#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <rtdm/rtdm.h>
#include <native/task.h>
#include <native/sem.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <rtdk.h>
#include <pthread.h>

#include "ecrt.h"
#define     Bool                              int
#define     false                             0
#define     true                              1
#define     ETHERCAT_STATUS_OP                0x08
#define     STATUS_SERVO_ENABLE_BIT           (0x04)
//master status
typedef enum  _SysWorkingStatus
{
    SYS_WORKING_POWER_ON,
    SYS_WORKING_SAFE_MODE,
    SYS_WORKING_OP_MODE,
    SYS_WORKING_LINK_DOWN,
    SYS_WORKING_IDLE_STATUS       //System is idle
}SysWorkingStatus;

typedef  struct  _GSysRunningParm
{
    SysWorkingStatus   m_gWorkStatus;
}GSysRunningParm;

GSysRunningParm    gSysRunning;

RT_TASK InterpolationTask;

int run = 1;
int ecstate = 0;
#define     CLOCK_TO_USE         CLOCK_REALTIME
#define     NSEC_PER_SEC         (1000000000L)
#define     TIMESPEC2NS(T)       ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
static int64_t  system_time_base = 0LL;
//Get the current system time
RTIME system_time_ns(void)
{
    struct timespec  rt_time;
    clock_gettime(CLOCK_TO_USE, &rt_time);
    RTIME time = TIMESPEC2NS(rt_time);
    return time - system_time_base;
}
/****************************************************************************/

// EtherCAT
ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domainServoInput = NULL;
static ec_domain_state_t domainServoInput_state = {};
static ec_domain_t *domainServoOutput = NULL;
static ec_domain_state_t domainServoOutput_state = {};

static uint8_t *domainOutput_pd = NULL;
static uint8_t *domainInput_pd = NULL;

static ec_slave_config_t *sc_asda[3];
static ec_slave_config_state_t sc_asda_state[3];
/****************************************************************************/
#define asda_Pos0 0, 0
#define asda_Pos1 0, 1
#define asda_Pos2 0, 2
#define asda 0x000001dd, 0x10305070
// offsets for PDO entries
static unsigned int  cntlwd[3];
static unsigned int  ipData[3];
static unsigned int  status[3];
static unsigned int  actpos[3];
static unsigned int  actvel[3];
// process data
ec_pdo_entry_reg_t domainServoOutput_regs[] = {
    {asda_Pos0, asda, 0x6040, 0x00, &cntlwd[0], NULL},
    {asda_Pos0, asda, 0x607a, 0x00, &ipData[0], NULL},
    {asda_Pos1, asda, 0x6040, 0x00, &cntlwd[1], NULL},
    {asda_Pos1, asda, 0x607a, 0x00, &ipData[1], NULL},
    {asda_Pos2, asda, 0x6040, 0x00, &cntlwd[2], NULL},
    {asda_Pos2, asda, 0x607a, 0x00, &ipData[2], NULL},
    {}
};
ec_pdo_entry_reg_t domainServoInput_regs[] = {
    {asda_Pos0, asda, 0x6064, 0x00, &actpos[0], NULL},
    {asda_Pos0, asda, 0x6041, 0x00, &status[0], NULL},
    {asda_Pos0, asda, 0x606c, 0x00, &actvel[0], NULL},
    {asda_Pos1, asda, 0x6064, 0x00, &actpos[1], NULL},
    {asda_Pos1, asda, 0x6041, 0x00, &status[1], NULL},
    {asda_Pos1, asda, 0x606c, 0x00, &actvel[1], NULL},
    {asda_Pos2, asda, 0x6064, 0x00, &actpos[2], NULL},
    {asda_Pos2, asda, 0x6041, 0x00, &status[2], NULL},
    {asda_Pos2, asda, 0x606c, 0x00, &actvel[2], NULL},
    {}
};
/****************************************************************************/
/* Master 0, Slave 0
 * Vendor ID:       0x000001dd
 * Product code:    0x10305070
 * Revision number: 0x02040608
 */
//《 Delta_ASDA2-E_rev4-00_XML_TSE_20160620.xml》
static ec_pdo_entry_info_t asda_pdo_entries_output[] = {
    { 0x6040, 0x00, 16 }, //control word
    { 0x607a, 0x00, 32 }  //TargetPosition

};

static ec_pdo_entry_info_t asda_pdo_entries_input[] = {
    { 0x6064, 0x00, 32 }, //actualPosition
    { 0x6041, 0x00, 16 }, //status word
    { 0x606c, 0x00, 32 }, //Velocity actual value
};

//RxPDO
static ec_pdo_info_t asda_pdo_1600[] = {
    { 0x1600, 2, asda_pdo_entries_output },
};
//TxPDO
static ec_pdo_info_t asda_pdo_1a00[] = {
    { 0x1A00, 3, asda_pdo_entries_input },
};

static ec_sync_info_t asda_syncs[] = {
//    { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
//    { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
    { 2, EC_DIR_OUTPUT, 1, asda_pdo_1600, EC_WD_DISABLE },
    { 3, EC_DIR_INPUT, 1, asda_pdo_1a00, EC_WD_DISABLE },
    { 0xff }
};
/****************************************************************************/
int ConfigPDO()
{
    /********************/
    printf("xenomai Configuring PDOs...\n");
    domainServoOutput = ecrt_master_create_domain(master);
    if (!domainServoOutput) {
        return -1;
    }
    domainServoInput = ecrt_master_create_domain(master);
    if (!domainServoInput) {
        return -1;
    }
    /********************/
    printf("xenomai Creating slave configurations...\n");
    sc_asda[0] =
        ecrt_master_slave_config(master, asda_Pos0, asda);
    if (!sc_asda[0]) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    sc_asda[1] =
        ecrt_master_slave_config(master, asda_Pos1, asda);
    if (!sc_asda[1]) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    sc_asda[2] =
        ecrt_master_slave_config(master, asda_Pos2, asda);
    if (!sc_asda[2]) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    /********************/
    if (ecrt_slave_config_pdos(sc_asda[0], EC_END, asda_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(sc_asda[1], EC_END, asda_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(sc_asda[2], EC_END, asda_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    /********************/
    if (ecrt_domain_reg_pdo_entry_list(domainServoOutput, domainServoOutput_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }
    if (ecrt_domain_reg_pdo_entry_list(domainServoInput, domainServoInput_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    fprintf(stderr, "Creating SDO requests...\n");
    for(int i=0;i<3;i++)
    {
        ecrt_slave_config_sdo8(sc_asda[i], 0x6060, 0, 8);
        ecrt_slave_config_sdo8(sc_asda[i], 0x60C2, 1, 1);
    }
    return 0;
}


/*****************************************************************************
 * Realtime task
 ****************************************************************************/

void rt_check_domain_state(void)
{
    ec_domain_state_t ds = {};
    ec_domain_state_t ds1 = {};
    //domainServoInput
    ecrt_domain_state(domainServoInput, &ds);
    if (ds.working_counter != domainServoInput_state.working_counter) {
        rt_printf("domainServoInput: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domainServoInput_state.wc_state) {
        rt_printf("domainServoInput: State %u.\n", ds.wc_state);
    }
    domainServoInput_state = ds;
    //domainServoOutput
    ecrt_domain_state(domainServoOutput, &ds1);
    if (ds1.working_counter != domainServoOutput_state.working_counter) {
        rt_printf("domainServoOutput: WC %u.\n", ds1.working_counter);
    }
    if (ds1.wc_state != domainServoOutput_state.wc_state) {
        rt_printf("domainServoOutput: State %u.\n", ds1.wc_state);
    }
    domainServoOutput_state = ds1;
}

/****************************************************************************/

void rt_check_master_state(void)
{
    ec_master_state_t ms;

	ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        rt_printf("%u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master_state.al_states) {
        rt_printf("AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master_state.link_up) {
        rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/****************************************************************************/
void check_slave_config_states(void)
{
    ec_slave_config_state_t s;
    ecrt_slave_config_state(sc_asda[0],&s);
    if (s.al_state != sc_asda_state[0].al_state)
        printf("sc_asda_state[0]: State 0x%02X.\n", s.al_state);
    if (s.online != sc_asda_state[0].online)
        printf("sc_asda_state[0]: %s.\n", s.online ? "online" : "offline");
    if (s.operational != sc_asda_state[0].operational)
        printf("sc_asda_state[0]: %soperational.\n",s.operational ? "" : "Not ");
    sc_asda_state[0] = s;

    ec_slave_config_state_t s1;
    ecrt_slave_config_state(sc_asda[1],&s1);
    if (s1.al_state != sc_asda_state[1].al_state)
        printf("sc_asda_state[1]: State 0x%02X.\n", s1.al_state);
    if (s1.online != sc_asda_state[1].online)
        printf("sc_asda_state[1]: %s.\n", s1.online ? "online" : "offline");
    if (s1.operational != sc_asda_state[1].operational)
        printf("sc_asda_state[1]: %soperational.\n",s1.operational ? "" : "Not ");
    sc_asda_state[1] = s1;

    ec_slave_config_state_t s2;
    ecrt_slave_config_state(sc_asda[2],&s2);
    if (s2.al_state != sc_asda_state[2].al_state)
        printf("sc_asda_state[2]: State 0x%02X.\n", s2.al_state);
    if (s2.online != sc_asda_state[2].online)
        printf("sc_asda_state[2]: %s.\n", s2.online ? "online" : "offline");
    if (s2.operational != sc_asda_state[2].operational)
        printf("sc_asda_state[2]: %soperational.\n",s2.operational ? "" : "Not ");
    sc_asda_state[2] = s2;
}
/****************************************************************************/
void ReleaseMaster()
{
    if(master)
    {
        printf("xenomai End of Program, release master\n");
        ecrt_release_master(master);
        master = NULL;
    }
}
/****************************************************************************/
int ActivateMaster()
{
    int ret;
    printf("xenomai Requesting master...\n");
    if(master)
        return 0;
    master = ecrt_request_master(0);
    if (!master) {
        return -1;
    }

    ConfigPDO();

    // configure SYNC signals for this slave
    ecrt_slave_config_dc(sc_asda[0], 0x0300, 1000000, 0, 0, 0);
    ecrt_slave_config_dc(sc_asda[1], 0x0300, 1000000, 0, 0, 0);
    ecrt_slave_config_dc(sc_asda[2], 0x0300, 1000000, 0, 0, 0);
    ecrt_master_application_time(master, system_time_ns());
    ret = ecrt_master_select_reference_clock(master, NULL);
    if (ret < 0) {
        fprintf(stderr, "xenomai Failed to select reference clock: %s\n",
                strerror(-ret));
        return ret;
    }

    /********************/
    printf("xenomai Activating master...\n");
    if (ecrt_master_activate(master)) {
        printf("xenomai Activating master...failed\n");
        return -1;
    }
    /********************/
    if (!(domainInput_pd = ecrt_domain_data(domainServoInput))) {
        fprintf(stderr, "xenomai Failed to get domain data pointer.\n");
        return -1;
    }
    if (!(domainOutput_pd = ecrt_domain_data(domainServoOutput))) {
        fprintf(stderr, "xenomai Failed to get domain data pointer.\n");
        return -1;
    }
    printf("xenomai Activating master...success\n");
    return 0;
}
/****************************************************************************/
void DriverEtherCAT()
{
    static int curpos = 0;
    //Just boot (need to wait for other operations to complete), return to wait for the next cycle
    if(gSysRunning.m_gWorkStatus == SYS_WORKING_POWER_ON)
        return ;

    static int cycle_counter = 0;
    cycle_counter++;
    if(cycle_counter >= 90*1000){
        cycle_counter = 0;
	run = 0;
    }

    // receive EtherCAT frames
    ecrt_master_receive(master);
    ecrt_domain_process(domainServoOutput);
    ecrt_domain_process(domainServoInput);
    rt_check_domain_state();

    if (!(cycle_counter % 500)) {
        rt_check_master_state();
        check_slave_config_states();
    }

    //State machine operation
    switch (gSysRunning.m_gWorkStatus)
    {
    case SYS_WORKING_SAFE_MODE:{
        //Check whether the master station is in OP mode, if not, adjust to OP mode
        rt_check_master_state();
        check_slave_config_states();
        if((master_state.al_states & ETHERCAT_STATUS_OP))
        {
            int tmp = true;
            for(int i=0;i<3;i++)
            {
                if(sc_asda_state[i].al_state != ETHERCAT_STATUS_OP)
                {
                    tmp = false;
                    break ;
                }
            }
            if(tmp)
            {
                ecstate = 0;
                gSysRunning.m_gWorkStatus = SYS_WORKING_OP_MODE;
                printf("xenomai SYS_WORKING_OP_MODE\n");
            }
        }
    }break;

    case SYS_WORKING_OP_MODE:
    {
        ecstate++;
        //Enable servo
        if(ecstate <= 16)
        {
            for(int i=0;i<3;i++)
            {
                switch (ecstate){
                case 1:
                    EC_WRITE_U16(domainOutput_pd + cntlwd[i], 0x80);       
                    break;
                case 7:
                    curpos = EC_READ_S32(domainInput_pd + actpos[i]);       
                    EC_WRITE_S32(domainOutput_pd + ipData[i], EC_READ_S32(domainInput_pd + actpos[i])); 
                    printf("x@rtITP >>> Axis %d current position = %d\n", i, curpos);
                    break;
                case 9:
                    EC_WRITE_U16(domainOutput_pd + cntlwd[i], 0x06);
                    break;
                case 11:
                    EC_WRITE_U16(domainOutput_pd + cntlwd[i], 0x07);
                    break;
                case 13:
                    EC_WRITE_U16(domainOutput_pd + cntlwd[i], 0xF);
                    break;
                }
            }
        }
        else {
            int tmp  = true;
            for(int i=0;i<3;i++)
            {
                if((EC_READ_U16(domainInput_pd + status[i]) & (STATUS_SERVO_ENABLE_BIT)) == 0)
                {
                    tmp = false;
                    break ;
                }
            }
            if(tmp)
            {
                ecstate = 0;
                gSysRunning.m_gWorkStatus = SYS_WORKING_IDLE_STATUS;
                printf("xenomai SYS_WORKING_IDLE_STATUS\n");
            }
        }
    }break;

    default:
    {
        if (!(cycle_counter % 1000)) {
            printf("curpos = %d\t",curpos);
            printf("asda0 actpos... %d\t",EC_READ_S32(domainInput_pd + actpos[0]));
            printf("asda1 actpos... %d\t",EC_READ_S32(domainInput_pd + actpos[1]));
            printf("asda2 actpos... %d\n",EC_READ_S32(domainInput_pd + actpos[2]));
        }
        curpos += 100;
        for(int i=0;i<3;i++)
        {
            EC_WRITE_S32(domainOutput_pd + ipData[i], curpos);
        }
    }break;
    }

    // write application time to master
    ecrt_master_application_time(master, system_time_ns());
    ecrt_master_sync_reference_clock(master);   
    ecrt_master_sync_slave_clocks(master);      

    // send process data
    ecrt_domain_queue(domainServoOutput);
    ecrt_domain_queue(domainServoInput);
    ecrt_master_send(master);
}
/****************************************************************************/
void InterpolationThread(void *arg)
{
    RTIME wait, previous;
    previous = rt_timer_read();
    wait = previous;

	while (run) {
        wait += 1000000; //1ms
        //Delay the calling task (absolute).Delay the execution of the calling task until a given date is reached.
        rt_task_sleep_until(wait);
        DriverEtherCAT();
	}
}

/****************************************************************************
 * Signal handler
 ***************************************************************************/

void signal_handler(int sig)
{
    run = 0;
}

/****************************************************************************
 * Main function
 ***************************************************************************/

int main(int argc, char *argv[])
{
    int ret;
    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    rt_print_auto_init(1);
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
    mlockall(MCL_CURRENT | MCL_FUTURE);

    gSysRunning.m_gWorkStatus = SYS_WORKING_POWER_ON;
    if(gSysRunning.m_gWorkStatus == SYS_WORKING_POWER_ON)
    {
        ActivateMaster();
        ecstate = 0;
        gSysRunning.m_gWorkStatus = SYS_WORKING_SAFE_MODE;
        printf("xenomai SYS_WORKING_SAFE_MODE\n"); 
    }

    ret = rt_task_create(&InterpolationTask, "InterpolationTask", 0, 99, T_FPU);
    if (ret < 0) {
        fprintf(stderr, "xenomai Failed to create task: %s\n", strerror(-ret));
        return -1;
    }

    printf("Starting InterpolationTask...\n");
    ret = rt_task_start(&InterpolationTask, &InterpolationThread, NULL);
    if (ret < 0) {
        fprintf(stderr, "xenomai Failed to start task: %s\n", strerror(-ret));
        return -1;
    }

	while (run) {
		rt_task_sleep(50000000);
	}

    printf("xenomai Deleting realtime InterpolationTask task...\n");
    rt_task_delete(&InterpolationTask);

    ReleaseMaster();
    return 0;
}

/****************************************************************************/















