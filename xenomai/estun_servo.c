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
#include <math.h>
#define PI 3.1415926535898

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

static ec_slave_config_t *sc_estun;
static ec_slave_config_state_t sc_estun_state;
/****************************************************************************/
#define estun_Pos0 0, 0
#define estun 0x0000060a, 0x00000001
// offsets for PDO entries
static unsigned int  cntlwd;
static unsigned int  ipData;
static unsigned int  modes_of_operation;
static unsigned int  status;
static unsigned int  actpos;
static unsigned int  modes_of_operation_display;
static unsigned int  Homing_method;
static unsigned int speed_during_search_for_switch;
static unsigned int speed_during_search_for_zero;
static unsigned int homing_acceleration;
static unsigned int home_offset;

static int cur_status;
static int cur_mode;
// process data
ec_pdo_entry_reg_t domainServoOutput_regs[] = {
    {estun_Pos0, estun, 0x6040, 0x00, &cntlwd, NULL},
    {estun_Pos0, estun, 0x607a, 0x00, &ipData, NULL},
    {estun_Pos0, estun, 0x6060, 0x00, &modes_of_operation, NULL},			//6060 mode selection
    {estun_Pos0, estun, 0x6098, 0x00, &Homing_method, NULL},			//Homing_method mode selection
    {estun_Pos0, estun, 0x6099, 0x01, &speed_during_search_for_switch, NULL},			//speed_during_search_for_switch
    {estun_Pos0, estun, 0x6099, 0x02, &speed_during_search_for_zero, NULL},			//speed_during_search_for_zero
    {estun_Pos0, estun, 0x609a, 0x00, &homing_acceleration, NULL},			//homing_acceleration
    {estun_Pos0, estun, 0x607c, 0x00, &home_offset, NULL},			//home_offset
    {}
};
ec_pdo_entry_reg_t domainServoInput_regs[] = {
    {estun_Pos0, estun, 0x6064, 0x00, &actpos, NULL},
    {estun_Pos0, estun, 0x6041, 0x00, &status, NULL},
    {estun_Pos0, estun, 0x6061, 0x00, &modes_of_operation_display, NULL},
    {}
};

/****************************************************************************/
/* Master 0, Slave 0, "ESTUN ProNet"
 * Vendor ID:       0x0000060a
 * Product code:    0x00000001
 * Revision number: 0x00000001
 */

ec_pdo_entry_info_t estun_pdo_entries[] = {
    //CoE CSP Mode (RX)
    {0x6040, 0x00, 16}, /* Controlword */
    {0x607a, 0x00, 32}, /* Target Position */
    {0x6060, 0x00, 8},  /*modes of operation*/
    {0x6098, 0x00, 8},  /*Homing method*/
    {0x6099, 0x01, 32},  /*Speed during search for switch*/
    {0x6099, 0x02, 32},  /*Speed during search for zero*/
    {0x609a, 0x00, 32},  /*homing_acceleration*/
    {0x607c, 0x00, 32},  /*home_offset*/
    //CoE CSP Mode (TX)
    {0x6041, 0x00, 16}, /* Statusword */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x6061, 0x00, 8},  /*modes of operation display*/
};

ec_pdo_info_t estun_pdos[] = {
    {0x1600, 8, estun_pdo_entries + 0}, /* CoE CSP Mode (RX) */
    {0x1a00, 3, estun_pdo_entries + 8}, /* CoE CSP Mode (TX) */
};

ec_sync_info_t estun_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, estun_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, estun_pdos + 1, EC_WD_DISABLE},
    {0xff}
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
    sc_estun =
        ecrt_master_slave_config(master, estun_Pos0, estun);
    if (!sc_estun) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    /********************/
    if (ecrt_slave_config_pdos(sc_estun, EC_END, estun_syncs)) {
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

    // ecrt_slave_config_sdo8(sc_estun, 0x6060, 0, 8);
    // ecrt_slave_config_sdo8(sc_estun, 0x60C2, 1, 1);

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
    ecrt_slave_config_state(sc_estun,&s);
    if (s.al_state != sc_estun_state.al_state)
        printf("sc_estun_state: State 0x%02X.\n", s.al_state);
    if (s.online != sc_estun_state.online)
        printf("sc_estun_state: %s.\n", s.online ? "online" : "offline");
    if (s.operational != sc_estun_state.operational)
        printf("sc_estun_state: %soperational.\n",s.operational ? "" : "Not ");
    sc_estun_state = s;


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
    ecrt_slave_config_dc(sc_estun, 0x0300, 1000000, 0, 0, 0);
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
    static int curpos_offset = 0;
    static int i = 0;
    //Just boot (need to wait for other operations to complete), return to wait for the next cycle
    if(gSysRunning.m_gWorkStatus == SYS_WORKING_POWER_ON)
        return ;

    static int cycle_counter = 0;
    cycle_counter++;
    if(cycle_counter >= 120*1000){
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

            if(sc_estun_state.al_state != ETHERCAT_STATUS_OP)
            {
                tmp = false;
                break ;
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
        if(ecstate <= 1000)
        {

            switch (ecstate){
            case 1:
                EC_WRITE_U8(domainOutput_pd + modes_of_operation, 8);
                break;
            case 200:
                EC_WRITE_U16(domainOutput_pd + cntlwd, 0x80);    //Error reset   
                break;
            case 300:
               // curpos = EC_READ_S32(domainInput_pd + actpos);   
                curpos_offset  = EC_READ_S32(domainInput_pd + actpos);   
                EC_WRITE_S32(domainOutput_pd + ipData, EC_READ_S32(domainInput_pd + actpos)); 
                printf("x@rtITP >>> Axis  current position = %d\n", curpos);
                break;
            case 400:
                EC_WRITE_U16(domainOutput_pd + cntlwd, 0x06);
                break;
            case 500:
                EC_WRITE_U16(domainOutput_pd + cntlwd, 0x07);
                break;
            case 600:
                EC_WRITE_U16(domainOutput_pd + cntlwd, 0xF);
                break;
            }

            // if(ecstate<10)
            // {
            //     switch (ecstate){
            //     case 1:
            //         EC_WRITE_U8(domainOutput_pd + modes_of_operation, 8); //csp
            //         //EC_WRITE_U8(domainOutput_pd + modes_of_operation, 1); //pp
            // EC_WRITE_U16(domainOutput_pd + cntlwd, 0x80); //error reset
            //         break;
            //     case 7:
            //         curpos = EC_READ_S32(domainInput_pd + actpos);       
            //         EC_WRITE_S32(domainOutput_pd + ipData, EC_READ_S32(domainInput_pd + actpos)); 
            //         printf("x@rtITP >>> Axis  current position = %d\n", curpos);
            //         break;
            //     }
            // }
            // else{
            //     cur_status = EC_READ_U16(domainInput_pd + status);
    
            // /************The following is mainly to ensure that the motor enters Operation Enable*******/
            // /**Number of rows 231 235 239 243 Switch the motor to OP mode**/
            //     if((cur_status & 0x004f) == 0x0040)
            //     {
            //         EC_WRITE_U16(domainOutput_pd + cntlwd, 0x0006);
            //         printf("0x0006\n");
            //     }
            //     else if((cur_status & 0x006f) == 0x0021)
            //     {
            //         EC_WRITE_U16(domainOutput_pd + cntlwd, 0x0007);
            //         printf("0x0007\n");
            //     }
            //     else if((cur_status & 0x006f) == 0x023)
            //     {
            //         EC_WRITE_U16(domainOutput_pd + cntlwd, 0x000f);
            //         printf("0x000f\n");
            //     }
            //     // else if((cur_status & 0x06f) == 0x027)
            //     // {
            //     //     EC_WRITE_S32(domainOutput_pd + ipData, curpos);
            //     //     EC_WRITE_U16(domainOutput_pd + cntlwd, 0x001f);
            //     // }
            // }

        }
        else {
            printf("enable servo success!\n");
            cur_mode= EC_READ_U8(domainInput_pd + modes_of_operation_display);
            printf("modes_of_operation_display 0x6061 = %d\n",cur_mode);

            int tmp  = true;

            cur_status = EC_READ_U16(domainInput_pd + status);
            printf("status_world 0x6041 = %d\n",cur_status);

            if((EC_READ_U16(domainInput_pd + status) & (STATUS_SERVO_ENABLE_BIT)) == 0)
            {
                tmp = false;
                break ;
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
        cur_status = EC_READ_U16(domainInput_pd + status);
        // if (!(cycle_counter % 1000)) {
        //     printf("cur_status: %d\t", cur_status);
        //     printf("curpos = %d\t",curpos);
        //     printf("asda0 actpos... %d\n",EC_READ_S32(domainInput_pd + actpos));
        // }
        // int tmp = cycle_counter%10000;
        // if(tmp < 4000){
        //     curpos -= 1000;
        // }
        // else if(tmp>=4000 && tmp <5000) {
        //     curpos -= 0;
        // }
        // else if(tmp >=5000 && tmp < 9000){EC_WRITE_S32(domainOutput_pd + ipData, curpos);
        //     curpos += 1000;
        // }else{
        //     curpos -= 0;
        // }


        if(cycle_counter < 50 * 1000){
            if (!(cycle_counter % 2000)) {
                cur_status = EC_READ_U16(domainInput_pd + status);
                printf("cur_status: %d\t", cur_status);
                printf("curpos = %d\t",curpos);
                printf("asda0 actpos... %d\n",EC_READ_S32(domainInput_pd + actpos));
                if(i>=500){
                    i=0;
                    curpos_offset = EC_READ_S32(domainInput_pd + actpos);
                }
            }


            if(i<500)
            {
                int distance = 100000;
                int points = 500;
                double x;
                // int i=0;
                x = (double)(i) / (double)(points - 1);
                curpos = curpos_offset + (int)(distance * (x - sin(2*PI*x) / 2 / PI));       
                i++;
            } 
            EC_WRITE_S32(domainOutput_pd + ipData, curpos);
        }

        // if(cycle_counter = 55*1000){
        //     EC_WRITE_U8(domainOutput_pd + modes_of_operation, 6);
        // }
        // if(cycle_counter = 55*1000 +200){
        //     EC_WRITE_U8(domainOutput_pd + Homing_method, 1);
        // }
        // if(cycle_counter = 55*1000 +400){
        //     EC_WRITE_U16(domainOutput_pd + cntlwd, 0x1f);
        // }

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

