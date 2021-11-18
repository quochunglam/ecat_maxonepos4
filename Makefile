# define the compiler
CC=arm-linux-gnueabihf-gcc

# application name
MAIN=EC_SERVO_TEST

# source files
SRCS=main.c

# CFLAGS
CFLAGS += -I /home/hpc/workspace/ethercat_install/usr/local/include
CFLAGS += -O3 -Wall -g

# LDLIBS
LDLIBS += -L /home/hpc/workspace/ethercat_install/usr/local/lib 
LDLIBS += -lethercat
LDLIBS += -lrt
LDLIBS += -lm

# OBJS
OBJS = $(SRCS:.c=.o)

all: $(MAIN)

$(MAIN): $(OBJS)
	$(CC) $(CFLAGS) -o $(MAIN) $(OBJS) $(LDLIBS)

clean:
	$(RM) *.o *~ $(MAIN)
