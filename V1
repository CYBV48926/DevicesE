#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <THREADSLib.h>

#define system_call_arguments_t sysargs
#define STATUS_SLEEP_BLOCKED 10
#define STATUS_DISK_BLOCKED  11
#include <Messaging.h>
#include <Scheduler.h>
#include <TList.h>
#include <libuser.h>
#include <SystemCalls.h>
#include <Devices.h>

/* Set the disk arm scheduling algorithm. */
#define DISK_ARM_ALG   DISK_ARM_ALG_FCFS

/* Globals and Data Structures for Clock Driver */
static int clockMbox;

typedef struct {
    int pid;
    int sleepTimeSeconds;
} ClockMessage;

typedef struct SleepNode {
    struct SleepNode* pNext;  /* Must be first for TList (offset 0) */
    struct SleepNode* pPrev;
    int pid;
    unsigned int wakeTime;    /* Time in microseconds when process should wake */
} SleepNode;

/* Ordering function to keep the sleep queue sorted by earliest wake time */
static int SleepOrder(void* p1, void* p2)
{
    SleepNode* n1 = (SleepNode*)p1;
    SleepNode* n2 = (SleepNode*)p2;

    if (n1->wakeTime < n2->wakeTime) return -1;
    if (n1->wakeTime > n2->wakeTime) return 1;
    return 0;
}

/* Prototypes */
static int ClockDriver(char*);
static int DiskDriver(char*);
extern int DevicesEntryPoint(char*);
static inline void checkKernelMode(const char* functionName);

void SysSleepHandler(system_call_arguments_t* args);
void SysDiskReadHandler(system_call_arguments_t* args);
void SysDiskWriteHandler(system_call_arguments_t* args);
void SysDiskInfoHandler(system_call_arguments_t* args);

/* System Calls Entry Point */
int SystemCallsEntryPoint(char* arg)
{
    char    buf[25];
    char    name[128];
    int     i;
    int     clockPID = 0;
    int     diskPids[THREADS_MAX_DISKS];
    int     status;

    checkKernelMode(__func__);

    /* Assign system call handlers */
    systemCallVector[SYS_SLEEP] = SysSleepHandler;
    systemCallVector[SYS_DISKREAD] = SysDiskReadHandler;
    systemCallVector[SYS_DISKWRITE] = SysDiskWriteHandler;
    systemCallVector[SYS_DISKINFO] = SysDiskInfoHandler;

    /* Initialize the process table */
    for (i = 0; i < MAXPROC; ++i)
    {
    }

    /* Create and start the clock driver */
    clockPID = k_spawn("Clock driver", ClockDriver, NULL, THREADS_MIN_STACK_SIZE, HIGHEST_PRIORITY);
    if (clockPID < 0)
    {
        console_output(TRUE, "start3(): Can't create clock driver\n");
        stop(1);
    }

    /* Create the disk drivers */
    for (i = 0; i < THREADS_MAX_DISKS; i++)
    {
        sprintf(buf, "%d", i);
        sprintf(name, "DiskDriver%d", i);
        diskPids[i] = k_spawn(name, DiskDriver, buf, THREADS_MIN_STACK_SIZE * 4, HIGHEST_PRIORITY);
        if (diskPids[i] < 0)
        {
            console_output(TRUE, "start3(): Can't create disk driver %d\n", i);
            stop(1);
        }
    }

    /* Create first user-level process and wait for it to finish */
    sys_spawn("DevicesEntryPoint", DevicesEntryPoint, NULL, 8 * THREADS_MIN_STACK_SIZE, 3);
    sys_wait(&status);

    return 0;
}

/* System Call Handlers */
void SysSleepHandler(system_call_arguments_t* args)
{
    ClockMessage msg;

    /* args->arg1 contains the number of seconds to sleep */
    msg.sleepTimeSeconds = (int)(long)args->arg1;
    msg.pid = k_getpid();

    /* Send the message to the clock driver without blocking */
    mailbox_send(clockMbox, &msg, sizeof(msg), FALSE);

    /* Safely block this process until the Clock Driver wakes it up */
    block(STATUS_SLEEP_BLOCKED);
}

void SysDiskReadHandler(system_call_arguments_t* args) { /* To be implemented */ }
void SysDiskWriteHandler(system_call_arguments_t* args) { /* To be implemented */ }
void SysDiskInfoHandler(system_call_arguments_t* args) { /* To be implemented */ }

/* Clock Driver */
static int ClockDriver(char* arg)
{
    int result;
    int status;
    TList sleepQueue;
    ClockMessage msg;
    SleepNode* head;

    /* Initialize the sleep queue (offset to pNext/pPrev is 0) */
    TListInitialize(&sleepQueue, 0, SleepOrder);

    /* Create the mailbox for receiving sleep requests */
    clockMbox = mailbox_create(MAXSLOTS, sizeof(ClockMessage));

    set_psr(get_psr() | PSR_INTERRUPTS);

    while (!signaled())
    {
        /* Wait for the hardware clock to tick */
        result = wait_device("clock", &status);
        if (result != 0)
        {
            return 0;
        }

        /* 1. Check for NEW sleep requests (non-blocking receive) */
        while (mailbox_receive(clockMbox, &msg, sizeof(msg), FALSE) == sizeof(msg))
        {
            SleepNode* newNode = (SleepNode*)malloc(sizeof(SleepNode));
            newNode->pid = msg.pid;

            /* read_time() returns microseconds. Convert seconds to microseconds. */
            newNode->wakeTime = read_time() + (msg.sleepTimeSeconds * 1000000);

            TListAddNodeInOrder(&sleepQueue, newNode);
        }

        /* 2. Wake up any processes whose time has come */
        head = (SleepNode*)sleepQueue.pHead;
        while (head != NULL && head->wakeTime <= read_time())
        {
            /* Pop the process off the queue */
            head = (SleepNode*)TListPopNode(&sleepQueue);

            /* Unblock it so the scheduler can run it again */
            unblock(head->pid);

            /* Free the node memory */
            free(head);

            /* Check the next process in line */
            head = (SleepNode*)sleepQueue.pHead;
        }
    }
    return 0;
}

/* Disk Driver Skeleton (Phase 3) */
static int DiskDriver(char* arg)
{
    int unit = atoi(arg);
    int status;
    char deviceName[16];

    /* Format the device name string (e.g., "disk0", "disk1") */
    sprintf(deviceName, "disk%d", unit);

    set_psr(get_psr() | PSR_INTERRUPTS);

    while (!signaled())
    {
        /* Temporarily block and wait for a disk interrupt so the driver
         * goes to sleep and stops hogging the CPU!
         */
        wait_device(deviceName, &status);
    }
    return 0;
}

/* Helper Functions */
struct psr_bits {
    unsigned int cur_int_enable : 1;
    unsigned int cur_mode : 1;
    unsigned int prev_int_enable : 1;
    unsigned int prev_mode : 1;
    unsigned int unused : 28;
};

union psr_values {
    struct psr_bits bits;
    unsigned int integer_part;
};

static inline void checkKernelMode(const char* functionName)
{
    union psr_values psrValue;

    psrValue.integer_part = get_psr();
    if (psrValue.bits.cur_mode == 0)
    {
        console_output(FALSE, "Kernel mode expected, but function called in user mode.\n");
        stop(1);
    }
}
