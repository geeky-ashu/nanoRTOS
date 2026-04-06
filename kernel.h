/*--------------------------------------------------------------------------------
 * Public API header  -  include in any translation unit that uses the RTOS.
 *--------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/

#ifndef KERNEL_H
#define KERNEL_H

#include <stdint.h>
#include <stdbool.h>

/*--------------------------------------------------------------------------------
 * Constants
 *--------------------------------------------------------------------------------*/

#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
#define MAX_TASKS      14
#define MAX_QUEUES     4   // maximum number of message queues
#define MAX_QUEUE_LEN  8   // maximum items per queue
#define MAX_EVENTS     4   // maximum number of event flag groups
#define WDT_DEFAULT    5000u // default watchdog period in ms (0 = disabled for that specific task)

// Task states
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_BLOCKED    3 // has run, but now blocked by semaphore
#define STATE_DELAYED    4 // has run, but now awaiting timer
#define STATE_RUN        5 // currently executing
#define STATE_SUSPENDED  6 // suspended  -  will not be scheduled until resumed

// Critical section  -  clears PRIMASK to block all maskable interrupts (including
// SysTick), preventing ISR-torn reads of kernel data from thread context.
// Keep critical sections short: no blocking calls, no UART inside.
#define taskENTER_CRITICAL()  __asm(" CPSID I ")
#define taskEXIT_CRITICAL()   __asm(" CPSIE I ")

// Pend a PendSV context switch.  DSB ensures the MMIO store to ICSR
// completes before subsequent instructions observe the pending state.
// ISB flushes the pipeline so the effect is visible immediately.
// Required per ARMv7-M TRM §B3.4.1 for memory-mapped NVIC writes.
#define pendSV()  do {                              \
    NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;       \
    __asm(" DSB ");                                 \
    __asm(" ISB ");                                 \
} while (0)

/*--------------------------------------------------------------------------------
 * Types
 *--------------------------------------------------------------------------------*/

// Task function pointer (FreeRTOS-style naming)
typedef void (*task_fn_t)(void);

// Semaphore  -  callers declare semaphore variables so struct must be visible here
typedef struct {
    char     *name;
    uint16_t  count;
    uint16_t  queueSize;
    uint32_t  processQueue[MAX_QUEUE_SIZE]; // stores pid of waiting tasks
} semaphore_t;

// Message queue  -  transfers uint32_t values between tasks.
// Blocks the sender when full and the receiver when empty.
typedef struct {
    char     *name;
    uint8_t   length;                       // capacity in items
    uint8_t   count;                        // items currently stored
    uint8_t   head;                         // dequeue index
    uint8_t   tail;                         // enqueue index
    uint32_t  buf[MAX_QUEUE_LEN];           // item storage
    uint32_t  sendWait[MAX_QUEUE_SIZE];     // PIDs blocked on full-queue send
    uint8_t   sendWaitSize;
    uint32_t  recvWait[MAX_QUEUE_SIZE];     // PIDs blocked on empty-queue recv
    uint8_t   recvWaitSize;
} queue_t;

// Event flag group  -  up to 32 independent bits, set by any task,
// waited on by any task (wait-any OR wait-all selectable).
typedef struct {
    char     *name;
    uint32_t  bits;                     // current set of asserted bits
    uint32_t  waitMask[MAX_QUEUE_SIZE]; // mask each blocked task is waiting for
    uint8_t   waitAll[MAX_QUEUE_SIZE];  // 1 = wait-all, 0 = wait-any
    uint32_t  waitPid[MAX_QUEUE_SIZE];  // PID of each blocked task
    uint8_t   waitSize;
} event_t;

// Task Control Block  -  exposed so the shell can read kernel state directly
struct _tcb
{
    uint8_t   state;           // see STATE_ values above
    void     *pid;             // used to uniquely identify thread
    void     *sp;              // saved stack pointer
    uint8_t   priority;        // 0=highest, 7=lowest
    uint8_t   currentPriority; // used for priority inheritance
    uint32_t  ticks;           // ticks until sleep complete
    char     *name;            // name of task used in ps command
    void     *semaphore;       // pointer to the semaphore blocking this thread
    uint8_t   skipCount;       // how many scheduler rounds this task has been skipped
    uint64_t  time;            // accumulated run time for CPU% calculation
    uint32_t  startTime;
    uint32_t  endTime;
    uint16_t  stackHwm;        // peak stack depth in words (high-water mark)
    uint32_t  queueMsg;        // pending send value or recv dest pointer (queue SVC)
    uint32_t  wdtTimeout;      // watchdog deadline in ms (0 = disabled)
    uint32_t  wdtCounter;      // counts down each SysTick ms; checkin resets it
    uint32_t  eventResult;     // bits that satisfied xEventGroupWaitBits (written by ISR)
};

// Task registry  -  populated by xTaskCreate, used by createBackgroundProcess
struct taskEntry
{
    void    *pid;
    char    *name;
    uint8_t  priority;
};

/*--------------------------------------------------------------------------------
 * Kernel globals (defined in kernel.c, accessed by shell in main.c)
 *--------------------------------------------------------------------------------*/

extern struct _tcb    tcb[MAX_TASKS];
extern struct taskEntry taskList[MAX_TASKS];
extern semaphore_t    semaphores[MAX_SEMAPHORES];
extern uint8_t        semaphoreCount;
extern uint64_t       totalTime;
extern uint64_t       avgTaskTime[MAX_TASKS];
extern uint32_t       uptimeTicks;
extern bool           stackOverflow;
extern char          *overflowTaskName;  // task name saved by systickIsr on sentinel failure
extern bool           preemptiveMode;
extern bool           preemptiveReady;
extern bool           priorityInheritance;
extern bool           threadDestroyed;
extern queue_t        queues[MAX_QUEUES];
extern uint8_t        queueCount;
extern event_t        events[MAX_EVENTS];
extern uint8_t        eventCount;
extern bool           wdtEnabled;   // global watchdog on/off switch

/*--------------------------------------------------------------------------------
 * RTOS kernel API
 *--------------------------------------------------------------------------------*/

void rtosInit(void);
void vTaskStartScheduler(void);

bool         xTaskCreate(task_fn_t fn, char name[], int priority);
void         vTaskDelete(task_fn_t fn);
void         vTaskPrioritySet(task_fn_t fn, uint8_t priority);
void         vTaskYield(void);
void         vTaskDelay(uint32_t ticks);
void         vTaskDelayUntil(uint32_t *lastWakeTime, uint32_t periodTicks);
void         vTaskSuspend(task_fn_t fn);
void         vTaskResume(task_fn_t fn);

semaphore_t* xSemaphoreCreateCounting(uint8_t count, char name[]);
void         xSemaphoreTake(semaphore_t *sem);
void         xSemaphoreGive(semaphore_t *sem);

queue_t*     xQueueCreate(uint8_t length, char name[]);
void         xQueueSend(queue_t *q, uint32_t value);
void         xQueueReceive(queue_t *q, uint32_t *value);

void         vTaskCheckin(void);   // reset this task's watchdog counter

event_t*     xEventGroupCreate(char name[]);
void         xEventGroupSetBits(event_t *eg, uint32_t bits);
uint32_t     xEventGroupWaitBits(event_t *eg, uint32_t mask, uint8_t waitAll);

/*--------------------------------------------------------------------------------
 * ISR handlers (referenced by startup vector table)
 *--------------------------------------------------------------------------------*/

void systickIsr(void);
void pendSvIsr(void);
void svCallIsr(void);
void hardFaultIsr(void);

/*------------------------------------------------------------------------------*/

#endif /* KERNEL_H */
