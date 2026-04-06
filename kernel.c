/*--------------------------------------------------------------------------------
 * Kernel implementation  -  scheduler, context switch ISRs, semaphore engine,
 * SVC handler.
 *--------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "kernel.h"
#include "uart.h"

/*------------------------------------------------------------------------------*/

struct _tcb     tcb[MAX_TASKS];
struct taskEntry taskList[MAX_TASKS];
semaphore_t     semaphores[MAX_SEMAPHORES];
uint8_t         semaphoreCount = 0;
queue_t         queues[MAX_QUEUES];
uint8_t         queueCount = 0;
event_t         events[MAX_EVENTS];
uint8_t         eventCount = 0;
bool            wdtEnabled = false;   // global watchdog on/off (toggled by shell)

uint32_t stack[MAX_TASKS][256];    // 1024-byte stack for each thread

uint32_t systemStackPointer;       // kernel stack pointer saved at rtosStart
uint32_t sp;                       // scratch SP used in context save/restore

uint64_t totalTime = 0;            // total accumulated ticks (used for CPU%)
uint32_t cpuStatsTimer = 10000;    // countdown to reset per-task time accumulators
uint64_t avgTaskTime[MAX_TASKS];   // EMA of per-task CPU% (units: hundredths of a percent)

uint8_t taskCurrent = 0;           // index of currently running task
uint8_t taskCount   = 0;           // total no of registered tasks

uint32_t uptimeTicks = 0;          // total 1ms ticks since boot
bool stackOverflow   = false;      // set when sentinel corruption detected
char    *overflowTaskName = NULL;  // name pointer saved by systickIsr for shell to print

#define STACK_SENTINEL 0xDEADC0DE

bool preemptiveMode   = false;     // false = cooperative, true = preemptive
bool preemptiveReady  = false;     // true once all tasks have run at least once
bool priorityInheritance = true;   // enable priority inheritance by default
bool threadDestroyed  = false;     // set when vTaskDelete succeeds; read by shell

static int  rtosScheduler(void);
static void deleteTaskSemaphore(semaphore_t *sem, task_fn_t fn);
static void setStackpointer(uint32_t sp);
static uint32_t getStackpointer(void);
static uint32_t getR0(void);
static uint32_t getR1(void);
static uint32_t getR2(void);
static uint32_t getSVCno(void);

/*------------------------------------------------------------------------------*/

void rtosInit(void)
{
    uint8_t taskIdx;
    taskCount = 0;
    for (taskIdx = 0; taskIdx < MAX_TASKS; taskIdx++)
    {
        tcb[taskIdx].state     = STATE_INVALID;
        tcb[taskIdx].pid       = 0;
        tcb[taskIdx].ticks     = 0;
        tcb[taskIdx].startTime = 0;
        tcb[taskIdx].endTime   = 0;
        tcb[taskIdx].time      = 0;
        tcb[taskIdx].skipCount = 0;
        tcb[taskIdx].stackHwm  = 0;
        tcb[taskIdx].queueMsg  = 0;
        tcb[taskIdx].wdtTimeout = 0;
        tcb[taskIdx].wdtCounter = 0;
        tcb[taskIdx].eventResult = 0;
        stack[taskIdx][0]      = STACK_SENTINEL;
    }
}

static int rtosScheduler(void)
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        if (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN)
        {
            if (tcb[task].skipCount >= tcb[task].currentPriority)
            {
                tcb[task].skipCount = 0;
                ok = true;
            }
            tcb[task].skipCount++;
        }
    }
    return task;
}

static void setStackpointer(uint32_t sp)
{
    __asm(" MOV SP, R0 ");
    __asm(" BX LR ");
}

static uint32_t getStackpointer(void)
{
    __asm(" MOV R0, SP ");
    __asm(" BX LR ");
}

void vTaskStartScheduler(void)
{
    task_fn_t fn;
    taskCurrent = rtosScheduler();
    systemStackPointer = getStackpointer();
    sp = (uint32_t)tcb[taskCurrent].sp;
    setStackpointer(sp);
    tcb[taskCurrent].state = STATE_RUN;
    fn = (task_fn_t)tcb[taskCurrent].pid;
    NVIC_ST_CTRL_R  = NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN;
    NVIC_ST_RELOAD_R = 0x9C40;               // 1 ms tick at 40 MHz
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
    (*fn)();                                 // launch first task  -  never returns
}

bool xTaskCreate(task_fn_t fn, char name[], int priority)
{
    __asm("  SVC  #01 ");
    return getR0() != 0;
}

void vTaskDelete(task_fn_t fn)
{
    __asm("  SVC #06 ");
}

void vTaskPrioritySet(task_fn_t fn, uint8_t priority)
{
    uint8_t taskIdx = 0;
    taskENTER_CRITICAL();          // protect dual-write of priority + currentPriority
    while (taskIdx < MAX_TASKS)
    {
        if (tcb[taskIdx].pid == fn)
        {
            tcb[taskIdx].priority        = priority;
            tcb[taskIdx].currentPriority = priority;
        }
        taskIdx++;
    }
    taskEXIT_CRITICAL();
}

semaphore_t* xSemaphoreCreateCounting(uint8_t count, char name[])
{
    semaphore_t *sem = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        sem = &semaphores[semaphoreCount++];
        sem->count     = count;
        sem->name      = name;
        sem->queueSize = 0;
    }
    return sem;
}

void xSemaphoreTake(semaphore_t *sem)
{
    __asm("  SVC #04 ");
}

void xSemaphoreGive(semaphore_t *sem)
{
    __asm("  SVC #05 ");
}

queue_t* xQueueCreate(uint8_t length, char name[])
{
    queue_t *q = NULL;
    if (queueCount < MAX_QUEUES && length <= MAX_QUEUE_LEN)
    {
        q               = &queues[queueCount++];
        q->name         = name;
        q->length       = length;
        q->count        = 0;
        q->head         = 0;
        q->tail         = 0;
        q->sendWaitSize = 0;
        q->recvWaitSize = 0;
    }
    return q;
}

void xQueueSend(queue_t *q, uint32_t value)
{
    __asm(" SVC #09 ");
}

void xQueueReceive(queue_t *q, uint32_t *value)
{
    __asm(" SVC #10 ");
}

// Reset this task's watchdog counter.  Must be called before wdtTimeout elapses.
void vTaskCheckin(void)
{
    tcb[taskCurrent].wdtCounter = tcb[taskCurrent].wdtTimeout;
}

event_t* xEventGroupCreate(char name[])
{
    event_t *eg = NULL;
    if (eventCount < MAX_EVENTS)
    {
        eg           = &events[eventCount++];
        eg->name     = name;
        eg->bits     = 0;
        eg->waitSize = 0;
    }
    return eg;
}

void xEventGroupSetBits(event_t *eg, uint32_t bits)
{
    __asm(" SVC #11 ");
}

uint32_t xEventGroupWaitBits(event_t *eg, uint32_t mask, uint8_t waitAll)
{
    __asm(" SVC #12 ");
    return tcb[taskCurrent].eventResult;
}

void vTaskYield(void)
{
    __asm("  SVC #02 ");
}

void vTaskDelay(uint32_t tick)
{
    __asm("  SVC #03 ");
}

void vTaskDelayUntil(uint32_t *lastWakeTime, uint32_t periodTicks)
{
    uint32_t target = *lastWakeTime + periodTicks;
    *lastWakeTime   = target;              // advance before sleeping
    uint32_t now    = uptimeTicks;
    if ((int32_t)(target - now) > 0)      // signed: handles 32-bit wrap safely
        vTaskDelay(target - now);
    // if overdue (body ran too long), skip sleep and try to catch up next period
}

void vTaskSuspend(task_fn_t fn)
{
    __asm("  SVC #07 ");
}

void vTaskResume(task_fn_t fn)
{
    __asm("  SVC #08 ");
}

void systickIsr(void)
{
    uint8_t taskIdx = 0;
    uptimeTicks++;
    while (taskIdx < MAX_TASKS)
    {
        if (tcb[taskIdx].state == STATE_DELAYED)
        {
            if (tcb[taskIdx].ticks > 0)
                tcb[taskIdx].ticks--;
            if (tcb[taskIdx].ticks == 0)
                tcb[taskIdx].state = STATE_READY;
        }
        if (cpuStatsTimer == 0)
        {
            uint64_t intervalCpu = 0;
            if (totalTime != 0)
                intervalCpu = (tcb[taskIdx].time * 10000) / totalTime;
            avgTaskTime[taskIdx] = (avgTaskTime[taskIdx] * 9 + intervalCpu) / 10;
            tcb[taskIdx].time = 0;
        }
        if (tcb[taskIdx].state != STATE_INVALID && stack[taskIdx][0] != STACK_SENTINEL)
        {
            stackOverflow    = true;
            overflowTaskName = tcb[taskIdx].name;  // shell task prints the message
        }
        // per-task software watchdog: count down and flag if expired
        if (wdtEnabled && tcb[taskIdx].wdtTimeout > 0
                && tcb[taskIdx].state != STATE_INVALID)
        {
            if (tcb[taskIdx].wdtCounter > 0)
                tcb[taskIdx].wdtCounter--;
            if (tcb[taskIdx].wdtCounter == 0)
            {
                // reuse overflowTaskName path  -  shell prints [WDT] prefix
                stackOverflow    = true;
                overflowTaskName = tcb[taskIdx].name;
                // reset so the message fires once, not every tick
                tcb[taskIdx].wdtCounter = tcb[taskIdx].wdtTimeout;
            }
        }
        taskIdx++;
    }
    if (cpuStatsTimer == 0)
    {
        cpuStatsTimer = 10000;
        totalTime = 0;
    }
    else
    {
        cpuStatsTimer--;
    }
    if (preemptiveMode)
        pendSV();
}

void pendSvIsr(void)
{
    __asm("  PUSH  {r4-r11} ");           // save callee-saved regs of outgoing task
    sp = getStackpointer();
    tcb[taskCurrent].sp = (void*)sp;

    {
        uint16_t depth = (uint16_t)((uint32_t*)&stack[taskCurrent][255] - (uint32_t*)sp);
        if (depth > tcb[taskCurrent].stackHwm)
            tcb[taskCurrent].stackHwm = depth;
    }

    setStackpointer(systemStackPointer);

    // accumulate run time for the outgoing task
    tcb[taskCurrent].endTime = NVIC_ST_CURRENT_R;
    if (tcb[taskCurrent].startTime != 0 && tcb[taskCurrent].startTime > tcb[taskCurrent].endTime)
    {
        tcb[taskCurrent].time += (tcb[taskCurrent].startTime - tcb[taskCurrent].endTime);
        totalTime             += (tcb[taskCurrent].startTime - tcb[taskCurrent].endTime);
    }
    if (tcb[taskCurrent].startTime != 0 && tcb[taskCurrent].startTime < tcb[taskCurrent].endTime)
    {
        tcb[taskCurrent].time += (tcb[taskCurrent].startTime + (0x9C40 - tcb[taskCurrent].endTime));
        totalTime             += (tcb[taskCurrent].startTime + (0x9C40 - tcb[taskCurrent].endTime));
    }

    taskCurrent = rtosScheduler();

    if (tcb[taskCurrent].state == STATE_READY)
    {
        tcb[taskCurrent].startTime = NVIC_ST_CURRENT_R;
        sp = (uint32_t)tcb[taskCurrent].sp;
        setStackpointer(sp);
        __asm("  POP  {r4-r11} ");        // restore callee-saved regs of incoming task
    }
    if (tcb[taskCurrent].state == STATE_UNRUN)
    {
        sp = (uint32_t)tcb[taskCurrent].sp;
        setStackpointer(sp);
        uint32_t volatile *stackAddress = (uint32_t*)sp;
        *stackAddress = (uint32_t)tcb[taskCurrent].pid;
        __asm("  LDR R5, [SP] ");
        __asm("  POP { R7 } ");           // clear one stack slot
        __asm("  MOV  R4, #0x1000000 ");
        __asm("  PUSH  { R4 } ");         // xPSR
        __asm("  PUSH  { R5 } ");         // PC (task entry)
        __asm("  MOV  R6, #0xFF000000 ");
        __asm("  ORR  R6, R6, #0x00FF0000 ");
        __asm("  ORR  R6, R6, #0x00FF0000 ");
        __asm("  ORR  R6, R6, #0x0000FF00 ");
        __asm("  ORR  R6, R6, #0x000000F9 ");
        __asm("  PUSH  { R6 } ");         // LR = EXC_RETURN (0xFFFFFFF9)
        __asm("  PUSH  { R12 } ");
        __asm("  PUSH  { r0- r3 } ");
        __asm("  BX  R6 ");               // exception return  -  launches the task
    }
}

static uint32_t getR0(void) { __asm(" BX LR "); }
static uint32_t getR1(void) { __asm(" MOV R0, R1 "); }
static uint32_t getR2(void) { __asm(" MOV R0, R2 "); }
static uint32_t getSVCno(void)
{
    __asm(" LDR R1, [SP, #88]");
    __asm(" SUB R1,  #2 ");
    __asm(" LDR R0 , [R1] ");
}

static void deleteTaskSemaphore(semaphore_t *sem, task_fn_t fn)
{
    uint8_t queueIdx = 0;
    while (queueIdx < sem->queueSize)
    {
        if (sem->processQueue[queueIdx] == (uint32_t)fn)
        {
            // compact: shift remaining entries down to preserve FIFO order
            uint8_t shiftIdx = queueIdx;
            while (shiftIdx < sem->queueSize - 1)
            {
                sem->processQueue[shiftIdx] = sem->processQueue[shiftIdx + 1];
                shiftIdx++;
            }
            sem->processQueue[sem->queueSize - 1] = 0;
            sem->queueSize--;
            // do not increment queueIdx  -  recheck the slot we just filled
        }
        else
        {
            queueIdx++;
        }
    }
}

/*--------------------------------------------------------------------------------
 * hardFaultIsr  -  extracts the exception frame the CPU pushed onto PSP/MSP
 * and prints each register before resetting. The assembly stub determines
 * which stack was active (PSP in thread mode, MSP in handler mode) and
 * passes its base address to the C handler as a pointer.
 *--------------------------------------------------------------------------------*/
void hardFaultHandler(uint32_t *frame)
{
    // Hardware exception frame layout (ARMv7-M §B1.5.6):
    //   frame[0]=R0  frame[1]=R1  frame[2]=R2  frame[3]=R3
    //   frame[4]=R12 frame[5]=LR  frame[6]=PC  frame[7]=xPSR
    char string[10];
    putsUart0("\n\r[HARDFAULT] stacked frame:\n\r");
    putsUart0("  PC   = 0x"); ltoa(frame[6], string); putsUart0(string); putsUart0("\n\r");
    putsUart0("  LR   = 0x"); ltoa(frame[5], string); putsUart0(string); putsUart0("\n\r");
    putsUart0("  xPSR = 0x"); ltoa(frame[7], string); putsUart0(string); putsUart0("\n\r");
    putsUart0("  R0   = 0x"); ltoa(frame[0], string); putsUart0(string); putsUart0("\n\r");
    putsUart0("  R1   = 0x"); ltoa(frame[1], string); putsUart0(string); putsUart0("\n\r");
    putsUart0("  R2   = 0x"); ltoa(frame[2], string); putsUart0(string); putsUart0("\n\r");
    putsUart0("  R3   = 0x"); ltoa(frame[3], string); putsUart0(string); putsUart0("\n\r");
    putsUart0("  R12  = 0x"); ltoa(frame[4], string); putsUart0(string); putsUart0("\n\r");
    // uart0TxIsr cannot preempt HardFault (lower priority); flush the circular
    // buffer directly so all the above lines actually reach the terminal.
    uart0TxFlush();
    // reset so the device recovers rather than spinning
    NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
    while (true) {}
}

/*--------------------------------------------------------------------------------
 * Assembly trampoline: tests EXC_RETURN bit 2 to decide which stack held
 * the frame, then calls the C handler with the correct pointer in R0.
 *--------------------------------------------------------------------------------*/
void hardFaultIsr(void)
{
    __asm(" TST   LR, #4          ");
    __asm(" ITE   EQ              ");
    __asm(" MRSEQ R0, MSP         ");
    __asm(" MRSNE R0, PSP         ");
    __asm(" B     hardFaultHandler");
}

void svCallIsr(void)
{
    uint32_t  r0        = getR0();
    uint32_t  r1        = getR1();
    uint32_t  r2        = getR2();
    uint32_t  svcNumber = getSVCno();
    task_fn_t fn        = (task_fn_t)r0;
    uint32_t  tick      = r0;
    semaphore_t *sem    = (semaphore_t*)r0;
    queue_t     *q      = (queue_t*)r0;
    event_t     *eg     = (event_t*)r0;
    char     *name      = (char*)r1;
    uint32_t  priority  = r2;
    uint8_t   newTaskIdx   = 0;
    uint32_t  fnThumbAddr;
    bool      found         = false;
    uint8_t   targetTaskIdx = 0;

    switch (svcNumber & 0xFF)
    {
    case 1:   // xTaskCreate
        if (taskCount < MAX_TASKS)
        {
            while (!found && (newTaskIdx < MAX_TASKS))
                found = (tcb[newTaskIdx++].pid == fn);

            if (!found)
            {
                newTaskIdx = 0;
                while (tcb[newTaskIdx].state != STATE_INVALID) { newTaskIdx++; }
                tcb[newTaskIdx].state          = STATE_UNRUN;
                tcb[newTaskIdx].pid            = fn;
                tcb[newTaskIdx].sp             = &stack[newTaskIdx][255];
                tcb[newTaskIdx].priority       = priority;
                tcb[newTaskIdx].currentPriority = priority;
                tcb[newTaskIdx].name           = name;
                taskList[newTaskIdx].pid       = fn;
                taskList[newTaskIdx].name      = name;
                taskList[newTaskIdx].priority  = priority;
                taskCount++;
            }
        }
        break;

    case 2:   // vTaskYield
        tcb[taskCurrent].state = STATE_READY;
        pendSV();
        break;

    case 3:   // vTaskDelay
        tcb[taskCurrent].ticks = tick;
        tcb[taskCurrent].state = STATE_DELAYED;
        pendSV();
        break;

    case 4:   // xSemaphoreTake
        tcb[taskCurrent].semaphore = sem;
        if (sem->count > 0)
        {
            sem->count--;
        }
        else
        {
            sem->processQueue[sem->queueSize] = (uint32_t)tcb[taskCurrent].pid;
            sem->queueSize++;
            tcb[taskCurrent].state = STATE_BLOCKED;
            uint8_t inheritIdx = 0;
            while (inheritIdx < MAX_TASKS && priorityInheritance)
            {
                if (tcb[inheritIdx].semaphore == sem && tcb[inheritIdx].state != STATE_BLOCKED)
                {
                    if (tcb[inheritIdx].currentPriority > tcb[taskCurrent].currentPriority)
                        tcb[inheritIdx].currentPriority = tcb[taskCurrent].currentPriority;
                }
                inheritIdx++;
            }
            pendSV();
        }
        break;

    case 5:   // xSemaphoreGive
        sem->count++;
        if (sem->queueSize > 0)
        {
            // unblock the longest-waiting task (index 0 = FIFO head)
            uint8_t waitTaskIdx = 0;
            while (waitTaskIdx < MAX_TASKS)
            {
                if ((uint32_t)tcb[waitTaskIdx].pid == sem->processQueue[0]
                        && tcb[waitTaskIdx].state != STATE_INVALID)
                    tcb[waitTaskIdx].state = STATE_READY;
                waitTaskIdx++;
            }
            // shift remaining waiters down
            uint8_t shiftIdx = 0;
            while (shiftIdx < sem->queueSize - 1)
            {
                sem->processQueue[shiftIdx] = sem->processQueue[shiftIdx + 1];
                shiftIdx++;
            }
            sem->processQueue[sem->queueSize - 1] = 0;
            sem->queueSize--;
            sem->count--;
            uint8_t restorePriorityIdx = 0;
            while (restorePriorityIdx < MAX_TASKS)
            {
                if (tcb[restorePriorityIdx].semaphore == sem && tcb[restorePriorityIdx].currentPriority != tcb[restorePriorityIdx].priority)
                    tcb[restorePriorityIdx].currentPriority = tcb[restorePriorityIdx].priority;
                restorePriorityIdx++;
            }
            pendSV();
        }
        break;

    case 6:   // vTaskDelete
        fnThumbAddr = ((uint32_t)fn) | 1;   // set ARM Thumb bit to match stored PID
        fn = (task_fn_t)fnThumbAddr;
        while (targetTaskIdx < MAX_TASKS)
        {
            if ((strcmp(tcb[targetTaskIdx].name, "SHELL") == 0) && (tcb[targetTaskIdx].pid == fn))
            {
                putsUart0("\n\rthread cannot be destroyed\n\r");
            }
            else if (tcb[targetTaskIdx].pid == fn)
            {
                tcb[targetTaskIdx].state    = STATE_INVALID;
                tcb[targetTaskIdx].pid      = 0;
                tcb[targetTaskIdx].ticks    = 0;
                tcb[targetTaskIdx].time     = 0;
                tcb[targetTaskIdx].sp       = 0;
                tcb[targetTaskIdx].semaphore = 0;
                taskCount--;
                // remove from any semaphore wait queues
                uint8_t semIdx;
                for (semIdx = 0; semIdx < semaphoreCount; semIdx++)
                    deleteTaskSemaphore(&semaphores[semIdx], fn);
                putsUart0("\n\rthread destroyed\n\r");
                threadDestroyed = true;
            }
            targetTaskIdx++;
        }
        break;

    case 7:   // vTaskSuspend
        fnThumbAddr = ((uint32_t)fn) | 1;
        fn = (task_fn_t)fnThumbAddr;
        while (targetTaskIdx < MAX_TASKS)
        {
            if (tcb[targetTaskIdx].pid == fn && tcb[targetTaskIdx].state != STATE_INVALID)
                tcb[targetTaskIdx].state = STATE_SUSPENDED;
            targetTaskIdx++;
        }
        pendSV();
        break;

    case 8:   // vTaskResume
        fnThumbAddr = ((uint32_t)fn) | 1;
        fn = (task_fn_t)fnThumbAddr;
        while (targetTaskIdx < MAX_TASKS)
        {
            if (tcb[targetTaskIdx].pid == fn && tcb[targetTaskIdx].state == STATE_SUSPENDED)
                tcb[targetTaskIdx].state = STATE_READY;
            targetTaskIdx++;
        }
        break;

    case 9:   // xQueueSend
    {
        uint32_t val = r1;
        if (q->recvWaitSize > 0)
        {
            // direct handoff: skip the buffer, write straight to the waiting receiver
            uint8_t wi = 0;
            while (wi < MAX_TASKS)
            {
                if ((uint32_t)tcb[wi].pid == q->recvWait[0]
                        && tcb[wi].state != STATE_INVALID)
                {
                    *(uint32_t*)tcb[wi].queueMsg = val;
                    tcb[wi].state = STATE_READY;
                    break;
                }
                wi++;
            }
            uint8_t si = 0;
            while (si < q->recvWaitSize - 1) { q->recvWait[si] = q->recvWait[si + 1]; si++; }
            q->recvWaitSize--;
            pendSV();
        }
        else if (q->count < q->length)
        {
            q->buf[q->tail] = val;
            q->tail         = (q->tail + 1) % q->length;
            q->count++;
            // no pendSV: let the sender continue; preemptive SysTick will schedule consumer
        }
        else
        {
            // queue full  -  block sender until a receiver drains a slot
            tcb[taskCurrent].queueMsg              = val;
            q->sendWait[q->sendWaitSize++]         = (uint32_t)tcb[taskCurrent].pid;
            tcb[taskCurrent].state                 = STATE_BLOCKED;
            pendSV();
        }
        break;
    }

    case 10:  // xQueueReceive
    {
        uint32_t *dest = (uint32_t*)r1;
        if (q->count > 0)
        {
            *dest   = q->buf[q->head];
            q->head = (q->head + 1) % q->length;
            q->count--;
            if (q->sendWaitSize > 0)
            {
                // a sender was waiting  -  absorb their pending value into the now-free slot
                uint8_t wi = 0;
                while (wi < MAX_TASKS)
                {
                    if ((uint32_t)tcb[wi].pid == q->sendWait[0]
                            && tcb[wi].state != STATE_INVALID)
                    {
                        q->buf[q->tail] = tcb[wi].queueMsg;
                        q->tail         = (q->tail + 1) % q->length;
                        q->count++;
                        tcb[wi].state   = STATE_READY;
                        break;
                    }
                    wi++;
                }
                uint8_t si = 0;
                while (si < q->sendWaitSize - 1) { q->sendWait[si] = q->sendWait[si + 1]; si++; }
                q->sendWaitSize--;
                pendSV();
            }
        }
        else
        {
            // queue empty  -  block receiver; save dest so the sender can write through it
            tcb[taskCurrent].queueMsg              = (uint32_t)dest;
            q->recvWait[q->recvWaitSize++]         = (uint32_t)tcb[taskCurrent].pid;
            tcb[taskCurrent].state                 = STATE_BLOCKED;
            pendSV();
        }
        break;
    }

    case 11:  // xEventGroupSetBits
    {
        uint32_t newBits = r1;
        eg->bits |= newBits;
        // wake any tasks whose wait condition is now satisfied
        uint8_t wi = 0;
        while (wi < eg->waitSize)
        {
            uint8_t satisfied = eg->waitAll[wi]
                ? ((eg->bits & eg->waitMask[wi]) == eg->waitMask[wi])
                : ((eg->bits & eg->waitMask[wi]) != 0);
            if (satisfied)
            {
                uint8_t ti = 0;
                while (ti < MAX_TASKS)
                {
                    if ((uint32_t)tcb[ti].pid == eg->waitPid[wi]
                            && tcb[ti].state != STATE_INVALID)
                    {
                        tcb[ti].eventResult = eg->bits & eg->waitMask[wi];
                        tcb[ti].state       = STATE_READY;
                        break;
                    }
                    ti++;
                }
                // compact waiter list
                uint8_t si = wi;
                while (si < eg->waitSize - 1)
                {
                    eg->waitMask[si] = eg->waitMask[si + 1];
                    eg->waitAll[si]  = eg->waitAll[si + 1];
                    eg->waitPid[si]  = eg->waitPid[si + 1];
                    si++;
                }
                eg->waitSize--;
                // clear bits consumed by a wait-all waiter
                if (eg->waitAll[wi]) eg->bits &= ~newBits;
            }
            else
            {
                wi++;
            }
        }
        pendSV();
        break;
    }

    case 12:  // xEventGroupWaitBits
    {
        uint32_t mask    = r1;
        uint8_t  waitAll = (uint8_t)r2;
        uint8_t  sat     = waitAll
            ? ((eg->bits & mask) == mask)
            : ((eg->bits & mask) != 0);
        if (sat)
        {
            tcb[taskCurrent].eventResult = eg->bits & mask;
            if (waitAll) eg->bits &= ~mask;   // consume bits on wait-all
        }
        else
        {
            uint8_t ws = eg->waitSize;
            eg->waitMask[ws] = mask;
            eg->waitAll[ws]  = waitAll;
            eg->waitPid[ws]  = (uint32_t)tcb[taskCurrent].pid;
            eg->waitSize++;
            tcb[taskCurrent].state = STATE_BLOCKED;
            pendSV();
        }
        break;
    }
    }
}

/*------------------------------------------------------------------------------*/
