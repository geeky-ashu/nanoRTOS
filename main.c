/*--------------------------------------------------------------------------------
 * Application layer  -  hardware init, task functions, shell, and main().
 *--------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "kernel.h"
#include "uart.h"

/*------------------------------------------------------------------------------*/

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))

#define PB1  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4))) // PF4
#define PB2  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4))) // PC4
#define PB3  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4))) // PC5
#define PB4  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4))) // PC6
#define PB5  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4))) // PC7

static semaphore_t *keyPressed;
static semaphore_t *keyReleased;
static semaphore_t *flashReq;
static semaphore_t *resource;
static semaphore_t *resource2;   // second resource  -  grabbed in reverse order by important
                                  // to create a potential deadlock under preemptive mode.
static queue_t     *msgQ;
static event_t     *sysEvents;   // event group: bit0=keyDown, bit1=keyUp, bit2=timerTick

#define EV_KEY_DOWN   (1u << 0)
#define EV_KEY_UP     (1u << 1)
#define EV_TIMER_TICK (1u << 2)

#define MAX_CHAR   80
#define max_fields  6

static char    str[MAX_CHAR + 1];
static uint8_t position[max_fields];
static uint8_t fields = 0;
static char    type[max_fields];

/*------------------------------------------------------------------------------*/

static void initHw(void)
{
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
                 | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    SYSCTL_GPIOHBCTL_R = 0;
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC
                   | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;

    // Port A  -  off-board LED outputs (PA5, PA7)
    GPIO_PORTA_DIR_R  |= 0xA0;
    GPIO_PORTA_DR2R_R |= 0xA0;
    GPIO_PORTA_DEN_R  |= 0xA0;

    // Port E  -  off-board LED outputs (PE1, PE3)
    GPIO_PORTE_DIR_R  |= 0x0A;
    GPIO_PORTE_DR2R_R |= 0x0A;
    GPIO_PORTE_DEN_R  |= 0x0A;

    // Port F  -  on-board blue LED (PF2)
    GPIO_PORTF_DIR_R  |= 0x04;
    GPIO_PORTF_DR2R_R |= 0x04;
    GPIO_PORTF_DEN_R  |= 0x04;

    // Port F  -  PB1 input with pull-up (PF4)
    GPIO_PORTF_DR2R_R |= 0x0A;
    GPIO_PORTF_DEN_R  |= 0x10;
    GPIO_PORTF_PUR_R  |= 0x10;

    // Port C  -  PB2-PB5 inputs with pull-ups (PC4-PC7)
    GPIO_PORTC_DR2R_R |= 0xF0;
    GPIO_PORTC_DEN_R  |= 0xF0;
    GPIO_PORTC_PUR_R  |= 0xF0;

    // UART0  -  PA0 (RX), PA1 (TX), 115200 baud 8N1
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    GPIO_PORTA_DEN_R  |= 3;
    GPIO_PORTA_AFSEL_R |= 3;
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
    UART0_CTL_R   = 0;
    UART0_CC_R    = UART_CC_CS_SYSCLK;
    UART0_IBRD_R  = 21;    // 40 MHz / (16 * 115200)
    UART0_FBRD_R  = 45;
    UART0_LCRH_R  = UART_LCRH_WLEN_8 | UART_LCRH_FEN;
    UART0_CTL_R   = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
    // UART0 TX interrupt  -  give it priority 5 (lower than SysTick/PendSV)
    // and enable in NVIC (IRQ5).  TX interrupt mask (TXIM) is left off here
    // and is armed dynamically by putcUart0 when the circular buffer has data.
    // NVIC_PRI1_R bits [15:13] control IRQ5 priority (top 3 bits of byte 1).
    NVIC_PRI1_R = (NVIC_PRI1_R & 0xFFFF1FFF) | (5 << 13);
    NVIC_EN0_R |= 1 << 5;   // enable UART0 interrupt (IRQ5)
}

static void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");
    __asm("WMS_LOOP1:   SUB  R1, #1");
    __asm("             CBZ  R1, WMS_DONE1");
    __asm("             NOP");
    __asm("             B    WMS_LOOP1");
    __asm("WMS_DONE1:   SUB  R0, #1");
    __asm("             CBZ  R0, WMS_DONE0");
    __asm("             B    WMS_LOOP0");
    __asm("WMS_DONE0:");
}

static uint8_t readPbs(void)
{
    uint8_t value = 0;
    if (!PB1) value |= 1;
    if (!PB2) value |= 2;
    if (!PB3) value |= 4;
    if (!PB4) value |= 8;
    if (!PB5) value |= 16;
    return value;
}

void idle(void)
{
    while (true)
    {
        vTaskCheckin();               // reset watchdog  -  idle always running
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        __asm(" WFI ");   // park CPU until next interrupt  -  reduces active power
        vTaskYield();
    }
}

/*--------------------------------------------------------------------------------
 * producer sends an incrementing counter into msgQ every 500 ms.
 * consumer receives it and accumulates a running total  -  queue depth
 * and waiter counts are visible via the QUEUE shell command.
 *--------------------------------------------------------------------------------*/
void producer(void)
{
    uint32_t count = 0;
    while (true)
    {
        xQueueSend(msgQ, count++);
        vTaskDelay(500);
    }
}

void consumer(void)
{
    uint32_t value;
    uint32_t total = 0;
    while (true)
    {
        xQueueReceive(msgQ, &value);   // blocks until data arrives
        total += value;                // accumulate  -  inspect state via QUEUE command
    }
}

/*--------------------------------------------------------------------------------
 * eventSetter fires every 1 s and asserts EV_TIMER_TICK.
 * eventWaiter blocks on wait-ANY: wakes on either a key press (EV_KEY_DOWN)
 * or the 1-second timer tick (EV_TIMER_TICK). This mirrors real UI patterns
 * where you want to react to whichever event arrives first.
 *--------------------------------------------------------------------------------*/
void eventSetter(void)
{
    while (true)
    {
        vTaskDelay(1000);
        xEventGroupSetBits(sysEvents, EV_TIMER_TICK);
    }
}

void eventWaiter(void)
{
    while (true)
    {
        // wait-any: wake on key press OR 1-second tick, whichever comes first
        xEventGroupWaitBits(sysEvents, EV_KEY_DOWN | EV_TIMER_TICK, 0);
        ORANGE_LED ^= 1;
    }
}

void flash4Hz(void)
{
    uint32_t lastWake = uptimeTicks;
    while (true)
    {
        GREEN_LED ^= 1;
        vTaskDelayUntil(&lastWake, 125);  // exact 250 ms period; no drift
    }
}

void oneshot(void)
{
    while (true)
    {
        xSemaphoreTake(flashReq);
        YELLOW_LED = 1;
        vTaskDelay(1000);
        YELLOW_LED = 0;
    }
}

static void partOfLengthyFn(void)
{
    waitMicrosecond(1000);
    vTaskYield();
}

void lengthyFn(void)
{
    uint16_t cycleCount;
    while (true)
    {
        vTaskCheckin();              // reset watchdog at top of each work cycle
        xSemaphoreTake(resource);    // grab resource first  (A → B order)
        xSemaphoreTake(resource2);   // then resource2
        for (cycleCount = 0; cycleCount < 4000; cycleCount++)
            partOfLengthyFn();
        RED_LED ^= 1;
        xSemaphoreGive(resource2);
        xSemaphoreGive(resource);
    }
}

void readKeys(void)
{
    uint8_t buttons;
    while (true)
    {
        xSemaphoreTake(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            vTaskYield();
        }
        xSemaphoreGive(keyPressed);
        xEventGroupSetBits(sysEvents, EV_KEY_DOWN);   // signal event flag waiters
        if (buttons & 1)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if (buttons & 2)
        {
            xSemaphoreGive(flashReq);
            RED_LED = 0;
        }
        if (buttons & 4)
            xTaskCreate(flash4Hz, "FLASH4HZ", 0);
        if (buttons & 8)
            vTaskDelete(flash4Hz);
        if (buttons & 16)
            vTaskPrioritySet(lengthyFn, 4);
        vTaskYield();
    }
}

void debounce(void)
{
    uint8_t count;
    while (true)
    {
        xSemaphoreTake(keyPressed);
        count = 10;
        while (count != 0)
        {
            vTaskDelay(10);
            if (readPbs() == 0) count--;
            else                count = 10;
        }
        xSemaphoreGive(keyReleased);
    }
}

void uncooperative(void)
{
    while (true)
    {
        while (readPbs() == 8) {}
        vTaskYield();
    }
}

/*--------------------------------------------------------------------------------
 * important grabs resource2 first, then resource (B -> A order).
 * If lengthyFn holds resource and important holds resource2 simultaneously,
 * neither can proceed  -  classic circular wait / deadlock.
 * Run WATCHDOG ON then PREEMPT ON then type DEADLOCK to observe.
 *--------------------------------------------------------------------------------*/
void important(void)
{
    while (true)
    {
        xSemaphoreTake(resource2);   // grab resource2 first  (B → A order)
        xSemaphoreTake(resource);    // then resource  -  may block if lengthyFn holds it
        vTaskCheckin();              // alive once both locks acquired
        BLUE_LED = 1;
        vTaskDelay(1000);
        BLUE_LED = 0;
        xSemaphoreGive(resource);
        xSemaphoreGive(resource2);
    }
}

static void getString(void)
{
    uint8_t count = 0;
    uint8_t charCount = 0;
    char ch;
    while (charCount < 80 && (ch = getcUart0()) != 0x0D)
    {
        if (ch == 0x08)
        {
            if (count > 0)
            {
                count--;
                putsUart0("\b \b");   // back, overwrite with space, back again
            }
        }
        else if (ch >= 0x20)           // printable ASCII only (space and above)
        {
            putcUart0(ch);
            str[count++] = ch;
            if (count > MAX_CHAR) return;
            charCount++;
        }
    }
    putsUart0("\n\r");               // move to new line after Enter
    str[count] = '\0';
}

static void parseInput(void)
{
    uint8_t charIdx, fieldIdx = 0;
    for (charIdx = 0; charIdx < 80; charIdx++)
    {
        if (str[charIdx] < 48 && str[charIdx] != 38)
            str[charIdx] = '\0';
    }
    for (charIdx = 0; charIdx < 80; charIdx++)
    {
        if (str[charIdx] >= 48 && str[charIdx] <= 57)
        {
            position[fieldIdx] = charIdx; fields++; type[fieldIdx++] = 'n';
            while (str[charIdx] >= 48 && str[charIdx] <= 57) charIdx++;
        }
        else if ((str[charIdx] >= 65 && str[charIdx] <= 90) || (str[charIdx] >= 97 && str[charIdx] <= 122))
        {
            position[fieldIdx] = charIdx; fields++; type[fieldIdx++] = 'a';
            while ((str[charIdx] >= 65 && str[charIdx] <= 90) || (str[charIdx] >= 97 && str[charIdx] <= 122)) charIdx++;
        }
        else if (str[charIdx] == 38)
        {
            position[fieldIdx] = charIdx; fields++; type[fieldIdx++] = 'a';
            while (str[charIdx] == 38) charIdx++;
        }
    }
}

static char* stringUpper(char *string)
{
    uint16_t charIdx = 0;
    while (string[charIdx] != '\0')
    {
        if (string[charIdx] >= 'a' && string[charIdx] <= 'z')
            string[charIdx] -= 32;
        charIdx++;
    }
    return string;
}

static bool isCommand(char command[], uint8_t minArgs)
{
    char *token = &str[position[0]];
    return (strcmp(stringUpper(token), command) == 0) && (fields > minArgs);
}

static uint16_t getNumber(uint8_t field)
{
    return atoi(&str[position[field]]);
}

static char* getStringArg(uint8_t field)
{
    return &str[position[field]];
}

static void getPidNumber(char *name)
{
    uint8_t taskIdx = 0;
    bool valid = false;
    char string[10];
    while (taskIdx < MAX_TASKS)
    {
        if (strcmp(tcb[taskIdx].name, stringUpper(name)) == 0)
        {
            ltoa((uint32_t)tcb[taskIdx].pid, string);
            putsUart0("\n\r"); putsUart0(string); putsUart0("\n\r");
            valid = true;
        }
        taskIdx++;
    }
    if (!valid) putsUart0("\n\rPlease enter valid name\n\r");
}

static void reniceTask(char *name, uint8_t priority)
{
    uint8_t taskIdx = 0;
    bool valid = false;
    while (taskIdx < MAX_TASKS)
    {
        if (strcmp(tcb[taskIdx].name, stringUpper(name)) == 0)
        {
            vTaskPrioritySet((task_fn_t)tcb[taskIdx].pid, priority);
            putsUart0("\n\rpriority updated\n\r");
            valid = true;
        }
        taskIdx++;
    }
    if (!valid) putsUart0("\n\rPlease enter valid name\n\r");
}

static void clearFields(void)
{
    fields = 0;
    uint8_t strIdx;
    for (strIdx = 0; strIdx < 80; strIdx++) str[strIdx] = '\0';
}

static void reset(void)
{
    NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
}

static const char *stateStr(uint8_t state)
{
    switch (state)
    {
    case STATE_UNRUN:     return "UNRUN    ";
    case STATE_READY:     return "READY    ";
    case STATE_BLOCKED:   return "BLOCKED  ";
    case STATE_DELAYED:   return "DELAYED  ";
    case STATE_RUN:       return "RUN      ";
    case STATE_SUSPENDED: return "SUSPENDED";
    default:              return "INVALID  ";
    }
}

/*--------------------------------------------------------------------------------
 * Deadlock detection: walk blocked tasks and check for circular waits.
 * Returns true and prints a report if a cycle is found.
 * Algorithm: for each blocked task, follow the "waiting on semaphore held by"
 * chain depth-first; if we revisit a task we have a cycle.
 *--------------------------------------------------------------------------------*/
static bool detectDeadlock(void)
{
    bool found = false;
    uint8_t i;
    char string[10];
    for (i = 0; i < MAX_TASKS; i++)
    {
        if (tcb[i].state != STATE_BLOCKED || tcb[i].semaphore == NULL)
            continue;
        uint8_t visited[MAX_TASKS];
        uint8_t vcount = 0;
        uint8_t cur = i;
        uint8_t depth;
        for (depth = 0; depth < MAX_TASKS; depth++)
        {
            uint8_t v;
            for (v = 0; v < vcount; v++)
                if (visited[v] == cur) { found = true; break; }
            if (found) break;
            visited[vcount++] = cur;
            semaphore_t *sem = (semaphore_t*)tcb[cur].semaphore;
            uint8_t holder = MAX_TASKS;
            uint8_t j;
            for (j = 0; j < MAX_TASKS; j++)
            {
                if (j != cur && tcb[j].state != STATE_INVALID
                        && tcb[j].semaphore == (void*)sem
                        && tcb[j].state != STATE_BLOCKED)
                { holder = j; break; }
            }
            if (holder == MAX_TASKS) break;
            cur = holder;
        }
        if (found)
        {
            putsUart0("\n\r[DEADLOCK] cycle detected: ");
            uint8_t v;
            for (v = 0; v < vcount; v++)
            {
                putsUart0(tcb[visited[v]].name);
                if (v < vcount - 1) putsUart0(" -> ");
            }
            putsUart0("\n\r");
            break;
        }
    }
    return found;
}

static void createBackgroundProcess(void)
{
    uint8_t taskIdx = 0;
    bool valid = false;
    while (taskIdx < MAX_TASKS)
    {
        if (strcmp(taskList[taskIdx].name, getStringArg(0)) == 0)
        {
            task_fn_t fn = (task_fn_t)taskList[taskIdx].pid;
            xTaskCreate(fn, taskList[taskIdx].name, taskList[taskIdx].priority);
            putsUart0("\n\radded background process\n\r");
            valid = true;
        }
        taskIdx++;
    }
    if (!valid) putsUart0("\n\rerror occured\n\r");
}

void shell(void)
{
    while (true)
    {
        // Print stack overflow message if SysTick detected a corrupted sentinel.
        // Done here, not in the ISR, because putsUart0 must not busy-wait in ISR.
        if (stackOverflow)
        {
            putsUart0("\n\r[FATAL] stack overflow");
            if (overflowTaskName != NULL)
            {
                putsUart0(": ");
                putsUart0(overflowTaskName);
                overflowTaskName = NULL;
            }
            putsUart0("\n\r");
            stackOverflow = false;
        }
        bool valid = false;
        getString();
        parseInput();

        if (isCommand("PS", 0))
        {
            // Snapshot kernel state under critical section so SysTick cannot
            // tear a 64-bit avgTaskTime read between its two 32-bit halves.
            uint32_t snapPid[MAX_TASKS];
            uint32_t snapCpu[MAX_TASKS];
            uint8_t  snapState[MAX_TASKS];
            uint8_t  snapPri[MAX_TASKS];
            uint16_t snapHwm[MAX_TASKS];
            char    *snapName[MAX_TASKS];
            uint8_t  snapCount = 0;

            taskENTER_CRITICAL();
            uint8_t taskIdx = 0;
            while (taskIdx < MAX_TASKS)
            {
                if (tcb[taskIdx].state != STATE_INVALID)
                {
                    snapPid[snapCount]   = (uint32_t)tcb[taskIdx].pid;
                    snapCpu[snapCount]   = (uint32_t)avgTaskTime[taskIdx];
                    snapState[snapCount] = tcb[taskIdx].state;
                    snapPri[snapCount]   = tcb[taskIdx].currentPriority;
                    snapHwm[snapCount]   = tcb[taskIdx].stackHwm;
                    snapName[snapCount]  = tcb[taskIdx].name;
                    snapCount++;
                }
                taskIdx++;
            }
            taskEXIT_CRITICAL();

            putsUart0("\n\t\tPID\t\tCPU%\t\tState\t\tPri\tStack\t\tName\n\r");
            uint8_t snapIdx = 0;
            char string[10];
            while (snapIdx < snapCount)
            {
                uint8_t length;
                uint32_t cpu = snapCpu[snapIdx];
                putsUart0("\t\t");
                ltoa(snapPid[snapIdx], string); putsUart0(string);
                putsUart0("\t\t");
                ltoa(cpu, string);
                length = strlen(string);
                if (length <= 2 && cpu != 0)
                {
                    putsUart0("0."); putsUart0(string);
                }
                else if (cpu != 0 && length > 2)
                {
                    string[length - 2] = '\0';
                    putsUart0(string); putsUart0(".");
                    ltoa(cpu, string);
                    length = strlen(string);
                    putsUart0(&string[length - 2]);
                }
                putsUart0("\t\t");
                putsUart0(stateStr(snapState[snapIdx]));
                putsUart0("\t");
                ltoa(snapPri[snapIdx], string); putsUart0(string);
                putsUart0("\t");
                ltoa(snapHwm[snapIdx], string); putsUart0(string);
                putsUart0("/256w\t\t");
                putsUart0(snapName[snapIdx]); putsUart0("\n\r");
                snapIdx++;
            }
            valid = true;
        }

        if (isCommand("KILL", 1))
        {
            // accept either a numeric PID or a task name
            uint8_t killIdx = 0;
            bool killedByName = false;
            char *arg = getStringArg(1);
            // try name match first
            while (killIdx < MAX_TASKS)
            {
                if (tcb[killIdx].state != STATE_INVALID &&
                    strcmp(tcb[killIdx].name, stringUpper(arg)) == 0)
                {
                    vTaskDelete((task_fn_t)tcb[killIdx].pid);
                    killedByName = true;
                    break;
                }
                killIdx++;
            }
            if (!killedByName)
            {
                task_fn_t pid = (task_fn_t)getNumber(1);
                vTaskDelete(pid);
            }
            if (!threadDestroyed)
                putsUart0("\n\rEnter valid PID or name\n\r");
            else
                threadDestroyed = false;
            valid = true;
        }

        if (isCommand("PIDOF", 1))
        {
            getPidNumber(getStringArg(1));
            valid = true;
        }

        if (isCommand("SEM", 0))
        {
            uint8_t count = 0;
            char string[10];
            putsUart0("\n\t\tCount\t\tWaiting\t\tHead PID\tName\n\r");
            while (count < semaphoreCount)
            {
                if (semaphores[count].name != '\0')
                {
                    putsUart0("\t\t");
                    ltoa(semaphores[count].count, string); putsUart0(string);
                    putsUart0("\t\t");
                    ltoa(semaphores[count].queueSize, string); putsUart0(string);
                    putsUart0("\t\t\t");
                    ltoa(semaphores[count].processQueue[0], string); putsUart0(string);
                    putsUart0("\t\t");
                    putsUart0(semaphores[count].name); putsUart0("\n\r");
                }
                count++;
            }
            valid = true;
        }

        if (isCommand("QUEUE", 0))
        {
            uint8_t qi = 0;
            char string[10];
            putsUart0("\n\t\tLen\tUsed\tTxWait\tRxWait\tName\n\r");
            while (qi < queueCount)
            {
                putsUart0("\t\t");
                ltoa(queues[qi].length,      string); putsUart0(string); putsUart0("\t");
                ltoa(queues[qi].count,       string); putsUart0(string); putsUart0("\t");
                ltoa(queues[qi].sendWaitSize,string); putsUart0(string); putsUart0("\t");
                ltoa(queues[qi].recvWaitSize,string); putsUart0(string); putsUart0("\t");
                putsUart0(queues[qi].name); putsUart0("\n\r");
                qi++;
            }
            valid = true;
        }

        if (isCommand("EVENTS", 0))
        {
            uint8_t ei = 0;
            char string[10];
            putsUart0("\n\t\tBits\t\tWaiting\tName\n\r");
            while (ei < eventCount)
            {
                putsUart0("\t\t0x");
                ltoa(events[ei].bits, string); putsUart0(string);
                putsUart0("\t\t");
                ltoa(events[ei].waitSize, string); putsUart0(string);
                putsUart0("\t");
                putsUart0(events[ei].name); putsUart0("\n\r");
                ei++;
            }
            valid = true;
        }

        if (isCommand("WATCHDOG", 1))
        {
            if (strcmp(stringUpper(getStringArg(1)), "ON") == 0)
            {
                uint8_t taskIdx = 0;
                while (taskIdx < MAX_TASKS)
                {
                    if (tcb[taskIdx].state != STATE_INVALID)
                    {
                        tcb[taskIdx].wdtTimeout = WDT_DEFAULT;
                        tcb[taskIdx].wdtCounter = WDT_DEFAULT;
                    }
                    taskIdx++;
                }
                wdtEnabled = true;
                putsUart0("\n\rwatchdog enabled (5 s)\n\r");
                valid = true;
            }
            else if (strcmp(stringUpper(getStringArg(1)), "OFF") == 0)
            {
                wdtEnabled = false;
                putsUart0("\n\rwatchdog disabled\n\r");
                valid = true;
            }
        }

        if (isCommand("DEADLOCK", 0))
        {
            if (!detectDeadlock())
                putsUart0("\n\rno deadlock detected\n\r");
            valid = true;
        }

        if (isCommand("RESET", 0))
        {
            reset();
            putsUart0("\n\rreset performed\n\r");
            valid = true;
        }

        if (isCommand("SETPRI", 2))
        {
            reniceTask(getStringArg(1), (uint8_t)getNumber(2));
            valid = true;
        }

        if (isCommand("UPTIME", 0))
        {
            uint32_t ticks = uptimeTicks;
            uint32_t secs  = ticks / 1000;
            uint32_t mins  = secs  / 60;
            uint32_t hrs   = mins  / 60;
            secs %= 60; mins %= 60;
            char string[10];
            putsUart0("\n\r");
            ltoa(hrs,  string); putsUart0(string); putsUart0("h ");
            ltoa(mins, string); putsUart0(string); putsUart0("m ");
            ltoa(secs, string); putsUart0(string); putsUart0("s\n\r");
            valid = true;
        }

        if (isCommand("HELP", 0))
        {
            putsUart0("\n\r");
            putsUart0("  PS                    list tasks: CPU%, state, priority, stack HWM\n\r");
            putsUart0("  KILL <pid|name>       delete a task\n\r");
            putsUart0("  PIDOF <name>          get PID of task\n\r");
            putsUart0("  SETPRI <name> <0-7>   set task priority (0=highest)\n\r");
            putsUart0("  SUSPEND <name>        suspend a task\n\r");
            putsUart0("  RESUME <name>         resume a suspended task\n\r");
            putsUart0("  SEM                   list semaphores\n\r");
            putsUart0("  QUEUE                 list message queues\n\r");
            putsUart0("  EVENTS                list event flag groups\n\r");
            putsUart0("  DEADLOCK              scan for circular semaphore waits\n\r");
            putsUart0("  WATCHDOG ON|OFF       per-task software watchdog (5 s)\n\r");
            putsUart0("  UPTIME                show time since boot\n\r");
            putsUart0("  PREEMPT ON|OFF        preemptive scheduling\n\r");
            putsUart0("  INHERIT ON|OFF        priority inheritance\n\r");
            putsUart0("  RESET                 system reset\n\r");
            putsUart0("  <name> &              restart background task\n\r");
            valid = true;
        }

        if (isCommand("SUSPEND", 1))
        {
            uint8_t taskIdx = 0;
            bool found = false;
            char *arg = stringUpper(getStringArg(1));
            while (taskIdx < MAX_TASKS)
            {
                if (tcb[taskIdx].state != STATE_INVALID &&
                    tcb[taskIdx].state != STATE_SUSPENDED &&
                    strcmp(tcb[taskIdx].name, arg) == 0)
                {
                    vTaskSuspend((task_fn_t)tcb[taskIdx].pid);
                    found = true;
                    break;
                }
                taskIdx++;
            }
            if (!found) putsUart0("\n\rtask not found\n\r");
            valid = true;
        }

        if (isCommand("RESUME", 1))
        {
            uint8_t taskIdx = 0;
            bool found = false;
            char *arg = stringUpper(getStringArg(1));
            while (taskIdx < MAX_TASKS)
            {
                if (tcb[taskIdx].state == STATE_SUSPENDED &&
                    strcmp(tcb[taskIdx].name, arg) == 0)
                {
                    vTaskResume((task_fn_t)tcb[taskIdx].pid);
                    found = true;
                    break;
                }
                taskIdx++;
            }
            if (!found) putsUart0("\n\rtask not found or not suspended\n\r");
            valid = true;
        }

        if (isCommand("PREEMPT", 1))
        {
            if (strcmp(stringUpper(getStringArg(1)), "ON") == 0)
            {
                uint8_t taskIdx = 0;
                if (!preemptiveReady)
                {
                    preemptiveReady = true;
                    while (taskIdx < MAX_TASKS)
                    {
                        if (tcb[taskIdx].state == STATE_UNRUN)
                        {
                            preemptiveReady = false;
                            break;
                        }
                        taskIdx++;
                    }
                }
                if (preemptiveReady) { preemptiveMode = true;  putsUart0("\n\rpreemptive mode on\n\r"); }
                else                  putsUart0("\n\rnot all tasks have run yet\n\r");
                valid = true;
            }
            else if (strcmp(stringUpper(getStringArg(1)), "OFF") == 0)
            {
                preemptiveMode = false;
                putsUart0("\n\rpreemptive mode off\n\r");
                valid = true;
            }
        }

        if (isCommand("INHERIT", 1))
        {
            if (strcmp(stringUpper(getStringArg(1)), "ON") == 0)
            {
                priorityInheritance = true;
                putsUart0("\n\rpriority inheritance on\n\r");
                valid = true;
            }
            else if (strcmp(stringUpper(getStringArg(1)), "OFF") == 0)
            {
                priorityInheritance = false;
                putsUart0("\n\rpriority inheritance off\n\r");
                valid = true;
            }
        }

        if (str[position[2]] == 38 || str[position[1]] == 38)
        {
            createBackgroundProcess();
            valid = true;
        }

        clearFields();
        if (!valid) putsUart0("\n\renter valid shell command\n\r");
        vTaskYield();
    }
}

int main(void)
{
    bool ok;

    initHw();
    rtosInit();

    GREEN_LED = 1; waitMicrosecond(250000);
    GREEN_LED = 0; waitMicrosecond(250000);

    keyPressed  = xSemaphoreCreateCounting(1, "keyPressed");
    keyReleased = xSemaphoreCreateCounting(0, "keyReleased");
    flashReq    = xSemaphoreCreateCounting(5, "flashReq");
    resource    = xSemaphoreCreateCounting(1, "resource");
    resource2   = xSemaphoreCreateCounting(1, "resource2");
    msgQ        = xQueueCreate(4, "msgQ");
    sysEvents   = xEventGroupCreate("sysEvents");

    ok  =  xTaskCreate(idle,          "IDLE",      7);
    ok &=  xTaskCreate(lengthyFn,     "LENGTHYFN", 6);
    ok &=  xTaskCreate(flash4Hz,      "FLASH4HZ",  2);
    ok &=  xTaskCreate(oneshot,       "ONESHOT",   2);
    ok &=  xTaskCreate(readKeys,      "READKEYS",  6);
    ok &=  xTaskCreate(debounce,      "DEBOUNCE",  6);
    ok &=  xTaskCreate(important,     "IMPORTANT", 0);
    ok &=  xTaskCreate(uncooperative, "UNCOOP",    5);
    ok &=  xTaskCreate(producer,      "PRODUCER",  5);
    ok &=  xTaskCreate(consumer,      "CONSUMER",  5);
    ok &=  xTaskCreate(eventSetter,   "EVTSETTER", 6);
    ok &=  xTaskCreate(eventWaiter,   "EVTWAITER", 6);
    ok &=  xTaskCreate(shell,         "SHELL",     4);

    if (ok)
        vTaskStartScheduler();   // never returns
    else
        RED_LED = 1;

    return 0;
}

/*------------------------------------------------------------------------------*/
