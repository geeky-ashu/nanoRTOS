# nanoRTOS

A preemptive/cooperative real-time operating system kernel written from scratch for the ARM Cortex-M4F,
targeting the TI TM4C123GH6PM microcontroller.

The goal is a complete, readable reference implementation of an RTOS: round-robin scheduler, counting
semaphores with priority inheritance, message queues, event flag groups, per-task software watchdog, and
runtime deadlock detection  -  all in roughly 1,000 lines of C with zero library dependencies beyond a
single vendor header.

Every mechanism is visible in the source. There is no generated code, no middleware, no HAL abstraction.
The context switch is ~20 lines of C calling one assembly instruction. If something breaks, you can read
exactly why.

---

## Table of Contents

- [Features](#features)
- [Architecture](#architecture)
- [Files](#files)
- [Tasks](#tasks)
- [IPC objects](#ipc-objects)
- [Shell reference](#shell-reference)
- [Demonstrating key behaviours](#demonstrating-key-behaviours)
- [Building](#building)

---

## Features

**Scheduler**
- Round-robin with 8 priority levels and weighted skip-counting
- Cooperative and preemptive (1 ms SysTick) modes switchable at runtime
- `vTaskSuspend` / `vTaskResume`  -  pause/unpause without destroying tasks
- `vTaskDelayUntil()`  -  jitter-free periodic execution against an absolute wake time
- Idle task calls `WFI` between ticks to reduce active power

**Synchronisation & IPC**
- Counting semaphores with FIFO wait queues and priority inheritance
- Message queues (`xQueueSend` / `xQueueReceive`)  -  blocking, backpressure-safe, direct handoff when a receiver is already waiting
- Event flag groups (`xEventGroupSetBits` / `xEventGroupWaitBits`)  -  wait-any or wait-all on a 32-bit bitmask

**Reliability**
- `taskENTER_CRITICAL` / `taskEXIT_CRITICAL`  -  PRIMASK-based critical sections
- Stack sentinel (`0xDEADC0DE`) at the bottom of every task stack; overflow reported to shell via flag (never inside ISR)
- Per-task software watchdog  -  `vTaskCheckin()` resets counter; SysTick fires fatal alert if any task misses its deadline
- HardFault handler  -  assembly trampoline extracts stacked exception frame, prints PC/LR/xPSR/R0–R3 over UART, flushes TX buffer, resets

**Observability**
- EMA-filtered per-task CPU% using hardware SysTick counter as a stopwatch
- Per-task stack high-water mark updated on every context switch
- PS snapshot taken under critical section to prevent torn 64-bit reads
- Runtime deadlock detection  -  walks blocked-task chain looking for circular semaphore waits

**UART driver** (`uart.c` / `uart.h`)
- Interrupt-driven TX via 128-byte circular buffer; `putcUart0` enqueues and arms TXIM, ISR drains asynchronously
- `uart0TxFlush()` for blocking drain inside HardFault handler where UART ISR cannot preempt
- Blocking RX poll with `vTaskYield()` + `vTaskCheckin()` so shell stays schedulable and watchdog-safe

---

## Architecture

**Context switch flow**

```
task calls vTaskYield / vTaskDelay / xSemaphoreTake / xQueueSend / xEventGroupWaitBits ...
  → SVC exception fires
    → svCallIsr updates task state, pends PendSV
      → pendSvIsr saves r4-r11, picks next task via scheduler
        → if READY: restores r4-r11, exception return resumes task
        → if UNRUN: builds fake exception frame, BX LR launches task
```

SysTick fires every 1 ms. In cooperative mode it only decrements sleep timers and checks watchdogs.
In preemptive mode it also pends PendSV, forcibly switching tasks on every tick.

**PendSV for context switch**  -  Fixed at lowest exception priority (0xFF). Guarantees context switches never interrupt peripheral handlers. SysTick tail-chains into PendSV in preemptive mode without returning to thread code.

**Fake exception frame for UNRUN tasks**  -  ARM hardware exception return reads xPSR, PC, LR, R12, R0–R3 from the stack. `pendSvIsr` constructs this frame manually with the task entry as PC and `EXC_RETURN = 0xFFFFFFF9` as LR, then `BX LR` launches the task cleanly.

**`vTaskDelayUntil` vs `vTaskDelay`**  -  `vTaskDelay(N)` sleeps N ticks from wake time, accumulating drift. `vTaskDelayUntil` records the target tick before sleeping, keeping `flash4Hz` period stable to within one tick regardless of task body runtime.

**EMA for CPU%**  -  SysTick down-counter sampled at every context switch gives µs-resolution timing. Every 10,000 ticks (10 s) the per-task share feeds `avg = avg×9/10 + sample/10`, smoothing bursts without a sliding-window buffer.

**Critical section for PS snapshot**  -  `avgTaskTime[]` is `uint64_t`; on a 32-bit core a read is two LDR instructions. SysTick can update the value between them in preemptive mode. PS snapshots the full TCB inside `CPSID I` / `CPSIE I`, then prints from the snapshot with interrupts re-enabled.

**Interrupt-driven TX rationale**  -  The old `putsUart0` busy-waited on the TX FIFO. Called from `systickIsr` (every 1 ms for stack overflow messages), a full FIFO would stall the ISR and cause every task to miss its deadline for the entire stall. The circular buffer decouples the ISR completely: SysTick sets a flag, the shell task prints, `uart0TxIsr` drains the FIFO asynchronously.

---

## Files

| File | Contents |
|------|----------|
| `kernel.h` | Public API, types, extern declarations |
| `kernel.c` | Scheduler, context switch ISRs, semaphore/queue/event engine, SVC handler, watchdog |
| `uart.h` | UART driver API |
| `uart.c` | TX circular buffer, `uart0TxIsr`, RX poll |
| `main.c` | Hardware init, task functions, shell, `main()` |

---

## Tasks

| Task | Priority | Role |
|------|----------|------|
| `idle` | 7 | Always runnable; WFI between yields; checks in watchdog |
| `lengthyFn` | 6 | Grabs `resource` → `resource2` (A→B lock order); long computation |
| `readKeys` | 6 | Reads push buttons, signals semaphores, sets `EV_KEY_DOWN` event bit |
| `debounce` | 6 | Hardware debounce for buttons |
| `producer` | 5 | Sends incrementing counter into `msgQ` every 500 ms |
| `consumer` | 5 | Blocks on `xQueueReceive`; accumulates running total |
| `uncooperative` | 5 | Demonstrates starvation in cooperative mode |
| `eventSetter` | 6 | Sets `EV_TIMER_TICK` every 1 s |
| `eventWaiter` | 6 | Wakes on `EV_KEY_DOWN` OR `EV_TIMER_TICK` (wait-any) |
| `flash4Hz` | 2 | Blinks green LED at 4 Hz via `vTaskDelayUntil` |
| `oneshot` | 2 | One-shot yellow LED flash on button press |
| `important` | 0 | High-priority; grabs `resource2` → `resource` (B→A lock order  -  deadlock-capable with `lengthyFn` under preemptive mode) |
| `shell` | 4 | UART command interface |

---

## IPC objects

| Object | Type | Purpose |
|--------|------|---------|
| `keyPressed` | semaphore (init 1) | Signals debounce that a key is down |
| `keyReleased` | semaphore (init 0) | Signals readKeys that key is released |
| `flashReq` | semaphore (init 5) | Triggers oneshot flash (buffered up to 5) |
| `resource` | semaphore (init 1) | Mutex between `lengthyFn` and `important` |
| `resource2` | semaphore (init 1) | Second mutex  -  enables circular-wait deadlock scenario |
| `msgQ` | queue (depth 4) | `producer` → `consumer` data transfer |
| `sysEvents` | event group | `EV_KEY_DOWN` (bit 0), `EV_TIMER_TICK` (bit 2) |

---

## Task states

| Value | State | Meaning |
|-------|-------|---------|
| 0 | INVALID | Slot empty |
| 1 | UNRUN | Registered, never scheduled |
| 2 | READY | Runnable, waiting its turn |
| 3 | BLOCKED | Waiting on semaphore, queue, or event |
| 4 | DELAYED | Sleeping via `vTaskDelay` |
| 5 | RUN | Currently executing |
| 6 | SUSPENDED | Paused via `vTaskSuspend` |

---

## Shell reference

Connect a serial terminal at 115,200 8N1.

| Command | Effect |
|---------|--------|
| `PS` | Tasks: PID, CPU% (EMA), state, priority, stack HWM |
| `KILL <pid\|name>` | Delete a task |
| `PIDOF <name>` | Print task PID |
| `SETPRI <name> <0-7>` | Change task priority at runtime |
| `SUSPEND <name>` | Suspend a task |
| `RESUME <name>` | Resume a suspended task |
| `SEM` | Show semaphores: count, waiters, head PID |
| `QUEUE` | Show message queues: capacity, used, send/recv waiters |
| `EVENTS` | Show event groups: current bits, waiter count |
| `DEADLOCK` | Scan for circular semaphore waits and print cycle |
| `WATCHDOG ON\|OFF` | Enable/disable per-task software watchdog (5 s timeout) |
| `UPTIME` | Time since boot as `Hh Mm Ss` |
| `PREEMPT ON\|OFF` | Enable/disable preemptive scheduling |
| `INHERIT ON\|OFF` | Enable/disable priority inheritance |
| `RESET` | System reset via NVIC APINT |
| `<name> &` | Restart a previously killed background task |
| `HELP` | Print command reference |

---

## Demonstrating key behaviours

**Priority inversion / inheritance**
Run `PS` while `lengthyFn` holds `resource`. `important` (pri 0) blocks behind it. With `INHERIT ON`, `lengthyFn`'s priority is raised to 0 while it holds the lock. With `INHERIT OFF`, `important` starves until `lengthyFn` releases.

**Deadlock**
`PREEMPT ON` → wait a few seconds → `DEADLOCK`. Under preemptive scheduling, `lengthyFn` (A→B) and `important` (B→A) may each hold one lock and block on the other. `DEADLOCK` prints the cycle: `LENGTHYFN -> IMPORTANT -> LENGTHYFN`.

**Watchdog**
`WATCHDOG ON`. Every task must call `vTaskCheckin()` within 5 s. Suspend a task with `SUSPEND <name>`  -  it stops checking in. After 5 s the shell prints `[FATAL] <name>` and resets.

**Event flags**
Press any button → `EV_KEY_DOWN` fires → `eventWaiter` wakes immediately (wait-any). Without a key press, `EV_TIMER_TICK` fires every 1 s and wakes it instead. Run `EVENTS` to observe the bits live.

**Message queue**
Run `QUEUE` repeatedly while the system runs. `count` cycles between 0 and 4 as `producer` fills and `consumer` drains. A blocked `consumer` (recvWait = 1) is visible when the queue is empty.

---

## Building

**Hardware:** EK-TM4C123GXL (Tiva C LaunchPad), ARM Cortex-M4F @ 40 MHz, UART0 at 115,200 8N1 on PA0/PA1. LEDs and buttons use bit-banding  -  pin assignments are at the top of `main.c`.

Import into Code Composer Studio (CCS) and target TM4C123GH6PM. The only external dependency is the TivaWare peripheral header `tm4c123gh6pm.h`. Wire `uart0TxIsr` to the UART0 Rx/Tx vector in `startup_ccs.c`.

Each task gets a 256-word (1 KB) stack. All sizing limits are in `kernel.h`:

| Constant | Default |
|----------|---------|
| `MAX_TASKS` | 14 |
| `MAX_SEMAPHORES` | 5 |
| `MAX_QUEUES` | 4 |
| `MAX_EVENTS` | 4 |
| `WDT_DEFAULT` | 5000 ms |

## License
