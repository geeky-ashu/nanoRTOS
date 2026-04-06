/*--------------------------------------------------------------------------------
 * UART0 driver  -  interrupt-driven TX circular buffer, blocking RX poll.
 * This is a hardware driver layer, separate from the RTOS scheduler.
 *--------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/

#ifndef UART_H
#define UART_H

#include <stdint.h>

/*------------------------------------------------------------------------------*/

// Transmit a single character via the circular TX buffer.
// Arms the UART0 TX interrupt to drain the buffer asynchronously.
// Must NOT be called from ISR context (except through uart0TxFlush).
void putcUart0(char value);

// Transmit a null-terminated string via the circular TX buffer.
void putsUart0(char *string);

// Receive one character, yielding the CPU while the RX FIFO is empty.
// Calls vTaskCheckin() - so the shell watchdog counter stays reset during input.
char getcUart0(void);

// Blocking drain of the TX circular buffer  -  bypasses the interrupt and
// writes directly to the FIFO.  Use only when interrupts are disabled or
// cannot fire (e.g., inside HardFault handler before system reset).
void uart0TxFlush(void);

// UART0 TX ISR  -  must be wired to the UART0 Rx/Tx vector in startup_ccs.c.
// Drains the circular buffer into the TX FIFO; disables TXIM when empty.
void uart0TxIsr(void);

/*------------------------------------------------------------------------------*/

#endif /* UART_H */
