/*--------------------------------------------------------------------------------
 * UART0 driver  -  interrupt-driven TX circular buffer, blocking RX poll.
 *--------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/

#include <string.h>
#include "tm4c123gh6pm.h"
#include "kernel.h"
#include "uart.h"

/*------------------------------------------------------------------------------*/

// Circular TX buffer.  Power-of-two size lets (idx & mask) replace modulo.
#define UART_TX_BUF_MASK  127u
static volatile char    uartTxBuf[UART_TX_BUF_MASK + 1];
static volatile uint8_t uartTxHead = 0;   // producer index (putcUart0)
static volatile uint8_t uartTxTail = 0;   // consumer index (uart0TxIsr)

/*------------------------------------------------------------------------------*/

// Enqueue one byte and arm the TX interrupt to drain the buffer.
// Spins only if the 128-byte buffer is completely full (rare under normal use).
void putcUart0(char value)
{
    uint8_t next = (uartTxHead + 1) & UART_TX_BUF_MASK;
    while (next == uartTxTail) {}          // spin if buffer full
    uartTxBuf[uartTxHead] = value;
    uartTxHead = next;
    UART0_IM_R |= UART_IM_TXIM;           // arm TX interrupt to drain buffer
}

void putsUart0(char *string)
{
    uint8_t len = strlen(string);
    uint8_t i;
    for (i = 0; i < len; i++)
        putcUart0(string[i]);
}

// Blocking drain  -  used by hardFaultHandler before reset, where the UART0
// TX interrupt cannot preempt the fault handler (lower exception priority).
void uart0TxFlush(void)
{
    while (uartTxTail != uartTxHead)
    {
        while (UART0_FR_R & UART_FR_TXFF) {}
        UART0_DR_R = uartTxBuf[uartTxTail];
        uartTxTail = (uartTxTail + 1) & UART_TX_BUF_MASK;
    }
}

// UART0 TX ISR  -  drains the circular buffer into the TX FIFO one byte at a
// time.  Disables TXIM when the buffer empties so it does not retrigger.
// Wire to the UART0 Rx/Tx vector slot in startup_ccs.c.
void uart0TxIsr(void)
{
    UART0_ICR_R = UART_ICR_TXIC;          // clear TX interrupt flag
    while (uartTxTail != uartTxHead && !(UART0_FR_R & UART_FR_TXFF))
    {
        UART0_DR_R  = uartTxBuf[uartTxTail];
        uartTxTail  = (uartTxTail + 1) & UART_TX_BUF_MASK;
    }
    if (uartTxTail == uartTxHead)
        UART0_IM_R &= ~UART_IM_TXIM;      // buffer empty  -  stop triggering
}

// Poll RX FIFO, yielding CPU while empty.  Calls vTaskCheckin() so the shell
// task resets its watchdog counter during idle keyboard wait.
char getcUart0(void)
{
    while (UART0_FR_R & UART_FR_RXFE)
    {
        vTaskCheckin();   // shell is alive  -  reset watchdog while waiting for input
        vTaskYield();
    }
    return UART0_DR_R & 0xFF;
}

/*------------------------------------------------------------------------------*/
