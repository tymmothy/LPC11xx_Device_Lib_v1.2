/******************************************************************************
 * @file:    uart_echo_irq.c
 * @purpose: Example / test program for LPC11xx UART interface
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    1. January 2012
 * @license: Simplified BSD License
 *
 * Echoes characters received on UART, using the UART's Tx/Rx IRQs to drive
 * buffers.
 *
 ******************************************************************************
 * @section License
 * Copyright (c) 2012, Timothy Twillman
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright notice,
 *        this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY TIMOTHY TWILLMAN ''AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL <COPYRIGHT HOLDER> ORCONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, ORCONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of Timothy Twilllman.
 *****************************************************************************/

/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "lpc11xx.h"
#include "lpc11xx/syscon.h"
#include "lpc11xx/gpio.h"
#include "lpc11xx/iocon.h"
#include "lpc11xx/uart.h"
#include "system_lpc11xx.h"


/* Defines ------------------------------------------------------------------*/

/* Default to 38400 baud */
#ifndef BAUD
# define BAUD 38400
#endif

/* # of bytes in UART FIFO */
#define TX_FIFO_SIZE (16)


/* File Local Variables -----------------------------------------------------*/

volatile uint8_t tx_buffer[16];
volatile uint8_t tx_buffer_start;
volatile uint8_t tx_buffer_end;

volatile uint8_t rx_buffer[16];
volatile uint8_t rx_buffer_start;
volatile uint8_t rx_buffer_end;


/* Functions ----------------------------------------------------------------*/

/** @brief  UART IRQ Handler; deals with UART buffering, etc.
  *
  * @return None.
  *
  * Moves characters into Rx buffer from UART & out of Tx buffer into UART.
  */
void UART0_IRQHandler(void)
{
    uint16_t tmp;
    int bindex;
    int tx_fifo_free;
    UART_Type *uart = UART0;


    /* Get the next pending UART interrupt ID and act on it */
    while ((tmp = UART_GetPendingInterruptID(uart)) != UART_InterruptID_None) {
        if (tmp == UART_InterruptID_RxDataAvailable) {
            /* UART has character(s) waiting... Read one. */
            tmp = UART_Recv(uart);

            /* Try to figure out whether it can be shoved in the rx buffer,
             *  and if so where
             */
            bindex = rx_buffer_start + 1;
            bindex &= (sizeof(rx_buffer) - 1);

            /* If there's room, put the character in the buffer and
             *  update the buffer's start index
             */
            if (bindex != rx_buffer_end) {
                tmp = rx_buffer[rx_buffer_start] = tmp;
                rx_buffer_start = bindex;
            }
        } else if (tmp == UART_InterruptID_TxEmpty) {
            tx_fifo_free = TX_FIFO_SIZE;

            while (tx_buffer_start != tx_buffer_end) {
                UART_Send(uart, tx_buffer[tx_buffer_end]);
                tx_buffer_end++;
                tx_buffer_end &= (sizeof(tx_buffer) - 1);
                tx_fifo_free--;
            }
            /* UART Tx has space available... see if there are any characters
             * waiting in the Tx buffer
             */
            if (tx_buffer_start != tx_buffer_end) {
                /* If so, send the next one & update the Tx buffer end index */
                UART_Send(uart, tx_buffer[tx_buffer_end]);
                tx_buffer_end++;
                tx_buffer_end &= (sizeof(tx_buffer) - 1);
            } else {
                /* Otherwise disable Tx interrupts until more characters are
                 *  put in the buffer.
                 */
                UART_DisableInterrupts(uart, UART_Interrupt_TxData);
            }
        }
    }
}


/** @brief  Determine whether there are any characters in the Rx buffer
  *
  * @return 1 if there are character(s) available, 0 otherwise.
  */
int buffered_uart_available(void)
{
    return (rx_buffer_start != rx_buffer_end);
}


/** @brief  Send a character via buffered UART
  *
  * @param  [in]  c        The character to send
  *
  * @return None.
  *
  * Blocks until the buffer is ready to accept the new character, then
  *  writes it.  Enables the UART Tx interrupt to make sure the
  *  character gets picked up.
  */
void buffered_uart_putchar(uint8_t c)
{
    int bindex;


    /* Wait until there is tx buffer space available */
    do {
        bindex = tx_buffer_start + 1;
        bindex &= (sizeof(tx_buffer) - 1);
    } while (bindex == tx_buffer_end);

    /* Put the new character in the buffer & update buffer start index */
    tx_buffer[tx_buffer_start] = c;
    tx_buffer_start = bindex;

    /* Make sure the UART's Tx IRQ is enabled */
    UART_EnableInterrupts(UART0, UART_Interrupt_TxData);
}


/** @brief  Send a string via buffered UART
  *
  * @param  [in]  string   The string to send
  *
  * @return None.
  *
  * Uses uart_putchar, so this is will block until the entire string
  *  has been buffered.
  */
void buffered_uart_putstr(const char *string)
{
    while (*string != '\0') {
        buffered_uart_putchar((uint8_t)(*string++));
    }
}


/** @brief  Receive a character via (buffered) UART
  *
  * @return The received character
  *
  * Blocks until the UART's Rx buffer has a character available to read.
  */
uint8_t buffered_uart_getchar(void)
{
    int bindex;
    uint8_t c;


    /* Wait until there's a character in the buffer */
    while (!buffered_uart_available());

    /* Get the character */
    c = rx_buffer[rx_buffer_end];

    /* Compute new buffer end index and set the position */
    bindex = rx_buffer_end + 1;
    bindex &= (sizeof(rx_buffer) - 1);
    rx_buffer_end = bindex;

    return c;
}


/** @brief  Initialize a UART for communication.
  *
  * @param  [in]  uart     The UART to initialize
  * @param  [in]  baud     Baud rate to configure on the UART
  *
  * @return None.
  */
void init_uart(UART_Type *uart, uint32_t baud)
{
    uint32_t uart_divisor;


    /* Determine the UART divisor for the given baud rate. */
    uart_divisor = ((SystemAHBClock / 16) + (baud / 2)) / baud;

    /* This should always be true -- currently only support a single UART...
     */
    if (uart == UART0) {
        /* Disable the UART IRQ while configuring it */
        NVIC_DisableIRQ(UART0_IRQn);

        /* Configure GPIO1.7 as TXD, GPIO1.6 as RXD */
        IOCON_SetPinConfig(IOCON_PinConfig_1_7_TXD0, IOCON_Mode_Normal);
        IOCON_SetPinConfig(IOCON_PinConfig_1_6_RXD0, IOCON_Mode_Normal);

        /* Enable the clock line to the UART */
        SYSCON_EnableAHBClockLines(SYSCON_AHBClockLine_UART0);

        /* Set the UART input clock to run at AHB bus speed */
        SYSCON_SetUART0ClockDivider(1);
    }

    /* Set the UART baud rate generator divisor to the value calculated */
    UART_SetDivisor(uart, uart_divisor);

    /* Set other standard UART settings */
    UART_SetWordLength(uart, UART_WordLength_8b);
    UART_SetStopBits(uart, UART_StopBits_1);
    UART_SetParity(uart, UART_Parity_None);

    /* Enable FIFOs on the UART (required for proper function) and flush them
     */
    UART_EnableFifos(uart);
    UART_FlushTxFifo(uart);
    UART_FlushRxFifo(uart);

    /* UART should interrupt on every incoming byte */
    UART_SetRxFifoTrigger(uart, UART_RxFifoTrigger_1);

    /* Enable transmitting on the UART */
    UART_EnableTx(uart);

    /* This should always be true -- currently only support a single UART...
     */
    if (uart == UART0) {
        /* Set UART IRQ priorty to lowest level */
        NVIC_SetPriority(UART0_IRQn, 2);

        /* Connect UART IRQ line */
        NVIC_EnableIRQ(UART0_IRQn);
    }

    /* UART needs a read to start the interrupt juices flowing... */
    UART_GetPendingInterruptID(uart);
}


/** @brief  Main function for UART example / test program.
  *
  * @return None (never returns).
  *
  * Sits in a loop waiting for characters on the given UART, then sends
  *  a string telling what it got.
  */
int main(void)
{
    uint8_t c;
    UART_Type *uart = UART0;


    /* Enable system clock to the GPIO block */
    SYSCON_EnableAHBClockLines(SYSCON_AHBClockLine_GPIO);

    /* Set pin as GPIO w/ no pullup/pulldown, and output */
    IOCON_SetPinConfig(IOCON_PinConfig_3_5_PIO, IOCON_Mode_Normal);
    GPIO_SetPinDirections(GPIO3, GPIO_Pin_5, GPIO_Direction_Out);

    /* Set pin to LOW */
    GPIO_WritePins(GPIO3, GPIO_Pin_5, 0);

    /* Set up UART... */
    init_uart(uart, BAUD);

    /* Set pin to HIGH */
    GPIO_WritePins(GPIO3, GPIO_Pin_5, GPIO_Pin_5);

    /* Enable UART Rx, Tx IRQ's */
    UART_EnableInterrupts(uart, UART_Interrupt_RxData);

    /* Set pin to LOW */
    GPIO_WritePins(GPIO3, GPIO_Pin_5, 0);

    UART_Send(uart, '!');

    buffered_uart_putstr("Now echoing characters: ");

    /* Set pin to HIGH */
    GPIO_WritePins(GPIO3, GPIO_Pin_5, GPIO_Pin_5);

    /* Loop forever, reporting what is received */
    while(1) {

        if (buffered_uart_available()) {
            c = buffered_uart_getchar();
            buffered_uart_putchar(c);
        }
    }
}

