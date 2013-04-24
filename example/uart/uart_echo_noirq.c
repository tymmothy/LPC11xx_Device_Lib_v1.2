/******************************************************************************
 * @file:    uart_echo_noirq.c
 * @purpose: Example / test program for LPC11xx UART interface
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    1. January 2012
 * @license: Simplified BSD License
 *
 * Echoes characters received on UART.
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


/* Functions ----------------------------------------------------------------*/

/** @brief  Determine whether the given UART has a character in the rcv buffer
  *
  * @param  [in]  uart     The UART to check
  *
  * @return 1 if the UART has at least 1 character available, 0 otherwise.
  */
int uart_available(UART_Type *uart)
{
    return (UART_GetLineStatus(uart) & UART_LineStatus_RxData) ? 1:0;
}


/** @brief  Send a character via UART
  *
  * @param  [in]  uart     The UART on which to send the character
  * @param  [in]  c        The character to send
  *
  * @return None.
  *
  * Blocks until the UART is ready to accept the new character, then
  *  writes it.
  */
void uart_putchar(UART_Type *uart, uint8_t c)
{
    /* Wait for tx holding register to empty */
    while ((UART_GetLineStatus(uart) & UART_LineStatus_TxEmpty) == 0);

    /* Send character */
    UART_Send(uart, c);
}


/** @brief  Send a string via UART
  *
  * @param  [in]  uart     The UART on which to send the string
  * @param  [in]  string   The string to send
  *
  * @return None.
  *
  * Uses uart_putchar, so this is will block until the entire string
  *  has been sent.
  */
void uart_putstr(UART_Type *uart, const char *string)
{
    while (*string != '\0') {
        uart_putchar(uart, (uint8_t)(*string++));
    }
}


/** @brief  Receive a character via UART
  *
  * @param  [in]  uart     The UART on which to receive the character
  *
  * @return The received character
  *
  * Blocks until the UART has a character available to read.
  */
uint8_t uart_getchar(UART_Type *uart)
{
    uint8_t c;


    /* Wait for a character to be ready */
    while ((UART_GetLineStatus(uart) & UART_LineStatus_RxData) == 0);

    /* Receive it */
    c = UART_Recv(uart);

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

    /* Enable transmitting on the UART */
    UART_EnableTx(uart);

    /* Clear the receive buffer register */
    UART_Recv(uart);
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
    /* Choose the UART to use for test */
    UART_Type *uart = UART0;
    uint8_t c;


    /* Set up UART ... */
    init_uart(uart, BAUD);

    uart_putstr(uart, "Now echoing characters: ");

    /* Loop forever, reporting what is received */
    while(1) {
        if (uart_available(uart)) {
            c = uart_getchar(uart);
            uart_putchar(uart, c);
        }
    }
}
