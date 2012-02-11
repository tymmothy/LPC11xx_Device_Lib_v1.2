/******************************************************************************
 * @file:    swuart.c
 * @purpose: Example software UART for LPC11xx using GPIO, Timer
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    15. January 2012
 * @license: Simplified BSD License
 ******************************************************************************
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
 * THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * TIMOTHY TWILLMAN OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, ORCONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of Timothy Twillman.
 *****************************************************************************/

/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "LPC11xx.h"
#include "LPC11xx_iocon.h"
#include "LPC11xx_syscon.h"
#include "LPC11xx_gpio.h"
#include "LPC11xx_ct16b.h"
#include "system_LPC11xx.h"


/* Defines ------------------------------------------------------------------*/

/* With prescaler set to 4, this is the number of timer ticks per bit. */
#define BIT_TICKS  ((F_CPU / BAUD) / 4)


/* Number of bytes in transmit buffer -- must be power of 2, >= 2 */
#ifndef SWUART_TX_BUF_LEN
# define SWUART_TX_BUF_LEN 16
#endif

/* Number of bytes in receive buffer -- must be power of 2, >= 2 */
#ifndef SWUART_RX_BUF_LEN
# define SWUART_RX_BUF_LEN 16
#endif


/* Types --------------------------------------------------------------------*/


/* File Local Variables -----------------------------------------------------*/

/* Software UART state variables
 *
 * Tx / Rx buffers are set up as circular buffers.  As bytes are inserted,
 * they get put at the end index (which is then updated).  When they are
 * removed, they are taken from the start index, which is then updated.
 */

static uint8_t tx_buf[SWUART_TX_BUF_LEN]; /*!< Software UART Tx Buffer     */
static int tx_buf_start;                  /*!< Tx buffer start index       */
static int tx_buf_end;                    /*!< Tx buffer end index         */
static int tx_byte;                       /*!< Currently transmitting byte */

static uint16_t rx_buf[SWUART_RX_BUF_LEN]; /*!< Software UART Rx Buffer     */
static int rx_buf_start;                   /*!< Rx buffer start index       */
static int rx_buf_end;                     /*!< Tx buffer end index         */
static int rx_byte;                        /*!< Currently receiving byte    */


/* Functions ----------------------------------------------------------------*/

/** @brief RX start bit detection IRQ handler
  * @return  None.
  *
  * GPIO pin interrupt that sets up timers / state to receive a byte when
  * the start bit has been detected.
  */
void PIO3_IRQHandler()
{
    uint16_t start_time;


    /* Get the timer count ASAP to minimize time skew */
    start_time = CT16B_GetCount(CT16B0);

    /* Disable interrupt (will be re-enabled by timer IRQ handler
     * when done reading byte
     */
    GPIO3->IE = 0;

    /* Clear the pin interrupt */
    GPIO3->IC = 0xfff;

    /* Set timer for 1.5 bits to aim for middle of next bit */
    CT16B_SetChannelMatchValue(CT16B0, 0, starttime + ((BIT_TICKS * 3) / 2));

    /* Enable timer interrupt */
    CT16B_SetChannelMatchControl(CT16B0, 0, CT16B_MatchControl_Interrupt);

    /* Set the stop bit that gets rotated through to indicate when
     * reception is done.  For 2 stop bits, replace the 1 with 3 here.
     */
    swuart.rx_byte = (1 << (8 + 2));
}


/** @brief Stash a received byte into the RX buffer
  * @param  bval   The received byte
  * @return  None.
  *
  * If there is space, puts into the buffer and updates the buffer end index.
  * If no space, the byte is dropped.
  */
static inline void swuart_rx_save(uint16_t bval)
{
    int new_buf_end;


    new_buf_end = rx_buf_end + 1;
    new_buf_end &= (sizeof(rx_buf) - 1);

    if (new_buf_end != rx_buf_start) {
        rx_buf[new_buf_end] = bval;
        rx_buf_end = new_buf_end;
    }
}


/** @brief Load the next byte from the TX buffer for transmission
  * @return  None.
  *
  * Tries to get the next byte from the transmit buffer; if it's empty
  * then this will disable the timer match interrupts for software UART
  * transmit.
  */
static inline void swuart_tx_load(void)
{
    int new_buf_start;


    if (tx_buf_start != tx_buf_end) {
        new_buf_start = tx_buf_start + 1;
        new_buf_start &= sizeof(tx_buf) - 1;

        tx_byte = tx_buf[new_buf_start];

        /* Set stop bit; shift over one to add the start bit @ bit0
         * Note: Change 0x100 to 0x300 for 2 stop bits
         */
        tx_byte = (tx_byte | 0x100) << 1;
        tx_buf_start = new_buf_start;
    } else {
        /* No more bytes in TX buffer; turn off TX match interrupts */
        CT16B_SetChannelMatchControl(CT16B0, 1, CT16B_MatchControl_None);
    }
}


/** @brief Timer IRQ handler for software UART
  * @return  None.
  *
  * Tries to get the next byte from the transmit buffer; if it's empty
  * then this will disable the timer match interrupts for software uart
  * transmit.
  */
void CT16B0_IRQHandler(void)
{
    uint16_t it;


    it = CT16B_GetPendingIT(CT16B1);

    CT16B_ClearPendingIT(CT16B1, it);

    /* Match Register 0 is for the RX bit timer */
    if (it & CT16B_IT_MR0) {
        swuart.rx_byte >>= 1;
        if (GPIO_ReadPins(SWUART_PORT, SWUART_RX_PIN)) {
            swuart.rx_byte |= 0x200;
        }

        /* Set counter to fire @ center of next bit */
        CT16B_SetChannelMatchValue(CT16B0, 0,
                                   CT16B_GetChannelMatchValue(CT16B1, 0)
                                   + BIT_TICKS);

        /* Check to see if the "byte done" bit has hit bit 0... if so,
         * reception is done.
         */
        if (swuart.rx_byte & 0x01) {
            /* Disable timer interrupt */
            CT16B_SetChannelMatchControl(CT16B0, 0, CT16B_MatchControl_None);

            SWUART_PORT->IC = 0xfff;
            GPIO_EnableIRQForPins(SWUART_PORT, SWUART_RX_PIN);

            swuart_rx_save(swuart.rx_byte >> 1);
        }
    }

    /* Match Register 1 is for the TX bit timer */
    if (it & CT16B_IT_MR1) {
        if (swuart.tx_byte == 0) {
            /* If byte is 0, means done sending (including the stop bit) */
            swuart_tx_load();
        } else {
            /* Write next bit and shift the remaining TX bytes over */
            GPIO_WritePins(SWUART_PORT, SWUART_TX_PIN, (swuart.tx_byte & 0x01) ? 0:SWUART_TX_PIN)
            swuart.tx_byte >>= 1;
        }
    }
}


int swuart_send_break()
{
    /* Don't send if there are currently bytes being transmitted */
    if (tx_buf_end != tx_buf_start) {
        return 0;
    }

    /* The 1 bit here will clear the BREAK condition when the timer
     * has finished.
     */
    tx_byte = 0x01;

    /* Set the BREAK (0 value) on the wire */
    GPIO_WritePins(SWUART_PORT, SWUART_TX_PIN, 0xfff);

    /* ... for at least 15 bit periods (bump to 16 just in case the
     * timer is partially through the current tick...)
     */
    CT16B_SetChannelMatchValue(CT16B0, 1,
                               CT16B_GetCount(CT16B0) + (BIT_TICKS * 16));

    /* Normal processing will continue when the interrupt handler picks up the
     * timer interrupt.
     */

    return 1;
}


int swuart_send(uint8_t byte)
{
    int new_buf_end;
    uint32_t primask;


    new_buf_end = tx_buf_end + 1;
    new_buf_end &= (sizeof(tx_buf) - 1);
    if (new_buf_end != tx_buf_start) {
        tx_buf[new_buf_end] = bval;
        tx_buf_end = new_buf_end;

        __asm__ __volatile__ ("    mrs    %0, primask":"=r"(primask));

        __disable_irq();
        if (CT16B_GetChannelMatchControl(CT16B0, 1) == CT16B_MatchControl_None) {
            CT16B_SetChannelMatchControl(CT16B0, 1, CT16B_MatchControl_Interrupt);
            /* Trigger match ASAP */
            CT16B_SetChannelMatchValue(CT16B0, 1, CT16B_GetCount(CT16B0) + 1);
        }
        __asm__ __volatile__ ("    msr    primask, %0"::"r"(primask));

        return 1;
    }

    return 0;
}


int swuart_recv()
{
    b = swuart.rxbuf[swuart.rxbuf_start];

    /* If b == 0 it's a BREAK
     * If bit 9 of b is set (stop bit received), it's a valid character.
     * Otherwise it's a line error.
     */
    if ((b & 0x100) == 0) {
        /* BREAK or line error */
        if (b == 0x00) {
            return -255;
        } else {
            return -1;
        }
    } else {
        return b;
    }
}


/** @brief  Main function for Software UART example
  *
  * @return None.
  */
int main(void)
{
    /* Enable system clock to the GPIO block */
    SYSCON_EnableAHBClockLines(SYSCON_AHBClockLine_IOCON
                             | SYSCON_AHBClockLine_GPIO);

    /* Set RX pin as GPIO w/ no pullup/pulldown, and input */
    IOCON_SetPinConfig(SWUART_RX_IOCONFIG, IOCON_Mode_Normal);
    GPIO_SetPinDirections(SWUART_PORT, SWUART_RX_PIN, GPIO_Direction_In);

    /* Set TX pin as GPIO w/ no pullup/pulldown, and output */
    IOCON_SetPinConfig(SWUART_RX_IOCONFIG, IOCON_Mode_Normal);
    GPIO_SetPinDirections(SWUART_PORT, SWUART_TX_PIN, GPIO_Direction_Out);

    /* Set TX pin to HIGH */
    GPIO_WritePins(SWUART_TX_PORT, SWUART_TX_PIN, SWUART_TX_PIN);

    /* Look for falling edge (start bit) */
    GPIO_EnableIRQForPins(SWUART_TX_PORT, SWUART_TX_PIN, GPIO_Sense_FallingEdge);

    while(1) {

    }
}
