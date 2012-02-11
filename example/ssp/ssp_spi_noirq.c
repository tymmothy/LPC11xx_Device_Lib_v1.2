/******************************************************************************
 * @file:    ssp_spi_noirq.c
 * @purpose: Example / test program for LPC11xx SSP interface
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    15. Januart 2012
 * @license: Simplified BSD License
 *
 * Blinks an LED connected to pin 3.5 at 0.5Hz.
 *
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

#include "LPC11xx.h"
#include "LPC11xx_iocon.h"
#include "LPC11xx_ssp.h"
#include "LPC11xx_syscon.h"
#include "LPC11xx_gpio.h"
#include "system_LPC11xx.h"


/* Defines ------------------------------------------------------------------*/

#define CS_PORT GPIO0
#define CS_PIN  GPIO_Pin_7

/* Functions ----------------------------------------------------------------*/

/** @brief Set up SSP0 for 8-bit SPI mode 0,0 operation with CS pin configured
  *
  * @return None.
  */
void ssp0_init()
{
    /*
     * Configure SPI interface pins
     */
    IOCON_SetPinConfig(IOCON_PinConfig_0_9_MOSI0, IOCON_Mode_Normal);
    IOCON_SetPinConfig(IOCON_PinConfig_0_8_MISO0, IOCON_Mode_Normal);
    IOCON_SetPinConfig(IOCON_PinConfig_0_6_SCK0, IOCON_Mode_Normal);

    /* Set SCK0 to be on GPIO0.6 */
    IOCON_SetSCK0Location(IOCON_SCK0Location_0_6);

    /* Configure CS pin */
    IOCON_SetPinConfig(IOCON_PinConfig_0_7_PIO0, IOCON_Mode_Normal);
    GPIO_SetPinDirections(CS_PORT, CS_PIN, GPIO_Direction_Out);

    /* Initially Chip Select High (not selected) */
    GPIO_WritePins(CS_PORT, CS_PIN, CS_PIN);

    /* Enable clock routing to SPI */
    SYSCON_EnableAHBClockLines(SYSCON_AHBClockLine_SSP0);

    /* Set SPI clock divider to 1 (divider == 1 -> pass through) */
    SYSCON_SetSSP0ClockDivider(1);

    /* Reset the SSP peripheral */
    SYSCON_AssertPeripheralResets(SYSCON_PeripheralReset_SSP0);
    SYSCON_DeassertPeripheralResets(SYSCON_PeripheralReset_SSP0);

    /* Set the word size to transfer to / from SPI Flash to 8 */
    SSP_SetWordSize(SSP0, 8);

    /* Set the frame format to SPI */
    SSP_SetFrameFormat(SSP0, SSP_FrameFormat_SPI);

    /* Standard SPI polarity / phase */
    SSP_SetClockPolarity(SSP0, SSP_ClockPolarity_Low);
    SSP_SetClockPhase(SSP0, SSP_ClockPhase_A);

    /* Set SPI Peripheral Clock Prescaler
     * (2 == divide PCLK by 2, which is minimum)
     */
    SSP_SetClockPrescaler(SSP0, 2); /* 2 -> 2 PCLK counts / prescaler count */
    SSP_SetClockRate(SSP0, 0);      /* 0 -> 1 prescaler count / bit         */

    /* Set Master Mode */
    SSP_SetMode(SSP0, SSP_Mode_Master);

    /* Enable the SSP */
    SSP_Enable(SSP0);
}


/** @brief  Write a buffer of data to ssp0
  * @param  buf    Pointer to buffered data
  * @param  len    length of data to send
  * @return None.
  */
void ssp0_write(const uint8_t *buf, uint16_t len)
{
    while (len--) {
        SSP_Xfer(SSP0, *buf++);
    }
}


/** @brief  Main function for SSP SPI example / test program.
  *
  * @return None.
  *
  * Sits in a loop sending data over SSP
  */
int main(void)
{
    /* Set the system tick timer to trigger 4 times every second.
     *  Divide SystemCoreClock here by 10 for 10Hz, etc.
     *
     *  NOTE: SysTick uses a 24-bit register for the divide
     *  value.  This means at maximum you can pass
     *  16 million-ish before overflowing the register; e.g.
     *  at 50MHz cpu operation you will need to divide
     *  SystemCoreClock by at least 3)
     */
    SysTick_Config(SystemCoreClock/4);

    /* Enable system clock to the GPIO block */
    SYSCON_EnableAHBClockLines(SYSCON_AHBClockLine_GPIO);

    /* Enable system clock to the IOCON block */
    SYSCON_EnableAHBClockLines(SYSCON_AHBClockLine_IOCON);

    /* Set up the SSP */
    ssp0_init();

    while(1) {
        ssp0_write("Testing...", 10);
    }
}
