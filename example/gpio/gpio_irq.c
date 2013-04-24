/******************************************************************************
 * @file:    gpio_irq.c
 * @purpose: Example / test program for LPC11xx GPIO w/interrupts
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    15. January 2012
 * @license: Simplified BSD License
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
#include "system_LPC11xx.h"


/* Defines ------------------------------------------------------------------*/

/* Functions ----------------------------------------------------------------*/

void PIO0_IRQHandler()
{
}

void PIO1_IRQHandler()
{
}

void PIO2_IRQHandler()
{
}

void PIO3_IRQHandler()
{
    uint16_t it;

    it = GPIO_GetPendingIT(GPIO3);
    GPIO_ClearPendingITStatusForPins(GPIO3, it);
}

/** @brief  Main function for GPIO w/IRQ example
  *
  * @return None.
  */
int main(void)
{
    /* Enable system clock to the GPIO block */
    SYSCON_EnableAHBClockLines(SYSCON_AHBClockLine_IOCON
                             | SYSCON_AHBClockLine_GPIO);

    /* Set pin as GPIO w/ no pullup/pulldown, and output */
    IOCON_SetPinConfig(GPIO_IOCONFIG, IOCON_Mode_Normal);
    GPIO_SetPinDirections(GPIO_PORT, GPIO_PIN, GPIO_Direction_Out);

    IOCON_SetPinConfig(GPIO_PIN_IN, IOCON_Mode_Normal);
    IOCON_SetPinDirection(GPIO_PORT, GPIO_PIN_IN, GPIO_Direction_In);

    /* Set pin to LOW */
    SetPinLow(GPIO_PORT, GPIO_PIN);

    GPIO_EnableIRQForPins(GPIO3,

    while(1);
}
