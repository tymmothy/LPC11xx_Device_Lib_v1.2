/******************************************************************************
 * @file:    ct16b.c
 * @purpose: Example for use of 16-bit Timer / Counters
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    3. Januart 2012
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
#include "LPC11xx_syscon.h"
#include "LPC11xx_ct16b.h"
#include "LPC11xx_gpio.h"
#include "LPC11xx_iocon.h"
#include "system_LPC11xx"


/* Functions ----------------------------------------------------------------*/


void CT16B0_IRQHandler()
{
   uint16_t it;


    it = CT16B_GetPendingIT(CT16B0);
    CT16B_ClearPendingIT(CT16B0, it);

    if (it & CT16B_IT_MR0) {
    }
}


void CT16B1_IRQHandler()
{
   uint16_t it;


    it = CT16B_GetPendingIT(CT16B1);
    CT16B_ClearPendingIT(CT16B1, it);

}


void init_ct16b0()
{

    /* Disable the timer to eliminate race conditions */
    CT16B_Disable(CT16B0);

    /* Reset count & prescaler count */
    CT16B_AssertReset(CT16B0);
    CT16B_DeassertReset(CT16B0);

    /* CT16B0 source = PCLK / 2 (prescaler = 1) */
    CT16B_SetPrescaler(CT16B0, 1);
    CT16B_SetMode(CT16B0, CT16B_Mode_Timer);

    CT16B_SetChannelMatchValue(CT16B0, 0, );
    CT16B_SetChannelMatchValue(CT16B0, 1, );

    CT16B_SetChannelMatchControl(CT16B0, 0, CT16B_MatchControl_Interrupt);
    CT16B_SetChannelMatchControl(CT16B0, 1, CT16B_MatchControl_Interrupt | CT16B_MatchControl_Reset);

    CT16B_Enable(CT16B0);

    NVIC_SetPriority(CT16B0_IRQn, 2);

    NVIC_EnableIRQ(CT16B0_IRQn);
}


void init_ct16b1()
{

    /* Disable the timer to eliminate race conditions */
    CT16B_Disable(CT16B1);

    /* Reset count & prescaler count */
    CT16B_AssertReset(CT16B1);
    CT16B_DeassertReset(CT16B1);

    /* CT16B1 source = PCLK / 2 (prescaler = 1) */
    CT16B_SetPrescaler(CT16B1, 1);
    CT16B_SetMode(CT16B1, CT16B_Mode_Counter);

    CT16B_SetChannelMatchValue(CT16B1, 0, );
    CT16B_SetChannelMatchValue(CT16B1, 1, );

    CT16B_SetChannelMatchControl(CT16B1, 0, CT16B_MatchControl_Interrupt);
    CT16B_SetChannelMatchControl(CT16B1, 1, CT16B_MatchControl_Interrupt | CT16B_MatchControl_Reset);

    CT16B_Enable(CT16B1);

    NVIC_SetPriority(CT16B1_IRQn, 2);
    NVIC_EnableIRQ(CT16B1_IRQn);
}


/** @brief  Main function for CT16B example / test program.
  *
  * @return None.
  */
int main(void)
{
    /* Enable system clock to GPIO, IOCON, CT16B blocks */
    SYSCON_EnableAHBClockLines(SYSCON_AHBClockLine_GPIO
                             | SYSCON_AHBClockLine_IOCON
                             | SYSCON_AHBClockLine_CT16B0
                             | SYSCON_AHBClockLine_CT16B1);

    NVIC_DisableIRQ(CT16B0_IRQn);

    /* Enable CT16B0 AHB Bus clocks */
    SYSCON_EnableAHBClockLines(SYSCON_AHBClockLine_CT16B0);

    init_ct16b0();
    init_ct16b1();

    while(1) {
        ssp0_write("Testing...", 10);
    }
}
