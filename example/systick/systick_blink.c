/******************************************************************************
 * @file:    systick_blink.c
 * @purpose: Example / test program for LPC11xx SysTick timer
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    3. Januart 2012
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
#include "LPC11xx_syscon.h"
#include "LPC11xx_gpio.h"
#include "LPC11xx_iocon.h"
#include "system_LPC11xx.h"


/* Defines ------------------------------------------------------------------*/

/* Default to using port GPIO3.5 */
#ifndef GPIO_PORT
# define GPIO_PORT GPIO3
#endif

#ifndef GPIO_PIN
# define GPIO_PIN GPIO_Pin_5
#endif

/* IO Configuration for the configured pin */
#ifndef GPIO_IOCONFIG
# define GPIO_IOCONFIG IOCON_PinConfig_3_5_PIO
#endif


/* Functions ----------------------------------------------------------------*/

/** @brief  Convenience function for setting a single GPIO pin high
  *
  * @return None.
  *
  * Sets the given pin of the given GPIO port high.
  */
void SetPinHigh(GPIO_Type *GPIO, uint16_t pin)
{
    GPIO_WritePins(GPIO, pin, pin);
}


/** @brief  Convenience function for setting a single GPIO pin low
  *
  * @return None.
  *
  * Sets the given pin of the given GPIO port low.
  */
void SetPinLow(GPIO_Type *GPIO, uint16_t pin)
{
    GPIO_WritePins(GPIO, pin, 0);
}


/** @brief  SysTick IRQ Handler
  *
  * @return None.
  *
  * Flashes an LED connected to pin 0.1 at 1/2 configured system tick rate
  */
void SysTick_Handler(void)
{
    static int pinVal = 0;
    
    
    if (pinVal == 0) {
        SetPinLow(GPIO_PORT, GPIO_PIN);
    } else {    
        SetPinHigh(GPIO_PORT, GPIO_PIN);
    }
    
    /* Invert the pin value */
    pinVal = !pinVal;
}


/** @brief  Main function for SysTick example / test program.
  *
  * @return None.
  *
  * Sits in a loop while SysTick interrupts drive blinking of
  *  LED on pin 0.1
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

    /* Set pin as GPIO w/ no pullup/pulldown, and output */
    IOCON_SetPinConfig(GPIO_IOCONFIG, IOCON_Mode_Normal);
    GPIO_SetPinDirections(GPIO_PORT, GPIO_PIN, GPIO_Direction_Out);
    
    /* Set pin to LOW */
    SetPinLow(GPIO_PORT, GPIO_PIN);
    
    /* Infinite loop (blinking happens in Systick handler) */
    while(1);
}
