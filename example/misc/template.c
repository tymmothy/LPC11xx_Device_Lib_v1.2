/******************************************************************************
 * @file:    
 * @purpose: 
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
#include "LPC11xx_gpio.h"
#include "LPC11xx_iocon.h"
#include "system_LPC11xx"


/* File Local Variables -----------------------------------------------------*/

static uint32_t ms_since_boot;


/* Functions ----------------------------------------------------------------*/

/** @brief  SysTick IRQ Handler
  *
  * @return None.
  *
  * Increments ms_since_boot once every millisecond.
  */
void SysTick_Handler(void)
{
    ms_since_boot++;
}


/** @brief  Get the # of milliseconds since boot.
  *
  * @return Number of milliseconds since chip boot.
  */
uint32_t millis(void)
{
   return ms_since_boot;
}


/** @brief  Delay the given number of milliseconds
  * @param  [in]  ms    Number of milliseconds to delay
  *
  * @return None.
  *
  * Waits for given # of milliseconds to pass, then returns.  Due to 
  * timing inaccuracies, may return up to 1 millisecond early.
  */
void delay(uint32_t ms)
{
    uint32_t start_ms;
    
    
    start_ms = millis();
    while ((millis() - start_ms) < ms);
}

