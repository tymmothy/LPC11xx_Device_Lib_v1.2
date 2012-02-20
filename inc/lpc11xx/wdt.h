/**************************************************************************//**
 * @file     wdt.h
 * @brief    Watchdog Timer Interface Header for NXP LPC Microcontrollers.
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. January 2012
 ******************************************************************************
 * @section Overview
 * This file gives a basic interface to NXP LPC microcontroller
 * WDT's.  It abstracts such things as setting the
 *
 * @note
 * This file does not handle the following necessary steps for WDT use:
 * - If the WDT oscillator is used as the input clock, its speed must be
 *   set and power enabled, and its clock input divider configured.
 * - The WDT's input clock must be set and its divider configured.
 * - The WDT's (AHB/APB/VPB) bus clock must be enabled.
 ******************************************************************************
 * @section License License
 * Licensed under a Simplified BSD License:
 *
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
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

#ifndef LPC_WDT_H_
#define LPC_WDT_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/

#include <stdint.h>
#include "lpc11xx.h"


/**
  * @defgroup WDT_AbstractionLayer WDT (Watchdog Timer) Abstraction Layer
  * @ingroup  LPC_Peripheral_AbstractionLayer
  * @{
  */

/* Types & Type-Related Definitions -----------------------------------------*/

/** @defgroup WDT_Types WDT Interface Types and Type-Related Definitions
  * @{
  */

/** @defgroup WDT_Feed_Values Watchdog Timer Feed Register Values
  * @{
  */

#define WDT_Feed_A                     (0xAA)              /*!< First "feed" value to update WDT */
#define WDT_Feed_B                     (0x55)              /*!< 2nd   "feed" value to update WDT */

/** @} */

/**
  * @}
  */

/* Inline Functions ---------------------------------------------------------*/

/** @defgroup WDT_InlineFunctions WDT Interface Inline Functions
  * @{
  */

/** @brief  Enable the Watchdog Timer (cannot be cleared w/o reset)
  * @return None.
  */
__INLINE static void WDT_Enable(void)
{
    WDT->MOD |= WDT_WDEN;
}

/** @brief  Determine Whether the Watchdog Timer is Enabled
  * @return 1 if the WDT is enabled, 0 otherwise
  */
__INLINE static uint32_t WDT_IsEnabled(void)
{
    return (WDT->MOD & WDT_WDEN) ? 1:0;
}

/** @brief  Enable WDT Resetting the Chip (cannot be cleared w/o reset)
  * @return None.
  */
__INLINE static void WDT_EnableChipReset(void)
{
   WDT->MOD |= WDT_WDRESET;
}

/** @brief  Determine Whether the Watchdog Chip Reset is Enabled
  * @return 1 if the WDT Chip Reset is enabled, 0 otherwise
  */
__INLINE static uint32_t WDT_ChipResetIsEnabled(void)
{
    return (WDT->MOD & WDT_WDRESET) ? 1:0;
}

/** @brief Check Whether the Watchdog Timer has Timed Out
  * @return 1 if the timer has timed out, 0 otherwise.
  */
__INLINE static uint8_t WDT_TimedOut(void)
{
    return (WDT->MOD & WDT_WDTOF) ? 1:0;
}

/** @brief Check Whether the Watchdog Timer Interrupt is Pending
  * @return 1 if the timer interrupt is pending, 0 otherwise
  */
__INLINE static uint8_t WDT_ITIsPending(void)
{
    return (WDT->MOD & WDT_WDINT) ? 1:0;
}

/** @brief Get the Current Value of the Watchdog Timer
  * @return Current value of the watchdog timer
  */
__INLINE static uint32_t WDT_GetCurrentValue(void)
{
    return WDT->TV;
}

/** @brief Feed the Watchdog Timer (load the timer constant)
  * @return None.
  *
  * This is necessary to load the current value in the Timer Constant
  *  register as the watchdog's Timer Reload value.  This is also necessary
  *  to start the watchdog, after it's been enabled, and enable changes to
  *  WDT settings.
  */
__INLINE static void WDT_Feed(void)
{
    uint32_t primask;

    __asm__ __volatile__ ("    mrs    %0, primask":"=r"(primask));

    __disable_irq();
    WDT->FEED = WDT_Feed_A;
    WDT->FEED = WDT_Feed_B;
    __asm__ __volatile__ ("    msr    primask, %0"::"r"(primask));
}

/** @brief Set the Watchdog's Timeout Constant (value that gets reloaded)
  * @param  Timeout  Number of watchdog ticks before the watchdog times out
  * @return None.
  */
__INLINE static void WDT_SetTimeout(uint32_t Timeout)
{
    WDT->TC = Timeout;
}

/** @brief Get the Reload Value of the Watchdog Timer
  * @return Reload value of the timer
  */
__INLINE static uint32_t WDT_GetTimeout(void)
{
    return WDT->TC;
}

/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
};
#endif

#endif /* #ifndef LPC_WDT_H_ */
