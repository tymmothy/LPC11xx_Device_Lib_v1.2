/**************************************************************************//**
 * @file     wdt.h
 * @brief    Watchdog Timer Interface Header for NXP LPC Microcontrollers.
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     January 2012
 ******************************************************************************
 * @section Overview
 * This file gives a basic interface to NXP LPC microcontroller
 * watchdog timers.
 *
 * @note
 * This file does not handle the following necessary steps for WDT use:
 * - If the WDT oscillator is used as the input clock, its speed must be
 *   set and power enabled, and its divider configured.
 * - The WDT's (AHB or APB/VPB) bus clock must be enabled.
 * - The WDT's input clock must be set and its divider configured.
 ******************************************************************************
 * @section License
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

/** @brief Enable the watchdog timer.
  *
  * @note
  * - Once enabled, the watchdog timer cannot be disabled without a chip reset.
  * - As a side-effect, this will clear the watchdog's "timed out" flag.  You probably want this.
  * - On L-series (LPC11xxL) parts, this will also lock the watchdog's clock source.
  */
__INLINE static void WDT_Enable(void)
{
    WDT->MOD |= WDT_WDEN;
}

/** @brief Test whether the watchdog timer is enabled.
  * @return             1 if the WDT is enabled, 0 otherwise.
  */
__INLINE static uint32_t WDT_IsEnabled(void)
{
    return (WDT->MOD & WDT_WDEN) ? 1:0;
}

/** @brief Enable watchdog timer chip reset on timeout.
  *
  * @note
  * - Once enabled, watchdog timer chip resetting cannot be disabled without a chip reset.
  * - As a side-effect, this will clear the watchdog's "timed out" flag.  You probably want this.
  */
__INLINE static void WDT_EnableChipReset(void)
{
   WDT->MOD |= WDT_WDRESET;
}

/** @brief Test whether the watchdog timer's chip reset mode is enabled.
  * @return             1 if chip resetting is enabled, 0 otherwise.
  */
__INLINE static uint32_t WDT_ChipResetIsEnabled(void)
{
    return (WDT->MOD & WDT_WDRESET) ? 1:0;
}

/** @brief Test whether the watchdog timer has timed out.
  * @return             1 if the watchdog timer has timed out, 0 otherwise.
  */
__INLINE static uint8_t WDT_IsTimedOut(void)
{
    return (WDT->MOD & WDT_WDTOF) ? 1:0;
}

/** @brief Test whether the watchdog timer interrupt is pending.
  * @return             1 if the timer interrupt is pending, 0 otherwise.
  */
__INLINE static uint8_t WDT_ITIsPending(void)
{
    return (WDT->MOD & WDT_WDINT) ? 1:0;
}

/** @brief Get the current value of the watchdog timer's counter.
  * @return             The current counter value.
  */
__INLINE static uint32_t WDT_GetCurrentValue(void)
{
    return WDT->TV;
}

/** @brief Feed the watchdog timer (load the timer constant).
  *
  * @note
  * This is necessary to load the current value in the Timer Constant
  * register as the watchdog's Timer Reload value.  This is also necessary
  * to start the watchdog, after it's been enabled, and enable changes to
  * WDT settings.
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

/** @brief Set the watchdog's timeout constant.
  * @param[in]  timeout      The number of ticks before the watchdog times out (range 0-16777215)
  *
  * @note
  * The watchdog timer's timeout constant is the number of watchdog ticks before
  * the watchdog timer times out (and interrupts or resets the microcontroller).  It is reloaded
  * into the watchdog timer's time value whenever a watchdog feed occurs.
  *
  * @sa WDT_Feed
  */
__INLINE static void WDT_SetTimeout(uint32_t timeout)
{
    lpclib_assert((timeout & ~WDT_TC_Mask) == 0);

    WDT->TC = timeout;
}

/** @brief Get the watchdog timer's current timeout constant.
  * @return             The current timeout constant value of the watchdog timer.
  *
  * @sa WDT_SetTimeout
  */
__INLINE static uint32_t WDT_GetTimeout(void)
{
    return WDT->TC;
}


#if defined(LPC11XXL)  /* L-series parts have windowed watchdog timer with additional features */

/** @brief Restrict WDT counter reloading to timer values between WINDOW value and 0.
  *
  * @note
  * - Window mode cannot be cleared without resetting the chip.
  * - As a side-effect, this will clear the watchdog's "timed out" flag.
  *
  * Windowed mode is meant to be used with the reset function of the WDT enabled.  In windowed
  * mode, a WDT feed will only be allowed between the time the WDT's counter reaches the window
  * value and 0; any feeds before this point will cause a chip reset.
  */
__INLINE static void WDT_EnableWindow(void)
{
    WDT->MOD |= WDT_WDPROTECT;
}

/** @brief Set the watchdog's 10-bit warning interrupt match value.
  * @param[in]  count        The WDT count match that will generate an interrupt (0-1023)
  *
  * @note
  * The warning interrupt is generated when the WDT counter's upper 14 bits are all 0 and the
  * bottom 10 bits match the warning interrupt count value.
  */
__INLINE static void WDT_SetWarningITCount(uint16_t count)
{
    lpclib_assert((count & ~WDT_WARNINT_Mask) == 0);

    WDT->WARNING = count;
}

/** @brief Get the watchdog's current 10-bit warning interrupt match value.
  * @return            The current WDT warning interrupt match value.
  *
  * @sa WDT_SetWarningITCount
  */
__INLINE static uint16_t WDT_GetWarningITCount(void)
{
    return WDT->WARNING;
}

/** @brief Set the maximum allowed WDT count value for feeding the WDT.
  * @param[in]  window       The new max allowed count value for feeding the WDT (range 0-16777215)
  *
  * @sa WDT_EnableWindow
  */
__INLINE static void WDT_SetWindow(uint32_t Window)
{
    lpclib_assert((Window & ~WDT_WINDOW_Mask) == 0);

    WDT->WINDOW = Window;
}

/** @brief Get the current maximum allowed WDT count value for feeding the WDT.
  * @return             The current maximum allowed WDT count for feeding the WDT.
  *
  * @sa WDT_EnableWindow
  */
__INLINE static void WDT_GetWindow(void)
{
    return WDT->WINDOW;
}

#endif /* #if defined(LPC11XXL) */

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
