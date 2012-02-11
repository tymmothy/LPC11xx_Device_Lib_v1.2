/******************************************************************************
 * @file:    pmu.h
 * @purpose: Header File for LPC11xx MCU Power Control Unit
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    1. June 2010
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

#ifndef LPC_PMU_H_
#define LPC_PMU_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/

#include <stdint.h>
#include "lpc11xx.h"

/**
  * @defgroup PMU_Access_Interface PMU Access-level Interface
  * @ingroup  LPC_Peripheral_Access_Layer
  * @{
  */

/* Inline Functions ---------------------------------------------------------*/

/**
  * @defgroup PMU_Inline_Functions PMU Access-level Inline Functions
  * @{
  */

/** @brief  Enable Deep Power Down Mode
  * @return None.
  */
__INLINE static void PMU_EnableDeepPowerDown(void)
{
    PMU->PCON |= PMU_DPDEN;
}

/** @brief  Disable Deep Power Down Mode
  * @return None.
  */
__INLINE static void PMU_DisableDeepPowerDown(void)
{
    PMU->PCON &= ~PMU_DPDEN;
}

/** @brief  Determine Whether Deep Power Down Mode is Enabled
  * @return 1 if DPD Mode is enabled, 0 otherwise
  */
__INLINE static uint32_t PMU_DeepPowerDownIsEnabled(void)
{
    return (PMU->PCON & PMU_DPDEN) ? 1:0;
}

/** @brief  Test whether the chip is in sleep mode
  * @return 1 if the CPU is in sleep mode, 0 otherwise.
  */
__INLINE static uint8_t PMU_IsInSleepMode(void)
{
    return (PMU->PCON & PMU_SLEEPFLAG);
}

/** @brief  Test whether the chip is in deep power down mode
  * @return 1 if the CPU is in deep power down mode, 0 otherwise.
  */
__INLINE static uint8_t PMU_IsInDeepPowerDownMode(void)
{
    return (PMU->PCON & PMU_DPDFLAG);
}

/** @brief  Enable Hysteresis on the Wakeup Pin
  * @return None.
  */
__INLINE static void PMU_EnableWakeupHysteresis(void)
{
    PMU->GPREG4 |= PMU_WAKEUPHYS;
}

/** @brief  Disable Hysteresis on the Wakeup Pin
  * @return None.
  */
__INLINE static void PMU_DisableWakeupHysteresis(void)
{
    PMU->GPREG4 &= ~PMU_WAKEUPHYS;
}

/** @brief Test whether hysteresis is enabled on the wakeup pin
  * @return 1 if hysteresis is enabled on the wakeup pin, 0 otherwise.
  */
__INLINE static uint8_t PMU_WakeupHysteresisIsEnabled(void)
{
    return (PMU->GPREG4 & PMU_WAKEUPHYS) ? 1:0;
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

#endif /* #ifndef LPC_PMU_H_ */
