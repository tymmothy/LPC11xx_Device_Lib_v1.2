/**************************************************************************//**
 * @file     flash.h
 * @brief    Flash Memory Interface Header for NXP LPC11xx Microcontrollers
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. January 2012
 ******************************************************************************
 * @section Overview
 * This file gives a basic interface to NXP LPC11xx Flash Memory Controllers.
 *
 * It allows setting of Flash wait states & generating 128-bit signatures
 * of Flash memory ranges.
 *
 * @note
 * This file does not handle the following necessary steps for FLASH controller
 * use:
 * - The FLASHCTRL (AHB or APB/VPB) bus clock line must be enabled
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

#ifndef NXP_LPC_FLASH_H_
#define NXP_LPC_FLASH_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

#include "lpc11xx.h"
#include "lpclib_assert.h"


/**
  * @defgroup FLASH_AbstractionLayer Flash Control Abstraction Layer
  * @ingroup  LPC_Peripheral_AbstractionLayer
  * @{
  */

/* Inline Functions ----------------------------------------------------------*/

/** @defgroup FLASH_InlineFunctions FLASH Interface Inline Functions
  * @{
  */

/** @brief Set the Flash fetch latency.
  * @param[in]  waits        The new number of wait states per Flash access (0 - 2)
  *
  * Must have 1 or 2 cycle latency @ >=20Mhz; 2 cycles @ >=40Mhz.
  */
__INLINE static void FLASH_SetWaitStates(unsigned int waits)
{
    lpclib_assert(waits <= 2);

    FLASH->FLASHCFG = (FLASH->FLASHCFG & ~FLASH_FLASHTIM_Mask) | waits;
}

/** @brief Get the currently configured Flash fetch latency.
  * @return                  The number of configured wait states per Flash access.
  */
__INLINE static unsigned int FLASH_GetWaitStates(void)
{
    return FLASH->FLASHCFG & (FLASH_FLASHTIM_Mask);
}

/** @brief Generate a signature for a range of flash memory.
  * @param[in]  start_addr   The start address of the flash memory (16-byte aligned)
  * @param[in]  end_addr     The end address of the flash memory   (16-byte aligned)
  * @param[out] result       Pointer to location to store the resulting signature (4x32b words)
  */
__INLINE static void FLASH_GenerateSignature(uint32_t start_addr,
                                             uint32_t end_addr,
                                             uint32_t result[4])
{
    /* Verify start & end are 16-byte aligned. */
    lpclib_assert((start_addr & 0x0f) == 0);
    lpclib_assert((end_addr & 0x0f) == 0);

    /* Clear the "DONE" flag */
    FLASH->FMSTATCLR = FLASH_SIG_DONE_CLR;

    /* Plug in start / end addresses & kick SIG_START to begin */
    FLASH->FMSSTART = start_addr >> 4;
    FLASH->FMSEND = (end_addr >> 4) | FLASH_SIG_START;

    /* Wait for signature generation to complete (if this is run from Flash,
     * it would block until finished anyhow)
     */
    while ((FLASH->FMSTAT & FLASH_SIG_DONE) == 0);

    /* Copy out the results */
    result[0] = FLASH->FMSW0;
    result[1] = FLASH->FMSW1;
    result[2] = FLASH->FMSW2;
    result[3] = FLASH->FMSW3;
}

/** @} */

/**
  * @}
  */


#ifdef __cplusplus
};
#endif

#endif /* #ifndef NXP_LPC_FLASH_H_ */
