/******************************************************************************
 * @file:    flash.h
 * @purpose: Flash Memory Interface Header for NXP LPC11xx Microcontrollers
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    1. January 2012
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
  * @defgroup FLASH_Access_Interface Flash Control Access-level Interface
  * @ingroup  LPC_Peripheral_Access_Layer
  * @{
  */

/* Types & Type-Related Definitions -----------------------------------------*/

/**
  * @defgroup FLASH_Access_Types FLASH Access-level Interface Types & Definitions
  * @{
  */

/** @defgroup FLASH_Latency_Types FLASH Latency Types
  * @{
  */

/*! Flash Control Latency (wait-state) Configuration Settings */
typedef enum {
    FLASH_Latency_0 = 0x00,            /*!< No Latency on Flash accesses     */
    FLASH_Latency_1,                   /*!< 1-cycle Flash access latency     */
    FLASH_Latency_2                    /*!< 2-cycle Flash access latency     */
} FLASH_Latency_Type;

/*! Macro to test whether the parameter is a valid Flash latency setting */
#define IS_FLASH_LATENCY_TYPE(Latency) (((Latency) == FLASH_Latency_0) \
                                     || ((Latency) == FLASH_Latency_1) \
                                     || ((Latency) == FLASH_Latency_2))

/** @} */

/**
  * @}
  */


/* Inline Functions ----------------------------------------------------------*/

/** @defgroup FLASH_Inline_Functions FLASH Access-level Inline Functions
  * @{
  */

/** @brief Set Flash latency (must be 1 if chip @ >=20Mhz, 2 if >=40Mhz)
  * @param  Latency   Number of wait states per Flash access
  * @return None.
  */
__INLINE static void FLASH_SetLatency(FLASH_Latency_Type Latency)
{
    lpclib_assert(IS_FLASHCFG_LATENCY_TYPE(Latency));

    FLASH->FLASHCFG = (FLASH->FLASHCFG & ~FLASH_FLASHTIM_Mask) | Latency;
}

/** @brief Get currently configured Flash latency
  * @return  Number of configured wait states per Flash access
  */
__INLINE static uint16_t FLASH_GetLatency(void)
{
    return FLASH->FLASHCFG & (FLASH_FLASHTIM_Mask);
}

/** @brief Generate a signature for flash memory
  * @param  start_addr  The (16-byte aligned) start address of memory block
  * @param  start_addr  The (16-byte aligned) end address of memory block
  * @param  result      Where the (128-bit / 4 word) resulting signature is put
  * @return  None.
  */
__INLINE static void FLASH_GenerateSignature(uint32_t start_addr,
                                             uint32_t end_addr,
                                             uint32_t result[4])
{
    /* Start/end addresses should be 16-byte aligned. */
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
