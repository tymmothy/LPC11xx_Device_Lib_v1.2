/**************************************************************************//**
 * @file     crp.h
 * @brief    Code Protection Interface Header for NXP LPC Microcontrollers.
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     January 2012
 ******************************************************************************
 * @section Overview
 * This file gives a basic interface to NXP LPC Code Read Protection.
 *
 * Protection is controlled by a special coded word at a specific location in
 * Flash memory; it indicates the types of restrictions placed
 * on the boot loader and debugging interfaces.
 *
 * @note
 * This file & the related crp.c source file require specific support of the
 * linker script(s) used to generate object code to place the CodeProtection
 * variable at the correct place in Flash for CRP to function.
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

#ifndef NXP_LPC_CRP_H_
#define NXP_LPC_CRP_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/

#include <stdint.h>


/**
  * @defgroup CRP_AbstractionLayer CRP (Code Read Protection) Abstraction Layer
  * @ingroup  LPC_Peripheral_AbstractionLayer
  * @{
  */

/* Definitions --------------------------------------------------------------*/

/**
  * @defgroup CRP_Definitions CRP Interface Definitions
  * @{
  */

/** @defgroup CRP_Settings Code Read Protection Values
  * @{
  */

#define CRP_DISABLE (0x00000000UL) /*!< No CRP Protection                                     */
#define CRP_NO_ISP  (0x4e697370UL) /*!< Prevent entering ISP mode on startup from PIO0_1      */
#define CRP_CRP1    (0x12345678UL) /*!< SWD access is disabled, partial FLASH updates allowed */
#define CRP_CRP2    (0x87654321UL) /*!< SWD access is disabled, ISP only allows erase         */
#define CRP_CRP3    (0x43218765UL) /*!< SWD access is disabled, no ISP from PIO0_1            */

/** @} */

/**
  * @}
  */

/* External Variables -------------------------------------------------------*/

/**
  * @defgroup CRP_Exported_Variables CRP Interface Exported Variables
  * @{
  */

/*! @brief Firmware Code Read Protection Configuration */
extern const uint32_t CodeProtection;

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
};
#endif

#endif /* #ifndef NXP_LPC_CRP_H_ */
