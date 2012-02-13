/**************************************************************************//**
 * @file     system_lpc11xx.h
 * @brief    Header File for low-level fuctions for LPC11xx CPUs
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. June 2010
 * @license  Simplified BSD License
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

#ifndef SYSTEM_LPC11XX_H_
#define SYSTEM_LPC11XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Exported Variables ---------------------------------------------------------------------------*/

/** @defgroup LPC11xx_System_Variables System-level Core Variables for LPC11xx MCUs
  * @{
  */

extern uint32_t SystemCoreClock;                           /*!< Speed of MCU Core Clock          */
extern uint32_t SystemAHBClock;                            /*!< Speed of AHB Bus                 */

/**
  * @}
  */


/* Exported Functions ---------------------------------------------------------------------------*/

/** @defgroup LPC11xx_System_Functions System-level Core Functions for LPC11xx MCUs
  * @{
  */

/** @brief  Set up system clocks & low-level hardware.
  *
  * @return None.
  *
  * Called on bring-up of CPU for hardware configuration.
  */
extern void SystemInit(void);

/** @brief  Update SystemCoreClock and SystemAHBClock variables to match current clock config
  *
  * @return None.
  *
  * Checks system clocking registers to determine current CPU core clock speed.
  */
extern void SystemCoreClockUpdate(void);

/**
  * @}
  */

#ifdef __cplusplus
};
#endif

#endif /* #ifndef SYSTEM_LPC11XX_H_ */
