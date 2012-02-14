/**************************************************************************//**
 * @file     i2c.h
 * @brief    I2C Interface Header File for NXP LPC Microcontrollers
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. January 2012
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

#ifndef NXP_LPC_I2C_H_
#define NXP_LPC_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/

#include <stdint.h>
#include "lpc11xx.h"
#include "lpclib_assert.h"


/**
  * @defgroup I2C_AbstractionLayer I2C (Inter-Integreated Circuit) Abstraction Layer
  * @ingroup  LPC_Peripheral_AbstractionLayer
  * @{
  */

/* Types & Type-Related Definitions -----------------------------------------*/

/** @defgroup I2C_Types I2C Interface Types and Type-Related Definitions
  * @{
  */

/** @defgroup I2C_Status_Type I2C Status Values
  * @{
  */

/*! @brief I2C State Machine Status Codes */
typedef enum {
    I2C_Status_Start = 0x08,
    I2C_Status_RepeatedStart = 0x10,
    I2C_Status_SLAW_Acked = 0x18,
    I2C_Status_SLAW_Nacked = 0x20,
    I2C_Status_Data_Acked = 0x28,
    I2C_Status_Data_Nacked = 0x30,
    I2C_Status_ArbitrationLost = 0x38
} I2C_Status_Type;

/** @} */

/**
  * @}
  */


/* Inline Functions ---------------------------------------------------------*/

/** @addtogroup I2C_InlineFunctions I2C Interface Inline Functions
  * @{
  */

/** @brief Initialize the I2C Interface
  * @param  Init  Settings to initialize the I2C with
  * @return None.
  */
__INLINE static void I2C_Init(I2C_Init_Type *Init)
{
}


/** @brief Enable the I2C Interface
  * @param  I2C   The I2C Interface to enable
  * @return None.
  */
__INLINE static void I2C_Enable(I2C_Type *I2C)
{
    I2C->CONSET |= I2C_I2EN;
}


/** @brief Disable the I2C Interface
  * @param  I2C   The I2C Interface to disable
  * @return None.
  */
__INLINE static void I2C_Disable(I2C_Type *I2C)
{
    I2C->CONCLR |= I2C_I2EN;
}


/** @brief Get the status of the I2C Interface
  * @param  I2C   The I2C Interface for which to get status
  * @return None.
  */
__INLINE static void I2C_GetStatus(I2C_Type *I2C)
{
    return I2C->STAT;
}


/** @brief Set one of the slave addresses of the I2C Interface
  * @param  I2C   The I2C Interface for which to set the address
  * @param  ANum  The number [0-3] of the slave address to set
  * @param  Addr  The address to assign
  * @param  IgnMask  The address to assign
  * @return None.
  */
__INLINE static void I2C_SetSlaveAddress(I2C_Type *I2C, unsigned int AddrNum, uint8_t Addr, uint8_t IgnMask)
{
    lpclib_assert(AddrNum <= 3);

    if (AddrNum == 0) {
        I2C->ADR0 = Addr;
    } else {
        ((uint32_t *)(&(I2C->ADR1)))[AddrNum - 1] = Addr;
    }

    ((uint32_t *)(&(I2C->MASK0)))[AddrNum] = Mask & I2C_MASK_Mask;
}


/** @brief Set the duty cycle of the I2C Interface
  * @param  I2C   The I2C Interface for which to set the duty cycle
  * @param  High  The count for SCL High time
  * @param  Low   The count for SCL Low time
  * @return None.
  */
__INLINE static void I2C_SetDutyCycle(I2C_Type *I2C, uint8_t HighCycles, uint8_t LowCycles)
{
    I2C->SCLH = HighCycles;
    I2C->SCLL = LowCycles;
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

#endif /* #ifndef NXP_LPC_I2C_H_ */
