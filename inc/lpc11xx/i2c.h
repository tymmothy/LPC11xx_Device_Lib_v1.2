/**************************************************************************//**
 * @file     i2c.h
 * @brief    I2C Interface Header File for NXP LPC Microcontrollers
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. January 2012
 ******************************************************************************
 * @section Overview
 * This file gives a basic interface to NXP LPC I2C controllers.
 *
 * @note
 * This file does not handle the following necessary steps for I2C use:
 * - The I2C (AHB/APB/VPB) bus clock line must be enabled
 * - In many cases, IO Pins must be configured for I2C use in the
 *   (IOCON/PINCONFIG) block
 * - For interrupt use, an interrupt handler must be declared and
 *   the I2C's interrupt line must be enabled in the microcontroller's
 *   interrupt controller.
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
    I2C_Status_BusError              = 0x00, /*!< Bus Error.                                     */

    /* Master Tx & Rx States */
    I2C_Status_MasterStart           = 0x08, /*!< Sent START condition                           */
    I2C_Status_MasterRepeatedStart   = 0x10, /*!< Repeatedly sent START                          */

    /* Master Tx States */
    I2C_Status_Master_TxAddr_Acked   = 0x18, /*!< Sent slave address + write, got ACK            */
    I2C_Status_Master_TxAddr_Nacked  = 0x20, /*!< Sent slave address + write, got NACK           */
    I2C_Status_Master_TxData_Acked   = 0x28, /*!< Sent data to slave, got ACK                    */
    I2C_Status_Master_TxData_Nacked  = 0x30, /*!< Sent data to slave, got NACK                   */
    I2C_Status_Mater_ArbLost         = 0x38, /*!< Lost arbitration                               */

    /* Master Rx States */
    I2C_Status_Master_RxAddr_Acked   = 0x40, /*!< Sent slave address + read, got ACK             */
    I2C_Status_Master_RxAddr_Nacked  = 0x48, /*!< Sent slave address + read, got NACK            */
    I2C_Status_Master_RxData_Acked   = 0x50, /*!< Received data from slave, sent ACK             */
    I2C_Status_Master_RxData_Nacked  = 0x58, /*!< Received data from slave, sent NACK            */

    /* Slave Rx States */
    I2C_Status_Slave_RxAddr          = 0x60, /*!< Received addr from master, sent ACK            */
    I2C_Status_Slave_RxAddr_ArbLost  = 0x68, /*!< Rcvd addr from master (lost arb'n), sent ACK   */
    I2C_Status_Slave_RxGC            = 0x70, /*!< Received Gen Call, sent ACK                    */
    I2C_Status_Slave_RxGC_ArbLost    = 0x78, /*!< Received Gen Call (lost arbitration), sent ACK */
    I2C_Status_Slave_RxData_Acked    = 0x80, /*!< Received data from master, sent ACK            */
    I2C_Status_Slave_RxData_Nacked   = 0x88, /*!< Received data from master, sent NACK           */
    I2C_Status_Slave_RxGCData_Acked  = 0x90, /*!< Received General Call + data, sent ACK         */
    I2C_Status_Slave_RxGCData_Nacked = 0x98, /*!< Received General Call + data, sent NACK        */
    I2C_Status_Slave_RxStop          = 0xa0, /*!< Received STOP                                  */

    /* Slave Tx States */
    I2C_Status_Slave_TxAddr          = 0xa8, /*!< Received addr from master, sent ACK            */
    I2C_Status_Slave_TxAddr_ArbLost  = 0xb0, /*!< Rcvd addr from master (lost arb'n), sent ACK   */
    I2C_Status_Slave_TxData_Acked    = 0xb8, /*!< Sent data to master, sent ACK                  */
    I2C_Status_Slave_TxData_Nacked   = 0xc0, /*!< Sent data to master, sent NACK                 */
    I2C_Status_Slave_TxDone          = 0xc8, /*!< Finished sending data to master                */

    /* Misc. */
    I2C_Status_NoInfo                = 0xf8, /*!< I2C controller is between states or idle       */
} I2C_Status_Type;

/** @} */

/**
  * @}
  */


/* Inline Functions ---------------------------------------------------------*/

/** @addtogroup I2C_InlineFunctions I2C Interface Inline Functions
  * @{
  */

/** @brief Enable an I2C controller.
  * @param  I2C         The I2C controller
  * @return             None.
  */
__INLINE static void I2C_Enable(I2C_Type *I2C)
{
    I2C->CONSET = I2C_I2EN;
}

/** @brief Disable an I2C controller.
  * @param  I2C         The I2C controller
  * @return             None.
  */
__INLINE static void I2C_Disable(I2C_Type *I2C)
{
    I2C->CONCLR = I2C_I2EN;
}

/** @brief Test whether an I2C controller is enabled.
  * @param  I2C         The I2C controller
  * @return             1 if the I2C interface is enabled, 0 otherwise.
  */
__INLINE static unsigned int I2C_IsEnabled(I2C_Type *I2C)
{
    return (I2C->CONSET & I2C_I2EN) ? 1:0;
}

/** @brief Get the status (current state) of an I2C controller.
  * @param  I2C         The I2C controller
  * @return             The current state of the I2C controller.
  */
__INLINE static unsigned int I2C_GetStatus(I2C_Type *I2C)
{
    return I2C->STAT;
}

/** @brief Set one of the slave addresses of an I2C controller.
  * @param  I2C         The I2C controller
  * @param  ANum        The number [0-3] of the slave address to set
  * @param  Addr        The address to assign
  * @param  IgnMask     Bits in the address to ignore when matching the address
  * @return             None.
  */
__INLINE static void I2C_Slave_SetAddress(I2C_Type *I2C, unsigned int AddrNum, uint8_t Addr, uint8_t IgnMask)
{
    lpclib_assert(AddrNum <= 3);

    if (AddrNum == 0) {
        I2C->ADR0 = Addr;
    } else {
        ((uint32_t *)(&(I2C->ADR1)))[AddrNum - 1] = Addr;
    }

    ((uint32_t *)(&(I2C->MASK0)))[AddrNum] = IgnMask & I2C_MASK_Mask;
}

/** @brief Set the duty cycle of an I2C controller.
  * @param  I2C         The I2C controller
  * @param  High        The number of clock cycles in an I2C cycle for which SCL is high
  * @param  Low         The number of clock cycles in an I2C cycle for which SCL is low
  * @return             None.
  */
__INLINE static void I2C_Master_SetDutyCycle(I2C_Type *I2C, uint8_t HighCycles, uint8_t LowCycles)
{
    I2C->SCLH = HighCycles;
    I2C->SCLL = LowCycles;
}

/** @brief Enable reception of General Call messages on an I2C controller.
  * @return             None.
  */
__INLINE static void I2C_Slave_EnableGeneralCall(I2C_Type *I2C)
{
    I2C->ADR0 |= I2C_GC;
}

/** @brief Disable reception of General Call messages on an I2C controller.
  * @return             None.
  */
__INLINE static void I2C_Slave_DisableGeneralCall(I2C_Type *I2C)
{
    I2C->ADR0 |= I2C_GC;
}

/** @brief Test whether reception of General Call messages is enabled on an I2C controller.
  * @return             1 if GC message reception is enabled, 0 otherwise.
  */

__INLINE static unsigned int I2C_Slave_GeneralCallIsEnabled(I2C_Type *I2C)
{
    return (I2C->ADR0 & I2C_GC) ? 1:0;
}

/** @brief Send a START condition on an I2C controller (initiate a transfer).
  * @return             None.
  */
__INLINE static void I2C_Master_SendStart(I2C_Type *I2C)
{
    I2C->CONSET = I2C_STA;
}

/** @brief Send a STOP condition on an I2C controller (conclude a transfer).
  * @return             None.
  */
__INLINE static void I2C_Master_SendStop(I2C_Type *I2C)
{
    I2C->CONSET = I2C_STO;
}

/** @brief Queue a byte for sending via the I2C controller.
  * @param  I2C         The I2C Controller
  * @param  b           The byte to send
  * @return             None.
  *
  * @note This doesn't initiate any transfers, just loads the next byte
  *       to be sent via the I2C controller.
  */
__INLINE static void I2C_SendByte(I2C_Type *I2C, uint8_t b)
{
    I2C->DAT = b;
}

/** @brief Read the last received byte from the I2C controller.
  * @param  I2C         The I2C Controller
  * @return             The last byte received.
  *
  * @note This doesn't initiate any transfers, just returns the last byte
  *       that the I2C controller received.
  */
__INLINE static uint8_t I2C_RecvByte(I2C_Type *I2C)
{
    return I2C->DAT;
}

/** @brief Test whether an I2C controller has any interrupts pending.
  * @param  I2C         The I2C controller
  * @return             1 if the I2C controller has interrupts pending, 0 otherwise.
  */
__INLINE static unsigned int I2C_ITIsPending(I2C_Type *I2C)
{
    return (I2C->STAT & I2C_SI) ? 1:0;
}

/** @brief Test whether an I2C controller is currently busy.
  * @param  I2C         The I2C controller
  * @return             1 if the I2C controller is busy, 0 otherwise.
  *
  * @note  The I2C controller is considered to be busy when the interrupt
  *        flag is not set.
  */
__INLINE static unsigned int I2C_IsBusy(I2C_Type *I2C)
{
    return (I2C->STAT & I2C_SI) == 0;
}

/** @brief Enable monitor mode on an I2C controller.
  * @param  I2C         The I2C controller
  * @return             None.
  */
__INLINE static void I2C_EnableMonitor(I2C_Type *I2C, unsigned int disableSCL, unsigned int matchAll)
{
    I2C->MMCTRL |= I2C_MM_ENA | (disableSCL ? I2C_ENA_SCL : 0) | (matchAll ? I2C_MATCH_ALL : 0);
}

/** @brief Test whether monitor mode is enabled on an I2C controller.
  * @param  I2C         The I2C controller
  * @return             1 if monitor mode is enabled, 0 otherwise.
  */
__INLINE static unsigned int I2C_MonitorIsEnabled(I2C_Type *I2C)
{
    return (I2C->MMCTRL & I2C_MM_ENA) ? 1:0;
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
