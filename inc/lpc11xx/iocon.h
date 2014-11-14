/**************************************************************************//**
 * @file     iocon.h
 * @brief    IO Configuration Interface Header for LPC11xx Microcontrollers
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. January 2012
 ******************************************************************************
 * @section Overview
 * This file gives a basic interface to the IO configuration block of
 * NXP LPC11xx microcontrollers.  It abstracts such things as configuring
 * the peripheral controlling IO pins, enabling pull-up/pull-down resistors,
 * and enabling digital mode for ADC input pins configured as GPIO.
 *
 * @note
 * This file does not handle the following necessary steps for IOCON use:
 * - The IOCON (AHB or APB/VPB) bus clock line must be enabled
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

#ifndef NXP_LPC_IOCON_H_
#define NXP_LPC_IOCON_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "lpc11xx.h"
#include "lpclib_assert.h"


/**
  * @defgroup IOCON_AbstractionLayer IOCON (IO Configuration Block) Abstraction Layer
  * @ingroup  LPC_Peripheral_AbstractionLayer
  * @{
  */

/* Types & Type-Related Definitions -----------------------------------------*/

/** @defgroup IOCON_Types IOCON Interface Types and Type-Related Definitions
  * @{
  */

/** @defgroup IOCON_Pin_Locations IOCON Block Pin Locations
  * @{
  */

/*! @brief IOCON Pin Locations */
typedef enum {
    IOCON_Pin_2_6 = 0x00,                                  /*!< Port 2 pin 6                     */
    IOCON_Pin_2_0 = 0x02,                                  /*!< Port 2 pin 0                     */
    IOCON_Pin_0_0,                                         /*!< Port 0 pin 0                     */
    IOCON_Pin_0_1,                                         /*!< Port 0 pin 1                     */
    IOCON_Pin_1_8,                                         /*!< Port 1 pin 8                     */
    IOCON_Pin_0_2 = 0x07,                                  /*!< Port 0 pin 2                     */
    IOCON_Pin_2_7,                                         /*!< Port 2 pin 7                     */
    IOCON_Pin_2_8,                                         /*!< Port 2 pin 8                     */
    IOCON_Pin_2_1,                                         /*!< Port 2 pin 1                     */
    IOCON_Pin_0_3,                                         /*!< Port 0 pin 3                     */
    IOCON_Pin_0_4,                                         /*!< Port 0 pin 4                     */
    IOCON_Pin_0_5,                                         /*!< Port 0 pin 5                     */
    IOCON_Pin_1_9,                                         /*!< Port 1 pin 9                     */
    IOCON_Pin_3_4,                                         /*!< Port 3 pin 4                     */
    IOCON_Pin_2_4,                                         /*!< Port 2 pin 4                     */
    IOCON_Pin_2_5,                                         /*!< Port 2 pin 5                     */
    IOCON_Pin_3_5,                                         /*!< Port 3 pin 5                     */
    IOCON_Pin_0_6,                                         /*!< Port 0 pin 6                     */
    IOCON_Pin_0_7,                                         /*!< Port 0 pin 7                     */
    IOCON_Pin_2_9,                                         /*!< Port 2 pin 9                     */
    IOCON_Pin_2_10,                                        /*!< Port 2 pin 10                    */
    IOCON_Pin_2_2,                                         /*!< Port 2 pin 2                     */
    IOCON_Pin_0_8,                                         /*!< Port 0 pin 8                     */
    IOCON_Pin_0_9,                                         /*!< Port 0 pin 9                     */
    IOCON_Pin_0_10,                                        /*!< Port 0 pin 10                    */
    IOCON_Pin_1_10,                                        /*!< Port 1 pin 10                    */
    IOCON_Pin_2_11,                                        /*!< Port 2 pin 11                    */
    IOCON_Pin_0_11,                                        /*!< Port 0 Pin 11                    */
    IOCON_Pin_1_0,                                         /*!< Port 1 Pin 0                     */
    IOCON_Pin_1_1,                                         /*!< Port 1 Pin 1                     */
    IOCON_Pin_1_2,                                         /*!< Port 1 Pin 2                     */
    IOCON_Pin_3_0,                                         /*!< Port 3 Pin 0                     */
    IOCON_Pin_3_1,                                         /*!< Port 3 Pin 1                     */
    IOCON_Pin_2_3,                                         /*!< Port 2 Pin 3                     */
    IOCON_Pin_1_3,                                         /*!< Port 1 Pin 3                     */
    IOCON_Pin_1_4,                                         /*!< Port 1 Pin 4                     */
    IOCON_Pin_1_11,                                        /*!< Port 1 Pin 11                    */
    IOCON_Pin_3_2,                                         /*!< Port 3 Pin 2                     */
    IOCON_Pin_1_5,                                         /*!< Port 1 Pin 5                     */
    IOCON_Pin_1_6,                                         /*!< Port 1 Pin 6                     */
    IOCON_Pin_1_7,                                         /*!< Port 1 Pin 7                     */
    IOCON_Pin_3_3                                          /*!< Port 3 Pin 3                     */
} IOCON_Pin_Type;

/*! @brief Macro to test whether parameter is a valid IO Pin */
#define IOCON_IS_PIN(Pin)  (((Pin) >= IOCON_Pin_2_6) \
                         && ((Pin) <= IOCON_Pin_3_3) \
                         && ((Pin) != 0x01)          \
                         && ((Pin) != 0x06))

/*! @brief Macro to test whether parameter is a valid ADC Pin */
#define IOCON_IS_AD_PIN(Pin)   (((Pin) == IOCON_Pin_0_11) \
                             || ((Pin) == IOCON_Pin_1_0)  \
                             || ((Pin) == IOCON_Pin_1_1)  \
                             || ((Pin) == IOCON_Pin_1_2)  \
                             || ((Pin) == IOCON_Pin_1_3)  \
                             || ((Pin) == IOCON_Pin_1_4)  \
                             || ((Pin) == IOCON_Pin_1_10) \
                             || ((Pin) == IOCON_Pin_1_11))

/*! @brief Macro to test whether parameter is a valid I2C Pin */
#define IOCON_IS_I2C_PIN(Pin)  (((Pin) == IOCON_Pin_0_4) \
                             || ((Pin) == IOCON_Pin_0_5))

#if defined(LPC11XXL) /* L-series Parts Only */
/*! @brief Macro to test whether parameter is a valid (pseudo) Open Drain Pin */
# define IOCON_IS_OD_PIN(Pin)  (IOCON_IS_PIN(Pin) && (!IOCON_IS_I2C_PIN(Pin)))
#endif

/** @} */

/** @defgroup IOCON_Function_Settings  IO Configuration Block Pin Function Settings
  * @{
  */

/*! @brief IOCON Pin Function settings
 *  These names are pretty non-descriptive, but are used to build more useful
 *  simplified pin settings below.
 */
typedef enum {
    IOCON_Function_Default = 0x00,                         /*!< Default Pin Function Setting     */
    IOCON_Function_Alt1,                                   /*!< Alternate Pin Function #1        */
    IOCON_Function_Alt2,                                   /*!< Alternate Pin Function #2        */
    IOCON_Function_Alt3,                                   /*!< Alternate Pin Function #3        */
} IOCON_Function_Type;

/*! @brief Macro to test whether parameter is a valid IOCON pin function */
#define IOCON_IS_FUNCTION(Function) \
                                  (((Function) == IOCON_Function_Default) \
                                || ((Function) == IOCON_Function_Alt1)    \
                                || ((Function) == IOCON_Function_Alt2)    \
                                || ((Function) == IOCON_Function_Alt3))

#define IOCON_Function_Mask            (0x03)              /*!< Function bits in IOCON registers */

/** @} */

/** @defgroup IOCON_Modes IO Configuration Pin Pullup / Pulldown Modes
  * N/A on I2C Pins IOCON_Pin_PIO_0_4, IOCON_Pin_PIO_0_5
  * @{
  */

/*! @brief IOCON Pin Mode Configuration Settings */
typedef enum {
    IOCON_Mode_Normal   = 0x00,                            /*!< Normal mode (no PU/PD resistors) */
    IOCON_Mode_PD       = 0x08,                            /*!< Pulldown resistor enabled        */
    IOCON_Mode_PU       = 0x10,                            /*!< Pullup resistor enabled          */
    IOCON_Mode_Repeater = 0x18                             /*!< Repeater Mode enabled on pin     */
} IOCON_Mode_Type;

/*! @brief Macro to test whether parameter is a valid IOCON pin mode */
#define IOCON_IS_MODE(Mode) (((Mode) == IOCON_Mode_Normal) \
                               || ((Mode) == IOCON_Mode_PD)     \
                               || ((Mode) == IOCON_Mode_PU)     \
                               || ((Mode) == IOCON_Mode_Repeater))

#define IOCON_Mode_Mask                (0x03 << 3)         /*!< Pin Mode bits in IOCON registers */
#define IOCON_Mode_Shift               (3)                 /*!< Shift of IOCON pin mode bits     */

/** @} */

/** @defgroup IOCON_I2CModes  IO Configuration I2C Modes
  * Applies to IOCON_Pin_PIO_0_4, IOCON_Pin_PIO_0_5 Only
  * @{
  */

/*! @brief IOCON I2C Pin Configuration Modes */
typedef enum {
    IOCON_I2CMode_I2C                  = 0x0000,           /*!< Pin configured for I2C IO        */
    IOCON_I2CMode_PIO                  = 0x0100,           /*!< Pin configured for GPIO          */
    IOCON_I2CMode_I2CFastPlus          = 0x0200            /*!< Pin config'd for I2C FastPlus IO */
} IOCON_I2CMode_Type;

/*! Macro to test whether parameter is a valid IOCON I2C mode */
#define IOCON_IS_I2C_MODE(Mode) (((Mode) == IOCON_I2CMode_I2C) \
                              || ((Mode) == IOCON_I2CMode_PIO) \
                              || ((Mode) == IOCON_I2CMode_I2CFastPlus))

#define IOCON_I2CMode_Mask             (0x03 << 8)         /*!< I2C Mode Bits in IOCON registers */

/** @} */

/** @defgroup IOCON_ADModes  IO Configuration Analog Modes
  * Applies to IOCON_Pin_PIO0_11, IOCON_Pin_PIO1_0, IOCON_Pin_PIO1_1,
  *            IOCON_Pin_PIO1_2,  IOCON_Pin_PIO1_3, IOCON_Pin_PIO1_4,
  *            IOCON_Pin_PIO1_11, IOCON_Pin_PIO1_10
  * @{
  */

/*! @brief IOCON Analog Capable Pin Configuration Modes */
typedef enum {
    IOCON_ADMode_Analog                = 0x00,             /*!< AD pin in analog mode            */
    IOCON_ADMode_Digital               = 0x80              /*!< AD pin in digital mode           */
} IOCON_ADMode_Type;

/*! @brief Macro to test whether parameter is a valid IOCON Analog/Digital Mode */
#define IOCON_IS_AD_MODE(Mode) (((Mode) == IOCON_ADMode_Analog) \
                             || ((Mode) == IOCON_ADMode_Digital))

/** @} */

#if defined(LPC11XXL)  /* L-series parts only */

/** @defgroup IOCON_ODModes  IO Configuration (pseudo) Open Drain Modes
  * Applies to non-I2C Pins
  * @{
  */

/*! @brief IOCON (pseudo) Open-Drain Pin Configuration Modes */
typedef enum {
    IOCON_ODMode_Normal                = 0x0000,           /*!< Pin configured for normal op     */
    IOCON_ODMode_OpenDrain             = 0x0400            /*!< Pin configured for open drain op */
} IOCON_ODMode_Type;

/*! @brief Macro to test whether parameter is a valid IOCON Open Drain Mode */
#define IOCON_IS_OD_MODE(Mode) (((Mode) == IOCON_ODMode_Normal) \
                             || ((Mode) == IOCON_ODMode_OpenDrain))

/** @} */

#endif /* #if defined(LPC11XXL) */

/** @defgroup IOCON_SCK0Locations SSP0 SCK Pin Selections
  * @{
  */

/*! @brief IOCON SSP0 SCK Pin Location Settings */
typedef enum {
    IOCON_SCK0Location_PIO0_10 = 0x00,                     /*!< SSP0 SCK routed to PIO0.10       */
    IOCON_SCK0Location_PIO2_11,                            /*!< SSP0 SCK routed to PIO2.11       */
    IOCON_SCK0Location_PIO0_6                              /*!< SSP0 SCK routed to PIO0.6        */
} IOCON_SCK0Location_Type;

/*! @brief Macro to test whether parameter is a valid SSP0 SCK pin location */
#define IOCON_IS_SCK0_LOCATION(Location) \
                                  (((Location) == IOCON_SCK0Location_PIO0_10) \
                                || ((Location) == IOCON_SCK0Location_PIO2_11) \
                                || ((Location) == IOCON_SCK0Location_PIO0_6))

/** @} */

/** @defgroup IOCON_DSR0Locations UART 0 DSR Pin Selection
  * @{
  */

/*! @brief IOCON UART0 DSR Pin Location Settings */
typedef enum {
    IOCON_DSR0Location_PIO2_1 = 0x00,                      /*!< UART0 DSR routed to PIO2.1       */
    IOCON_DSR0Location_PIO3_1                              /*!< UART0 DSR routed to PIO3.1       */
} IOCON_DSR0Location_Type;

/*! @brief Macro to test whether parameter is a valid UART0 DSR pin location */
#define IOCON_IS_DSR0_LOCATION(Location) \
                                   (((Location) == IOCON_DSR0Location_PIO2_1) \
                                 || ((Location) == IOCON_DSR0Location_PIO3_1))

/** @} */

/** @defgroup IOCON_DCD0Locations UART 0 DCD Pin Selection
  * @{
  */

/*! @brief IOCON UART0 DCD Pin Location Settings */
typedef enum {
    IOCON_DCD0Location_PIO2_2 = 0x00,                      /*!< UART0 DCD routed to PIO2.2       */
    IOCON_DCD0Location_PIO3_2                              /*!< UART0 DCD routed to PIO3.2       */
} IOCON_DCD0Location_Type;

/*! @brief Macro to test whether parameter is a valid UART0 DCD pin location */
#define IOCON_IS_DCD0_LOCATION(Location) \
                                   (((Location) == IOCON_DCD0Location_PIO2_2) \
                                 || ((Location) == IOCON_DCD0Location_PIO3_2))

/** @} */

/** @defgroup IOCON_RI0Locations UART 0 RI Pin Selection
  * @{
  */

/*! @brief IOCON UART0 RI Pin Location Settings */
typedef enum {
    IOCON_RI0Location_PIO2_3 = 0x00,                       /*!< UART0 RI routed to PIO2.3        */
    IOCON_RI0Location_PIO3_3                               /*!< UART0 RI routed to PIO3.3        */
} IOCON_RI0Location_Type;

/*! @brief Macro to test whether parameter is a valid UART0 RI pin location */
#define IOCON_IS_RI0_LOCATION(Location) \
                                   (((Location) == IOCON_RI0Location_PIO2_3) \
                                 || ((Location) == IOCON_RI0Location_PIO3_3))

/** @} */

/** @defgroup IOCON_PinConfigs Simplified Pin Configuration Definitions
  * @{
  */
#define IOCON_PinConfig_2_6_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_2_6)  /*!< Pin 2.6 configured as PIO                    */
#define IOCON_PinConfig_2_0_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_2_0)  /*!< Pin 2.0 configured as PIO                    */
#define IOCON_PinConfig_2_0_DTR0         (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_2_0)  /*!< Pin 2.0 configured as UART0 DTR              */
#define IOCON_PinConfig_2_0_SSEL1        (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_2_0)  /*!< Pin 2.0 configured as SSP1 SSEL              */
#define IOCON_PinConfig_0_0_Reset        (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_0_0)  /*!< Pin 0.0 configured as /RESET                 */
#define IOCON_PinConfig_0_0_PIO          (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_0_0)  /*!< Pin 0.0 configured as PIO                    */
#define IOCON_PinConfig_0_1_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_0_1)  /*!< Pin 0.1 configured as PIO                    */
#define IOCON_PinConfig_0_1_CLKOUT       (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_0_1)  /*!< Pin 0.1 configured as CLKOUT                 */
#define IOCON_PinConfig_0_1_CT32B0_MAT2  (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_0_1)  /*!< Pin 0.1 configured as CT32B0 Match Output 2  */
#define IOCON_PinConfig_1_8_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_1_8)  /*!< Pin 1.8 configured as PIO                    */
#define IOCON_PinConfig_1_8_CT16B1_CAP0  (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_1_8)  /*!< Pin 1.8 configured as CT16B0 Capture Input 0 */
#define IOCON_PinConfig_0_2_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_0_2)  /*!< Pin 0.2 configured as PIO                    */
#define IOCON_PinConfig_0_2_SSEL0        (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_0_2)  /*!< Pin 0.2 configured as SSP0 SSEL              */
#define IOCON_PinConfig_0_2_CT16B0_CAP0  (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_0_2)  /*!< Pin 0.2 configured as CT16B0 Capture Input 0 */
#define IOCON_PinConfig_2_7_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_2_7)  /*!< Pin 2.7 configured as PIO                    */
#define IOCON_PinConfig_2_8_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_2_8)  /*!< Pin 2.8 configured as PIO                    */
#define IOCON_PinConfig_2_1_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_2_1)  /*!< Pin 2.1 configured as PIO                    */
#define IOCON_PinConfig_2_1_DSR0         (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_2_1)  /*!< Pin 2.1 configured as UART0 DSR              */
#define IOCON_PinConfig_2_1_SCK1         (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_2_1)  /*!< Pin 2.1 configured as SSP1 SCK               */
#define IOCON_PinConfig_0_3_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_0_3)  /*!< Pin 0.3 configured as PIO                    */
#define IOCON_PinConfig_0_4_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_0_4)  /*!< Pin 0.4 configured as PIO                    */
#define IOCON_PinConfig_0_4_SCL0         (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_0_4)  /*!< Pin 0.4 configured as I2C0 SCL               */
#define IOCON_PinConfig_0_5_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_0_5)  /*!< Pin 0.5 configured as PIO                    */
#define IOCON_PinConfig_0_5_SDA0         (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_0_5)  /*!< Pin 0.5 configured as I2C0 SDA               */
#define IOCON_PinConfig_1_9_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_1_9)  /*!< Pin 1.9 configured as PIO                    */
#define IOCON_PinConfig_1_9_CT16B1_MAT0  (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_1_9)  /*!< Pin 1.9 configured as CT16B1 Match Output 0  */
#define IOCON_PinConfig_3_4_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_3_4)  /*!< Pin 3.4 configured as PIO                    */
#define IOCON_PinConfig_2_4_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_2_4)  /*!< Pin 2.4 configured as PIO                    */
#define IOCON_PinConfig_2_5_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_2_5)  /*!< Pin 2.5 configured as PIO                    */
#define IOCON_PinConfig_3_5_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_3_5)  /*!< Pin 3.5 configured as PIO                    */
#define IOCON_PinConfig_0_6_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_0_6)  /*!< Pin 0.6 configured as PIO                    */
#define IOCON_PinConfig_0_6_SCK0         (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_0_6)  /*!< Pin 0.6 configured as SSP0 SCK               */
#define IOCON_PinConfig_0_7_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_0_7)  /*!< Pin 0.7 configured as PIO                    */
#define IOCON_PinConfig_0_7_CTS0         (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_0_7)  /*!< Pin 0.7 configured as UART0 CTS              */
#define IOCON_PinConfig_2_9_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_2_9)  /*!< Pin 2.9 configured as PIO                    */
#define IOCON_PinConfig_2_10_PIO         (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_2_10) /*!< Pin 2.10 configured as PIO                   */
#define IOCON_PinConfig_2_2_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_2_2)  /*!< Pin 2.2 configured as PIO                    */
#define IOCON_PinConfig_2_2_DCD0         (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_2_2)  /*!< Pin 2.2 configured as UART0 DCD              */
#define IOCON_PinConfig_2_2_MISO1        (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_2_2)  /*!< Pin 2.2 configured as SSP1 MISO              */
#define IOCON_PinConfig_0_8_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_0_8)  /*!< Pin 0.8 configured as PIO                    */
#define IOCON_PinConfig_0_8_MISO0        (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_0_8)  /*!< Pin 0.8 configured as SSP0 MISO              */
#define IOCON_PinConfig_0_8_CT16B0_MAT0  (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_0_8)  /*!< Pin 0.8 configured as CT16B0 Match Output 0  */
#define IOCON_PinConfig_0_9_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_0_9)  /*!< Pin 0.9 configured as PIO                    */
#define IOCON_PinConfig_0_9_MOSI0        (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_0_9)  /*!< Pin 0.9 configured as SSP0 MOSI              */
#define IOCON_PinConfig_0_9_CT16B0_MAT1  (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_0_9)  /*!< Pin 0.9 configured as CT16B0 Match Output 1  */
#define IOCON_PinConfig_0_10_SWCLK       (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_0_10) /*!< Pin 0.10 configured as debug SWCLK pin       */
#define IOCON_PinConfig_0_10_PIO         (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_0_10) /*!< Pin 0.10 configured as PIO                   */
#define IOCON_PinConfig_0_10_SCK0        (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_0_10) /*!< Pin 0.10 configured as SSP0 SCK              */
#define IOCON_PinConfig_0_10_CT16B0_MAT2 (((((uint16_t)IOCON_Function_Alt3) << 8))    | IOCON_Pin_0_10) /*!< Pin 0.10 configured as CT16B0 Match Output 2 */
#define IOCON_PinConfig_1_10_PIO         (((((uint16_t)IOCON_Function_Default \
                                                       | IOCON_ADMode_Digital) << 8)) | IOCON_Pin_1_10) /*!< Pin 1.10 configured as PIO                   */
#define IOCON_PinConfig_1_10_AD6         (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_1_10) /*!< Pin 1.10 configured as ADC Input 6           */
#define IOCON_PinConfig_1_10_CT16B1_MAT1 (((((uint16_t)IOCON_Function_Alt2 \
                                                    | IOCON_ADMode_Digital) << 8))    | IOCON_Pin_1_10) /*!< Pin 1.10 configured as CT16B1 Match Output 1 */
#define IOCON_PinConfig_2_11_PIO         (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_2_11) /*!< Pin 2.11 configured as PIO                   */
#define IOCON_PinConfig_2_11_SCK0        (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_2_11) /*!< Pin 2.11 configured as SSP0 SCK              */
#define IOCON_PinConfig_0_11_PIO         (((((uint16_t)IOCON_Function_Alt1 \
                                                    | IOCON_ADMode_Digital) << 8))    | IOCON_Pin_0_11) /*!< Pin 0.11 configured as PIO                   */
#define IOCON_PinConfig_0_11_AD0         (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_0_11) /*!< Pin 0.11 configured as ADC Input 0           */
#define IOCON_PinConfig_0_11_CT32B0_MAT3 (((((uint16_t)IOCON_Function_Alt3 \
                                                    | IOCON_ADMode_Digital) << 8))    | IOCON_Pin_0_11) /*!< Pin 0.11 configured as CT32B0 Match Output 3 */
#define IOCON_PinConfig_1_0_PIO          (((((uint16_t)IOCON_Function_Alt1 \
                                                    | IOCON_ADMode_Digital) << 8))    | IOCON_Pin_1_0)  /*!< Pin 1.0 configured as PIO                    */
#define IOCON_PinConfig_1_0_AD1          (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_1_0)  /*!< Pin 1.0 configured as ADC Input 1            */
#define IOCON_PinConfig_1_0_CT32B1_CAP0  (((((uint16_t)IOCON_Function_Alt3 \
                                                    | IOCON_ADMode_Digital) << 8))    | IOCON_Pin_1_0)  /*!< Pin 1.0 configured as CT32B1 Capture Input 0 */
#define IOCON_PinConfig_1_1_PIO          (((((uint16_t)IOCON_Function_Alt1 \
                                                    | IOCON_ADMode_Digital) << 8))    | IOCON_Pin_1_1)  /*!< Pin 1.1 configured as PIO                    */
#define IOCON_PinConfig_1_1_AD2          (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_1_1)  /*!< Pin 1.1 configured as ADC Input 2            */
#define IOCON_PinConfig_1_1_CT32B1_MAT0  (((((uint16_t)IOCON_Function_Alt3 \
                                                    | IOCON_ADMode_Digital) << 8))    | IOCON_Pin_1_1)  /*!< Pin 1.1 configured as CT32B1 Match Output 0  */
#define IOCON_PinConfig_1_2_PIO          (((((uint16_t)IOCON_Function_Alt1 \
                                                    | IOCON_ADMode_Digital) << 8))    | IOCON_Pin_1_2)  /*!< Pin 1.2 configured as PIO                    */
#define IOCON_PinConfig_1_2_AD3          (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_1_2)  /*!< Pin 1.2 configured as ADC Input 3            */
#define IOCON_PinConfig_1_2_CT32B1_MAT1  (((((uint16_t)IOCON_Function_Alt3 \
                                                    | IOCON_ADMode_Digital) << 8))    | IOCON_Pin_1_2)  /*!< Pin 1.2 configured as CT32B1 Match Output 1  */
#define IOCON_PinConfig_3_0_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_3_0)  /*!< Pin 3.0 configured as PIO                    */
#define IOCON_PinConfig_3_0_DTR0         (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_3_0)  /*!< Pin 3.0 configured as UART0 DTR              */
#define IOCON_PinConfig_3_1_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_3_1)  /*!< Pin 3.1 configured as PIO                    */
#define IOCON_PinConfig_3_1_DSR0         (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_3_1)  /*!< Pin 3.1 configured as UART0 DSR              */
#define IOCON_PinConfig_2_3_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_2_3)  /*!< Pin 2.3 configured as PIO                    */
#define IOCON_PinConfig_2_3_RI0          (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_2_3)  /*!< Pin 2.3 configured as UART0 RI               */
#define IOCON_PinConfig_2_3_MOSI1        (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_2_3)  /*!< Pin 2.3 configured as SSP1 MOSI              */
#define IOCON_PinConfig_1_3_SWDIO        (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_1_3)  /*!< Pin 1.3 configured as debug SWDIO            */
#define IOCON_PinConfig_1_3_PIO          (((((uint16_t)IOCON_Function_Alt1 \
                                                    | IOCON_ADMode_Digital) << 8))    | IOCON_Pin_1_3)  /*!< Pin 1.3 configured as PIO                    */
#define IOCON_PinConfig_1_3_AD4          (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_1_3)  /*!< Pin 1.3 configured as ADC Input 4            */
#define IOCON_PinConfig_1_3_CT32B1_MAT2  (((((uint16_t)IOCON_Function_Alt3 \
                                                    | IOCON_ADMode_Digital) << 8))    | IOCON_Pin_1_3)  /*!< Pin 1.3 configured as CT32B1 Match Output 2  */
#define IOCON_PinConfig_1_4_PIO          (((((uint16_t)IOCON_Function_Default \
                                                       | IOCON_ADMode_Digital) << 8)) | IOCON_Pin_1_4)  /*!< Pin 1.4 configured as PIO                    */
#define IOCON_PinConfig_1_4_AD5          (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_1_4)  /*!< Pin 1.4 configured as ADC Input 5            */
#define IOCON_PinConfig_1_4_CT32B1_MAT3  (((((uint16_t)IOCON_Function_Alt2 \
                                                    | IOCON_ADMode_Digital) << 8))    | IOCON_Pin_1_4)  /*!< Pin 1.4 configured as CT32B1 Match Output 3  */
#define IOCON_PinConfig_1_11_PIO         (((((uint16_t)IOCON_Function_Default \
                                                       | IOCON_ADMode_Digital) << 8)) | IOCON_Pin_1_11) /*!< Pin 1.11 configured as PIO                   */
#define IOCON_PinConfig_1_11_AD7         (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_1_11) /*!< Pin 1.11 configured as ADC Input 7           */
#define IOCON_PinConfig_3_2_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_3_2)  /*!< Pin 3.2 configured as PIO                    */
#define IOCON_PinConfig_3_2_DCD0         (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_3_2)  /*!< Pin 3.2 configured as UART0 DCD              */
#define IOCON_PinConfig_1_5_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_1_5)  /*!< Pin 1.5 configured as PIO                    */
#define IOCON_PinConfig_1_5_RTS0         (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_1_5)  /*!< Pin 1.5 configured as UART0 RTS              */
#define IOCON_PinConfig_1_5_CT32B0_CAP0  (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_1_5)  /*!< Pin 1.5 configured as CT32B0 Capture Input 0 */
#define IOCON_PinConfig_1_6_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_1_6)  /*!< Pin 1.6 configured as PIO                    */
#define IOCON_PinConfig_1_6_RXD0         (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_1_6)  /*!< Pin 1.6 configured as UART0 RXD              */
#define IOCON_PinConfig_1_6_CT32B0_MAT0  (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_1_6)  /*!< Pin 1.6 configured as CT32B0 Match Output 0  */
#define IOCON_PinConfig_1_7_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_1_7)  /*!< Pin 1.7 configured as PIO                    */
#define IOCON_PinConfig_1_7_TXD0         (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_1_7)  /*!< Pin 1.7 configured as UART0 TXD              */
#define IOCON_PinConfig_1_7_CT32B0_MAT1  (((((uint16_t)IOCON_Function_Alt2) << 8))    | IOCON_Pin_1_7)  /*!< Pin 1.7 configured as CT32B0 Match Output 1  */
#define IOCON_PinConfig_3_3_PIO          (((((uint16_t)IOCON_Function_Default) << 8)) | IOCON_Pin_3_3)  /*!< Pin 3.3 configured as PIO                    */
#define IOCON_PinConfig_3_3_RI0          (((((uint16_t)IOCON_Function_Alt1) << 8))    | IOCON_Pin_3_3)  /*!< Pin 3.3 configured as UART0 RI               */

/*! @brief Type used for passing pin configuration settings */
typedef uint32_t IOCON_PinConfig_Type;

/** @} */

/**
  * @}
  */


/* Inline Functions ---------------------------------------------------------*/

/** @addtogroup IOCON_InlineFunctions IOCON Interface Inline Functions
  * @{
  */

/** @brief Set the function for an IO pin.
  * @param[in]  pin          An IOCON pin number
  * @param[in]  function     The new pin function
  */
__INLINE static void IOCON_SetPinFunction(IOCON_Pin_Type pin, IOCON_Function_Type function)
{
    lpclib_assert(IOCON_IS_PIN(pin));
    lpclib_assert(IOCON_IS_FUNCTION(function));

    ((__IO uint32_t *)IOCON)[pin] = (((__IO uint32_t *)IOCON)[pin] & ~IOCON_Function_Mask)
                                    | function;
}

/** @brief Get the current function of an IO pin.
  * @param[in]  pin          An IOCON pin number
  * @return                  A token indicating the current pin function.
  */
__INLINE static IOCON_Function_Type IOCON_GetPinFunction(IOCON_Pin_Type pin)
{
    return ((__IO uint32_t *)IOCON)[pin] & IOCON_Function_Mask;
}

/** @brief Simple IO pin configuration function.
  * @param[in]  config       A pin/function hybrid configuration setting
  * @param[in]  mode         The resistor/repeater mode to configure for the pin
  */
__INLINE static void IOCON_SetPinConfig(IOCON_PinConfig_Type config, IOCON_Mode_Type mode)
{
    uint16_t pin = config & 0xff;


    lpclib_assert(IOCON_IS_PIN(pin));
    lpclib_assert(IOCON_IS_FUNCTION(config >> 8));
    lpclib_assert(IOCON_IS_MODE(mode));

    ((__IO uint32_t *)IOCON)[pin] =
        (((__IO uint32_t *)IOCON)[pin] & ~(IOCON_Function_Mask | IOCON_Mode_Mask))
        | (config >> 8) | mode;
}

/** @brief Enable hysteresis on an IO pin.
  * @param[in]  pin          An IOCON pin number
  */
__INLINE static void IOCON_EnablePinHysteresis(IOCON_Pin_Type pin)
{
    lpclib_assert(IOCON_IS_PIN(pin));

    ((__IO uint32_t *)IOCON)[pin] |= IOCON_HYS;
}

/** @brief  Disable hysteresis on an IO pin.
  * @param[in]  pin          An IOCON pin number
  */
__INLINE static void IOCON_DisablePinHysteresis(IOCON_Pin_Type pin)
{
    lpclib_assert(IOCON_IS_PIN(pin));

    ((__IO uint32_t *)IOCON)[pin] &= ~IOCON_HYS;
}

/** @brief  Get the current hysteresis setting of an IO pin.
  * @param[in]  pin          An IOCON pin number
  * @return                  1 if hysteresis is enabled on the pin, 0 otherwise.
  */
__INLINE unsigned int IOCON_PinHysteresisIsEnabled(IOCON_Pin_Type pin)
{
    return (((__IO uint32_t *)IOCON)[pin] & IOCON_HYS) ? 1:0;
}

/** @brief Set the mode (resistor/repeater setting) of an IO pin.
  * @param[in]  pin          An IOCON pin number
  * @param[in]  mode         The new mode setting
  */
__INLINE static void IOCON_SetPinMode(IOCON_Pin_Type pin,
                                      IOCON_Mode_Type mode)
{
    lpclib_assert(IOCON_IS_PIN(pin));
    lpclib_assert(IOCON_IS_MODE(mode));

    ((__IO uint32_t *)IOCON)[pin] = (((__IO uint32_t *)IOCON)[pin] & ~IOCON_Mode_Mask)
                                    | mode;
}

/** @brief  Get the current mode (resistor/repeater setting) of an IO pin.
  * @param[in]  pin          An IOCON pin number
  * @return                  The current pin mode.
  */
__INLINE static IOCON_Mode_Type IOCON_GetPinMode(IOCON_Pin_Type pin)
{
    lpclib_assert(IOCON_IS_PIN(pin));

    return (((__IO uint32_t *)IOCON)[pin] & IOCON_Mode_Mask);
}

/** @brief Set Pin I2C Mode
  * @param[in]  pin          An IOCON pin number (IOCON_Pin_Pin_PIO_0_4/0_5 Only)
  * @param[in]  i2c_mode     The new I2C mode
  */
__INLINE static void IOCON_SetPinI2CMode(IOCON_Pin_Type pin,
                                         IOCON_I2CMode_Type i2c_mode)
{
    lpclib_assert(IOCON_IS_I2C_PIN(pin));
    lpclib_assert(IOCON_IS_I2C_MODE(i2c_mode));

    ((__IO uint32_t *)IOCON)[pin] = (((__IO uint32_t *)IOCON)[pin] & ~IOCON_I2C_Mask)
                                    | i2c_mode;
}

/** @brief Get the current I2C mode of an IO pin.
  * @param[in]  pin          An IOCON pin number (IOCON_Pin_Pin_PIO_0_4/0_5 Only)
  * @return                  The current I2C mode of the pin.
  */
__INLINE static IOCON_I2CMode_Type IOCON_GetPinI2CMode(IOCON_Pin_Type pin)
{
    lpclib_assert(IOCON_IS_I2C_PIN(pin));

    return ((__IO uint32_t *)IOCON)[pin] & IOCON_I2C_Mask;
}

/** @brief Enable analog mode on an IO pin.
  * @param[in]  pin          An IOCON pin number (ADC input pins only)
  */
__INLINE static void IOCON_EnablePinAnalogMode(IOCON_Pin_Type pin)
{
    lpclib_assert(IOCON_IS_AD_PIN(pin));

    /* Note: AD bit is 0 when Analog Mode is enabled */
    ((__IO uint32_t *)IOCON)[pin] &= ~IOCON_AD;
}

/** @brief Disable analog mode on an IO pin.
  * @param[in]  pin          An IOCON pin number (ADC input pins only)
  */
__INLINE static void IOCON_DisablePinAnalogMode(IOCON_Pin_Type pin)
{
    lpclib_assert(IOCON_IS_AD_PIN(pin));

    /* Note: AD bit is 0 when Analog Mode is enabled */
    ((__IO uint32_t *)IOCON)[pin] |= IOCON_AD;
}

/** @brief Test whether analog mode is enabled on an IO pin.
  * @param[in]  pin          An IOCON pin number (ADC input pins only)
  * @return                  1 if AD mode is enabled on the pin, 0 otherwise.
  */
__INLINE static int IOCON_PinAnalogModeIsEnabled(IOCON_Pin_Type pin)
{
   lpclib_assert(IOCON_IS_AD_PIN(pin));

    return (((__IO uint32_t *)IOCON)[pin] & IOCON_AD) ? 0:1;
}

#if defined(LPC11XXL)  /* L-series parts only; pseudo Open-Drain Mode */

/** @brief Enable pseudo open-drain mode on an IO pin.
  * @param[in]  pin          An IOCON pin number (non-I2C pins only)
  */
__INLINE static void IOCON_EnablePinODMode(IOCON_Pin_Type pin)
{
    lpclib_assert(IOCON_IS_OD_PIN(pin));

    ((__IO uint32_t *)IOCON)[pin] |= IOCON_OD;
}

/** @brief Disable pseudo open-drain mode on an IO pin.
  * @param[in]  pin          An IOCON pin number (non-I2C pins only)
  */
__INLINE static void IOCON_DisablePinOpenDrainMode(IOCON_Pin_Type pin)
{
    lpclib_assert(IOCON_IS_OD_PIN(pin));

    ((__IO uint32_t *)IOCON)[pin] &= ~IOCON_OD;
}

#endif /* #if defined(LPC11XXL) */

/** @brief Set the location of SSP0's SCK pin.
  * @param[in]  loc          The location to route the signal.
  */
__INLINE static void IOCON_SetSCK0Location(IOCON_SCK0Location_Type loc)
{
    lpclib_assert(IOCON_IS_SCK0_LOCATION(loc));

    IOCON->SCK0_LOC = loc;
}

/** @brief Get the current location of SSP0's SCK pin.
  * @return                  The current location of the SCK pin.
  */
__INLINE static IOCON_SCK0Location_Type IOCON_GetSCK0Location(void)
{
    return IOCON->SCK0_LOC;
}

/** @brief Set the location of UART0's DSR pin.
  * @param[in]  loc          The location to route the signal.
  */
__INLINE static void IOCON_SetDSR0Location(IOCON_DSR0Location_Type loc)
{
    lpclib_assert(IOCON_IS_DSR0_LOCATION(loc));

    IOCON->DSR0_LOC = loc;
}

/** @brief Get the current location of UART0's DSR pin.
  * @return                  The current location of the DSR pin.
  */
__INLINE static IOCON_DSR0Location_Type IOCON_GetDSR0Location(void)
{
    return IOCON->DSR0_LOC;
}

/** @brief Set the location of UART0's DCD pin.
  * @param[in]  loc          The location to route the signal.
  */
__INLINE static void IOCON_SetDCD0Location(IOCON_DCD0Location_Type loc)
{
    lpclib_assert(IOCON_IS_DCD0_LOCATION(loc));

    IOCON->DCD0_LOC = loc;
}

/** @brief Get the current location of UART0's DCD pin.
  * @return                  The current location of the DCD pin.
  */
__INLINE static IOCON_DCD0Location_Type IOCON_GetDCD0Location(void)
{
    return IOCON->DCD0_LOC;
}

/** @brief Set the location of UART0's RI pin.
  * @param[in]  loc          The location to route the signal.
  */
__INLINE static void IOCON_SetRI0Location(IOCON_RI0Location_Type loc)
{
    lpclib_assert(IOCON_IS_RI0_LOCATION(loc));

    IOCON->RI0_LOC = loc;
}

/** @brief Get the current location of UART0's RI pin.
  * @return                  The current location of the RI pin.
  */
__INLINE static IOCON_RI0Location_Type IOCON_GetRI0Location(void)
{
    return IOCON->RI0_LOC;
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

#endif /* #ifndef NXP_LPC_IOCON_H_ */
