/**************************************************************************//**
 * @file     gpio.h
 * @brief    General Purpose IO Interface Header File for NXP LPC11xx MCU's.
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. January 2012
 ******************************************************************************
 * @section Overview
 * This file gives a basic interface to NXP LPC11[C]xx microcontroller
 * GPIO.  It abstracts such things as setting pin directions, setting /
 * clearing pins, setting up interrupts on pins and reading pins.
 *
 * @note
 * This file does not handle the following necessary steps for GPIO use:
 * - The GPIO (AHB/APB/VPB) bus clock line must be enabled
 * - In many cases, IO Pins must be configured for GPIO use in the IOCON block
 * - For interrupt use, an interrupt handler must be declared and
 *   the GPIO port's interrupt line must be enabled in the microcontroller's
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

#ifndef NXP_LPC_GPIO_H_
#define NXP_LPC_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "lpc11xx.h"
#include "lpclib_assert.h"


/**
  * @defgroup GPIO_AbstractionLayer GPIO (General Purpose IO) Abstraction Layer
  * @ingroup  LPC_Peripheral_AbstractionLayer
  * @{
  */
/* Definitions --------------------------------------------------------------*/

/*! @brief Number of pins per GPIO port. */
#define GPIO_NUM_PINS   (12)

/* Types & Type-Related Definitions -----------------------------------------*/

/** @defgroup GPIO_Types GPIO Interface Types and Type-Related Definitions
  * @{
  */

/** @defgroup GPIO_Directions GPIO Direction Types
  * @{
  */

/*! @brief GPIO pin direction settings */
typedef enum {
    GPIO_Direction_In  = 0,            /*!< GPIO pin configured for input    */
    GPIO_Direction_Out = 1             /*!< GPIO pin configured for output   */
} GPIO_DirectionType;

/*! @brief Macro to test whether parameter is valid GPIO pin direction value */
#define GPIO_IS_DIRECTION_TYPE(Direction) (((Direction) == GPIO_Direction_In) \
                                        || ((Direction) == GPIO_Direction_Out))

/** @} */

/** @defgroup GPIO_SenseConditions GPIO Sense Conditions
  * @{
  */

/** @brief GPIO pin sense condition configurations
  * These are the types of possible interrupt triggers.
  */
typedef enum {
    GPIO_Sense_FallingEdge  = 0,      /*!< Sense GPIO signal falling edge    */
    GPIO_Sense_Low          = 1,      /*!< Sense GPIO signal low level       */
    GPIO_Sense_RisingEdge   = 2,      /*!< Sense GPIO signal rising edge     */
    GPIO_Sense_High         = 3,      /*!< Sense GPIO signal high level      */
    GPIO_Sense_BothEdges    = 4       /*!< Sense GPIO signal both edges      */
} GPIO_Sense_Type;

/*! @brief Macro to test whether the parameter is a valid pin sense config */
#define GPIO_IS_SENSE_TYPE(Sense)  (((Sense) == GPIO_Sense_FallingEdge) \
                                 || ((Sense) == GPIO_Sense_Low)         \
                                 || ((Sense) == GPIO_Sense_RisingEdge)  \
                                 || ((Sense) == GPIO_Sense_High)        \
                                 || ((Sense) == GPIO_Sense_BothEdges))

/** @} */

/** @defgroup GPIO_Pins GPIO pins
  * @{
  */

#define GPIO_PinMask_Pin0    (1 << 0)      /*!< GPIO Pin 0                       */
#define GPIO_PinMask_Pin1    (1 << 1)      /*!< GPIO Pin 1                       */
#define GPIO_PinMask_Pin2    (1 << 2)      /*!< GPIO Pin 2                       */
#define GPIO_PinMask_Pin3    (1 << 3)      /*!< GPIO Pin 3                       */
#define GPIO_PinMask_Pin4    (1 << 4)      /*!< GPIO Pin 4                       */
#define GPIO_PinMask_Pin5    (1 << 5)      /*!< GPIO Pin 5                       */
#define GPIO_PinMask_Pin6    (1 << 6)      /*!< GPIO Pin 6                       */
#define GPIO_PinMask_Pin7    (1 << 7)      /*!< GPIO Pin 7                       */
#define GPIO_PinMask_Pin8    (1 << 8)      /*!< GPIO Pin 8                       */
#define GPIO_PinMask_Pin9    (1 << 9)      /*!< GPIO Pin 9                       */
#define GPIO_PinMask_Pin10   (1 << 10)     /*!< GPIO Pin 10                      */
#define GPIO_PinMask_Pin11   (1 << 11)     /*!< GPIO Pin 11                      */

/*! @brief Mask of no GPIO pins */
#define GPIO_PinMask_NONE    (0x0000)

/*! @brief Mask of all GPIO pins */
#define GPIO_PinMask_ALL     (0x0fff)

/*! @brief Mask of all valid GPIO lines */
#define GPIO_Pin_Mask    (0x00000fffUL)

/** @} */

/**
  * @}
  */


/* Inline Functions ---------------------------------------------------------*/

/**
  * @defgroup GPIO_InlineFunctions GPIO Interface Inline Functions
  *
  * @{
  */

/** @brief Write the given bits to the given GPIO port's pins
  * @param  GPIO        The GPIO port
  * @param  Pins        A bitmask of pins on the GPIO port
  * @param  PinValues   A bitmask of values to apply to those pins
  * @return             None.
  *
  * Bits will be essentially ANDed with Pins in hardware, and written to those
  * pins.
  * e.g. if Pins == GPIO_Pin_3 | GPIO_Pin_4 and Bits = GPIO_Pin_4, pin 3
  * on the GPIO port will be set low, and pin 4 will be set high (dependent
  * of course on the data direction settings of the port)
  */
__INLINE static void GPIO_WritePins(GPIO_Type *GPIO, uint16_t Pins, uint16_t PinValues)
{
    lpclib_assert((Pins & ~GPIO_Pin_Mask) == 0);
    lpclib_assert((PinValues & ~GPIO_Pin_Mask) == 0);

    GPIO->SELDATA[Pins] = PinValues;
}

/** @brief Read the logic state of the given pins.
  * @param  GPIO        The GPIO port to write to
  * @param  Pins        The pins on the GPIO port to read
  * @return             None.
  *
  * The return value will have the states of the pins as individual bits.
  *  e.g. if Pins == GPIO_Pin_3 and pin 3 on that port is high, it will
  *  return a binary "1" in bit position 3.
  */
__INLINE static uint16_t GPIO_ReadPins(GPIO_Type *GPIO, uint16_t Pins)
{
    lpclib_assert((Pins & ~GPIO_Pin_Mask) == 0);

    return GPIO->SELDATA[Pins];
}

/** @brief Set the state of the given pins to High (if configured for output)
  * @param  GPIO      The GPIO port to write to
  * @param  Pins      The pins on the GPIO port to affect
  * @return None.
  */
__INLINE static void GPIO_SetPins(GPIO_Type *GPIO, uint16_t Pins)
{
    lpclib_assert((Pins & ~GPIO_Pin_Mask) == 0);

    GPIO->SELDATA[Pins] = Pins;
}

/** @brief Set the state of the given pins to Low (if configured for output)
  * @param  GPIO      The GPIO port to write to
  * @param  Pins      The pins on the GPIO port to affect
  * @return None.
  */
__INLINE static void GPIO_ClearPins(GPIO_Type *GPIO, uint16_t Pins)
{
    lpclib_assert((Pins & ~GPIO_Pin_Mask) == 0);

    GPIO->SELDATA[Pins] = 0;
}

/** @brief Invert the state of the given pins
  * @param  GPIO      The GPIO port whose pins will be toggled
  * @param  Pins      The pins on the GPIO port to affect
  * @return None.
  */
__INLINE static void GPIO_InvertPins(GPIO_Type *GPIO, uint16_t Pins)
{
    lpclib_assert((Pins & ~GPIO_Pin_Mask) == 0);

    GPIO->SELDATA[Pins] ^= Pins;
}

/** @brief Set the directions of specified pins of the GPIO port
  * @param  GPIO      The GPIO port to read from
  * @param  Pins      Pins for which to configure direction
  * @param  Direction The direction to set for given pins
  * @return None.
  */
__INLINE static void GPIO_SetPinDirections(GPIO_Type *GPIO, uint16_t Pins,
                                           GPIO_DirectionType Direction)
{
    lpclib_assert((Pins & ~GPIO_Pin_Mask) == 0);
    lpclib_assert(GPIO_IS_DIRECTION_TYPE(Direction));

    if (Direction) {
        GPIO->DIR |= Pins;
    } else {
        GPIO->DIR &= ~Pins;
    }
}

/** @brief Get the direction of a single pin of the GPIO port
  * @param  GPIO   The GPIO port to read from
  * @param  Pin    The pin for which to get the direction
  * @return None.
  */
__INLINE static uint16_t GPIO_GetPinDirection(GPIO_Type *GPIO, unsigned int Pin)
{
    lpclib_assert(Pin < GPIO_NUM_PINS);

    return (GPIO->DIR & Pin) ? GPIO_Direction_In:GPIO_Direction_Out;
}

/** @brief Set the interrupt level / edge sensing configuration for given pins
  * @param  GPIO   The GPIO port for which to set the sensing settings
  * @param  Pins   The pins for which to set the sensing configurations
  * @param  Sense  The sensing configuration for the pin
  * @return None.
  */
__INLINE static void GPIO_SetPinITSenseTypes(GPIO_Type *GPIO, uint16_t Pins,
                                             GPIO_Sense_Type Sense)
{
    lpclib_assert((Pins & ~GPIO_Pin_Mask) == 0);
    lpclib_assert(GPIO_IS_SENSE_TYPE(Sense));

    if (Sense == GPIO_Sense_BothEdges) {
        GPIO->IBE |= Pins;
    } else {
        GPIO->IBE &= ~Pins;
    }

    if ((Sense == GPIO_Sense_Low) || (Sense == GPIO_Sense_High)) {
        GPIO->IS |= Pins;
    } else {
        GPIO->IS &= ~Pins;
    }

    if ((Sense == GPIO_Sense_RisingEdge) || (Sense == GPIO_Sense_High)) {
        GPIO->IEV |= Pins;
    } else {
        GPIO->IEV &= ~Pins;
    }
}

/** @brief Get the interrupt level / edge sensing configuration for a given pin
  * @param  GPIO    The GPIO port for which to get the sensing configuration
  * @param  Pin     The Pin for which to configure direction
  * @return The sensing configuration of the pin
  */
__INLINE static GPIO_SenseType GPIO_GetPinITSenseType(GPIO_Type *GPIO, uint16_t Pin)
{
    lpclib_assert((Pin & ~GPIO_Pin_Mask) == 0);

    if (GPIO->IS & Pin) {
        if (GPIO->IEV & Pin) {
            return GPIO_Sense_High;
        } else {
            return GPIO_Sense_Low;
        }
    } else {
        if (GPIO->IBE & Pin) {
            return GPIO_Sense_BothEdges;
        } else if (GPIO->IEV & Pin) {
            return GPIO_Sense_RisingEdge;
        } else {
            return GPIO_Sense_FallingEdge;
        }
    }
}

/** @brief Enable IRQs for specified pins on a given GPIO port
  * @param  GPIO  The GPIO port for which to enable pin interrupts
  * @param  Pins  ORed values of all pins for which to enable interrupts
  * @return None.
  */
__INLINE static void GPIO_EnableIRQForPins(GPIO_Type *GPIO, uint16_t Pins)
{
    lpclib_assert((Pins & ~GPIO_Pin_Mask) == 0);

    GPIO->IE |= Pins;
}

/** @brief Disable IRQs for specified pins on a given GPIO port
  * @param  GPIO  The GPIO port for which to disable pin interrupts
  * @param  Pins  ORed values of all pins for which to disable interrupts
  * @return None.
  */
__INLINE static void GPIO_DisableIRQForPins(GPIO_Type *GPIO, uint16_t Pins)
{
    lpclib_assert((Pins & ~GPIO_Pin_Mask) == 0);

    GPIO->IE &= ~Pins;
}

/** @brief Get the raw interrupt states for pins on a given GPIO port
  * @param  GPIO  The GPIO port for which to get raw interrupts
  * @return Raw interrupt status for each pin of the GPIO port
  *
  * Each bit [0-11] in the return value gives the raw interrupt
  *  status for the corresponding GPIO pin regardless of interrupt
  *  masking.
  */
__INLINE static uint16_t GPIO_GetRawITStatusForPort(GPIO_Type *GPIO)
{
    return GPIO->RIS;
}

/** @brief Get the pending interrupts for a given GPIO port
  * @param  GPIO  The GPIO port for which to get pending interrupts
  * @return Pending interrupt status for each pin of the GPIO port
  *
  * Each bit [0-11] in the return value gives the pending interrupt
  *  status for the corresponding GPIO pin.
  */
__INLINE static uint16_t GPIO_GetPendingITStatusForPort(GPIO_Type *GPIO)
{
    return GPIO->MIS;
}

/** @brief Clear pending edge-triggered interrupts on given pins
  * @param  GPIO  The GPIO port for which to clear interrupts
  * @param  Pins  The pins of the port for which to clear interrupts
  * @return None.
  */
__INLINE static void GPIO_ClearPendingITStatusForPins(GPIO_Type *GPIO, uint16_t Pins)
{
    lpclib_assert((Pins & ~GPIO_Pin_Mask) == 0);

    GPIO->IC |= Pins;
    /* Prevent spurious interrupts if clearing in ISR */
    __asm__ __volatile__ ("    nop\n    nop\n");
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

#endif /* #ifndef NXP_LPC_GPIO_H_ */
