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
 * - The GPIO (AHB or APB/VPB) bus clock line must be enabled
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
  * (sense interrupt triggers.
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

#define GPIO_Pin_0    (1 << 0)      /*!< GPIO Pin 0                       */
#define GPIO_Pin_1    (1 << 1)      /*!< GPIO Pin 1                       */
#define GPIO_Pin_2    (1 << 2)      /*!< GPIO Pin 2                       */
#define GPIO_Pin_3    (1 << 3)      /*!< GPIO Pin 3                       */
#define GPIO_Pin_4    (1 << 4)      /*!< GPIO Pin 4                       */
#define GPIO_Pin_5    (1 << 5)      /*!< GPIO Pin 5                       */
#define GPIO_Pin_6    (1 << 6)      /*!< GPIO Pin 6                       */
#define GPIO_Pin_7    (1 << 7)      /*!< GPIO Pin 7                       */
#define GPIO_Pin_8    (1 << 8)      /*!< GPIO Pin 8                       */
#define GPIO_Pin_9    (1 << 9)      /*!< GPIO Pin 9                       */
#define GPIO_Pin_10   (1 << 10)     /*!< GPIO Pin 10                      */
#define GPIO_Pin_11   (1 << 11)     /*!< GPIO Pin 11                      */

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

/** @brief Set the logic states of all pins of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin_values   A bitmask of values to apply to the pins
  */
__INLINE static void GPIO_WritePort(GPIO_Type *gpio, uint32_t pin_values)
{
    lpclib_assert((pin_values & ~GPIO_Pin_Mask) == 0);

    gpio->DATA = pin_values;
}

/** @brief Read the logic state of all pins of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @return                  A bitmask of pin logic states.
  */
__INLINE static uint32_t GPIO_ReadPort(GPIO_Type *gpio)
{
    return gpio->DATA;
}

/** @brief Write the given bits to the specified pins of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin_mask     A bitmask of GPIO pins
  * @param[in]  pin_values   A bitmask of values to apply to those pins
  *
  * @note
  * Bits will be essentially ANDed with Pins in hardware, and written to those
  * pins.
  * e.g. if Pins == GPIO_Pin_3 | GPIO_Pin_4 and Bits = GPIO_Pin_4, pin 3
  * on the GPIO port will be set low, and pin 4 will be set high (dependent
  * of course on the data direction settings of the port)
  */
__INLINE static void GPIO_WritePins(GPIO_Type *gpio, uint32_t pin_mask, uint32_t pin_values)
{
    lpclib_assert((pin_mask & ~GPIO_Pin_Mask) == 0);
    lpclib_assert((pin_values & ~GPIO_Pin_Mask) == 0);

    gpio->SELDATA[pin_mask] = pin_values;
}

/** @brief Read the logic state of the specified pins of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin_mask     A bitmask of GPIO pins
  * @return                  A bitmask of pin logic states (only those specified in mask are valid).
  *
  * @note
  * - Only pins specified in the mask will have valid entries in the returned value.
  */
__INLINE static uint32_t GPIO_ReadPins(GPIO_Type *gpio, uint32_t pin_mask)
{
    lpclib_assert((pin_mask & ~GPIO_Pin_Mask) == 0);

    return gpio->SELDATA[pin_mask];
}

/** @brief Set the state of the specified pins of a GPIO port to high.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin_mask     A bitmask of GPIO pins
  */
__INLINE static void GPIO_SetPinsHigh(GPIO_Type *gpio, uint32_t pin_mask)
{
    lpclib_assert((pin_mask & ~GPIO_Pin_Mask) == 0);

    gpio->SELDATA[pin_mask] = pin_mask;
}

/** @brief Set the state of the specified pins of a GPIO port to low.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin_mask     A bitmask of GPIO pins
  */
__INLINE static void GPIO_SetPinsLow(GPIO_Type *gpio, uint32_t pin_mask)
{
    lpclib_assert((pin_mask & ~GPIO_Pin_Mask) == 0);

    gpio->SELDATA[pin_mask] = 0;
}

/** @brief Set the directions of the specified pins of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin_mask     A bitmask of GPIO pins
  * @param[in]  direction    The direction to set for the specified pins
  */
__INLINE static void GPIO_SetPinDirections(GPIO_Type *gpio, uint32_t pin_mask,
                                           GPIO_DirectionType direction)
{
    lpclib_assert((pin_mask & ~GPIO_Pin_Mask) == 0);
    lpclib_assert(GPIO_IS_DIRECTION_TYPE(direction));

    if (direction) {
        gpio->DIR |= pin_mask;
    } else {
        gpio->DIR &= ~pin_mask;
    }
}

/** @brief Invert the logic state of the specified pins of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin_mask     A bitmask of GPIO pins
  */
__INLINE static void GPIO_InvertPins(GPIO_Type *gpio, uint32_t pin_mask)
{
    lpclib_assert((pin_mask & ~GPIO_Pin_Mask) == 0);

    gpio->SELDATA[pin_mask] ^= pin_mask;
}

/** @brief Set the logic state of a single pin of a GPIO port to high.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin          A GPIO pin number (0 - 11)
  */
__INLINE static void GPIO_SetPinHigh(GPIO_Type *gpio, unsigned int pin)
{
    lpclib_assert(pin < GPIO_NUM_PINS);

    gpio->SELDATA[1 << pin] = (1 << pin);
}

/** @brief Set the logic state of a single pin of a GPIO port to low.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin          A GPIO pin number (0 - 11)
  */
__INLINE static void GPIO_SetPinLow(GPIO_Type *gpio, unsigned int pin)
{
    lpclib_assert(pin < GPIO_NUM_PINS);

    gpio->SELDATA[1 << pin] = 0;
}

/** @brief Invert the logic state of a single pin of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin          A GPIO pin number (0 - 11)
  */
__INLINE static void GPIO_InvertPin(GPIO_Type *gpio, unsigned int pin)
{
    lpclib_assert(pin < GPIO_NUM_PINS);

    gpio->SELDATA[1 << pin] ^= (1 << pin);
}

/** @brief Set the direction of a single pin of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin          A GPIO pin number (0 - 11)
  * @param[in]  direction    The new pin direction
  */
__INLINE static void GPIO_SetPinDirection(GPIO_Type *gpio, unsigned int pin, GPIO_DirectionType direction)
{
    lpclib_assert(pin < GPIO_NUM_PINS);
    lpclib_assert(GPIO_IS_DIRECTION_TYPE(direction));

    if (direction) {
        gpio->DIR |= (1 << pin);
    } else {
        gpio->DIR &= ~(1 << pin);
    }
}

/** @brief Get the direction of a single pin of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin          A GPIO pin number (0 - 11)
  * @return                  The pin's current direction.
  */
__INLINE static uint16_t GPIO_GetPinDirection(GPIO_Type *gpio, unsigned int pin)
{
    lpclib_assert(pin < GPIO_NUM_PINS);

    return (gpio->DIR & (1 << pin)) ? GPIO_Direction_In:GPIO_Direction_Out;
}

/** @brief Set the sensing configuration for the specified pins of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin_mask     A bitmask of GPIO pins
  * @param[in]  sense        The new sensing setting for the specified pins
  */
__INLINE static void GPIO_SetSenseConfigForPins(GPIO_Type *gpio, uint32_t pin_mask,
                                             GPIO_Sense_Type sense)
{
    lpclib_assert((pin_mask & ~GPIO_Pin_Mask) == 0);
    lpclib_assert(GPIO_IS_SENSE_TYPE(sense));

    if (sense == GPIO_Sense_BothEdges) {
        gpio->IBE |= pin_mask;
    } else {
        gpio->IBE &= ~pin_mask;
    }

    if ((sense == GPIO_Sense_Low) || (sense == GPIO_Sense_High)) {
        gpio->IS |= pin_mask;
    } else {
        gpio->IS &= ~pin_mask;
    }

    if ((sense == GPIO_Sense_RisingEdge) || (sense == GPIO_Sense_High)) {
        gpio->IEV |= pin_mask;
    } else {
        gpio->IEV &= ~pin_mask;
    }
}

/** @brief Get the sensing configuration for the specified pin of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin          A GPIO pin number (0 - 11)
  * @return                  The current sensing setting of the pin.
  */
__INLINE static GPIO_Sense_Type GPIO_GetPinSenseConfig(GPIO_Type *gpio, unsigned int pin)
{
    uint32_t pin_mask;


    lpclib_assert(pin < GPIO_NUM_PINS);

    pin_mask = (1 << pin);

    if (gpio->IS & pin_mask) {
        if (gpio->IEV & pin_mask) {
            return GPIO_Sense_High;
        } else {
            return GPIO_Sense_Low;
        }
    } else {
        if (gpio->IBE & pin_mask) {
            return GPIO_Sense_BothEdges;
        } else if (gpio->IEV & pin_mask) {
            return GPIO_Sense_RisingEdge;
        } else {
            return GPIO_Sense_FallingEdge;
        }
    }
}

/** @brief Enable sense interrupts for the specified pins of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin_mask     A bitmask of GPIO pins
  */
__INLINE static void GPIO_EnableInterruptsForPins(GPIO_Type *gpio, uint32_t pin_mask)
{
    lpclib_assert((pin_mask & ~GPIO_Pin_Mask) == 0);

    gpio->IE |= pin_mask;
}

/** @brief Disable sense interrupts for the specified pins of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin_mask     A bitmask of GPIO pins
  */
__INLINE static void GPIO_DisableInterruptsForPins(GPIO_Type *gpio, uint32_t pin_mask)
{
    lpclib_assert((pin_mask & ~GPIO_Pin_Mask) == 0);

    gpio->IE &= ~pin_mask;
}

/** @brief Get the raw interrupt states for the specified pins of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @return                  A bitmask of raw interrupt status bits.
  *
  * @note
  * Each bit [0-11] in the return value gives the raw interrupt
  * status for the corresponding GPIO pin regardless of interrupt
  * masking.
  */
__INLINE static uint32_t GPIO_GetRawInterruptsMask(GPIO_Type *gpio)
{
    return gpio->RIS;
}

/** @brief Get the pending sense interrupt states for the specified pins of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @return                  A bitmask of pending interrupt status bits.
  *
  * @note
  * Each bit [0-11] in the return value gives the pending interrupt
  * status for the corresponding GPIO pin.
  */
__INLINE static uint32_t GPIO_GetPendingInterruptsMask(GPIO_Type *gpio)
{
    return gpio->MIS;
}

/** @brief Clear pending edge-triggered sense interrupts for the specified pins of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin_mask     A bitmask of GPIO pins
  */
__INLINE static void GPIO_ClearPendingInterruptsForPins(GPIO_Type *gpio, uint32_t pin_mask)
{
    lpclib_assert((pin_mask & ~GPIO_Pin_Mask) == 0);

    gpio->IC |= pin_mask;

    /* Prevent spurious interrupts if clearing in ISR */
    __asm__ __volatile__ ("    nop\n    nop\n");
}

/** @brief Clear pending edge-triggered sense interrupts for the specified pins of a GPIO port.
  * @param[in]  gpio         A pointer to a GPIO instance
  * @param[in]  pin          A GPIO pin number (0 - 11)
  */
__INLINE static void GPIO_ClearPendingInterruptsForPin(GPIO_Type *gpio, unsigned int pin)
{
    lpclib_assert(pin < GPIO_NUM_PINS);

    gpio->IC |= (1 << pin);

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
