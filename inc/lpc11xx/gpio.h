/*****************************************************************************
 * @file:    gpio.h
 * @purpose: General Purpose IO Interface Header File for NXP LPC11xx MCU's
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
 *****************************************************************************
 *
 * This implementation uses bit representations for pin numbers which allows
 *  functions to work on multiple pins at the same time.
 *
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
  * @defgroup GPIO_Access_Interface General Purpose IO Access-level Interface
  * @ingroup  LPC_Peripheral_Access_Layer
  * @{
  */

/* Types & Type-Related Definitions -----------------------------------------*/

/** @defgroup GPIO_Access_Types GPIO Access-level Types & Definitions
  * @{
  */

/** @defgroup GPIO_Directions General Purpose IO Direction Types
  * @{
  */

/*! GPIO Pin Direction configurations */
typedef enum {
    GPIO_Direction_In  = 0,            /*!< GPIO Pin configured for input    */
    GPIO_Direction_Out = 1             /*!< GPIO Pin configured for output   */
} GPIO_DirectionType;

/*! Macro to test whether parameter is a valid GPIO Pin Direction value */
#define GPIO_IS_DIRECTION_TYPE(Direction)  (((Direction) == GPIO_Direction_In) \
                                         || ((Direction) == GPIO_Direction_Out))

/** @} */

/** @defgroup GPIO_SenseConditions General Purpose IO Sense Conditions
  * @{
  */

/*! GPIO Pin Sense Condition Configurations */
typedef enum {
    GPIO_Sense_FallingEdge  = 0,      /*!< Sense GPIO Signal Falling Edge    */
    GPIO_Sense_Low          = 1,      /*!< Sense GPIO Signal Low Level       */
    GPIO_Sense_RisingEdge   = 2,      /*!< Sense GPIO Signal Rising Edge     */
    GPIO_Sense_High         = 3,      /*!< Sense GPIO Signal High Level      */
    GPIO_Sense_BothEdges    = 4       /*!< Sense GPIO Signal Both Edges      */
} GPIO_SenseType;

/*! Macro to test whether the parameter is a valid Pin Sense Configuration */
#define GPIO_IS_SENSE_TYPE(Sense)  (((Sense) == GPIO_Sense_FallingEdge) \
                                 || ((Sense) == GPIO_Sense_Level)       \
                                 || ((Sense) == GPIO_Sense_RisingEdge)  \
                                 || ((Sense) == GPIO_Sense_BothEdges))

/** @} */

/** @defgroup GPIO_Pins General Purpose IO Pins
  * @{
  */

#define GPIO_Pin_0       (1 << 0)      /*!< GPIO Pin 0                       */
#define GPIO_Pin_1       (1 << 1)      /*!< GPIO Pin 1                       */
#define GPIO_Pin_2       (1 << 2)      /*!< GPIO Pin 2                       */
#define GPIO_Pin_3       (1 << 3)      /*!< GPIO Pin 3                       */
#define GPIO_Pin_4       (1 << 4)      /*!< GPIO Pin 4                       */
#define GPIO_Pin_5       (1 << 5)      /*!< GPIO Pin 5                       */
#define GPIO_Pin_6       (1 << 6)      /*!< GPIO Pin 6                       */
#define GPIO_Pin_7       (1 << 7)      /*!< GPIO Pin 7                       */
#define GPIO_Pin_8       (1 << 8)      /*!< GPIO Pin 8                       */
#define GPIO_Pin_9       (1 << 9)      /*!< GPIO Pin 9                       */
#define GPIO_Pin_10      (1 << 10)     /*!< GPIO Pin 10                      */
#define GPIO_Pin_11      (1 << 11)     /*!< GPIO Pin 11                      */

/*! Macro to test whether the parameter is a valid GPIO Pin value */
#define GPIO_IS_PIN_TYPE(Pin)    (((Pin) == GPIO_Pin_0)  \
                               || ((Pin) == GPIO_Pin_1)  \
                               || ((Pin) == GPIO_Pin_2)  \
                               || ((Pin) == GPIO_Pin_3)  \
                               || ((Pin) == GPIO_Pin_4)  \
                               || ((Pin) == GPIO_Pin_5)  \
                               || ((Pin) == GPIO_Pin_6)  \
                               || ((Pin) == GPIO_Pin_7)  \
                               || ((Pin) == GPIO_Pin_8)  \
                               || ((Pin) == GPIO_Pin_9)  \
                               || ((Pin) == GPIO_Pin_10) \
                               || ((Pin) == GPIO_Pin_11))

/*! No GPIO Pins (NOTE: Not a valid "pin" for GPIO_IS_PIN_TYPE checking)     */
#define GPIO_Pin_NONE    (0x0000)

/*! All GPIO Pins (NOTE: Not a valid "pin" for GPIO_IS_PIN_TYPE checking)    */
#define GPIO_Pin_ALL     (0x0fff)

/*! Mask of all valid GPIO lines                                             */
#define GPIO_Pin_Mask    (0x00000fffUL)

/** @} */

/**
  * @}
  */


/* Inline Functions ---------------------------------------------------------*/

/**
  * @defgroup GPIO_Inline_Functions General Purpose IO Inline Functions
  *
  * @{
  */

/** @brief Write the given bits to the given pins
  * @param  GPIO      The GPIO port to write to
  * @param  Pins      The pins on the GPIO port to affect
  * @param  PinValues The bit values to apply to those pins
  * @return None.
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

    GPIO->SELDATA[Pins] = PinValues;
}

/** @brief Read the logic state of the given pins
  * @param  GPIO      The GPIO port to write to
  * @param  Pins      The pins on the GPIO port to affect
  * @return None.
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
__INLINE static uint16_t GPIO_GetPinDirection(GPIO_Type *GPIO, uint16_t Pin)
{
    lpclib_assert(GPIO_IS_PIN_TYPE(Pin));

    return (GPIO->DIR & Pin) ? GPIO_Direction_In:GPIO_Direction_Out;
}

/** @brief Set the interrupt level / edge sensing configuration for given pins
  * @param  GPIO   The GPIO port for which to set the sensing settings
  * @param  Pins   The pins for which to set the sensing configurations
  * @param  Sense  The sensing configuration for the pin
  * @return None.
  */
__INLINE static void GPIO_SetPinITSenseTypes(GPIO_Type *GPIO, uint16_t Pins,
                                             GPIO_SenseType Sense)
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
