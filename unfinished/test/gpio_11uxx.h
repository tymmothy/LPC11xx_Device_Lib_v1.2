/*****************************************************************************
 * @file:    LPC11uxx_gpio.h
 * @purpose: General Purpose IO Interface Header for LPC11uxx Microcontrollers
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    1. January 2012
 *
 * This implementation uses bit representations for pin numbers which allows
 *  functions to work on multiple pins at the same time.
 *
 *****************************************************************************/

#ifndef LPC11XX_GPIO_H_
#define LPC11XX_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/

#include <stdint.h>
#include "LPC11uxx.h"
#include "LPC11xx_lib_assert.h"


/** @addtogroup GPIO
  * @{
  */

/* Types & Definitions -------------------------------------------------------*/

/** @addtogroup GPIO_Types
  * @{
  */

/** @defgroup GPIO_Directions
  * @{
  */
typedef enum {
    GPIO_Direction_In  = 0,
    GPIO_Direction_Out = 1
} GPIO_DirectionType;
#define GPIO_IS_DIRECTION_TYPE(Direction)  (((Direction) == GPIO_Direction_In) \
                                         || ((Direction) == GPIO_Direction_Out))

/**
  * @}
  */

/** @defgroup GPIO_Sense
  * @{
  */
typedef enum {
    GPIO_Sense_FallingEdge  = 0,
    GPIO_Sense_Low          = 1,
    GPIO_Sense_RisingEdge   = 2,
    GPIO_Sense_High         = 3,
    GPIO_Sense_BothEdges    = 4
} GPIO_SenseType;
#define GPIO_IS_SENSE_TYPE(Sense)  (((Sense) == GPIO_Sense_FallingEdge) \
                                 || ((Sense) == GPIO_Sense_Level)       \
                                 || ((Sense) == GPIO_Sense_RisingEdge)  \
                                 || ((Sense) == GPIO_Sense_BothEdges))
/**
  * @}
  */

/** @defgroup GPIO_Pins
  * @{
  */
#define GPIO_Pin_0       (1UL << 0)
#define GPIO_Pin_1       (1UL << 1)
#define GPIO_Pin_2       (1UL << 2)
#define GPIO_Pin_3       (1UL << 3)
#define GPIO_Pin_4       (1UL << 4)
#define GPIO_Pin_5       (1UL << 5)
#define GPIO_Pin_6       (1UL << 6)
#define GPIO_Pin_7       (1UL << 7)
#define GPIO_Pin_8       (1UL << 8)
#define GPIO_Pin_9       (1UL << 9)
#define GPIO_Pin_10      (1UL << 10)
#define GPIO_Pin_11      (1UL << 11)
#define GPIO_Pin_12      (1UL << 12)
#define GPIO_Pin_13      (1UL << 13)
#define GPIO_Pin_14      (1UL << 14)
#define GPIO_Pin_15      (1UL << 15)
#define GPIO_Pin_16      (1UL << 16)
#define GPIO_Pin_17      (1UL << 17)
#define GPIO_Pin_18      (1UL << 18)
#define GPIO_Pin_19      (1UL << 19)
#define GPIO_Pin_20      (1UL << 20)
#define GPIO_Pin_21      (1UL << 21)
#define GPIO_Pin_22      (1UL << 22)
#define GPIO_Pin_23      (1UL << 23)
#define GPIO_Pin_24      (1UL << 24)
#define GPIO_Pin_25      (1UL << 25)
#define GPIO_Pin_26      (1UL << 26)
#define GPIO_Pin_27      (1UL << 27)
#define GPIO_Pin_28      (1UL << 28)
#define GPIO_Pin_29      (1UL << 29)
#define GPIO_Pin_30      (1UL << 30)
#define GPIO_Pin_31      (1UL << 31)
#define GPIO_Pin_32      (1UL << 32)

#define GPIO_IS_PIN_TYPE(Pin)    (((Pin) & ((Pin) - 1))) == 0)

/*! No GPIO Pins (NOTE: Not a valid "pin" for GPIO_IS_PIN_TYPE checking) */
#define GPIO_Pin_NONE    (0x00000000UL)

/*! All GPIO Pins (NOTE: Not a valid "pin" for GPIO_IS_PIN_TYPE checking) */
#define GPIO_Pin_ALL     (0xffffffffUL)

/**
  * @}
  */

/**
  * @}
  */

/* Inline Functions ---------------------------------------------------------*/

/** @defgroup GPIO_Inline_Functions
  * @{
  */

/** @brief Write the given bits to the given pins
  * @param  GPIO      The GPIO port to write to
  * @param  Pins      The pins on the GPIO port to affect
  * @param  Bits      The bit values to apply to those pins
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
    lpclib_assert((Pins & ~GPIO_PIO_Mask) == 0);

    if (GPIO == GPIO0) {
        GPIO->mask
    } else if (GPIO == GPIO1) {
    }
    GPIO->MASK0 = Pins;
    GPIO->
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
    lpclib_assert((Pins & ~GPIO_PIO_Mask) == 0);

    return GPIO->SELDATA[Pins];
}

/** @brief Set the state of the given pins to High (if configured for output)
  * @param  GPIO      The GPIO port to write to
  * @param  Pins      The pins on the GPIO port to affect
  * @return None.
  */
__INLINE static void GPIO_SetPins(GPIO_Type *GPIO, uint16_t Pins)
{
    lpclib_assert((Pins & ~GPIO_PIO_Mask) == 0);

    GPIO->SELDATA[Pins] = Pins;
}

/** @brief Set the state of the given pins to Low (if configured for output)
  * @param  GPIO      The GPIO port to write to
  * @param  Pins      The pins on the GPIO port to affect
  * @return None.
  */
__INLINE static void GPIO_ClearPins(GPIO_Type *GPIO, uint16_t Pins)
{
    lpclib_assert((Pins & ~GPIO_PIO_Mask) == 0);

    GPIO->SELDATA[Pins] = 0;
}

/** @brief Invert the state of the given pins
  * @param  GPIO      The GPIO port whose pins will be toggled
  * @param  Pins      The pins on the GPIO port to affect
  * @return None.
  */
__INLINE static void GPIO_InvertPins(GPIO_Type *GPIO, uint16_t Pins)
{
    lpclib_assert((Pins & ~GPIO_PIO_Mask) == 0);

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
    lpclib_assert((Pins & ~GPIO_PIO_Mask) == 0);
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
  * @return None.
  */
__INLINE static void GPIO_SetPinITSenseTypes(GPIO_Type *GPIO, uint16_t Pins,
                                       GPIO_SenseType Sense)
{
    lpclib_assert((Pins & ~GPIO_PIO_Mask) == 0);
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
    lpclib_assert((Pin & ~GPIO_PIO_Mask) == 0);

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
    lpclib_assert((Pins & ~GPIO_PIO_Mask) == 0);

    GPIO->IE |= Pins;
}

/** @brief Disable IRQs for specified pins on a given GPIO port
  * @param  GPIO  The GPIO port for which to disable pin interrupts
  * @param  Pins  ORed values of all pins for which to disable interrupts
  * @return None.
  */
__INLINE static void GPIO_DisableIRQForPins(GPIO_Type *GPIO, uint16_t Pins)
{
    lpclib_assert((Pins & ~GPIO_PIO_Mask) == 0);

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
    lpclib_assert((Pins & ~GPIO_PIO_Mask) == 0);

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

#endif /* #ifndef LPC11XX_GPIO_H_ */
