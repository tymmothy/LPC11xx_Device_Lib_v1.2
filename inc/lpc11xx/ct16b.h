/**************************************************************************//**
 * @file     ct16b.h
 * @brief    16 Bit Counter/Timer Interface Header for NXP LPC Microcontrollers
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. January 2012
 ******************************************************************************
 * @section Overview
 * This file gives a basic interface to NXP LPC CT16B Counter / Timer
 * peripherals.  These peripherals can operate as timers, in various counter
 * modes, and also have built-in PWM hardware; this is in contrast to timers
 * on some older chips which have much more basic counter modes and do not have
 * PWM hardware.
 *
 * This interface offers basic things like setting the mode of the Counter /
 * timer, setting up the prescaler, getting the current count value,
 * configuring interrupts and timer match values, dealing with interrupts,
 * etc.
 *
 * @note
 * This file does not handle the following necessary steps for CT16B use:
 * - The CT16B's input clock must be enabled
 * - (if using Match or Capture pins) IO Pins must be set up externally
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

#ifndef NXP_LPC_CT16B_H_
#define NXP_LPC_CT16B_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "lpc11xx.h"
#include "lpc11xx/lib_assert.h"


/**
  * @defgroup CT16B_AbstractionLayer CT16B (16-bit Counter/Timer) Abstraction Layer
  * @ingroup  LPC_Peripheral_AbstractionLayer
  * @{
  */

/* Typedefs -----------------------------------------------------------------*/

/**
  * @defgroup CT16B_Types CT16B Interface Types and Type-Related Definitions
  * @{
  */

/** @defgroup CT16B_Interrupt_Types CT16B Interrupt Sources
  * NOTE: LPC11xx doesn't support captures 1-3.
  * @{
  */

#if ! (defined(LPC11XX) || defined(LPC11Cxx))
# define CT16B_IT_Mask                       (0xff)        /*!< All valid bits in interrupt reg  */
#else
# define CT16B_IT_Mask                       (0x1f)        /*!< All valid bits in interrupt reg  */
#endif

#define CT16B_IT_MR0                        (1 << 0)       /*!< Match on match register 0        */
#define CT16B_IT_MR1                        (1 << 1)       /*!< Match on match register 1        */
#define CT16B_IT_MR2                        (1 << 2)       /*!< Match on match register 2        */
#define CT16B_IT_MR3                        (1 << 3)       /*!< Match on match register 3        */
#define CT16B_IT_CR0                        (1 << 4)       /*!< Trigger of Capture Input 0       */

#if ! (defined(LPC11XX) || defined(LPC11Cxx))
# define CT16B_IT_CR1                       (1 << 5)       /*!< Trigger of capture input 1       */
# define CT16B_IT_CR2                       (1 << 6)       /*!< Trigger of capture input 2       */
# define CT16B_IT_CR3                       (1 << 7)       /*!< Trigger of capture input 3       */
#endif

/** @} */

/** @defgroup CT16B_Match_Control_Types CT16B Actions to Take on a Match
  * @{
  */

/* Note: These are actions per match register.  Actions can be ORed together. */
#define CT16B_MatchControl_Mask             (0x07)         /*!< All valid match Control bits     */

#define CT16B_MatchControl_None             (0x00)         /*!< Do nothing on match              */
#define CT16B_MatchControl_Interrupt        (1 << 0)       /*!< Generate IRQ on match            */
#define CT16B_MatchControl_Reset            (1 << 1)       /*!< Reset the timer/counter on match */
#define CT16B_MatchControl_Stop             (1 << 2)       /*!< Stop the timer/counter on match  */

/** @} */

/** @defgroup CT16B_Capture_Control_Types CT16B External Capture Settings
  * @{
  */

/* Note: These are settings per capture control register.  Actions can be ORed together. */
#define CT16B_CaptureControl_Mask           (0x07)         /*!< All valid capture control bits   */

#define CT16B_CaptureControl_None           (0x00)         /*!< Do not monitor capture pin       */
#define CT16B_CaptureControl_RisingEdges    (1 << 0)       /*!< Capture count on rising edges    */
#define CT16B_CaptureControl_FallingEdges   (1 << 1)       /*!< Capture count on falling edges   */
#define CT16B_CaptureControl_Interrupt      (1 << 2)       /*!< Generate an interrupt on capture */

/** @} */

/** @defgroup CT16B_Ext_Match_Control_Types CT16B External Match Output Controls
  * @{
  */

/*! @brief CT16B external match control actions; one action per external match output */
typedef enum {
    CT16B_ExtMatchControl_None = 0x00,                     /*!< No output on timer/ctr match     */
    CT16B_ExtMatchControl_Clear,                           /*!< Set output lo on timer/ctr match */
    CT16B_ExtMatchControl_Set,                             /*!< Set output hi on timer/ctr match */
    CT16B_ExtMatchControl_Toggle,                          /*!< Toggle output on timer/ctr match */
} CT16B_ExtMatchControl_Type;

/*! @brief Macro to test whether parameter is a valid counter / timer external match control setting */
#define CT16B_IS_EXT_MATCH_CONTROL(Control) (((Control) == CT16B_ExtMatchControl_None)  \
                                          || ((Control) == CT16B_ExtMatchControl_Clear) \
                                          || ((Control) == CT16B_ExtMatchControl_Set)   \
                                          || ((Control) == CT16B_ExtMatchControl_Toggle))

/** @} */

/** @defgroup CT16B_Mode_Types CT16B Mode Configuration Controls
  * @{
  */

/*! @brief CT16B peripheral mode configurations */
typedef enum {
    CT16B_Mode_Timer = 0x00,                               /*!< Timer mode                       */
    CT16B_Mode_CountRisingEdges,                           /*!< Count mode; count rising edges   */
    CT16B_Mode_CountFallingEdges,                          /*!< Count mode; count falling edges  */
    CT16B_Mode_CountAllEdges,                              /*!< Count mode; count all edges      */
} CT16B_Mode_Type;

/*! @brief Macro to test whether parameter is a valid counter / timer mode setting */
#define CT16B_IS_MODE(Mode)  (((Mode) == CT16B_Mode_Timer)             \
                           || ((Mode) == CT16B_Mode_CountRisingEdges)  \
                           || ((Mode) == CT16B_Mode_CountFallingEdges) \
                           || ((Mode) == CT16B_Mode_CountAllEdges))

/** @} */

/**
  * @}
  */

/* Inline Functions ---------------------------------------------------------*/

/**
  * @defgroup CT16B_InlineFunctions CT16B Interface Inline Functions
  *
  * @{
  */

/** @brief  Enable a timer.
  * @param  Timer       The timer
  * @return             None.
  */
__INLINE static void CT16B_Enable(CT16B_Type *Timer)
{
    Timer->TCR |= CT16B_CE;
}

/** @brief  Disable a timer.
  * @param  Timer       The timer
  * @return             None.
  */
__INLINE static void CT16B_Disable(CT16B_Type *Timer)
{
    Timer->TCR &= ~CT16B_CE;
}

/** @brief  Test whether a timer is enabled.
  * @param  Timer       The timer
  * @return             1 if the timer/counter is enabled, 0 otherwise.
  */
__INLINE static uint8_t CT16B_IsEnabled(CT16B_Type *Timer)
{
    return (Timer->TCR & CT16B_CE) ? 1:0;
}

/** @brief  Set the (timing / counting) operation mode of a timer.
  * @param  Timer       The timer
  * @param  Mode        The new timing / counting mode
  * @return             None.
  */
__INLINE static void CT16B_SetMode(CT16B_Type *Timer, CT16B_Mode_Type Mode)
{
    lpclib_assert(CT16B_IS_MODE(Mode));
    Timer->CTCR = Mode;
}

/** @brief  Get the current (timing / counting) mode of a timer.
  * @param  Timer       The timer
  * @return             The current timing / counting mode of the timer.
  */
__INLINE static CT16B_Mode_Type CT16B_GetMode(CT16B_Type *Timer)
{
    return Timer->CTCR;
}

/** @brief  Reset a timer (it will stay reset until cleared).
  * @param  Timer       The timer
  * @return             None.
  */
__INLINE static void CT16B_AssertReset(CT16B_Type *Timer)
{
    Timer->TCR |= CT16B_CR;
}

/** @brief  Clear the reset state of a timer.
  * @param  Timer       The timer
  * @return None.
  */
__INLINE static void CT16B_DeassertReset(CT16B_Type *Timer)
{
    Timer->TCR &= ~CT16B_CR;
}

/** @brief Test whether a timer is being held in reset.
  * @return             1 if the timer is being held in reset, 0 otherwise.
  */
__INLINE static uint8_t CT16B_ResetIsAsserted(CT16B_Type *Timer)
{
    return (Timer->TCR & CT16B_CR) ? 1:0;
}

/** @brief  Get a bitmask of pending interrupts for a timer.
  * @param  Timer       The timer
  * @return             A bitmask of pending interrupts.
  */
__INLINE static uint8_t CT16B_GetPendingIT(CT16B_Type *Timer)
{
    return Timer->IR;
}

/** @brief  Clear pending interrupts on a timer.
  * @param  Timer       The timer
  * @param  Interrupts  A bitmask of pending interrupts to clear
  * @return             None.
  */
__INLINE static void CT16B_ClearPendingIT(CT16B_Type *Timer, uint8_t Interrupts)
{
    lpclib_assert((Interrupts & ~CT16B_IT_Mask) == 0);
    Timer->IR = Interrupts;
}

/** @brief  Set a timer's count value.
  * @param  Timer       The timer
  * @param  Count       The new count value
  * @return             None.
  */
__INLINE static void CT16B_SetCount(CT16B_Type *Timer, uint16_t Count)
{
    Timer->TC = Count;
}

/** @brief  Get a timer's current count value.
  * @param  Timer       The timer
  * @return             The timer's current count value.
  */
__INLINE static uint16_t CT16B_GetCount(CT16B_Type *Timer)
{
    return Timer->TC;
}

/** @brief  Set the prescaler value of a timer.
  * @param  Timer       The timer
  * @param  Prescaler   The new prescaler value (0 - 65535).
  * @return             None.
  *
  * The prescaler of a timer is the # of input clocks or counts before
  * the timer increments.  Changing this changes the number of
  * counts before each increment of the timer/counter.
  *
  * A prescaler value of 0 means that the timer will increment every
  * PCLK; 1 means every 2 PCLKS and so on -- unlike many other interfaces
  * in this library, this one is 0-based.
  */
__INLINE static void CT16B_SetPrescaler(CT16B_Type *Timer, uint16_t Prescaler)
{
    Timer->PR = Prescaler;
}

/** @brief  Get a timer's current prescaler value.
  * @param  Timer       The timer
  * @return             The timer's current prescaler value
  *
  * The prescaler of a timer is the # of input clocks or counts before
  * the timer increments.
  */
__INLINE static uint16_t CT16B_GetPrescaler(CT16B_Type *Timer)
{
    return Timer->PR;
}

/** @brief  Set a timer's prescaler count value.
  * @param  Timer       The timer
  * @param  Count       The new prescaler count value
  * @return             None.
  *
  * This is the number of prescaler "ticks" thus far on the way to the
  * next count increment.
  */
__INLINE static void CT16B_SetPrescalerCount(CT16B_Type *Timer, uint16_t Count)
{
    Timer->PC = Count;
}

/** @brief Get a timer's current prescaler count value.
  * @param  Timer       The timer
  * @return             The timer's current prescaler count value.
  *
  * This is the number of prescaler "ticks" thus far on the way to the
  * next count increment.
  */
__INLINE static uint16_t CT16B_GetPrescalerCount(CT16B_Type *Timer)
{
    return Timer->PC;
}

/** @brief Set the actions that will happen on a timer's count matching a match register.
  * @param  Timer       The timer
  * @param  Channel     The timer's channel for which to configure match actions
  * @param  Control     A bitmask of actions to take on a count match
  * @return             None.
  */
__INLINE static void CT16B_SetChannelMatchControl(CT16B_Type *Timer, unsigned int Channel, uint8_t Control)
{
    lpclib_assert(Channel <= 3);
    lpclib_assert((Control & ~CT16B_MatchControl_Mask) == 0);

    Timer->MCR = (Timer->MCR & ~(0x07 << (Channel * 3))) | (Control << (Channel * 3));
}

/** @brief Set the actions that will happen on a timer's count matching a match register.
  * @param  Timer       The timer
  * @param  Channel     The timer's channel for which to get actions
  * @return             A bitmask of actions taken on a count match.
  */
__INLINE static uint8_t CT16B_GetChannelMatchControl(CT16B_Type *Timer, unsigned int Channel)
{
    lpclib_assert(Channel <= 3);

    return (Timer->MCR >> (Channel * 3)) & 0x07;
}

/** @brief  Set the match value on the specified channel of a timer.
  * @param  Timer       The timer
  * @param  Channel     The timer's channel for which to set the match value
  * @param  Count       The new match value
  * @return             None.
  */
__INLINE static void CT16B_SetChannelMatchValue(CT16B_Type *Timer, unsigned int Channel, uint16_t Count)
{
    lpclib_assert(Channel <= 3);

    ((__IO uint32_t *)&Timer->MR0)[Channel] = Count;
}

/** @brief  Get the match value on the specified match channel of a timer.
  * @param  Timer      The timer
  * @param  Channel    The timer's channel for which to set the match value
  * @return            The match value on the specified channel of the timer.
  */
__INLINE static uint16_t CT16B_GetChannelMatchValue(CT16B_Type *Timer, unsigned int Channel)
{
    lpclib_assert(Channel <= 3);

    return ((__IO uint32_t *)&Timer->MR0)[Channel];
}

/** @brief Set the config of the external match line for the specified channel of a timer. *
  * @param  Timer       The timer
  * @param  Channel     The timer's channel
  * @param  Control     The new function of the channel's external match line
  * @return             None.
  *
  * Note that regardless of whether a pin is configured for external match duty,
  * tthe external match bit reflects the state of the external match value.
  */
__INLINE static void CT16B_SetChannelExtMatchControl(CT16B_Type *Timer, unsigned int Channel, CT16B_ExtMatchControl_Type Control)
{
    lpclib_assert(Channel <= 3);
    lpclib_assert(CT16B_IS_EXT_MATCH(Control));

    Timer->EMR = (Timer->EMR & ~(0x03 << ((Channel * 2) + 4))) | (Control << ((Channel * 2) + 4));
}

/** @brief Get the current config of the external match line for the specified channel of a timer.
  * @param  Timer       The timer
  * @param  Channel     The timer's channel
  * @return             The current function of the channel's external match line.
  */
__INLINE static CT16B_ExtMatchControl_Type CT16B_GetChannelExtMatchControl(CT16B_Type *Timer, unsigned int Channel)
{
    lpclib_assert(Channel <= 3);

    return (Timer->EMR >> ((Channel * 2) + 4)) & 0x03;
}

/** @brief Set the external match bit value for the specified channel of a timer.
  * @param  Timer       The timer
  * @param  Channel     The timer's channel
  * @param  Value       Non-zero: sets the external match bit / zero: clears the bit
  * @return             None.
  */

__INLINE static void CT16B_SetChannelExtMatchBit(CT16B_Type *Timer, unsigned int Channel, unsigned int Value)
{
    lpclib_assert(Channel <= 3);

    if (Value) {
        Timer->EMR |= (1 << Channel);
    } else {
        Timer->EMR &= ~(1 << Channel);
    }
}

/** @brief Get the external match bit value for the specified channel of a timer.
  * @param  Timer       The timer
  * @param  Channel     The timer's channel
  * @return             The external match bit value for the specified timer's channel.
  */
__INLINE static uint8_t CT16B_GetChannelExtMatchBit(CT16B_Type *Timer, unsigned int Channel)
{
    lpclib_assert(Channel <= 3);

    return (Timer->EMR & (1 << Channel)) ? 1:0;
}

/** @brief  Enable PWM operation on the specified channel of a timer.
  * @param  Timer       The timer
  * @param  Channel     The timer's channel
  * @return             None.
  *
  * If enabled, a PWM channel controls the external match pin / bit; if disabled,
  *  it's under external match control.
  */
__INLINE static void CT16B_EnableChannelPWM(CT16B_Type *Timer, unsigned int Channel)
{
    lpclib_assert(Channel <= 3);

    Timer->PWMC |= (1 << Channel);
}

/** @brief  Disable PWM operation on the specified channel of a timer.
  * @param  Timer       The timer
  * @param  Channel     The timer's channel
  * @return             None.
  *
  * If enabled, a PWM channel controls the external match pin / bit; if disabled,
  *  it's under external match control.
  */
__INLINE static void CT16B_DisableChannelPWM(CT16B_Type *Timer, unsigned int Channel)
{
    lpclib_assert(Channel <= 3);

    Timer->PWMC &= ~(1 << Channel);
}

/** @brief  Test whether the specified channel on a timer is configured for PWM.
  * @param  Timer       The timer
  * @param  Channel     The channel on the timer
  * @return             1 if the channel is configured for PWM operation, 0 otherwise.
  *
  * If enabled, a PWM channel controls the external match pin / bit; if disabled,
  *  it's under external match control.
  */
__INLINE static uint8_t CT16B_ChannelPWMIsEnabled(CT16B_Type *Timer, unsigned int Channel)
{
    lpclib_assert(Channel <= 3);

    return (Timer->PWMC & (1 << Channel)) ? 1:0;
}

/** @brief  Configure a capture input channel on a timer.
  * @param  Timer      The timer
  * @param  Channel    The channel on the timer
  * @param  Control    A bitmask of capture input configuration bits.
  * @return            None.
  *
  * MUST be CT16B_CaptureControl_None if counter mode is selected on timer.
  */
__INLINE static void CT16B_SetCaptureControl(CT16B_Type *Timer, unsigned int Channel, uint8_t Control)
{
#ifdef LPC11XX
    /* LPC11xx has only one capture channel */
    lpclib_assert(Channel == 0);
#else
    lpclib_assert(Channel <= 3);
#endif

    lpclib_assert((Control & ~CT16B_CaptureControl_Mask) == 0);

    Timer->CCR = (Timer->CCR & ~(CT16B_CaptureControl_Mask << (3 * Channel)))
                  | (Control << (3 * Channel));
}

/** @brief  Get the current configuration for a timer's capture input channel.
  * @param  Timer       The timer
  * @param  Channel     The channel on the timer
  * @return             A bitmask of configuration bits for the specified channel.
  */
__INLINE static uint8_t CT16B_GetCaptureControl(CT16B_Type *Timer, unsigned int Channel)
{
#ifdef LPC11XX
    /* LPC11xx has only one capture channel */
    lpclib_assert(Channel == 0);
#else
    lpclib_assert(Channel <= 3);
#endif

    return ((Timer->CCR >> (3 * Channel)) & CT16B_CaptureControl_Mask);
}

/** @brief  Get a timer channel's captured count value.
  * @param  Timer       The timer
  * @param  Channel     The channel on the timer
  * @return             The captured count vale for the specified channel on the timer.
  *
  * NOTE: (only 1 count capture channel -- Channel 0 -- on LPC11xx)
  */
__INLINE static uint16_t CT16B_GetCaptureValue(CT16B_Type *Timer, unsigned int Channel)
{
#ifdef LPC11XX
    /* LPC11xx has only one capture channel */
    lpclib_assert(Channel == 0);
#else
    lpclib_assert(Channel <= 3);
#endif

    return ((uint16_t *)&(Timer->CR0))[Channel];
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

#endif /* #ifndef NXP_LPC_CT16B_H_ */
