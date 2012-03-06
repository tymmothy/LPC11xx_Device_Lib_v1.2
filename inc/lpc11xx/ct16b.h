/**************************************************************************//**
 * @file     ct16b.h
 * @brief    16 Bit Counter/Timer Interface Header for NXP LPC Microcontrollers
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     January 2012
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
 * - The CT16B's (AHB or APB/VPB) bus clock line must be enabled
 * - (if using Match or Capture pins) IO Pins must be set up
 * - For interrupt use, an interrupt handler must be declared and
 *   the CT16B's interrupt line must be enabled in the microcontroller's
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

#ifndef NXP_LPC_CT16B_H_
#define NXP_LPC_CT16B_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "lpc11xx.h"
#include "lpclib_assert.h"


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

#if ! (defined(LPC11XX) || defined(LPC11CXX))
# define CT16B_Interrupt_Mask                (0xff)        /*!< All valid bits in interrupt reg  */
#else
# define CT16B_Interrupt_Mask                (0x1f)        /*!< All valid bits in interrupt reg  */
#endif

#define CT16B_InterruptMask_MR0              (1 << 0)       /*!< Match on match register 0       */
#define CT16B_InterruptMask_MR1              (1 << 1)       /*!< Match on match register 1       */
#define CT16B_InterruptMask_MR2              (1 << 2)       /*!< Match on match register 2       */
#define CT16B_InterruptMask_MR3              (1 << 3)       /*!< Match on match register 3       */
#define CT16B_InterruptMask_CR0              (1 << 4)       /*!< Trigger of Capture Input 0      */

#if ! (defined(LPC11XX) || defined(LPC11CXX)) /* Not on LPC11xx/LPC11Cxx */
# define CT16B_InterruptMask_CR1             (1 << 5)       /*!< Trigger of capture input 1      */
# define CT16B_InterruptMask_CR2             (1 << 6)       /*!< Trigger of capture input 2      */
# define CT16B_InterruptMask_CR3             (1 << 7)       /*!< Trigger of capture input 3      */
#endif

/*! @brief Number of timer match channels supported */
#define CT16B_NUM_MATCH_CHANNELS            (4)

/*! @brief Number of capture channels supported */
#if ! (defined(LPC11XX) || defined(LPC11CXX))
# define CT16B_NUM_CAPTURE_CHANNELS         (4)
#else
# define CT16B_NUM_CAPTURE_CHANNELS         (1)
#endif

/** @} */

/** @defgroup CT16B_Match_Config_Types CT16B Match Action Settings
  * @{
  */

/* Note: These are actions per match channel.  Actions can be ORed together. */
#define CT16B_MatchConfigMask_Mask          (0x07)         /*!< All valid match config bits      */

#define CT16B_MatchConfigMask_None          (0)            /*!< Do nothing on match              */
#define CT16B_MatchConfigMask_Interrupt     (1 << 0)       /*!< Generate IRQ on match            */
#define CT16B_MatchConfigMask_Reset         (1 << 1)       /*!< Reset the timer/counter on match */
#define CT16B_MatchConfigMask_Stop          (1 << 2)       /*!< Stop the timer/counter on match  */

/** @} */

/** @defgroup CT16B_Capture_Config_Types CT16B External Capture Settings
  * @{
  */

/* Note: These are settings per capture control register.  Actions can be ORed together. */
#define CT16B_CaptureConfigMask_Mask         (0x07)        /*!< All valid capture config bits    */

#define CT16B_CaptureConfigMask_None         (0)           /*!< Do not monitor capture pin       */
#define CT16B_CaptureConfigMask_RisingEdges  (1 << 0)      /*!< Capture count on rising edges    */
#define CT16B_CaptureConfigMask_FallingEdges (1 << 1)      /*!< Capture count on falling edges   */
#define CT16B_CaptureConfigMask_Interrupt    (1 << 2)      /*!< Generate an interrupt on capture */

/** @} */

/** @defgroup CT16B_Ext_Match_Config_Types CT16B External Match Output Settings
  * @{
  */

/*! @brief CT16B configuration bits for external output channels on matches */
#define CT16B_ExtMatchConfigMask_Mask       (0x03)         /*!< All valid ext match config bits  */

#define CT16B_ExtMatchConfigMask_None       (0)            /*!< No output on timer/ctr match     */
#define CT16B_ExtMatchConfigMask_Clear      (1 << 0)       /*!< Set output lo on timer/ctr match */
#define CT16B_ExtMatchConfigMask_Set        (1 << 1)       /*!< Set output hi on timer/ctr match */
#define CT16B_ExtMatchConfigMask_Toggle     (0x03)         /*!< Toggle output on timer/ctr match */

/** @} */

/** @defgroup CT16B_Mode_Types CT16B Mode Configuration Controls
  * @{
  */

/*! @brief CT16B peripheral mode configurations */
typedef enum {
    CT16B_Mode_Timer = 0,                                  /*!< Timer mode                       */
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
  * @{
  */

/** @brief Enable a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  */
__INLINE static void CT16B_Enable(CT16B_Type *timer)
{
    timer->TCR |= CT16B_CE;
}

/** @brief Disable a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  */
__INLINE static void CT16B_Disable(CT16B_Type *timer)
{
    timer->TCR &= ~CT16B_CE;
}

/** @brief Test whether a CT16B counter/timer is enabled.
  * @param[in] timer        A pointer to the CT16B instance
  * @return                 1 if the timer/counter is enabled, 0 otherwise.
  */
__INLINE static unsigned int CT16B_IsEnabled(CT16B_Type *timer)
{
    return (timer->TCR & CT16B_CE) ? 1:0;
}

/** @brief Set the timing/counting mode of a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  mode         The new timing / counting mode
  */
__INLINE static void CT16B_SetMode(CT16B_Type *timer, CT16B_Mode_Type mode)
{
    lpclib_assert(CT16B_IS_MODE(mode));

    timer->CTCR = mode;
}

/** @brief Get the current timing/counting mode of a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @return                  The current timing/counting mode.
  *
  * @sa CT16B_SetMode
  */
__INLINE static CT16B_Mode_Type CT16B_GetMode(CT16B_Type *timer)
{
    return timer->CTCR;
}

/** @brief Assert reset on a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  *
  * @note
  * The counter/timer will stay in reset until cleared.
  *
  * @sa CT16B_DeassertReset
  */
__INLINE static void CT16B_AssertReset(CT16B_Type *timer)
{
    timer->TCR |= CT16B_CR;
}

/** @brief Clear the reset condition of a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  *
  * @sa CT16B_AssertReset
  */
__INLINE static void CT16B_ClearReset(CT16B_Type *timer)
{
    timer->TCR &= ~CT16B_CR;
}

/** @brief Test whether a CT16B counter/timer is being held in reset.
  * @param[in]  timer        A pointer to the CT16B instance
  * @return                  1 if the CT16B is being held in reset, 0 otherwise.
  */
__INLINE static unsigned int CT16B_ResetIsAsserted(CT16B_Type *timer)
{
    return (timer->TCR & CT16B_CR) ? 1:0;
}

/** @brief Get a bitmask of interrupts pending on a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @return                  A bitmask of pending interrupts.
  */
__INLINE static uint32_t CT16B_GetPendingInteruptMask(CT16B_Type *timer)
{
    return timer->IR;
}

/** @brief Clear pending interrupts on a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  it_mask      A bitmask of interrupts to clear
  */
__INLINE static void CT16B_ClearPendingInterruptMask(CT16B_Type *timer, uint32_t it_mask)
{
    lpclib_assert((it_mask & ~CT16B_Interrupt_Mask) == 0);
    timer->IR = it_mask;
}

/** @brief Set a CT16B counter/timer's count value.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  count        The new count value (0-65535)
  */
__INLINE static void CT16B_SetCount(CT16B_Type *timer, uint32_t count)
{
    timer->TC = count;
}

/** @brief Get a CT16B counter/timer's current count value.
  * @param[in]  timer        A pointer to the CT16B instance
  * @return                  The current count value (0-65535).
  */
__INLINE static uint32_t CT16B_GetCount(CT16B_Type *timer)
{
    return timer->TC;
}

/** @brief Set the prescaler value of a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  prescaler    The new prescaler value (0 - 65535).
  *
  * @note
  * The prescaler of a timer is the # of input clocks or counts before
  * the timer increments.  Changing this changes the number of
  * counts before each increment of the timer/counter.
  *
  * A prescaler value of 0 means that the timer will increment every
  * PCLK; 1 means every 2 PCLKS and so on -- unlike many other interfaces
  * in this library, this one is 0-based.
  */
__INLINE static void CT16B_SetPrescaler(CT16B_Type *timer, uint32_t prescaler)
{
    timer->PR = prescaler;
}

/** @brief Get a CT16B counter/timer's current prescaler value.
  * @param[in]  timer        A pointer to the CT16B instance
  * @return                  The current prescaler value (0-65535).
  *
  * @sa CT16B_SetPrescaler
  */
__INLINE static uint32_t CT16B_GetPrescaler(CT16B_Type *timer)
{
    return timer->PR;
}

/** @brief Set a CT16B counter/timer's prescaler count value.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  count        The new prescaler count value (0-65535)
  *
  * @note
  * This sets the number of prescaler "ticks" thus far on the way to the
  * next count increment.
  */
__INLINE static void CT16B_SetPrescalerCount(CT16B_Type *timer, uint32_t count)
{
    timer->PC = count;
}

/** @brief Get a CT16B counter/timer's current prescaler count value.
  * @param[in]  timer        A pointer to the CT16B instance
  * @return                  The current prescaler count value (0-65535).
  *
  * @sa CT16B_SetPrescalerCount
  */
__INLINE static uint32_t CT16B_GetPrescalerCount(CT16B_Type *timer)
{
    return timer->PC;
}

/** @brief Set the actions that will happen on a CT16B match channel being triggered.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  channel      A match channel # on the CT16B
  * @param[in]  config_mask  A bitmask of actions to take on a match
  */
__INLINE static void CT16B_SetConfigForMatchChannel(CT16B_Type *timer, unsigned int channel,
                                                    uint32_t config_mask)
{
    lpclib_assert(channel < CT16B_NUM_MATCH_CHANNELS);
    lpclib_assert((config_mask & ~CT16B_MatchConfigMask_Mask) == 0);

    timer->MCR = (timer->MCR & ~(CT16B_MatchConfigMask_Mask << (channel * 3)))
                                 | (config_mask << (channel * 3));
}

/** @brief Get a bitmask of actions that will happen on a CT16B match channel being triggered.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  channel      A match channel # on the CT16B
  * @return                  A bitmask of actions taken on a match.
  */
__INLINE static uint32_t CT16B_GetConfigForMatchChannel(CT16B_Type *timer, unsigned int channel)
{
    lpclib_assert(channel < CT16B_NUM_MATCH_CHANNELS);

    return (timer->MCR >> (channel * 3)) & CT16B_MatchConfigMask_Mask;
}

/** @brief Set the match value on the specified match channel of a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  channel      A match channel # on the CT16B
  * @param[in]  count        The new match value (0-65535)
  */
__INLINE static void CT16B_SetCountForMatchChannel(CT16B_Type *timer, unsigned int channel,
                                                   uint32_t count)
{
    lpclib_assert(channel < CT16B_NUM_MATCH_CHANNELS);

    ((__IO uint32_t *)&timer->MR0)[channel] = count;
}

/** @brief Get the match value on the specified match channel of a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  channel      A match channel # on the CT16B
  * @return                  The current match value on the channel (0-65535).
  */
__INLINE static uint32_t CT16B_GetCountForMatchChannel(CT16B_Type *timer, unsigned int channel)
{
    lpclib_assert(channel < CT16B_NUM_MATCH_CHANNELS);

    return ((__IO uint32_t *)&timer->MR0)[channel];
}

/** @brief Set the function of the external match line for a channel of a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  channel      A match channel # on the CT16B
  * @param[in]  config_mask  A bitmask of configuration bits for the external match line
  *
  * @note
  * Note that regardless of whether a pin is configured for external match duty,
  * the external match bit reflects the state of the external match value.
  */
__INLINE static void CT16B_SetConfigForExtMatchChannel(CT16B_Type *timer, unsigned int channel,
                                                       uint32_t config_mask)
{
    lpclib_assert(channel < CT16B_NUM_MATCH_CHANNELS);
    lpclib_assert((config_mask & ~CT16B_ExtMatchConfigMask_Mask) == 0);

    timer->EMR = (timer->EMR & ~(CT16B_ExtMatchConfigMask_Mask << ((channel * 2) + 4)))
                                 | (config_mask << ((channel * 2) + 4));
}

/** @brief Get the current function of the ext match line for a channel of a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  channel      A match channel # on the CT16B
  * @return                  The current function of the channel's external match line.
  */
__INLINE static uint32_t CT16B_GetConfigForExtMatchChannel(CT16B_Type *timer, unsigned int channel)
{
    lpclib_assert(channel < CT16B_NUM_MATCH_CHANNELS);

    return (timer->EMR >> ((channel * 2) + 4)) & CT16B_ExtMatchConfigMask_Mask;
}

/** @brief Set the ext match bit value for the specified channel of a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  channel      A match channel # on the CT16B
  * @param[in]  value        Non-zero: sets the external match bit, zero: clears the bit
  */

__INLINE static void CT16B_SetBitValueForExtMatchChannel(CT16B_Type *timer, unsigned int channel,
                                                         unsigned int value)
{
    lpclib_assert(channel < CT16B_NUM_MATCH_CHANNELS);

    if (value) {
        timer->EMR |= (1 << channel);
    } else {
        timer->EMR &= ~(1 << channel);
    }
}

/** @brief Get the ext match bit value for the specified channel of a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  channel      A match channel # on the CT16B
  * @return                  The external match bit value for the specified channel.
  */
__INLINE static unsigned int CT16B_GetBitValueForExtMatchChannel(CT16B_Type *timer,
                                                                 unsigned int channel)
{
    lpclib_assert(channel < CT16B_NUM_MATCH_CHANNELS);

    return (timer->EMR & (1 << channel)) ? 1:0;
}

/** @brief Enable PWM mode on a match channel of a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  channel      A match channel # on the CT16B
  *
  * @note
  * - When enabled, a match will set the PWM output high.  When the count is reset to 0,
  *   the output will go low.
  * - There can be 3 match channels per CT16B used as PWM channels; the fourth is used
  *   to set the PWM cycle length.
  */
__INLINE static void CT16B_EnablePWMForMatchChannel(CT16B_Type *timer, unsigned int channel)
{
    lpclib_assert(channel <= CT16B_NUM_MATCH_CHANNELS);

    timer->PWMC |= (1 << channel);
}

/** @brief Disable PWM mode on a match channel of a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  channel      A match channel # on the CT16B
  *
  * @sa CT16B_EnableChannelPWM
  */
__INLINE static void CT16B_DisablePWMForMatchChannel(CT16B_Type *timer, unsigned int channel)
{
    lpclib_assert(channel < CT16B_NUM_MATCH_CHANNELS);

    timer->PWMC &= ~(1 << channel);
}

/** @brief Test whether PWM mode is enabled on a match channel of a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  channel      A match channel # on the CT16B
  * @return                  1 if the channel is configured for PWM operation, 0 otherwise.
  *
  * @sa CT16B_EnableChannelPWM
  */
__INLINE static unsigned int CT16B_PWMIsEnabledForMatchChannel(CT16B_Type *timer, unsigned int channel)
{
    lpclib_assert(channel < CT16B_NUM_MATCH_CHANNELS);

    return (timer->PWMC & (1 << channel)) ? 1:0;
}

/** @brief Configure a capture channel on a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  channel      A capture channel # on the CT16B
  * @param[in]  config_mask  A bitmask of configuration bits for the specified channel
  *
  * MUST be CT16B_CaptureConfig_None if counter mode is selected on timer.
  */
__INLINE static void CT16B_SetConfigForCaptureChannel(CT16B_Type *timer, unsigned int channel,
                                                      uint32_t config_mask)
{
    lpclib_assert(channel < CT16B_NUM_CAPTURE_CHANNELS);
    lpclib_assert((config_mask & ~CT16B_CaptureConfigMask_Mask) == 0);

    timer->CCR = (timer->CCR & ~(CT16B_CaptureConfigMask_Mask << (3 * channel)))
                  | (config_mask << (3 * channel));
}

/** @brief Get the current configuration bits for a capture channel on a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  channel      A capture channel # on the CT16B
  * @return                  A bitmask of configuration bits for the specified channel.
  */
__INLINE static uint32_t CT16B_GetConfigForCaptureChannel(CT16B_Type *timer, unsigned int channel)
{
    lpclib_assert(channel < CT16B_NUM_CAPTURE_CHANNELS);

    return ((timer->CCR >> (3 * channel)) & CT16B_CaptureConfig_Mask);
}

/** @brief Get the count value on a capture channel on a CT16B counter/timer.
  * @param[in]  timer        A pointer to the CT16B instance
  * @param[in]  channel      A capture channel # on the CT16B
  * @return                  The channel's captured count value.
  */
__INLINE static uint32_t CT16B_GetCountForCaptureChannel(CT16B_Type *timer, unsigned int channel)
{
    lpclib_assert(channel < CT16B_NUM_CAPTURE_CHANNELS);

    return (&(timer->CR0))[channel];
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
