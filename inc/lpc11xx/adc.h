/**************************************************************************//**
 * @file     adc.h
 * @brief    ADC Access Interface Header for NXP LPC Microcontrollers.
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. January 2012
 ******************************************************************************
 * @section Overview
 * This file gives a basic interface to NXP LPC Analog to Digital
 * Converters; enabling ADC channels, triggering conversions,
 * reading values from ADC channels, working with ADC interrupts, etc.
 *
 * @note
 * This file does not handle the following necessary steps for ADC use:
 * - Pins must be set up in for AD input
 * - Power to the ADC must be enabled
 * - The ADC's (AHB/APB/VPB) bus clock line must be enabled
 * - For interrupt use, an interrupt handler must be declared and
 *   the ADC's interrupt line must be enabled in the microcontroller's
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

#ifndef NXP_LPC_ADC_H_
#define NXP_LPC_ADC_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/

#include <stdint.h>
#include "lpc11xx.h"
#include "lpclib_assert.h"


/**
  * @defgroup ADC_AbstractionLayer ADC (Analog-to-Digital Converter) Abstraction Layer
  * @ingroup  LPC_Peripheral_AbstractionLayer
  * @{
  */

/* Types & Type-Related Definitions -----------------------------------------*/

/** @defgroup ADC_Types ADC Interface Types and Type-Related Definitions
  * @{
  */

/** @defgroup ADC_Channels ADC Channels
  * @{
  */

/*! @brief Type for passing around single ADC Channels */
typedef unsigned int ADC_Channel_Type;

/*! @brief Number of input channels per ADC */
#define ADC_NUM_CHANNELS               (7)                 /*!< Number of channels per ADC       */

/*! @brief Macro to test whether parameter is a valid (single) ADC Channel */
#define ADC_IS_CHANNEL(Channel)        ((Channel) <= ADC_NUM_CHANNELS)

/** @} */

/** @defgroup ADC_ChannelMasks ADC Channel Bitmasks
  * @{
  */

#define ADC_ChannelMask_Mask           (0x00ff)            /*!< Mask of all ADC Channel bits     */

#define ADC_ChannelMask_AD0            (1 << 0)            /*!< AD0 Input                        */
#define ADC_ChannelMask_AD1            (1 << 1)            /*!< AD1 Input                        */
#define ADC_ChannelMask_AD2            (1 << 2)            /*!< AD2 Input                        */
#define ADC_ChannelMask_AD3            (1 << 3)            /*!< AD3 Input                        */
#define ADC_ChannelMask_AD4            (1 << 4)            /*!< AD4 Input                        */
#define ADC_ChannelMask_AD5            (1 << 5)            /*!< AD5 Input                        */
#define ADC_ChannelMask_AD6            (1 << 6)            /*!< AD6 Input                        */
#define ADC_ChannelMask_AD7            (1 << 7)            /*!< AD7 Input                        */

/*! @brief Macro to test whether parameter is a valid ADC ChannelMask */
#define ADC_IS_CHANNELMASK(Mask)        (((Mask) & ~(ADC_ChannelMask_Mask)) == 0)

/** @brief Type for passing ADC Channel bitmasks.
  * 1 bit per channel; channels can be ORed together to specify multiple.
  * e.g.
  * ADC_ChannelMask_Type Channels = ADC_ChannelMask_AD0 | ADC_ChannelMask_AD4;
  */
typedef uint8_t ADC_ChannelMask_Type;

/** @} */

/** @defgroup ADC_Burst_Resolutions ADC Burst Resolution Settings
  * @{
  */

/*! @brief ADC Burst Resolution configuration values */
typedef enum {
    ADC_BurstResolution_10Bits = 0x00,                     /*!< 10 Bit Resolution (11 Cycles)    */
    ADC_BurstResolution_9Bits,                             /*!<  9 Bit Resolution (10 Cycles)    */
    ADC_BurstResolution_8Bits,                             /*!<  8 Bit Resolution ( 9 Cycles)    */
    ADC_BurstResolution_7Bits,                             /*!<  7 Bit Resolution ( 8 Cycles)    */
    ADC_BurstResolution_6Bits,                             /*!<  6 Bit Resolution ( 7 Cycles)    */
    ADC_BurstResolution_5Bits,                             /*!<  5 Bit Resolution ( 6 Cycles)    */
    ADC_BurstResolution_4Bits,                             /*!<  4 Bit Resolution ( 5 Cycles)    */
    ADC_BurstResolution_3Bits                              /*!<  3 Bit Resolution ( 4 Cycles)    */
} ADC_BurstResolution_Type;

/*! @brief Macro to test whether parameter is a valid ADC burst resolution setting */
#define ADC_IS_BURST_RESOLUTION(Resolution) (((Resolution) == ADC_BurstResolution_10Bits) \
                                          || ((Resolution) == ADC_BurstResolution_9Bits)  \
                                          || ((Resolution) == ADC_BurstResolution_8Bits)  \
                                          || ((Resolution) == ADC_BurstResolution_7Bits)  \
                                          || ((Resolution) == ADC_BurstResolution_6Bits)  \
                                          || ((Resolution) == ADC_BurstResolution_5Bits)  \
                                          || ((Resolution) == ADC_BurstResolution_4Bits)  \
                                          || ((Resolution) == ADC_BurstResolution_3Bits))
/** @} */

/** @defgroup ADC_StartConversion_Settings ADC Start Conversion Settings
  * @{
  */

/*! @brief ADC Start Conversion configuration values */
typedef enum {
    ADC_StartConversion_None = 0x00,          /*!< Stop ADC                                      */
    ADC_StartConversion_Now,                  /*!< Start a Conversion Now                        */
    ADC_StartConversion_PIO0_2_Rising,        /*!< Start Conversion on  Rising Edge/PIO0_2       */
    ADC_StartConversion_PIO1_5_Rising,        /*!< Start Conversion on  Rising Edge/PIO1_5       */
    ADC_StartConversion_CT32B0_MAT0_Rising,   /*!< Start Conversion on  Rising Edge/CT32B0_MAT0  */
    ADC_StartConversion_CT32B0_MAT1_Rising,   /*!< Start Conversion on  Rising Edge/CT32B0_MAT1  */
    ADC_StartConversion_CT16B0_MAT0_Rising,   /*!< Start Conversion on  Rising Edge/CT16B0_MAT0  */
    ADC_StartConversion_CT16B0_MAT1_Rising,   /*!< Start Conversion on  Rising Edge/CT16B1_MAT0  */
    ADC_StartConversion_PIO0_2_Falling = 0x0a,/*!< Start Conversion on Falling Edge/PIO0_2       */
    ADC_StartConversion_PIO1_5_Falling,       /*!< Start Conversion on Falling Edge/PIO1_5       */
    ADC_StartConversion_CT32B0_MAT0_Falling,  /*!< Start Conversion on Falling Edge/CT32B0_MAT0  */
    ADC_StartConversion_CT32B0_MAT1_Falling,  /*!< Start Conversion on Falling Edge/CT32B0_MAT1  */
    ADC_StartConversion_CT16B0_MAT0_Falling,  /*!< Start Conversion on Falling Edge/CT16B0_MAT0  */
    ADC_StartConversion_CT16B0_MAT1_Falling   /*!< Start Conversion on Falling Edge/CT16B1_MAT0  */
} ADC_StartConversion_Type;

/*! @brief Macro to test whether parameter is a valid ADC start conversion setting */
#define ADC_IS_STARTCONVERSION(Start) (((Start) == ADC_StartConversion_None)                \
                                    || ((Start) == ADC_StartConversion_Now)                 \
                                    || ((Start) == ADC_StartConversion_PIO0_2_Rising)       \
                                    || ((Start) == ADC_StartConversion_PIO0_2_Falling)      \
                                    || ((Start) == ADC_StartConversion_PIO1_5_Rising)       \
                                    || ((Start) == ADC_StartConversion_PIO1_5_Falling)      \
                                    || ((Start) == ADC_StartConversion_CT32B0_MAT0_Rising)  \
                                    || ((Start) == ADC_StartConversion_CT32B0_MAT0_Falling) \
                                    || ((Start) == ADC_StartConversion_CT32B0_MAT1_Rising)  \
                                    || ((Start) == ADC_StartConversion_CT32B0_MAT1_Falling) \
                                    || ((Start) == ADC_StartConversion_CT16B0_MAT0_Rising)  \
                                    || ((Start) == ADC_StartConversion_CT16B0_MAT0_Falling) \
                                    || ((Start) == ADC_StartConversion_CT16B0_MAT1_Rising)  \
                                    || ((Start) == ADC_StartConversion_CT16B0_MAT1_Falling))

/** @} */

/**
  * @}
  */


/* Inline Functions ---------------------------------------------------------*/


/** @defgroup ADC_InlineFunctions ADC Interface Inline Functions
  * @{
  */

/** @brief Set the bitmask of enabled ADC channels.
  * @param  ADC         The ADC peripheral
  * @param  ChannelMask A bitmask of ADC channels to enable (1 bit per channel)
  * @return             None.
  */
__INLINE static void ADC_SetEnabledChannelMask(ADC_Type *ADC, ADC_ChannelMask_Type ChannelMask)
{
    lpclib_assert(ADC_IS_CHANNELMASK(ChannelMask));

    ADC->CR = (ADC->CR & ~ADC_SEL_Mask) | (ChannelMask << ADC_SEL_Shift);
}

/** @brief Get a bitmask of the enabled ADC channels.
  * @param  ADC         The ADC peripheral
  * @return             A bitmask of enabled ADC channels (1 bit per channel)
  */
__INLINE static ADC_ChannelMask_Type ADC_GetEnabledChannelMask(ADC_Type *ADC)
{
    return (ADC->CR & ADC_SEL_Mask) >> ADC_SEL_Shift;
}

/** @brief Enable an ADC channel.
  * @param  ADC         The ADC peripheral
  * @param  Channel     The ADC channel number to enable (single channel)
  * @return             None.
  */
__INLINE static void ADC_EnableChannel(ADC_Type *ADC, ADC_Channel_Type Channel)
{
    lpclib_assert(ADC_IS_CHANNEL(Channel));

    ADC->CR |= (1 << (Channel + ADC_SEL_Shift));
}

/** @brief Disable an ADC channel.
  * @param  ADC         The ADC peripheral
  * @param  Channel     Channel # to disable (single channel)
  * @return             None.
  */
__INLINE static void ADC_DisableChannel(ADC_Type *ADC, ADC_Channel_Type Channel)
{
    lpclib_assert(ADC_IS_CHANNEL(Channel));

    ADC->CR &= ~(1 << (Channel + ADC_SEL_Shift));
}

/** @brief Test whether an ADC Channel is enabled.
  * @param  ADC         The ADC peripheral
  * @param  Channel     The channel # to test
  * @return             1 if the channel is enabled, 0 otherwise.
  *
  * @sa ADC_EnableChannel
  * @sa ADC_SetChannelMask
  */
__INLINE static unsigned int ADC_ChannelIsEnabled(ADC_Type *ADC, unsigned int Channel)
{
    lpclib_assert(Channel <= ADC_NUM_CHANNELS);

    return (ADC->CR & (1 << (Channel + ADC_SEL_Shift))) ? 1:0;
}

/** @brief Set the bitmask of enabled ADC channel interrupts.
  * @param  ADC         The ADC peripheral
  * @param  ChannelMask Bitmask of ADC inputs for which to enable interrupts (1 bit / channel)
  * @return             None.
  *
  * This uses a bitmask, _NOT_ a single channel.
  *
  * Bits 0-7 of ChannelMask indicate whether to enable interrupts for the
  * same-numbered channel.
  */
__INLINE static void ADC_SetEnabledITChannelMask(ADC_Type *ADC, ADC_ChannelMask_Type ChannelMask)
{
    lpclib_assert(ADC_IS_CHANNELMASK(ChannelMask));

    ADC->INTEN = (ADC->INTEN & ~ADC_ADINTEN_Mask) | (ChannelMask << ADC_ADINTEN_Shift);
}

/** @brief Get a bitmask indicating ADC channels with enabled interrupts.
  * @param  ADC         The ADC peripheral
  * @return             Bitmask of ADC inputs for which interrupts are enabled (1 bit / channel)
  *
  * This uses a bitmask, _NOT_ a single channel.
  *
  * Bits 0-7 indicate whether interrupts are enabled on the same-numbered channel.
  */
__INLINE static ADC_ChannelMask_Type ADC_GetEnabledITChannelMask(ADC_Type *ADC)
{
    return ADC->INTEN;
}

/** @brief Enable interrupts for the given ADC channel.
  * @param  ADC         The ADC peripheral
  * @param  Channel     The ADC channel (0 - 7)
  * @return             None.
  *
  * This enables interrupts on the given ADC Channel (single channel; _NOT_ bitmask)
  */
__INLINE static void ADC_EnableITForChannel(ADC_Type *ADC, ADC_Channel_Type Channel)
{
    lpclib_assert(ADC_IS_CHANNEL(Channel));

    ADC->INTEN |= (1 << Channel);
}

/** @brief Disable interrupts for the given ADC channel.
  * @param  ADC         The ADC peripheral
  * @param  Channel     The ADC channel (0 - 7)
  * @return             None.
  *
  * This disables interrupts on the given ADC Channel (single channel; _NOT_ bitmask)
  */
__INLINE static void ADC_DisableITForChannel(ADC_Type *ADC, ADC_Channel_Type Channel)
{
    lpclib_assert(ADC_IS_CHANNEL(Channel));

    ADC->INTEN &= ~(1 << Channel);
}

/** @brief Determine whether interrupts for a given ADC channel is enabled.
  * @param  ADC         The ADC peripheral
  * @param  Channel     The ADC channel (0 - 7)
  * @return             1 if interrupts are enabled on given channel, 0 otherwise.
  *
  * This checks interrupt enable bit on the given ADC Channel (single channel; _NOT_ bitmask)
  */
__INLINE static unsigned int ADC_ITIsEnabledForChannel(ADC_Type *ADC, ADC_Channel_Type Channel)
{
    lpclib_assert(ADC_IS_CHANNEL(Channel));

    return (ADC->INTEN & (1 << Channel)) ? 1:0;
}

/** @brief Get a bitmask indicating ADC channels with completed conversions.
  * @param  ADC         The ADC peripheral
  * @return             A bitmask of ADC inputs with completed conversions.
  *
  * This uses a bitmask, _NOT_ a single channel.
  *
  * Bits 0-7 indicate a "done" condition on the same-numbered channel.
  */
__INLINE static ADC_ChannelMask_Type ADC_GetDoneChannelMask(ADC_Type *ADC)
{
    return (ADC->STAT & ADC_STATDONE_Mask) >> ADC_STATDONE_Shift;
}

/** @brief Test whether a single ADC channel has completed its conversion.
  * @param  ADC         The ADC peripheral
  * @param  Channel     The ADC channel (0 - 7)
  * @return             1 if the channel's conversion is complete, 0 otherwise.
  */
__INLINE static unsigned int ADC_ChannelIsDone(ADC_Type *ADC, ADC_Channel_Type Channel)
{
    lpclib_assert(ADC_IS_CHANNEL(Channel));

    return (ADC->STAT & (1 << Channel)) ? 1:0;
}

/** @brief Get a bitmask indicating ADC channels with overrun conditions.
  * @param  ADC         The ADC peripheral
  * @return             A bitmask of ADC inputs with overrun conditions.
  *
  * This uses a bitmask, _NOT_ a single channel.
  *
  * Bits 0-7 indicate an "overrun" condition on the same-numbered channel.
  *
  * Overruns occur when a conversion is lost -- a new conversion is made on
  * an ADC channel before the last conversion has been read out by software.
  */
__INLINE static ADC_ChannelMask_Type ADC_GetOverrunChannelMask(ADC_Type *ADC)
{
    return (ADC->STAT & ADC_STATOVERRUN_Mask) >> ADC_STATOVERRUN_Shift;
}

/** @brief Test whether an ADC channel has an overrun condition.
  * @param  ADC         The ADC peripheral
  * @param  Channel     The ADC channel (0 - 7)
  * @return             1 if the channel has an overrun condition, 0 otherwise.
  *
  * Overruns occur when a conversion is lost -- a new conversion is made on
  * an ADC channel before the last conversion has been read out by software.
  */
__INLINE static ADC_ChannelMask_Type ADC_ChannelIsOverrun(ADC_Type *ADC, ADC_Channel_Type Channel)
{
    return (ADC->STAT & (1 << (Channel + ADC_STATOVERRUN_Shift))) ? 1:0;
}

/** @brief Read the value from a single ADC channel.
  * @param  ADC         The ADC peripheral
  * @param  Channel     The ADC channel (0 - 7)
  * @return             The conversion value from the given channel.
  *
  * This gets the value on the given ADC Channel (single channel; _NOT_ bitmask).
  *
  * Returned values are 16-bit, left-aligned (max. 10 bit resolution)
  */
__INLINE static uint16_t ADC_ReadChannel(ADC_Type *ADC, ADC_Channel_Type Channel)
{
    lpclib_assert(ADC_IS_CHANNEL(Channel));

    return (((uint32_t *)&(ADC->DR0))[Channel] & ADC_V_VREF_Mask) >> ADC_V_VREF_Shift;
}

/** @brief Enable the ADC's global done interrupt.
  * @param  ADC         The ADC peripheral
  * @return             None.
  *
  * The ADC global done interrupt is triggered when any channel completes
  * a conversion, regardless of the state of the ADC channel interrupt mask.
  */
__INLINE static void ADC_EnableGlobalDoneIT(ADC_Type *ADC)
{
    ADC->INTEN |= ADC_ADGINTEN;
}

/** @brief Disable the ADC's global done interrupt.
  * @param  ADC         The ADC peripheral
  * @return             None.
  *
  * @sa ADC_EnableGlobalDoneIT
  */
__INLINE static void ADC_DisableGlobalDoneIT(ADC_Type *ADC)
{
    ADC->INTEN &= ~ADC_ADGINTEN;
}

/** @brief Test whether the ADC's global done interrupt is enabled.
  * @param  ADC         The ADC peripheral
  * @return             1 if the global done IRQ is enabled, 0 otherwise.
  *
  * @sa ADC_EnableGlobalDoneIT
  */
__INLINE static unsigned int ADC_GlobalDoneITIsEnabled(ADC_Type *ADC)
{
    return (ADC->INTEN & ADC_ADGINTEN) ? 1:0;
}

/** @brief Read the ADC global result register.
  * @param  ADC         The ADC peripheral
  * @return             A hybrid value with the last raw ADC reading + done / overrun flags.
  */
__INLINE static uint32_t ADC_ReadGlobalResult(ADC_Type *ADC)
{
    return ADC->GDR;
}

/** @brief Test whether the ADC has pending interrupts.
  * @param  ADC         The ADC peripheral
  * @return             1 if there are interrupts pending, 0 otherwise.
  */
__INLINE static unsigned int ADC_ITIsPending(ADC_Type *ADC)
{
    return (ADC->STAT & ADC_ADINT) ? 1:0;
}

/** @brief Test whether the ADC is currently busy with a conversion.
  * @param  ADC         The ADC peripheral
  * @return             1 if the ADC is busy, 0 otherwise.
  */
__INLINE static unsigned int ADC_IsBusy(ADC_Type *ADC)
{
    return ((ADC->GDR & ADC_DONE) == 0);
}

/** @brief Set the ADC clock divisor.
  * @param  ADC         The ADC peripheral
  * @param  Divisor     The new ADC clock divisor (valid values are 1 - 256)
  * @return             None.
  *
  * The end ADC clock rate should be <= 4.5MHz for proper ADC operation, so if
  * e.g. the AHB clock is running at 48MHz the ADC clock divisor should be
  * at least 11 giving a 4.3636MHz ADC clock rate.
  */
__INLINE static void ADC_SetClockDivisor(ADC_Type *ADC, uint16_t Divisor)
{
    lpclib_assert(Divisor >= 1);
    lpclib_assert(Divisor <= 0x100);
    ADC->CR = (ADC->CR & ~ADC_CLKDIV_Mask) | ((Divisor - 1) << ADC_CLKDIV_Shift);
}

/** @brief Get the current ADC clock divisor.
  * @param  ADC         The ADC peripheral
  * @return             The Current ADC clock divisor.
  *
  * @sa ADC_SetClockDivisor
  */
__INLINE static uint16_t ADC_GetClockDivisor(ADC_Type *ADC)
{
    return ((ADC->CR & ADC_CLKDIV_Mask) >> ADC_CLKDIV_Shift) + 1;
}

/** @brief Enable burst mode on the ADC.
  * @param  ADC         The ADC peripheral
  * @return             None.
  *
  * Burst mode is the ADC's hardware-controlled scan mode.  When enabled,
  * the ADC will do repeated conversions on each of the enabled ADC input
  * channels, at the rate specified by the burst resolution.  Channels
  * are scanned from lowest numbered to highest (then starting back
  * again at lowest).
  *
  * @sa ADC_SetBurstResolution
  */
__INLINE static void ADC_EnableBurstMode(ADC_Type *ADC)
{
    ADC->CR |= ADC_BURST;
}

/** @brief Disable burst mode on the ADC.
  * @param  ADC         The ADC peripheral
  * @return             None.
  *
  * @sa ADC_EnableBurstMode
  */
__INLINE static void ADC_DisableBurstMode(ADC_Type *ADC)
{
    ADC->CR &= ~ADC_BURST;
}

/** @brief Test whether burst mode is enabled on the ADC.
  * @param  ADC         The ADC peripheral
  * @return             1 if burst mode is enabled, 0 otherwise
  *
  * @sa ADC_EnableBurstMode
  */
__INLINE static unsigned int ADC_BurstModeIsEnabled(ADC_Type *ADC)
{
    return (ADC->CR & ADC_BURST) ? 1:0;
}

/** @brief Set the ADC burst mode conversion resolution.
  * @param  ADC         The ADC peripheral
  * @param  Resolution  The new conversion resolution
  * @return             None.
  *
  * The burst resolution determines the number of valid bits in a
  * burst mode ADC conversion.  The amount of time to complete an
  * ADC conversion will vary based on the required resolution (it takes
  * 1 extra ADC clock cycle for each extra bit of resolution).
  */
__INLINE static void ADC_SetBurstResolution(ADC_Type *ADC, ADC_BurstResolution_Type Resolution)
{
    lpclib_assert(ADC_IS_BURST_RESOLUTION(Resolution));

    ADC->CR = (ADC->CR & ~ADC_CLKS_Mask) | (Resolution << ADC_CLKS_Shift);
}

/** @brief Get the current ADC burst mode conversion resolution.
  * @param  ADC         The ADC peripheral
  * @return             The current burst mode conversion resolution.
  *
  * @sa ADC_SetBurstResolution
  */
__INLINE static ADC_BurstResolution_Type ADC_GetBurstResolution(ADC_Type *ADC)
{
    return ((ADC->CR & ADC_CLKS_Mask) >> ADC_CLKS_Shift);
}

/** @brief Set the ADC conversion trigger source.
  * @param  ADC         The ADC peripheral
  * @param  Start       The new conversion trigger
  * @return             None.
  *
  * Sets the event that will trigger an ADC conversion.  The events can
  * be ADC_StartConversion_None (which will stop conversions),
  * ADC_StartConversion_Now (which will trigger an immediate conversion),
  * or one of several IO / timer based events.
  */
__INLINE static void ADC_SetStartConversionTrigger(ADC_Type *ADC, ADC_StartConversion_Type Start)
{
    lpclib_assert(ADC_IS_STARTCONVERSION(Start));
    /* Note: EDGE is the bit above the START bits which is why this works */
    ADC->CR = (ADC->CR & ~(ADC_START_Mask | ADC_EDGE)) | (Start << ADC_START_Shift);
}

/** @brief Get the current ADC conversion trigger source.
  * @param  ADC         The ADC peripheral
  * @return             The current ADC start mode.
  *
  * @sa ADC_SetStartConversionMode
  */
__INLINE static ADC_StartConversion_Type ADC_GetStartConversionTrigger(ADC_Type *ADC)
{
    return ((ADC->CR & (ADC_START_Mask | ADC_EDGE)) >> ADC_START_Shift);
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

#endif /* #ifndef NXP_LPC_ADC_H_ */
