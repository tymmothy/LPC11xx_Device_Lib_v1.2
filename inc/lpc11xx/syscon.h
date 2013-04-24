/**************************************************************************//**
 * @file     syscon.h
 * @brief    System Control Block Interface Header for LPC11xx Microcontrollers
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. June 2010
 ******************************************************************************
 * @section Overview
 * This file gives a basic interface to NXP LPC11xx microcontroller
 * system configuration blocks.  It abstracts the interfaces to:
 *  - Memory remapping
 *  - Peripheral resetting
 *  - System, WDT, peripheral, IRC, PLL, AHB clocks
 *  - Brownout detector
 *  - System start logic
 *  - System power down / wake settings
 *  - Reading LPC11xx device ID's
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

#ifndef LPC_SYSCON_H_
#define LPC_SYSCON_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/

#include <stdint.h>
#include "lpc11xx.h"
#include "lpclib_assert.h"


/**
  * @defgroup SYSCON_AbstractionLayer SYSCON (System Control Block) Abstraction Layer
  * @ingroup  LPC_Peripheral_AbstractionLayer
  * @{
  */

/* Types & Type-Related Definitions -----------------------------------------*/

/** @defgroup SYSCON_Types SYSCON Interface Types and Type-Related Definitions
  * @{
  */

/** @defgroup SYSCON_RemapMem SYSCON first meory page / ISR vector remapping settings.
  * @{
  */

/*! @brief First page / ISR vector memory remapping options */
typedef enum {
    SYSCON_RemapMem_Bootloader = 0x00,                     /*!< ISR vector mapped from bootldr.  */
    SYSCON_RemapMem_RAM,                                   /*!< ISR vector mapped from RAM       */
    SYSCON_RemapMem_FLASH,                                 /*!< ISR vector mapped from Flash     */
} SYSCON_RemapMem_Type;

/*! @brief Macro to test whether parameter is a valid memory remapping setting */
#define SYSCON_IS_REMAP_MEM(RemapMem) (((RemapMem) == SYSCON_RemapMem_Bootloader)  \
                                    || ((RemapMem) == SYSCON_RemapMem_RAM)         \
                                    || ((RemapMem) == SYSCON_RemapMem_FLASH))

/** @} */

/** @defgroup SYSCON_PeripheralResets SYSCON Peripheral Reset Control Bits (active low)
  * @{
  */

#define SYSCON_PeripheralReset_SSP0       (1 << 0)         /*!< Reset control for SSP0 device    */
#define SYSCON_PeripheralReset_I2C0       (1 << 1)         /*!< Reset control for I2C0 device    */
#define SYSCON_PeripheralReset_SSP1       (1 << 2)         /*!< Reset control for SSP1 device    */

#if defined(LPC11CXX)  /*! CAN parts only */
# define SYSCON_PeripheralReset_CAN0      (1 << 3)         /*!< Reset control for CAN0 device    */
#endif

/** @} */

/** @defgroup SYSCON_SysOscFreqRanges SYSCON System Oscillator Frequency Range
  * @{
  */

/*! @brief System oscillator frequency range settings for external crystals */
typedef enum {
    SYSCON_SysOscFreqRange_1_20 = 0,                       /*!< Setting for  1-20 MHz ext. xtal  */
    SYSCON_SysOscFreqRange_15_25                           /*!< Setting for 15-25 MHz ext. xtal  */
} SYSCON_SysOscFreqRange_Type;

/*! @brief Macro to test whether parameter is a valid system oscillator frequency range value */
#define SYSCON_IS_SYSOSC_FREQ_RANGE(Range) (((Range) == SYSCON_SysOscFreqRange_1_20) \
                                         || ((Range) == SYSCON_SysOscFreqRange_15_25))

/** @} */

/** @defgroup SYSCON_WDTOscFreqs SYSCON Watchdog Timer Oscillator Frequency Settings
  * @{
  */

/*! @brief Watchdog frequency values */
typedef enum {
    SYSCON_WDTOscFreq_0_5_Mhz = 0x01,                      /*!< Run WDT oscillator @ 0.5 Mhz     */
    SYSCON_WDTOscFreq_0_8_Mhz = 0x02,                      /*!< Run WDT oscillator @ 0.8 Mhz     */
    SYSCON_WDTOscFreq_1_1_Mhz = 0x03,                      /*!< Run WDT oscillator @ 1.1 Mhz     */
    SYSCON_WDTOscFreq_1_4_Mhz = 0x04,                      /*!< Run WDT oscillator @ 1.4 Mhz     */
    SYSCON_WDTOscFreq_1_6_Mhz = 0x05,                      /*!< Run WDT oscillator @ 1.6 Mhz     */
    SYSCON_WDTOscFreq_1_8_Mhz = 0x06,                      /*!< Run WDT oscillator @ 1.8 Mhz     */
    SYSCON_WDTOscFreq_2_0_Mhz = 0x07,                      /*!< Run WDT oscillator @ 2.0 Mhz     */
    SYSCON_WDTOscFreq_2_2_Mhz = 0x08,                      /*!< Run WDT oscillator @ 2.2 Mhz     */
    SYSCON_WDTOscFreq_2_4_Mhz = 0x09,                      /*!< Run WDT oscillator @ 2.4 Mhz     */
    SYSCON_WDTOscFreq_2_6_Mhz = 0x0a,                      /*!< Run WDT oscillator @ 2.6 Mhz     */
    SYSCON_WDTOscFreq_2_7_Mhz = 0x0b,                      /*!< Run WDT oscillator @ 2.7 Mhz     */
    SYSCON_WDTOscFreq_2_9_Mhz = 0x0c,                      /*!< Run WDT oscillator @ 2.9 Mhz     */
    SYSCON_WDTOscFreq_3_1_Mhz = 0x0d,                      /*!< Run WDT oscillator @ 3.1 Mhz     */
    SYSCON_WDTOscFreq_3_2_Mhz = 0x0e,                      /*!< Run WDT oscillator @ 3.2 Mhz     */
    SYSCON_WDTOscFreq_3_4_Mhz = 0x0f                       /*!< Run WDT oscillator @ 3.4 Mhz     */
} SYSCON_WDTOscFreq_Type;

/*! @brief Macro to test whether parameter is a valid watchdog frequency value */
#define SYSCON_IS_WDT_OSC_FREQ(Freq) (((Freq) == SYSCON_WDTOscFreq_0_5_Mhz) \
                                   || ((Freq) == SYSCON_WDTOscFreq_0_8_Mhz) \
                                   || ((Freq) == SYSCON_WDTOscFreq_1_1_Mhz) \
                                   || ((Freq) == SYSCON_WDTOscFreq_1_4_Mhz) \
                                   || ((Freq) == SYSCON_WDTOscFreq_1_6_Mhz) \
                                   || ((Freq) == SYSCON_WDTOscFreq_1_8_Mhz) \
                                   || ((Freq) == SYSCON_WDTOscFreq_2_0_Mhz) \
                                   || ((Freq) == SYSCON_WDTOscFreq_2_2_Mhz) \
                                   || ((Freq) == SYSCON_WDTOscFreq_2_4_Mhz) \
                                   || ((Freq) == SYSCON_WDTOscFreq_2_6_Mhz) \
                                   || ((Freq) == SYSCON_WDTOscFreq_2_7_Mhz) \
                                   || ((Freq) == SYSCON_WDTOscFreq_2_9_Mhz) \
                                   || ((Freq) == SYSCON_WDTOscFreq_3_1_Mhz) \
                                   || ((Freq) == SYSCON_WDTOscFreq_3_2_Mhz) \
                                   || ((Freq) == SYSCON_WDTOscFreq_3_4_Mhz))

/** @} */

/** @defgroup SYSCON_ResetSources SYSCON Sytem Reset Source Flags
  * @{
  */

#define SYSCON_ResetSource_PowerOn        (1 << 0)         /*!< System reset from power on reset */
#define SYSCON_ResetSource_External       (1 << 1)         /*!< System reset from external reset */
#define SYSCON_ResetSource_Watchdog       (1 << 2)         /*!< System reset from watchdog timer */
#define SYSCON_ResetSource_Brownout       (1 << 3)         /*!< System reset from brownout       */
#define SYSCON_ResetSource_SysReset       (1 << 4)         /*!< System reset from NVIC reset     */

/** @} */

/** @defgroup SYSCON_AHBClockLines AHB Clock Control Lines
  * @{
  */

#define SYSCON_AHBClockLine_SYS         (1UL << 0)         /*!< AHB clock line to system block   */
#define SYSCON_AHBClockLine_ROM         (1UL << 1)         /*!< AHB clock line to ROM            */
#define SYSCON_AHBClockLine_RAM         (1UL << 2)         /*!< AHB clock line to RAM            */
#define SYSCON_AHBClockLine_FLASHCTRL   (1UL << 3)         /*!< AHB clock line to Flash control  */
#define SYSCON_AHBClockLine_FLASH       (1UL << 4)         /*!< AHB clock line to Flash memory   */
#define SYSCON_AHBClockLine_I2C0        (1UL << 5)         /*!< AHB clock line to I2C0           */
#define SYSCON_AHBClockLine_GPIO        (1UL << 6)         /*!< AHB clock line to GPIO           */
#define SYSCON_AHBClockLine_CT16B0      (1UL << 7)         /*!< AHB clock line to 16B timer 0    */
#define SYSCON_AHBClockLine_CT16B1      (1UL << 8)         /*!< AHB clock line to 16B timer 1    */
#define SYSCON_AHBClockLine_CT32B0      (1UL << 9)         /*!< AHB clock line to 32B timer 0    */
#define SYSCON_AHBClockLine_CT32B1      (1UL << 10)        /*!< AHB clock line to 32B timer 1    */
#define SYSCON_AHBClockLine_SSP0        (1UL << 11)        /*!< AHB clock line to SSP0           */
#define SYSCON_AHBClockLine_UART0       (1UL << 12)        /*!< AHB clock line to UART0          */
#define SYSCON_AHBClockLine_ADC0        (1UL << 13)        /*!< AHB clock line to ADC0           */
#define SYSCON_AHBClockLine_WDT         (1UL << 15)        /*!< AHB clock line to watchdog timer */
#define SYSCON_AHBClockLine_IOCON       (1UL << 16)        /*!< AHB clock line to IO config      */
#define SYSCON_AHBClockLine_SSP1        (1UL << 18)        /*!< AHB clock line to SSP1           */

#if defined(LPC11CXX)  /* CAN parts */
# define SYSCON_AHBClockLine_CAN0       (1UL << 17)        /*!< AHB clock line to CAN0           */
#endif

/** @} */

/** @defgroup SYSCON_SysPLLPVals System PLL "P" Values (divisor values)
  * @{
  */

/*! @brief System PLL P (divisor) value settings -- divisor is 2 times the set PVal value */
typedef enum {
    SYSCON_SysPLLPVal_1 = 0x00,                            /*!< Divide by 2                      */
    SYSCON_SysPLLPVal_2 = 0x01,                            /*!< Divide by 4                      */
    SYSCON_SysPLLPVal_4 = 0x02,                            /*!< Divide by 8                      */
    SYSCON_SysPLLPVal_8 = 0x03                             /*!< Divide by 16                     */
} SYSCON_SysPLLPVal_Type;

/*! @brief Macro to test whether parameter is a valid system oscillator frequency range value */
#define SYSCON_IS_SYSPLL_PVAL_TYPE(PVal) (((PVal) == SYSCON_SysPLLPVal_1) \
                                       || ((PVal) == SYSCON_SysPLLPVal_2) \
                                       || ((PVal) == SYSCON_SysPLLPVal_4) \
                                       || ((PVal) == SYSCON_SysPLLPVal_8))

/** @} */

/** @defgroup SYSCON_SysPLLClockSources System PLL Clock Source Values
  * @{
  */

/*! @brief System PLL clock sources */
typedef enum {
    SYSCON_SysPLLClockSource_IRC = 0x00,                   /*!< PLL source: internal RC osc.     */
    SYSCON_SysPLLClockSource_SysOsc                        /*!< PLL source: system oscillator    */
} SYSCON_SysPLLClockSource_Type;

/*! @brief Macro to test whether parameter is a valid PLL clock source value */
#define SYSCON_IS_SYSPLL_CLOCK_SOURCE(Source) (((Source) == SYSCON_SysPLLClockSource_IRC) \
                                            || ((Source) == SYSCON_SysPLLClockSource_SysOsc))

/** @} */

/** @defgroup SYSCON_MainClockSources System Main (CPU) Clock Source Values
  * @{
  */

/*! @brief Main CPU Clock Sources */
typedef enum {
    SYSCON_MainClockSource_IRC = 0x00,                     /*!< Main clock source: IRC osc       */
    SYSCON_MainClockSource_SysPLLIn,                       /*!< Main clock source: Sys PLL in    */
    SYSCON_MainClockSource_WDTOsc,                         /*!< Main clock source: WDT osc       */
    SYSCON_MainClockSource_SysPLLOut                       /*!< Main clock source: Sys PLL out   */
} SYSCON_MainClockSource_Type;

/*! @brief Macro to test whether parameter is a valid main clock source value */
#define SYSCON_IS_MAIN_CLOCK_SOURCE(Source) (((Source) == SYSCON_MainClockSource_IRC) \
                                     || ((Source) == SYSCON_MainClockSource_SysPLLIn) \
                                     || ((Source) == SYSCON_MainClockSource_WDTOsc)   \
                                     || ((Source) == SYSCON_MainClockSource_SysPLLOut))

/** @} */

/** @defgroup SYSCON_WDTClockSources Watchdog Clock Source Values
  * @{
  */

/*! @brief Watchdog timer clock sources */
typedef enum {
    SYSCON_WDTClockSource_IRC = 0x00,                      /*!< WDT clock source: IRC oscillator */
    SYSCON_WDTClockSource_MainClock,                       /*!< WDT clock source: Main clock     */
    SYSCON_WDTClockSource_WDTOsc                           /*!< WDT clock source: WDT oscillator */
} SYSCON_WDTClockSource_Type;

/*! @brief Macro to test whether parameter is a valid Watchdog Timer Clock Source value */
#define SYSCON_IS_WDT_CLOCK_SOURCE(Source) (((Source) == SYSCON_WDTClockSource_IRC)       \
                                         || ((Source) == SYSCON_WDTClockSource_MainClock) \
                                         || ((Source) == SYSCON_WDTClockSource_WDTOsc))

/** @} */

/** @defgroup SYSCON_CLKOUTSources CLKOUT (clock output pin) Clock Source Values
  * @{
  */

/*! @brief CLKOUT clock sources */
typedef enum {
    SYSCON_CLKOUTSource_IRC = 0x00,                        /*!< CLKOUT Clock source: IRC osc     */
    SYSCON_CLKOUTSource_SYSOsc,                            /*!< CLKOUT clock source: System osc. */
    SYSCON_CLKOUTSource_WDTOsc,                            /*!< CLKOUT clock source: WDT osc     */
    SYSCON_CLKOUTSource_MainClock                          /*!< CLKOUT clock source: Main clock  */
} SYSCON_CLKOUTSource_Type;

/*! @brief Macro to test whether parameter is a valid CLKOUT clock source value */
#define SYSCON_IS_CLKOUT_SOURCE(Source) (((Source) == SYSCON_CLKOUTSource_IRC)       \
                                       || ((Source) == SYSCON_CLKOUTSource_SYSOsc)   \
                                       || ((Source) == SYSCON_CLKOUTSource_WDTOsc)   \
                                       || ((Source) == SYSCON_CLKOUTSource_MainClock))

/** @} */

/** @defgroup SYSCON_BODResetVoltages Brownout Detector Reset Voltage Levels
  * @{
  */

/*! @brief Brownout detector reset voltage levels */
typedef enum {
    SYSCON_BODResetVoltage_1V46 = 0x00,                    /*!< BOD resets MCU at 1.46v          */
    SYSCON_BODResetVoltage_2V06,                           /*!< BOD resets MCU at 2.06v          */
    SYSCON_BODResetVoltage_2V35,                           /*!< BOD resets MCU at 2.35v          */
    SYSCON_BODResetVoltage_2V80,                           /*!< BOD resets MCU at 2.8v           */
} SYSCON_BODResetVoltage_Type;

/*! @brief Macro to test whether parameter is a valid brownout detector reset voltage value */
#define SYSCON_IS_BODRESETVOLTAGE(Voltage) (((Voltage) == SYSCON_BODResetVoltage_1V46) \
                                         || ((Voltage) == SYSCON_BODResetVoltage_2V06) \
                                         || ((Voltage) == SYSCON_BODResetVoltage_2V35) \
                                         || ((Voltage) == SYSCON_BODResetVoltage_2V80))

/** @} */

/** @defgroup SYSCON_BODInterruptVoltages Brownout Detector Interrupt Voltage Levels
  * @{
  */

/*! @brief Brownout detector interrupt voltage levels */
typedef enum {
    SYSCON_BODInterruptVoltage_1V65 = 0x00,                /*!< BOD IRQ triggered at 1.65v       */
    SYSCON_BODInterruptVoltage_2V22 = 0x04,                /*!< BOD IRQ triggered at 2.22v       */
    SYSCON_BODInterruptVoltage_2V52 = 0x08,                /*!< BOD IRQ triggered at 2.52v       */
    SYSCON_BODInterruptVoltage_2V80 = 0x0c                 /*!< BOD IRQ triggered at 2.8v        */
} SYSCON_BODInterruptVoltage_Type;

/*! @brief Macro to test whether parameter is a valid brownout detector interrupt voltage value */
#define SYSCON_IS_BODINTERRUPTVOLTAGE(Voltage) (((Voltage) == SYSCON_BODInterruptVoltage_1V65) \
                                             || ((Voltage) == SYSCON_BODInterruptVoltage_2V22) \
                                             || ((Voltage) == SYSCON_BODInterruptVoltage_2V52) \
                                             || ((Voltage) == SYSCON_BODInterruptVoltage_2V80))

/** @} */

/** @defgroup SYSCON_WakeupEdges Wake-up (Start Logic) Edge Triggers
  * @{
  */

/*! @brief Wake-up trigger edge settings */
typedef enum {
    SYSCON_WakeupEdge_Falling = 0x00,                  /*!< Wake-up triggers on falling edge */
    SYSCON_WakeupEdge_Rising                           /*!< Wake-up triggers on rising edge  */
} SYSCON_WakeupEdge_Type;

/*! @brief Macro to test whether parameter is a valid wake-up edge value */
#define SYSCON_IS_WAKEUPEDGE(Edge) (((Edge) == SYSCON_WakeupEdge_Falling) \
                                       || ((Edge) == SYSCON_WakeupEdge_Rising))

/** @} */

/** @defgroup SYSCON_WakeupInputs Start Logic Input Pins
  * @{
  */

#define SYSCON_WakeupInput_Mask        (0x1fff)            /*!< Bitmask of all wakeup inputs     */

#define SYSCON_WakeupInput_0           (1 << 0)            /*!< Wakeup input 0  (PIO0_0)         */
#define SYSCON_WakeupInput_1           (1 << 1)            /*!< Wakeup input 1  (PIO0_1)         */
#define SYSCON_WakeupInput_2           (1 << 2)            /*!< Wakeup input 2  (PIO0_2)         */
#define SYSCON_WakeupInput_3           (1 << 3)            /*!< Wakeup input 3  (PIO0_3)         */
#define SYSCON_WakeupInput_4           (1 << 4)            /*!< Wakeup input 4  (PIO0_4)         */
#define SYSCON_WakeupInput_5           (1 << 5)            /*!< Wakeup input 5  (PIO0_5)         */
#define SYSCON_WakeupInput_6           (1 << 6)            /*!< Wakeup input 6  (PIO0_6)         */
#define SYSCON_WakeupInput_7           (1 << 7)            /*!< Wakeup input 7  (PIO0_7)         */
#define SYSCON_WakeupInput_8           (1 << 8)            /*!< Wakeup input 8  (PIO0_8)         */
#define SYSCON_WakeupInput_9           (1 << 9)            /*!< Wakeup input 9  (PIO0_9)         */
#define SYSCON_WakeupInput_10          (1 << 10)           /*!< Wakeup input 10 (PIO0_10)        */
#define SYSCON_WakeupInput_11          (1 << 11)           /*!< Wakeup input 11 (PIO0_11)        */
#define SYSCON_WakeupInput_12          (1 << 12)           /*!< Wakeup input 12 (PIO1_0)         */

/** @} */

/** @defgroup SYSCON_AnalogPowerLines System Analog Power Line Control Bits
  * @{
  */

#define SYSCON_AnalogPowerLine_IRCOut  (1 << 0)            /*!< Power for IRC osc output         */
#define SYSCON_AnalogPowerLine_IRC     (1 << 1)            /*!< Power for internal RC osc        */
#define SYSCON_AnalogPowerLine_FLASH   (1 << 2)            /*!< Power for FLASH                  */
#define SYSCON_AnalogPowerLine_BOD     (1 << 3)            /*!< Power for brownout detector      */
#define SYSCON_AnalogPowerLine_ADC0    (1 << 4)            /*!< Power for ADC 0                  */
#define SYSCON_AnalogPowerLine_SysOsc  (1 << 5)            /*!< Power for system oscillator      */
#define SYSCON_AnalogPowerLine_WDTOsc  (1 << 6)            /*!< Power for WDT oscillator         */
#define SYSCON_AnalogPowerLine_SysPLL  (1 << 7)            /*!< Power for system PLL             */

/** @} */

/** @defgroup SYSCON_PowerModes System Power Modes
  * @{
  */

/*! @brief System Power Modes */
typedef enum {
    SYSCON_PowerMode_Sleep,            /*!< Sleep mode                       */
    SYSCON_PowerMode_Awake,            /*!< Awake from sleep mode            */
    SYSCON_PowerMode_Run,              /*!< Run mode                         */
} SYSCON_PowerMode_Type;

/*! @brief Macro to test whether parameter is a valid system power mode value */
#define SYSCON_IS_POWERMODE(Mode) (((Mode) == SYSCON_PowerMode_Sleep) \
                                || ((Mode) == SYSCON_PowerMode_Awake) \
                                || ((Mode) == SYSCON_PowerMode_Run))

/** @} */

/**
  * @}
  */


/* Inline Functions ---------------------------------------------------------*/

/**
  * @defgroup SYSCON_InlineFunctions SYSCON Interface Inline Functions
  * @{
  */

/** @brief Map the ISR vector to the specified memory area.
  * @param[in]  remap        The area to which to map the ISR vector
  *
  * This is used to change memory area where the interrupt vectors are found.
  */
__INLINE static void SYSCON_SetMemRemap(SYSCON_RemapMem_Type remap)
{
    lpclib_assert(SYSCON_IS_REMAP_MEM(remap));

    SYSCON->SYSMEMREMAP = remap;
}

/** @brief Return the area of memory to which the ISR vector is mapped.
  * @return                  The current area to which the ISR vector is mapped.
  */
__INLINE static SYSCON_RemapMem_Type SYSCON_GetMemRemap(void)
{
    return SYSCON->SYSMEMREMAP;
}

/** @brief Assert the reset lines of the specified peripherals.
  * @param[in]  reset_mask   A bitmask of the peripherals for which to assert reset.
  */
__INLINE static void SYSCON_AssertPeripheralResets(unsigned int reset_mask)
{
    lpclib_assert((reset_mask & ~SYSCON_PRESETCTRL_RESET_Mask) == 0);

    /* Note: Reset lines are active low */
    SYSCON->PRESETCTRL &= ~reset_mask;
}

/** @brief De-Assert the reset lines of the specified peripherals.
  * @param[in]  reset_mask   A bitmask of the peripherals for which to de-assert reset.
  */
__INLINE static void SYSCON_DeassertPeripheralResets(unsigned int reset_mask)
{
    lpclib_assert((reset_mask & ~SYSCON_PRESETCTRL_RESET_Mask) == 0);

    /* Note: Reset lines are active low */
    SYSCON->PRESETCTRL |= reset_mask;
}

/** @brief Get a bitmask of peripherals for which reset is currently asserted.
  * @return                  A bitmask of the peripherals for which reset is asserted.
  */
__INLINE static unsigned int SYSCON_GetAssertedPeripheralResets(void)
{
    return (~(SYSCON->PRESETCTRL)) & SYSCON_PRESETCTRL_RESET_Mask;
}

/** @brief Test whether the system PLL is locked.
  * @return                  1 if the system PLL is locked, 0 otherwise.
  */
__INLINE static unsigned int SYSCON_SysPLLIsLocked(void)
{
    return (SYSCON->SYSPLLSTAT & SYSCON_SYSPLLSTAT_LOCKED) ? 1:0;
}

/** @brief Enable bypass of the system oscillator.
  */
__INLINE static void SYSCON_EnableSysOscBypass(void)
{
    SYSCON->SYSOSCCTRL |= SYSCON_SYSOSCCTRL_BYPASS;
}

/** @brief Disable bypass of the system oscillator.
  */
__INLINE static void SYSCON_DisableSysOscBypass(void)
{
    SYSCON->SYSOSCCTRL &= ~SYSCON_SYSOSCCTRL_BYPASS;
}

/** @brief Test whether the system oscillator is currently bypassed.
  * @return                  1 if the system oscillator is bypassed, 0 otherwise.
  */
__INLINE static unsigned int SYSCON_SysOscBypassIsEnabled(void)
{
    return (SYSCON->SYSOSCCTRL & SYSCON_SYSOSCCTRL_BYPASS) ? 1:0;
}

/** @brief Set the frequency range of the crystal feeding the system oscillator.
  * @param[in]  range        The new frequency range of the xtal.
  */
__INLINE static void SYSCON_SetSysOscFreqRange(SYSCON_SysOscFreqRange_Type range)
{
    lpclib_assert(SYSCON_IS_SYSOSC_FREQ_RANGE(range));

    SYSCON->SYSOSCCTRL = (SYSCON->SYSOSCCTRL & ~SYSCON_SYSOSCCTRL_FREQRANGE) | range;
}

/** @brief Get the current (configured) frequency range of the crystal feeding the system oscillator.
  * @return                  The frequency range of the crystal.
  */
__INLINE static SYSCON_SysOscFreqRange_Type SYSCON_GetSysOscFreqRange(void)
{
    return SYSCON->SYSOSCCTRL & SYSCON_SYSOSCCTRL_FREQRANGE;
}

/** @brief Set the watchdog timer oscillator divider.
  * @param[in]  divider      The new divider value (an even number between 2 & 64)
  */
__INLINE static void SYSCON_SetWDTOscDivider(unsigned int divider)
{
    lpclib_assert((divider <= 64) && (divider != 0));
    lpclib_assert((divider & 1) == 0);

    SYSCON->WDTOSCCTRL = (SYSCON->WDTOSCCTRL & ~SYSCON_WDTOSCCTRL_DIV_Mask)
                          | ((divider >> 1) - 1);
}

/** @brief Get the current divider of the watchdog timer oscillator.
  * @return                  The current watchdog timer oscillator divider (even #; 2 <= n <= 64).
  */
__INLINE static unsigned int SYSCON_GetWDTOscDivider(void)
{
    return ((SYSCON->WDTOSCCTRL & SYSCON_WDTOSCCTRL_DIV_Mask) + 1) << 1;
}

/** @brief Set the frequency of the watchdog timer oscillator.
  * @param[in]  freq         The new watchdog timer oscillator frequency setting
  */
__INLINE static void SYSCON_SetWDTOscFreq(SYSCON_WDTOscFreq_Type freq)
{
    lpclib_assert(SYSCON_IS_WDT_OSC_FREQ(freq));

    SYSCON->WDTOSCCTRL = (SYSCON->WDTOSCCTRL & ~SYSCON_WDTOSCCTRL_FREQSEL_Mask)
                         | (freq << SYSCON_WDTOSCCTRL_FREQSEL_Shift);
}

/** @brief Get the current frequency setting of the watchdog timer oscillator.
  * @return                  The currently watchdog timer oscillator frequency setting.
  */
__INLINE static SYSCON_WDTOscFreq_Type SYSCON_GetWDTOscFreq(void)
{
    return (SYSCON_WDTOscFreq_Type)((SYSCON->WDTOSCCTRL & SYSCON_WDTOSCCTRL_FREQSEL_Mask)
            >> SYSCON_WDTOSCCTRL_FREQSEL_Shift);
}

/** @brief Set the internal RC oscillator trim value.
  * @param[in]  trim         The new RC oscillator trim value (0 - 255)
  *
  * @note
  * This does not normally need to be adjusted; it's set by the system boot ROM.
  */
__INLINE static void SYSCON_SetIRCTrim(unsigned int trim)
{
    lpclib_assert(trim <= 255);

    SYSCON->IRCCTRL = trim;
}

/** @brief Get the current internal RC oscillator trim value.
  * @return                  The current RC oscillator trim value (0-255).
  */
__INLINE static unsigned int SYSCON_GetIRCTrim(void)
{
    return SYSCON->IRCCTRL;
}

/** @brief Get a bitmask of sources that triggered the previous system reset(s).
  * @return                  A bitmask of the previous reset sources.
  *
  * @note
  * Sources are sticky & must be cleared by the application each reset or they
  * will stay set through the next reset.
  */
__INLINE static uint32_t SYSCON_GetResetSources(void)
{
    return SYSCON->SYSRESSTAT;
}

/** @brief Clear the system reset trigger sources.
  * @param[in]  reset_mask   A bitmask of reset sources to clear.
  *
  * @sa SYSCON_GetResetSources
  */
__INLINE static void SYSCON_ClearResetSources(uint32_t reset_mask)
{
    lpclib_assert((reset_mask & ~SYSCON_SYSRESSTAT_STAT_Mask) == 0);

    SYSCON->SYSRESSTAT = reset_mask;
}

/** @brief Set the system PLL clock source.
  * @param[in]  source       The new clock source
  *
  * @note
  * Need to call SYSCON_EnableSysPLLClockSourceUpdate after setting, and wait for
  * the source to be updated (SYSCON_SysPLLClockSourceIsUpdated() == 1)
  */
__INLINE static void SYSCON_SetSysPLLClockSource(SYSCON_SysPLLClockSource_Type source)
{
    lpclib_assert(SYSCON_IS_SYSPLL_CLOCK_SOURCE(source));

    SYSCON->SYSPLLCLKSEL = source;
}

/** @brief Get the current system PLL clock source.
  * @return                  The current clock source.
  */
__INLINE static SYSCON_SysPLLClockSource_Type SYSCON_GetSysPLLClockSource(void)
{
    return SYSCON->SYSPLLCLKSEL;
}

/** @brief Enable updating of the system PLL clock source.
  *
  * @note
  * This is required after updating the clock source in order for the actual
  * change to occur.
  */
__INLINE static void SYSCON_EnableSysPLLClockSourceUpdate(void)
{
    SYSCON->SYSPLLCLKUEN = SYSCON_SYSPLLUEN_ENA;
    SYSCON->SYSPLLCLKUEN = 0;
    SYSCON->SYSPLLCLKUEN = SYSCON_SYSPLLUEN_ENA;
}

/** @brief Test whether the system PLL clock source has updated.
  * @return                  1 if an update has occurred, 0 otherwise.
  *
  * @note
  * Generally will be called in a while() loop after the source has been updated
  * and SYSCON_EnableSysPLLClockSourceUpdate() has been called to ensure the
  * update happens before continuing.
  */
__INLINE static unsigned int SYSCON_SysPLLClockSourceIsUpdated(void)
{
    return (SYSCON->SYSPLLCLKUEN & SYSCON_SYSPLLUEN_ENA) ? 1:0;
}

/** @brief Set the system PLL clock scaler.
  * @param[in]  scaler       The new clock scaler setting
  *
  * @note
  * The scaler value is a hybrid of PLL M (multiplier) and P (divider)
  * values.
  */
__INLINE static void SYSCON_SetSysPLLClockScaler(unsigned int scaler)
{
    lpclib_assert((scaler & ~(SYSCON_SYSPLLCTRL_MSEL_Mask | SYSCON_SYSPLLCTRL_PSEL_Mask)) == 0);

    SYSCON->SYSPLLCTRL = scaler;
}

/** @brief Get the current system PLL scaler (M and P) setting.
  * @return                  The current system PLL scaler setting.
  *
  * @note
  * The scaler value is a hybrid of PLL M (multiplier) and P (divider)
  * values.
  */
__INLINE static unsigned int SYSCON_GetSysPLLClockScaler(void)
{
    return (SYSCON->SYSPLLCTRL & (SYSCON_SYSPLLCTRL_MSEL_Mask | SYSCON_SYSPLLCTRL_PSEL_Mask));
}

/** @brief Set the system PLL M (multiplier) value.
  * @param[in]  m            The new PLL multiplier value
  */
__INLINE static void SYSCON_SetSysPLLMVal(unsigned int m)
{
    lpclib_assert((m & ~(SYSCON_SYSPLLCTRL_MSEL_Mask)) == 0);

    SYSCON->SYSPLLCTRL = (SYSCON->SYSPLLCTRL & ~SYSCON_SYSPLLCTRL_MSEL_Mask) | m;
}

/** @brief  Get the current system PLL M (multiplier) value.
  * @return                  The current system PLL M value.
  */
__INLINE static uint8_t SYSCON_GetSysPLLMVal(void)
{
    return (SYSCON->SYSPLLCTRL & SYSCON_SYSPLLCTRL_MSEL_Mask);
}

/** @brief Set the system PLL P (divider) value.
  * @param[in]  p            The new PLL divider value
  */
__INLINE static void SYSCON_SetSysPLLPVal(unsigned int p)
{
    lpclib_assert(SYSCON_IS_SYSPLL_PVAL_TYPE(p));

    SYSCON->SYSPLLCTRL = (SYSCON->SYSPLLCTRL & ~SYSCON_SYSPLLCTRL_PSEL_Mask)
                          | (p << SYSCON_SYSPLLCTRL_PSEL_Shift);
}

/** @brief Get the current system PLL P (divider) value.
  * @return                  The current system PLL P value.
  */
__INLINE static unsigned int SYSCON_GetSysPLLPVal(void)
{
    return (SYSCON->SYSPLLCTRL & SYSCON_SYSPLLCTRL_PSEL_Mask)
            >> SYSCON_SYSPLLCTRL_PSEL_Shift;
}

/** @brief Set the main system clock source.
  * @param[in]  source       The new clock source.
  *
  * @note
  * Call SYSCON_EnableMainClockSourceUpdate after setting, and wait for
  * the source to be updated (SYSCON_MainClockSourceIsUpdated() == 1)
  */
__INLINE static void SYSCON_SetMainClockSource(SYSCON_MainClockSource_Type source)
{
    lpclib_assert(SYSCON_IS_MAIN_CLOCK_SOURCE(source));

    SYSCON->MAINCLKSEL = source;
}

/** @brief  Get the current main clock source.
  * @return                  The current clock source.
  */
__INLINE static SYSCON_MainClockSource_Type SYSCON_GetMainClockSource(void)
{
    return SYSCON->MAINCLKSEL;
}

/** @brief Enable updating of the main clock source.
  *
  * @note
  * This is required after updating the clock source in order for the actual
  * change to occur.
  */
__INLINE static void SYSCON_EnableMainClockSourceUpdate(void)
{
    SYSCON->MAINCLKUEN = SYSCON_MAINCLKUEN_ENA;
    SYSCON->MAINCLKUEN = 0;
    SYSCON->MAINCLKUEN = SYSCON_MAINCLKUEN_ENA;
}

/** @brief Test whether the main clock source has updated.
  * @return                  1 if an update has occurred, 0 otherwise.
  *
  * @note
  * Generally will be called in a while() loop after the source has been updated
  * and SYSCON_EnableMainClockSourceUpdate() has been called to ensure the update
  * happens before continuing.
  */
__INLINE static unsigned int SYSCON_MainClockSourceIsUpdated(void)
{
    return (SYSCON->MAINCLKUEN & SYSCON_MAINCLKUEN_ENA) ? 1:0;
}

/** @brief Set the divider for the AHB clock.
  * @param[in]  divider      The new AHB clock divider (0-255; 0 means disabled).
  */
__INLINE static void SYSCON_SetAHBClockDivider(unsigned int divider)
{
    lpclib_assert(divider <= 255);

    SYSCON->SYSAHBCLKDIV = divider;
}

/** @brief Get the current AHB clock divider.
  * @return                  The current clock divider.
  */
__INLINE static unsigned int SYSCON_GetAHBClockDivider(void)
{
    return SYSCON->SYSAHBCLKDIV;
}

/** @brief Enable the specified clock lines to devices on the AHB bus.
  * @param[in]  clockline_mask   A bitmask of AHB clock lines
  */
__INLINE static void SYSCON_EnableAHBClockLines(uint32_t clockline_mask)
{
    lpclib_assert((clockline_mask & ~SYSCON_SYSAHBCLKCTRL_Mask) == 0);

    SYSCON->SYSAHBCLKCTRL |= clockline_mask;
}

/** @brief Disable the specified clock lines to devices on the AHB bus.
  * @param[in]  clockline_mask   A bitmask of AHB clock lines
  */
__INLINE static void SYSCON_DisableAHBClockLines(uint32_t clockline_mask)
{
    lpclib_assert((clockline_mask & ~SYSCON_SYSAHBCLKCTRL_Mask) == 0);

    SYSCON->SYSAHBCLKCTRL &= ~clockline_mask;
}

/** @brief Get a bitmask of the currently enabled AHB clock lines.
  * @return                  A bitmask of currently enabled AHB clock lines.
  */
__INLINE static uint32_t SYSCON_GetEnabledAHBClockLines(void)
{
    return SYSCON->SYSAHBCLKCTRL & SYSCON_SYSAHBCLKCTRL_Mask;
}

/** @brief Set SSP0's main clock divider.
  * @param[in]  divider      The new clock divider (0-255; 0 disables).
  */
__INLINE static void SYSCON_SetSSP0ClockDivider(unsigned int divider)
{
    lpclib_assert(divider <= 255);

    SYSCON->SSP0CLKDIV = divider;
}

/** @brief Get SSP0's main clock divider.
  * @return                  The current SSP0 main clock divider.
  */
__INLINE static unsigned int SYSCON_GetSSP0ClockDivider(void)
{
    return SYSCON->SSP0CLKDIV;
}

/** @brief Set SSP1's main clock divider.
  * @param[in]  divider      The new SSP1 main clock divider (0-255; 0 disables).
  */
__INLINE static void SYSCON_SetSSP1ClockDivider(unsigned int divider)
{
    lpclib_assert(divider <= 255);

    SYSCON->SSP1CLKDIV = divider;
}

/** @brief Get SSP1's main clock divider.
  * @return                  The current clock divider.
  */
__INLINE static unsigned int SYSCON_GetSSP1ClockDivider(void)
{
    return SYSCON->SSP1CLKDIV;
}

/** @brief Set UART0's main clock divider.
  * @param[in]  divider      The new UART0 main clock divider (0-255; 0 disables).
  */
__INLINE static void SYSCON_SetUART0ClockDivider(unsigned int divider)
{
    lpclib_assert(divider <= 255);

    SYSCON->UART0CLKDIV = divider;
}

/** @brief Get UART0's main clock divider.
  * @return                  The current clock divider.
  */
__INLINE static unsigned int SYSCON_GetUART0ClockDivider(void)
{
    return SYSCON->UART0CLKDIV;
}

/** @brief Set the watchdog timer clock source.
  * @param[in]  source       The new clock source.
  *
  * @note
  * Call SYSCON_EnableWDTClockSourceUpdate after setting, and wait for
  * the source to be updated (SYSCON_WDTClockSourceIsUpdated() == 1)
  */
__INLINE static void SYSCON_SetWDTClockSource(SYSCON_WDTClockSource_Type source)
{
    lpclib_assert(SYSCON_IS_WDT_CLOCK_SOURCE(source));

    SYSCON->WDTCLKSEL = source;
}

/** @brief  Get the current watchdog timer clock source.
  * @return                  The current clock source.
  */
__INLINE static SYSCON_WDTClockSource_Type SYSCON_GetWDTClockSource(void)
{
    return SYSCON->WDTCLKSEL;
}

/** @brief Enable updating of the watchdog timer clock source.
  *
  * @note
  * This is required after updating the clock source in order for the actual
  * change to occur.
  */
__INLINE static void SYSCON_EnableWDTClockSourceUpdate(void)
{
    SYSCON->WDTCLKUEN = SYSCON_WDTCLKUEN_ENA;
    SYSCON->WDTCLKUEN = 0;
    SYSCON->WDTCLKUEN = SYSCON_WDTCLKUEN_ENA;
}

/** @brief  Test whether the watchdog timer's clock source has successfully updated.
  * @return 1 if the source has updated, 0 otherwise.
  *
  * Generally will be called in a while() loop after the source has been updated
  * and WDTCLK_EnableSourceUpdate() has been called.
  */
__INLINE static unsigned int SYSCON_WDTClockSourceIsUpdated(void)
{
    return (SYSCON->WDTCLKUEN & SYSCON_WDTCLKUEN_ENA) ? 1:0;
}

/** @brief Set the divider for the watchdog timer input clock.
  * @param[in]  divider      The new AHB clock divider (0-255; 0 means disabled).
  */
__INLINE static void SYSCON_SetWDTClockDivider(unsigned int divider)
{
    lpclib_assert(divider <= 255);

    SYSCON->WDTCLKDIV = divider;
}

/** @brief Get the current watchdog timer input clock divider.
  * @return                  The current clock divider.
  */
__INLINE static unsigned int SYSCON_GetWDTClockDivider(void)
{
    return SYSCON->WDTCLKDIV;
}

/** @brief Set the CLKOUT clock source.
  * @param[in]  source       The new clock source
  *
  * @note
  * Need to call SYSCON_EnableCLKOUTSourceUpdate after setting, and wait for
  * the source to be updated (SYSCON_CLKOUTSourceIsUpdated() == 1)
  */
__INLINE static void SYSCON_SetCLKOUTSource(SYSCON_CLKOUTSource_Type source)
{
    lpclib_assert(SYSCON_IS_CLKOUT_SOURCE(source));

    SYSCON->CLKOUTCLKSEL = source;
}

/** @brief Get the current CLKOUT clock source.
  * @return                  The current clock source.
  */
__INLINE static SYSCON_CLKOUTSource_Type SYSCON_GetCLKOUTSource(void)
{
    return SYSCON->CLKOUTCLKSEL;
}

/** @brief Enable updating of the CLKOUT clock source.
  *
  * @note
  * This is required after updating the clock source in order for the actual
  * change to occur.
  */
__INLINE static void SYSCON_EnableCLKOUTSourceUpdate(void)
{
    SYSCON->CLKOUTUEN = SYSCON_CLKOUTUEN_ENA;
    SYSCON->CLKOUTUEN = 0;
    SYSCON->CLKOUTUEN = SYSCON_CLKOUTUEN_ENA;
}

/** @brief Test whether the CLKOUT clock source has successfully updated.
  * @return                  1 if the source has updated, 0 otherwise.
  *
  * Generally will be called in a while() loop after the source has been updated
  * and CLKOUT_EnableSourceUpdate() has been called.
  */
__INLINE static unsigned int SYSCON_CLKOUTSourceIsUpdated(void)
{
    return (SYSCON->CLKOUTUEN & SYSCON_CLKOUTUEN_ENA) ? 1:0;
}

/** @brief Set the CLKOUT input clock divider.
  * @param[in] divider       The new clock divider (0-255; 0 means disabled).
  */
__INLINE static void SYSCON_SetCLKOUTDivider(unsigned int divider)
{
    lpclib_assert(divider <= 255);

    SYSCON->CLKOUTDIV = divider;
}

/** @brief Get the current CLKOUT clock divider.
  * @return                  The current clock divider.
  */
__INLINE static unsigned int SYSCON_GetCLKOUTDivider(void)
{
    return SYSCON->CLKOUTDIV;
}

/** @brief Set the brownout detector's reset trigger voltage.
  * @param[in]  voltage      The new reset trigger voltage
  */
__INLINE static void SYSCON_SetBODResetVoltage(SYSCON_BODResetVoltage_Type voltage)
{
    lpclib_assert(SYSCON_IS_BODRESETVOLTAGE(voltage));

    SYSCON->BODCTRL = (SYSCON->BODCTRL & ~SYSCON_BODCTRL_BODRSTLEV_Mask) | voltage;
}

/** @brief Get the brownout detector's current reset trigger voltage.
  * @return                  The current reset trigger voltage.
  */
__INLINE static SYSCON_BODResetVoltage_Type SYSCON_GetBODResetVoltage(void)
{
    return SYSCON->BODCTRL & SYSCON_BODCTRL_BODRSTLEV_Mask;
}

/** @brief Set the brownout detector interrupt trigger voltage.
  * @param[in]  voltage      The new interrupt trigger voltage
  */
__INLINE static void SYSCON_SetBODInterruptVoltage(SYSCON_BODInterruptVoltage_Type voltage)
{
    lpclib_assert(SYSCON_IS_BODINTERRUPTVOLTAGE(voltage));

    SYSCON->BODCTRL = (SYSCON->BODCTRL & ~SYSCON_BODCTRL_BODINTVAL_Mask) | voltage;
}

/** @brief Get the brownout detector's current interrupt trigger voltage.
  * @return                  The current interrupt trigger voltage.
  */
__INLINE static SYSCON_BODInterruptVoltage_Type SYSCON_GetBODInterruptVoltage(void)
{
    return SYSCON->BODCTRL & SYSCON_BODCTRL_BODINTVAL_Mask;
}

/** @brief Enable chip resets from the brownout detector.
  */
__INLINE static void SYSCON_EnableBODChipReset(void)
{
    SYSCON->BODCTRL |= SYSCON_BODCTRL_BODRSTENA;
}

/** @brief Disable chip resets from the brownout detector.
  */
__INLINE static void SYSCON_DisableBODChipReset(void)
{
    SYSCON->BODCTRL |= SYSCON_BODCTRL_BODRSTENA;
}

/** @brief Test whether brownout detector chip resets are enabled.
  * @return                  1 if chip resets are enabled, 0 otherwise.
  */
__INLINE static unsigned int SYSCON_BODChipResetIsEnabled(void)
{
    return (SYSCON->BODCTRL & SYSCON_BODCTRL_BODRSTENA) ? 1:0;
}

/** @brief Set the SysTick calibration value.
  * @param[in]  cal          The new calibration value (0-4095)
  */
__INLINE static void SYSCON_SetSysTickCalibration(uint32_t cal)
{
    lpclib_assert(cal <= 4095);

    SYSCON->SYSTCKCAL = cal;
}

/** @brief Get the current SysTick calibration value.
  * @return                  The current SysTick calibration value (0-4095).
  */
__INLINE static uint32_t SYSCON_GetSysTickCalibration(void)
{
    return SYSCON->SYSTCKCAL;
}

/** @brief Set the edges that will trigger a wake-up ("start logic") on the specified inputs.
  * @param[in]  input_mask   A bitmask of wake-up inputs
  * @param[in]  edge         The edge setting on which to trigger a wake-up on those inputs
  */
__INLINE static void SYSCON_SetWakeupEdgesForInputs(uint32_t input_mask,
                                                    SYSCON_WakeupEdge_Type edge)
{
    lpclib_assert((input_mask & ~SYSCON_WakeupInput_Mask) == 0);
    lpclib_assert(SYSCON_IS_WAKEUPEDGE(edge));

    if (edge == SYSCON_WakeupEdge_Falling) {
        SYSCON->STARTAPRP0 &= ~input_mask;
    } else {
        SYSCON->STARTAPRP0 |= input_mask;
    }
}

/** @brief Get a bitmask of wake-up ("start logic") inputs configured to trigger on the given edge.
  * @return             A bitmask of wake-up inputs configured to trigger on the given edge type.
  */
__INLINE static uint32_t SYSCON_GetStartLogicInputsWithEdge(SYSCON_WakeupEdge_Type edge)
{
     lpclib_assert(SYSCON_IS_WAKEUPEDGE(edge));

    if (edge == SYSCON_WakeupEdge_Falling) {
        return (~SYSCON->STARTAPRP0) & SYSCON_WakeupInput_Mask;
    } else {
        return SYSCON->STARTAPRP0;
    }
}

/** @brief Enable wake-up ("start logic") triggers from the specified inputs.
  * @param[in]  input_mask       A bitmask of wake-up inputs
  */
__INLINE static void SYSCON_EnableWakeupInputs(uint32_t input_mask)
{
    lpclib_assert((input_mask & ~SYSCON_WakeupInput_Mask) == 0);

    SYSCON->STARTERP0 |= input_mask;
}

/** @brief Disable wake-up ("start logic") triggers from the specified inputs.
  * @param[in]  input_mask       A bitmask of wake-up inputs
  */
__INLINE static void SYSCON_DisableWakeupInputs(uint32_t input_mask)
{
    lpclib_assert((input_mask & ~SYSCON_WakeupInput_Mask) == 0);

    SYSCON->STARTERP0 &= ~input_mask;
}

/** @brief Get a bitmask of currently enabled wake-up ("start logic") inputs.
  * @return              A bitmask of the enabled wake-up inputs.
  */
__INLINE static uint32_t SYSCON_GetEnabledWakeupInputs(void)
{
    return SYSCON->STARTERP0;
}

/** @brief Reset the specified wake-up ("start logic") inputs.
  * @param[in]  input_mask   A bitmask of wake-up inputs to reset
  *
  * @note
  * Start logic inputs need to be reset each time they are used to re-enable
  * triggering.
  */
__INLINE static void SYSCON_ResetWakeupInputs(uint32_t input_mask)
{
    lpclib_assert((input_mask & ~SYSCON_WakeupInput_Mask) == 0);

    SYSCON->STARTRSRP0CLR |= input_mask;
}

/** @brief Get the triggered states for all wake-up ("start logic") inputs.
  * @return             A bitmask of inputs for which start logic has been triggered.
  *
  * @note
  * Start logic inputs need to be reset each time they are used to re-enable
  * triggering.
  */
__INLINE static uint32_t SYSCON_GetTriggeredStartLogicInputs(void)
{
    return SYSCON->STARTSRP0;
}

/** @brief Initialize the system's analog power configuration.
  * This function sets reserved bits to required values and clears the
  * sleep configuration.
  */
__INLINE static void SYSCON_InitAnalogPowerLines(void)
{
    SYSCON->PDSLEEPCFG = SYSCON_PDSLEEPCFG_Required;
    SYSCON->PDAWAKECFG = (SYSCON->PDAWAKECFG & SYSCON_PDAWAKECFG_Mask) | SYSCON_PDAWAKECFG_Required;
    SYSCON->PDRUNCFG = (SYSCON->PDRUNCFG & SYSCON_PDRUNCFG_Mask) | SYSCON_PDRUNCFG_Required;
}

/** @brief Enable power to specified analog power blocks for the given power mode.
  * @param[in]  mode           A system power mode
  * @param[in]  powerline_mask A bitmask of analog power control lines
  */
__INLINE static void SYSCON_EnableAnalogPowerLinesForMode(SYSCON_PowerMode_Type mode,
                                                          uint32_t powerline_mask)
{
    lpclib_assert(SYSCON_IS_POWERMODE(mode));

    if (mode == SYSCON_PowerMode_Sleep) {
        lpclib_assert((powerline_mask & ~SYSCON_PDSLEEPCFG_Mask) == 0);
        SYSCON->PDSLEEPCFG = (SYSCON->PDSLEEPCFG & ~powerline_mask) | SYSCON_PDSLEEPCFG_Required;
    } else if (mode == SYSCON_PowerMode_Awake) {
        lpclib_assert((powerline_mask & ~SYSCON_PDAWAKECFG_Mask) == 0);
        SYSCON->PDAWAKECFG = (SYSCON->PDAWAKECFG & ~powerline_mask) | SYSCON_PDAWAKECFG_Required;
    } else { /* SYSCON_PowerMode_Run */
        lpclib_assert((powerline_mask & ~SYSCON_PDRUNCFG_Mask) == 0);
        SYSCON->PDRUNCFG = (SYSCON->PDRUNCFG & ~powerline_mask) | SYSCON_PDRUNCFG_Required;
    }
}

/** @brief Disable power to specified analog power blocks for the given power mode.
  * @param[in]  mode           A system power mode
  * @param[in]  powerline_mask A bitmask of analog power control lines
  */
__INLINE static void SYSCON_DisableAnalogPowerLinesForMode(SYSCON_PowerMode_Type mode,
                                                           uint32_t powerline_mask)
{
    lpclib_assert(SYSCON_IS_POWERMODE(mode));

    if (mode == SYSCON_PowerMode_Sleep) {
        lpclib_assert((powerline_mask & ~SYSCON_PDSLEEPCFG_Mask) == 0);
        SYSCON->PDSLEEPCFG |= (SYSCON_PDSLEEPCFG_Required | powerline_mask);
    } else if (mode == SYSCON_PowerMode_Awake) {
        lpclib_assert((powerline_mask & ~SYSCON_PDAWAKECFG_Mask) == 0);
        SYSCON->PDAWAKECFG |= (SYSCON_PDAWAKECFG_Required | powerline_mask);
    } else { /* SYSCON_PowerMode_Run */
        lpclib_assert((powerline_mask & ~SYSCON_PDRUNCFG_Mask) == 0);
        SYSCON->PDRUNCFG |= (SYSCON_PDRUNCFG_Required | powerline_mask);
    }
}

/** @brief Get a bitmask of analog power lines that are enabled for the given power mode.
  * @param[in]  mode         A system power mode
  * @return                  A bitmask of analog power lines that are enabled for the given mode.
  */
__INLINE static uint32_t SYSCON_GetEnabledAnalogPowerLinesForMode(SYSCON_PowerMode_Type mode)
{
    if (mode == SYSCON_PowerMode_Sleep) {
        return ~(SYSCON->PDSLEEPCFG & SYSCON_PDSLEEPCFG_Mask);
    } else if (mode == SYSCON_PowerMode_Awake) {
        return ~(SYSCON->PDAWAKECFG & SYSCON_PDAWAKECFG_Mask);
    } else { /* SYSCON_PowerMode_Run */
        return ~(SYSCON->PDRUNCFG & SYSCON_PDRUNCFG_Mask);
    }
}

/** @brief Get the microcontroller device ID.
  * @return                  The device ID.
  */
__INLINE static uint32_t SYSCON_GetDeviceID(void)
{
    return SYSCON->DEVICE_ID;
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

#endif /* #ifndef LPC_SYSCON_H_ */

