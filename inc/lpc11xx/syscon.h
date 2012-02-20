/**************************************************************************//**
 * @file     syscon.h
 * @brief    System Control Block Interface Header for LPC11xx Microcontrollers
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. June 2010
 ******************************************************************************
 * @section Overview
 * This file gives a basic interface to NXP LPC11xx microcontroller
 * system configuration blocks.  It abstracts such things as setting
 * remapping ISR memory, enabling/disabling system clock lines, configuring
 * the system brownout detector, and getting device ID's.
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
 ******************************************************************************
 * This file defines types and functions for using the LPC11xx System Control
 * block:
 *  - Memory Remapping
 *  - Peripheral Resetting
 *  - System, WDT, Peripheral, IRC, PLL, AHB Clocks
 *  - Brownout Detector
 *  - System Start Logic
 *  - System Power Down / Wake Settings
 *  - LPC11xx Device ID's
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

#define SYSCON_ResetSource_POR            (1 << 0)         /*!< System reset from power on reset */
#define SYSCON_ResetSource_EXT            (1 << 1)         /*!< System reset from external reset */
#define SYSCON_ResetSource_WDT            (1 << 2)         /*!< System reset from watchdog timer */
#define SYSCON_ResetSource_BOD            (1 << 3)         /*!< System reset from brownout       */
#define SYSCON_ResetSource_SYSRST         (1 << 4)         /*!< System reset from NVIC reset     */

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
#define SYSCON_IS_BOD_RESET_VOLTAGE(Voltage) (((Voltage) == SYSCON_BODResetVoltage_1V46) \
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
#define SYSCON_IS_BOD_INTERRUPT_VOLTAGE(Voltage) (((Voltage) == SYSCON_BODInterruptVoltage_1V65) \
                                               || ((Voltage) == SYSCON_BODInterruptVoltage_2V22) \
                                               || ((Voltage) == SYSCON_BODInterruptVoltage_2V52) \
                                               || ((Voltage) == SYSCON_BODInterruptVoltage_2V80))

/** @} */

/** @defgroup SYSCON_StartLogicEdges Start Logic Edge Triggers
  * @{
  */

/*! @brief Start Logic Trigger Edges */
typedef enum {
    SYSCON_StartLogicEdge_Falling = 0x00,                  /*!< Start logic trig on falling edge */
    SYSCON_StartLogicEdge_Rising                           /*!< Start logic trig on rising edge  */
} SYSCON_StartLogicEdge_Type;

/*! @brief Macro to test whether parameter is a valid start logic edge value */
#define SYSCON_IS_START_LOGIC_EDGE(Edge) (((Edge) == SYSCON_StartLogicEdge_Falling) \
                                       || ((Edge) == SYSCON_StartLogicEdge_Rising))

/** @} */

/** @defgroup SYSCON_StartLogicInputs Start Logic Input Pins
  * @{
  */

#define SYSCON_StartLogicInput_PIO0_0  (1 << 0)            /*!< System start logic input PIO0_0  */
#define SYSCON_StartLogicInput_PIO0_1  (1 << 1)            /*!< System start logic input PIO0_1  */
#define SYSCON_StartLogicInput_PIO0_2  (1 << 2)            /*!< System start logic input PIO0_2  */
#define SYSCON_StartLogicInput_PIO0_3  (1 << 3)            /*!< System start logic input PIO0_3  */
#define SYSCON_StartLogicInput_PIO0_4  (1 << 4)            /*!< System start logic input PIO0_4  */
#define SYSCON_StartLogicInput_PIO0_5  (1 << 5)            /*!< System start logic input PIO0_5  */
#define SYSCON_StartLogicInput_PIO0_6  (1 << 6)            /*!< System start logic input PIO0_6  */
#define SYSCON_StartLogicInput_PIO0_7  (1 << 7)            /*!< System start logic input PIO0_7  */
#define SYSCON_StartLogicInput_PIO0_8  (1 << 8)            /*!< System start logic input PIO0_8  */
#define SYSCON_StartLogicInput_PIO0_9  (1 << 9)            /*!< System start logic input PIO0_9  */
#define SYSCON_StartLogicInput_PIO0_10 (1 << 10)           /*!< System start logic input PIO0_10 */
#define SYSCON_StartLogicInput_PIO0_11 (1 << 11)           /*!< System start logic input PIO0_11 */
#define SYSCON_StartLogicInput_PIO1_0  (1 << 12)           /*!< System start logic input PIO1_0  */

/*! @brief Type for passing start logic input masks */
typedef uint16_t SYSCON_StartLogicInputs_Type;

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

/*! @brief Type for passing analog power line control bits */
typedef uint16_t SYSCON_AnalogPowerLines_Type;

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
#define SYSCON_IS_POWER_MODE(Mode)  (((Mode) == SYSCON_PowerMode_Sleep) \
                                  || ((Mode) == SYSCON_PowerMode_Awake) \
                                  || ((Mode) == SYSCON_PowerMode_Run))

/** @} */

/** @defgroup LPC11xx_DeviceIDs LPC11xx Device ID Values
  * @{
  */

#define DeviceID_LPC1111_101           (0x041e502bUL)         /*!< LPC1111  w/8K  FLASH / 2K RAM */
#define DeviceID_LPC1111_102           (0x2516d02bUL)         /*!< LPC1111L w/8K  FLASH / 2K RAM */
#define DeviceID_LPC1111_201           (0x0416502bUL)         /*!< LPC1111  w/8K  FLASH / 4K RAM */
#define DeviceID_LPC1111_202           (0x2516902bUL)         /*!< LPC1111L w/8K  FLASH / 4K RAM */
#define DeviceID_LPC1112_101           (0x042d502bUL)         /*!< LPC1112  w/16K FLASH / 2K RAM */
#define DeviceID_LPC1112_102           (0x2524d02bUL)         /*!< LPC1112L w/16K FLASH / 2K RAM */
#define DeviceID_LPC1112_201           (0x0425502bUL)         /*!< LPC1112  w/16K FLASH / 4K RAM */
#define DeviceID_LPC1112_202           (0x2524902bUL)         /*!< LPC1112L w/16K FLASH / 2K RAM */
#define DeviceID_LPC1113_201           (0x0434502bUL)         /*!< LPC1113  w/24K FLASH / 4K RAM */
#define DeviceID_LPC1113_202           (0x2532902bUL)         /*!< LPC1113L w/24K FLASH / 4K RAM */
#define DeviceID_LPC1113_301           (0x0434102bUL)         /*!< LPC1113  w/24K FLASH / 8K RAM */
#define DeviceID_LPC1113_302           (0x2532102bUL)         /*!< LPC1113L w/24K FLASH / 4K RAM */
#define DeviceID_LPC1114_201           (0x0444502bUL)         /*!< LPC1114  w/32K FLASH / 4K RAM */
#define DeviceID_LPC1114_202           (0x2540902bUL)         /*!< LPC1114L w/32K FLASH / 4K RAM */
#define DeviceID_LPC1114_301           (0x0444102bUL)         /*!< LPC1114  w/32K FLASH / 8K RAM */
#define DeviceID_LPC1114_302           (0x2540102bUL)         /*!< LPC1114L w/32K FLASH / 8K RAM */
#define DeviceID_LPC11C12_301          (0x1421102bUL)         /*!< LPC11C12 w/16K FLASH / 8K RAM */
#define DeviceID_LPC11C14_301          (0x1440102bUL)         /*!< LPC11C14 w/32K FLASH / 8K RAM */
#define DeviceID_LPC11C22_301          (0x1431102bUL)         /*!< LPC11C22 w/16K FLASH / 8K RAM */
#define DeviceID_LPC11C24_301          (0x1430102bUL)         /*!< LPC11C24 w/32K FLASH / 8K RAM */

/** @} */

/**
  * @}
  */


/* Inline Functions ---------------------------------------------------------*/

/**
  * @defgroup SYSCON_InlineFunctions SYSCON Interface Inline Functions
  * @{
  */

/** @brief  Remap the ISR vector area of memory to the specified region
  * @param  RemapMem  The area to map to the ISR vector area
  * @return None.
  *
  * This is used to change memory area where the interrupt vectors are found.
  */
__INLINE static void SYSCON_SetMemRemap(SYSCON_RemapMem_Type RemapMem)
{
    lpclib_assert(SYSCON_IS_REMAP_MEM(RemapMem));

    SYSCON->SYSMEMREMAP = RemapMem;
}

/** @brief  Return the area of memory currently remapped to the ISR Vector Memory Region
  * @return The currently remapped region
  */
__INLINE static SYSCON_RemapMem_Type SYSCON_GetMemRemap(void)
{
    return SYSCON->SYSMEMREMAP;
}

/** @brief  Assert the reset lines of specified peripherals.
  * @param  PeripheralResets  Mask of the peripherals to assert reset on
  * @return None.
  */
__INLINE static void SYSCON_AssertPeripheralResets(unsigned int PeripheralResets)
{
    lpclib_assert((PeripheralResets & ~SYSCON_PRESETCTRL_RESET_Mask) == 0);

    /* Note: Reset lines are active low */
    SYSCON->PRESETCTRL &= ~PeripheralResets;
}

/** @brief  De-assert the reset lines of specified peripherals.
  * @param  PeripheralResets  Mask of the peripherals to de-assert reset on
  * @return None.
  */
__INLINE static void SYSCON_DeassertPeripheralResets(unsigned int PeripheralResets)
{
    lpclib_assert((PeripheralResets & ~SYSCON_PRESETCTRL_RESET_Mask) == 0);

    /* Note: Reset lines are active low */
    SYSCON->PRESETCTRL |= PeripheralResets;
}

/** @brief  Get the mask specifying peripherals currently held in reset.
  * @return A mask of the peripherals being held in reset.
  */
__INLINE static unsigned int SYSCON_GetAssertedPeripheralResets(void)
{
    return (~(SYSCON->PRESETCTRL)) & SYSCON_PRESETCTRL_RESET_Mask;
}

/** @brief  Test whether the system PLL is locked.
  * @return 1 if the System PLL is locked, 0 otherwise.
  */
__INLINE static unsigned int SYSCON_SysPLLIsLocked(void)
{
    return (SYSCON->SYSPLLSTAT & SYSCON_SYSPLLSTAT_LOCKED) ? 1:0;
}

/** @brief  Enable bypass of the system oscillator.
  * @return None.
  */
__INLINE static void SYSCON_EnableSysOscBypass(void)
{
    SYSCON->SYSOSCCTRL |= SYSCON_SYSOSCCTRL_BYPASS;
}

/** @brief  Disable bypass of the system oscillator.
  * @return None.
  */
__INLINE static void SYSCON_DisableSysOscBypass(void)
{
    SYSCON->SYSOSCCTRL &= ~SYSCON_SYSOSCCTRL_BYPASS;
}

/** @brief  Test whether the system oscillator is currently bypassed.
  * @return 1 if the system oscillator is bypassed, 0 otherwise.
  */
__INLINE static unsigned int SYSCON_SysOscBypassIsEnabled(void)
{
    return (SYSCON->SYSOSCCTRL & SYSCON_SYSOSCCTRL_BYPASS) ? 1:0;
}

/** @brief  Set the frequency range of the xtal feeding the system oscillator.
  * @param  Range    Value specifying the frequency range of the xtal.
  * @return None.
  */
__INLINE static void SYSCON_SetSysOscFreqRange(SYSCON_SysOscFreqRange_Type Range)
{
    lpclib_assert(SYSCON_IS_SYSOSC_FREQ_RANGE(Range));

    SYSCON->SYSOSCCTRL = (SYSCON->SYSOSCCTRL & ~SYSCON_SYSOSCCTRL_FREQRANGE) | Range;
}

/** @brief  Get the set frequency range of the xtal feeding the system oscillator.
  * @return A value specifying the frequency range of the xtal.
  */
__INLINE static SYSCON_SysOscFreqRange_Type SYSCON_GetSysOscFreqRange(void)
{
    return SYSCON->SYSOSCCTRL & SYSCON_SYSOSCCTRL_FREQRANGE;
}

/** @brief  Set the divider of the watchdog timer's oscillator
  * @param  Divider   The value by which to divide the WDT clock
  * @return None.
  *
  * Divider must be an even # between 2 and 64 inclusive.
  */
__INLINE static void SYSCON_SetWDTOscDivider(unsigned int Divider)
{
    lpclib_assert((Divider <= 64) && (Divider != 0));
    lpclib_assert((Divider & 1) == 0);

    SYSCON->WDTOSCCTRL = (SYSCON->WDTOSCCTRL & ~SYSCON_WDTOSCCTRL_DIV_Mask)
                          | ((Divider >> 1) - 1);
}

/** @brief  Get the current divider of the watchdog timer's oscillator.
  * @return The current divider value (# between 2 & 64 inclusive)
  */
__INLINE static unsigned int SYSCON_GetWDTOscDivider(void)
{
    return ((SYSCON->WDTOSCCTRL & SYSCON_WDTOSCCTRL_DIV_Mask) + 1) << 1;
}

/** @brief  Set the frequency of the watchdog timer's oscillator.
  * @param  OscFreq  The WDT oscillator frequency setting (of WDT_OscFreq_Type)
  * @return None.
  */
__INLINE static void SYSCON_SetWDTOscFreq(SYSCON_WDTOscFreq_Type OscFreq)
{
    lpclib_assert(SYSCON_IS_WDT_OSC_FREQ(OscFreq));

    SYSCON->WDTOSCCTRL = (SYSCON->WDTOSCCTRL & ~SYSCON_WDTOSCCTRL_FREQSEL_Mask)
                         | (OscFreq << SYSCON_WDTOSCCTRL_FREQSEL_Shift);
}

/** @brief  Get the current frequency setting of the watchdog timer oscillator.
  * @return The currently configured WDT oscillator frequency setting.
  */
__INLINE static SYSCON_WDTOscFreq_Type SYSCON_GetWDTOscFreq(void)
{
    return (SYSCON_WDTOscFreq_Type)((SYSCON->WDTOSCCTRL & SYSCON_WDTOSCCTRL_FREQSEL_Mask)
            >> SYSCON_WDTOSCCTRL_FREQSEL_Shift);
}

/** @brief  Set the internal RC oscillator's trim value.
  * @param  Trim Value to set (0 - 255)
  * @return None.
  *
  * Not normally necessary to be adjusted; set by system boot ROM.
  */
__INLINE static void SYSCON_SetIRCTrim(unsigned int Trim)
{
    lpclib_assert((Trim & ~SYSCON_IRCCTL_TRIM_Mask) == 0);

    SYSCON->IRCCTRL = Trim;
}

/** @brief  Get the configured internal RC oscillator's trim value (0 - 255).
  * @return The configured IRC trim value.
  */
__INLINE static unsigned int SYSCON_GetIRCTrim(void)
{
    return SYSCON->IRCCTRL;
}

/** @brief  Get the source(s) that triggered the previous system reset(s).
  * @return A bitmask of the reset sources.
  *
  * Sources are sticky & must be cleared by the application.
  */
__INLINE static uint32_t SYSCON_GetResetSources(void)
{
    return SYSCON->SYSRESSTAT;
}

/** @brief  Clear reset trigger sources.
  * @param  sources  A bitmask of reset sources to clear
  * @return None.
  */
__INLINE static void SYSCON_ClearResetSource(uint32_t sources)
{
    lpclib_assert((sources & ~SYSCON_SYSRESSTAT_STAT_Mask) == 0);

    SYSCON->SYSRESSTAT = sources;
}

/** @brief  Set the system PLL's clock source.
  * @param  Source   The new clock source
  * @return None.
  *
  * Need to call SYSPLL_EnableSourceUpdate after setting, and wait for
  *  the source to be updated (SYSPLL_SourceIsUpdated() == 1)
  */
__INLINE static void SYSCON_SetSysPLLClockSource(SYSCON_SysPLLClockSource_Type Source)
{
    lpclib_assert(SYSCON_IS_SYSPLL_CLOCK_SOURCE(Source));

    SYSCON->SYSPLLCLKSEL = Source;
}

/** @brief  Get the system PLL's current clock source.
  * @return The current clock source.
  */
__INLINE static SYSCON_SysPLLClockSource_Type SYSCON_GetSysPLLClockSource(void)
{
    return SYSCON->SYSPLLCLKSEL;
}

/** @brief  Enable updating of the system PLL's clock source.
  * @return None.
  *
  * Used after changing the system PLL clock source to make the change active.
  */
__INLINE static void SYSCON_EnableSysPLLClockSourceUpdate(void)
{
    SYSCON->SYSPLLCLKUEN = SYSCON_SYSPLLUEN_ENA;
    SYSCON->SYSPLLCLKUEN = 0;
    SYSCON->SYSPLLCLKUEN = SYSCON_SYSPLLUEN_ENA;
}

/** @brief  Test whether the system PLL's clock source has successfully updated.
  * @return 1 if the source has updated, 0 otherwise.
  *
  * Generally will be called in a while() loop after the source has been updated
  * and SYSPLL_EnableSourceUpdate() has been called.
  */
__INLINE static unsigned int SYSCON_SysPLLClockSourceIsUpdated(void)
{
    return (((uint16_t)(SYSCON->SYSPLLCLKUEN)) & ((uint16_t)SYSCON_SYSPLLUEN_ENA)) ? 1:0;
}

/** @brief  Set the system PLL's clock scaler setting.
  * @param  Scaler  The new clock scaler setting
  * @return None.
  *
  * The scaler value is a hybrid of PLL M (multiplier) and P (divider)
  * values.
  */
__INLINE static void SYSCON_SetSysPLLClockScaler(unsigned int Scaler)
{
    lpclib_assert((Scaler & ~(SYSCON_SYSPLLCTRL_MSEL_Mask | SYSCON_SYSPLLCTRL_PSEL_Mask)) == 0);

    SYSCON->SYSPLLCTRL = Scaler;
}

/** @brief  Get the system PLL's current scaler (M and P) setting.
  * @return The system PLL's current scaler setting.
  *
  * The scaler value is a hybrid of PLL M (multiplier) and P (divider)
  * values.
  */
__INLINE static unsigned int SYSCON_GetSysPLLClockScaler(void)
{
    return (SYSCON->SYSPLLCTRL & (SYSCON_SYSPLLCTRL_MSEL_Mask | SYSCON_SYSPLLCTRL_PSEL_Mask));
}

/** @brief  Set the system PLL's M (multiplier) value
  * @param  MVal  The new PLL multiplier value
  * @return None.
  */
__INLINE static void SYSCON_SetSysPLLMVal(unsigned int MVal)
{
    lpclib_assert((MVal & ~(SYSCON_SYSPLLCTRL_MSEL_Mask)) == 0);

    SYSCON->SYSPLLCTRL = (SYSCON->SYSPLLCTRL & ~SYSCON_SYSPLLCTRL_MSEL_Mask) | MVal;
}

/** @brief  Get the system PLL's current M (multiplier) value.
  * @return The current system PLL M value
  */
__INLINE static uint8_t SYSCON_GetSysPLLMVal(void)
{
    return (SYSCON->SYSPLLCTRL & SYSCON_SYSPLLCTRL_MSEL_Mask);
}

/** @brief  Set the system PLL's P (divider) value.
  * @param  PVal  The new PLL divider value
  * @return None.
  */
__INLINE static void SYSCON_SetSysPLLPVal(unsigned int PVal)
{
    lpclib_assert(SYSCON_IS_SYSPLL_PVAL_TYPE(PVal));

    SYSCON->SYSPLLCTRL = (SYSCON->SYSPLLCTRL & ~SYSCON_SYSPLLCTRL_PSEL_Mask)
                          | (PVal << SYSCON_SYSPLLCTRL_PSEL_Shift);
}

/** @brief  Get the system PLL's current P (divider) value.
  * @return The current system PLL P value
  */
__INLINE static unsigned int SYSCON_GetSysPLLPVal(void)
{
    return (SYSCON->SYSPLLCTRL & SYSCON_SYSPLLCTRL_PSEL_Mask)
            >> SYSCON_SYSPLLCTRL_PSEL_Shift;
}

/** @brief  Set the main system clock source.
  * @param  Source  The new clock source.
  * @return None.
  *
  * Call MAINCLK_EnableSourceUpdate after setting, and wait for
  *  the source to be updated (MAINCLK_SourceIsUpdated() == 1)
  */
__INLINE static void SYSCON_SetMainClockSource(SYSCON_MainClockSource_Type Source)
{
    lpclib_assert(SYSCON_IS_MAIN_CLOCK_SOURCE(Source));

    SYSCON->MAINCLKSEL = Source;
}

/** @brief  Get the main clock's currently configured clock source.
  * @return The current clock source.
  */
__INLINE static SYSCON_MainClockSource_Type SYSCON_GetMainClockSource(void)
{
    return SYSCON->MAINCLKSEL;
}

/** @brief  Enable updating of the main system clock's source.
  * @return None.
  *
  * Used after changing the main system clock source to make the change active.
  */
__INLINE static void SYSCON_EnableMainClockSourceUpdate(void)
{
    SYSCON->MAINCLKUEN = SYSCON_MAINCLKUEN_ENA;
    SYSCON->MAINCLKUEN = 0;
    SYSCON->MAINCLKUEN = SYSCON_MAINCLKUEN_ENA;
}

/** @brief  Test whether the main system clock source has successfully updated.
  * @return 1 if the source has updated, 0 otherwise.
  *
  * Generally will be called in a while() loop after the source has been updated
  * and MAINCLK_EnableSourceUpdate() has been called.
  */
__INLINE static unsigned int SYSCON_MainClockSourceIsUpdated(void)
{
    return (SYSCON->MAINCLKUEN & SYSCON_MAINCLKUEN_ENA) ? 1:0;
}

/** @brief  Set the value by which the main clock will be divided to feed the AHB clock.
  * @param  Divider  The new AHB clock divider.
  * @return None.
  *
  * Divider must be an integer from 0-255 inclusive (0 == disabled)
  */
__INLINE static void SYSCON_SetAHBClockDivider(uint32_t Divider)
{
    lpclib_assert((Divider & ~SYSCON_SYSAHBCLKDIV_Mask) == 0);

    SYSCON->SYSAHBCLKDIV = Divider;
}

/** @brief  Get the value by which the main clock is divided to feed the AHB clock.
  * @return The current clock divider.
  */
__INLINE static uint32_t SYSCON_GetAHBClockDivider(void)
{
    return SYSCON->SYSAHBCLKDIV;
}

/** @brief  Enable the specified clock lines to devices on the AHB bus.
  * @param  Clocklines  A bitmask of clock lines to enable
  * @return None.
  */
__INLINE static void SYSCON_EnableAHBClockLines(uint32_t Clocklines)
{
    lpclib_assert((Clocklines & ~SYSCON_SYSAHBCLKCTRL_Mask) == 0);

    SYSCON->SYSAHBCLKCTRL |= Clocklines;
}

/** @brief  Disable the specified clock lines from devices on the AHB bus.
  * @param  Clocklines  A bitmask of clock lines to disable
  * @return None.
  */
__INLINE static void SYSCON_DisableAHBClockLines(uint32_t Clocklines)
{
    lpclib_assert((Clocklines & ~SYSCON_SYSAHBCLKCTRL_Mask) == 0);

    SYSCON->SYSAHBCLKCTRL &= ~Clocklines;
}

/** @brief  Get the currently enabled clock lines to devices on the AHB Bus.
  * @return A bitmask of the currently enabled AHB bus clock lines.
  */
__INLINE static uint32_t SYSCON_GetEnabledAHBClockLines(void)
{
    return SYSCON->SYSAHBCLKCTRL & SYSCON_SYSAHBCLKCTRL_Mask;
}

/** @brief  Set the value by which the main clock will be divided to feed the SSP0 clock.
  * @param  Divider  The new SSP0 clock divider.
  * @return None.
  *
  * Divider must be an integer from 0-255 inclusive (0 == disabled)
  */
__INLINE static void SYSCON_SetSSP0ClockDivider(unsigned int Divider)
{
    lpclib_assert((Divider & ~SYSCON_SSP0CLKDIV_Mask) == 0);

    SYSCON->SSP0CLKDIV = Divider;
}

/** @brief  Get the value by which the main clock is divided to feed the SSP0 clock.
  * @return The current SSP0 clock divider.
  */
__INLINE static unsigned int SYSCON_GetSSP0ClockDivider(void)
{
    return SYSCON->SSP0CLKDIV;
}

/** @brief  Set the value by which the main clock will be divided to feed the SSP1 clock.
  * @param  Divider  The new SSP1 clock divider.
  * @return None.
  *
  * Divider must be an integer from 0-255 inclusive (0 == disabled)
  */
__INLINE static void SYSCON_SetSSP1ClockDivider(unsigned int Divider)
{
    lpclib_assert((Divider & ~SYSCON_SSP1CLKDIV_Mask) == 0);

    SYSCON->SSP1CLKDIV = Divider;
}

/** @brief  Get the value by which the main clock is divided to feed the SSP1 clock.
  * @return The current SSP1 clock divider.
  */
__INLINE static unsigned int SYSCON_GetSSP1ClockDivider(void)
{
    return SYSCON->SSP1CLKDIV;
}

/** @brief  Set the value by which the main clock will be divided to feed the UART0 clock.
  * @param  Divider  The new UART0 clock divider.
  * @return None.
  *
  * Divider must be an integer from 0-255 inclusive (0 == disabled)
  */
__INLINE static void SYSCON_SetUART0ClockDivider(unsigned int Divider)
{
    lpclib_assert((Divider & ~SYSCON_UART0CLKDIV_Mask) == 0);

    SYSCON->UART0CLKDIV = Divider;
}

/** @brief  Get the value by which the main clock is divided to feed the UART0 clock.
  * @return The current UART0 clock divider.
  */
__INLINE static unsigned int SYSCON_GetUART0ClockDivider(void)
{
    return SYSCON->UART0CLKDIV;
}

/** @brief  Set the watchdog timer's clock source.
  * @param  Source   The new WDT clock source
  * @return None.
  *
  * Call WDTCLK_EnableSourceUpdate after setting, and wait for
  *  the source to be updated (WDTCLK_SourceIsUpdated() == 1)
  */
__INLINE static void SYSCON_SetWDTClockSource(SYSCON_WDTClockSource_Type Source)
{
    lpclib_assert(SYSCON_IS_WDT_CLOCK_SOURCE(Source));

    SYSCON->WDTCLKSEL = Source;
}

/** @brief  Get the current watchdog timer clock source.
  * @return The current WDT clock source.
  */
__INLINE static SYSCON_WDTClockSource_Type SYSCON_GetWDTClockSource(void)
{
    return SYSCON->WDTCLKSEL;
}

/** @brief  Enable updating of the watchdog timer's clock source.
  * @return None.
  *
  * Used after changing the watchdog timer's clock source to make the change active.
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
    return (((uint16_t)SYSCON->WDTCLKUEN) & ((uint16_t)SYSCON_WDTCLKUEN_ENA)) ? 1:0;
}

/** @brief  Set the value by which the watchdog timer's clock input will be divided.
  * @param  Divider  The new clock divider.
  * @return None.
  *
  * Divider must be an integer from 0 to 255 inclusive (0 == WDT clock disabled).
  */
__INLINE static void SYSCON_SetWDTClockDivider(unsigned int Divider)
{
    lpclib_assert((Divider & ~SYSCON_WDTCLKDIV_Mask) == 0);

    SYSCON->WDTCLKDIV = Divider;
}

/** @brief  Get the watchdog timer's current clock divider.
  * @return The current WDT clock divider.
  */
__INLINE static unsigned int SYSCON_GetWDTClockDivider(void)
{
    return SYSCON->WDTCLKDIV;
}

/** @brief  Set the clock source for the CLKOUT pin.
  * @param  Source   The new CLKOUT clock source.
  * @return None.
  *
  * Call CLKOUT_EnableSourceUpdate after setting, and wait for
  *  the source to be updated (CLKOUT_SourceIsUpdated() == 1)
  */
__INLINE static void SYSCON_SetCLKOUTSource(SYSCON_CLKOUTSource_Type Source)
{
    lpclib_assert(SYSCON_IS_CLKOUT_SOURCE(Source));

    SYSCON->CLKOUTCLKSEL = Source;
}

/** @brief  Get the current CLKOUT clock source.
  * @return The current CLKOUT source.
  */
__INLINE static SYSCON_CLKOUTSource_Type SYSCON_GetCLKOUTSource(void)
{
    return SYSCON->CLKOUTCLKSEL;
}

/** @brief  Enable updating of CLKOUT's clock source.
  * @return None.
  *
  * Used after changing the CLKOUT clock source to make the change active.
  */
__INLINE static void SYSCON_EnableCLKOUTSourceUpdate(void)
{
    SYSCON->CLKOUTUEN = SYSCON_CLKOUTUEN_ENA;
    SYSCON->CLKOUTUEN = 0;
    SYSCON->CLKOUTUEN = SYSCON_CLKOUTUEN_ENA;
}

/** @brief  Test whether CLKOUT's clock source has successfully updated.
  * @return 1 if the source has updated, 0 otherwise.
  *
  * Generally will be called in a while() loop after the source has been updated
  * and CLKOUT_EnableSourceUpdate() has been called.
  */
__INLINE static unsigned int SYSCON_CLKOUTSourceIsUpdated(void)
{
    return (SYSCON->CLKOUTUEN & SYSCON_CLKOUTUEN_ENA) ? 1:0;
}

/** @brief  Set the value by which CLKOUT's input clock will be divided.
  * @param  Divider  The new clock divider.
  * @return None.
  *
  * Divider must be an integer from 0 to 255 inclusive (0 == CLKOUT clock disabled).
  */
__INLINE static void SYSCON_SetCLKOUTDivider(unsigned int Divider)
{
    lpclib_assert((Divider & ~SYSCON_CLKOUTDIV_Mask) == 0);

    SYSCON->CLKOUTDIV = Divider;
}

/** @brief  Get CLKOUT's current clock divider.
  * @return The current CLKOUT input clock divider.
  */
__INLINE static unsigned int SYSCON_GetCLKOUTDivider(void)
{
    return SYSCON->CLKOUTDIV;
}

/** @brief  Set the brownout detector's reset trigger voltage.
  * @param  Voltage   The new reset trigger voltage
  * @return None.
  */
__INLINE static void SYSCON_SetBODResetVoltage(SYSCON_BODResetVoltage_Type Voltage)
{
    lpclib_assert(SYSCON_IS_BOD_RESET_VOLTAGE(Voltage));

    SYSCON->BODCTRL = (SYSCON->BODCTRL & ~SYSCON_BODCTRL_BODRSTLEV_Mask) | Voltage;
}

/** @brief  Get the brownout detector's current reset trigger voltage.
  * @return The current reset trigger voltage.
  */
__INLINE static SYSCON_BODResetVoltage_Type SYSCON_GetBODResetVoltage(void)
{
    return SYSCON->BODCTRL & SYSCON_BODCTRL_BODRSTLEV_Mask;
}

/** @brief  Set the brownout detector's interrupt trigger voltage
  * @param  Voltage   The new interrupt trigger voltage
  * @return None.
  */
__INLINE static void SYSCON_SetBODInterruptVoltage(SYSCON_BODInterruptVoltage_Type Voltage)
{
    lpclib_assert(SYSCON_IS_BOD_INTERRUPT_VOLTAGE(Voltage));

    SYSCON->BODCTRL = (SYSCON->BODCTRL & ~SYSCON_BODCTRL_BODINTVAL_Mask) | Voltage;
}

/** @brief  Get the brownout detector's current interrupt trigger voltage.
  * @return The current interrupt trigger voltage.
  */
__INLINE static SYSCON_BODInterruptVoltage_Type SYSCON_GetBODInterruptVoltage(void)
{
    return SYSCON->BODCTRL & SYSCON_BODCTRL_BODINTVAL_Mask;
}

/** @brief  Enable resetting of the chip by the brownout detector.
  * @return None.
  */
__INLINE static void SYSCON_EnableBODChipReset(void)
{
    SYSCON->BODCTRL |= SYSCON_BODCTRL_BODRSTENA;
}

/** @brief  Disable resetting of the chip by the brownout detector.
  * @return None.
  */
__INLINE static void SYSCON_DisableBODChipReset(void)
{
    SYSCON->BODCTRL |= SYSCON_BODCTRL_BODRSTENA;
}

/** @brief  Test whether the brownout detector is configured to reset the chip on low voltage.
  * @return 1 if BOD chip reset is enabled, 0 otherwise.
  */
__INLINE static unsigned int SYSCON_BODChipResetIsEnabled(void)
{
    return (SYSCON->BODCTRL & SYSCON_BODCTRL_BODRSTENA) ? 1:0;
}

/** @brief  Set the SysTick calibration value.
  * @param  Calibration  The new SysTick calibration value (0-4095)
  * @return None.
  */
__INLINE static void SYSCON_SetSysTickCalibration(uint16_t Calibration)
{
    lpclib_assert((Calibration & ~SYSCON_SYSTCKCAL_CAL_Mask) == 0);

    SYSCON->SYSTCKCAL = Calibration;
}

/** @brief  Get the current SysTick calibration value.
  * @return The current SysTick calibration value (0-4095).
  */
__INLINE static uint16_t SYSCON_GetSysTickCalibration(void)
{
    return SYSCON->SYSTCKCAL;
}

/** @brief  Set the start logic edges for the specified inputs.
  * @param  Inputs      A bitmask of start logic inputs
  * @param  Edge        The edge setting on which to trigger start logic on those inputs
  * @return None.
  */
__INLINE static void SYSCON_SetStartLogicEdges(SYSCON_StartLogicInputs_Type Inputs,
                                               SYSCON_StartLogicEdge_Type Edge)
{
    lpclib_assert((Inputs & ~SYSCON_STARTAPRP0_Mask) == 0);
    lpclib_assert(SYSCON_IS_START_LOGIC_EDGE(Edge));

    if (Edge == SYSCON_StartLogicEdge_Falling) {
        SYSCON->STARTAPRP0 &= ~Inputs;
    } else {
        SYSCON->STARTAPRP0 |= Inputs;
    }
}

/** @brief  Get the start logic inputs configured to trigger on the given edge.
  * @return A bitmask of start logic inputs configured to trigger on the given edge type.
  */
__INLINE static SYSCON_StartLogicInputs_Type SYSCON_GetStartLogicInputsWithEdge(SYSCON_StartLogicEdge_Type Edge)
{
     lpclib_assert(SYSCON_IS_START_LOGIC_EDGE(Edge));

    if (Edge == SYSCON_StartLogicEdge_Falling) {
        return (~SYSCON->STARTAPRP0) & SYSCON_STARTAPRP0_Mask;
    } else {
        return SYSCON->STARTAPRP0;
    }
}

/** @brief  Enable start logic for the specified inputs.
  * @param  Inputs      A bitmask of start logic inputs.
  * @return None.
  */
__INLINE static void SYSCON_EnableStartLogicInputs(SYSCON_StartLogicInputs_Type Inputs)
{
    lpclib_assert((Inputs & ~SYSCON_STARTEPRP0_Mask) == 0);

    SYSCON->STARTERP0 |= Inputs;
}

/** @brief  Disable start logic for the specified inputs.
  * @param  Inputs      A bitmask of start logic inputs.
  * @return None.
  */
__INLINE static void SYSCON_DisableStartLogicInputs(SYSCON_StartLogicInputs_Type Inputs)
{
    lpclib_assert((Inputs & ~SYSCON_STARTEPRP0_Mask) == 0);

    SYSCON->STARTERP0 &= ~Inputs;
}

/** @brief  Get the currently enabled start logic inputs.
  * @return A bitmask of start logic inputs for which start logic is enabled.
  */
__INLINE static SYSCON_StartLogicInputs_Type SYSCON_GetEnabledStartLogicInputs(void)
{
    return SYSCON->STARTERP0;
}

/** @brief  Reset the specified start logic inputs.
  * @param  Inputs   A bitmask of start logic inputs to reset
  * @return None.
  *
  * Start logic inputs need to be reset each time they are used to re-enable
  * triggering.
  */
__INLINE static void SYSCON_ResetStartLogicInputs(SYSCON_StartLogicInputs_Type Inputs)
{
    lpclib_assert((Inputs & ~SYSCON_STARTRSRP0CLR_Mask) == 0);

    SYSCON->STARTRSRP0CLR |= Inputs;
}

/** @brief  Get the triggered states for all start logic inputs.
  * @return A bitmask of inputs for which start logic has been triggered.
  *
  * Start logic inputs need to be reset each time they are used to re-enable
  * triggering.
  */
__INLINE static SYSCON_StartLogicInputs_Type SYSCON_GetTriggeredStartLogicInputs(void)
{
    return SYSCON->STARTSRP0;
}

/** @brief  Initialize the system's analog power configuration
  * @return None.
  *
  * Sets reserved bits to required values.  Clears sleep config.
  */
__INLINE static void SYSCON_InitAnalogPowerLines(void)
{
    SYSCON->PDSLEEPCFG = SYSCON_PDSLEEPCFG_Required;
    SYSCON->PDAWAKECFG = (SYSCON->PDAWAKECFG & 0xff) | SYSCON_PDAWAKECFG_Required;
    SYSCON->PDRUNCFG = (SYSCON->PDRUNCFG & 0xff) | SYSCON_PDRUNCFG_Required;
}

/** @brief  Enable power to analog power blocks for the specified power mode.
  * @param  PowerMode  The power mode in which this setting should apply
  * @param  PowerLines The blocks to which power should be enabled
  * @return None.
  *
  * Valid PowerModes:
  * - SYSCON_PowerMode_Sleep
  * - SYSCON_PowerMode_Awake
  * - SYSCON_PowerMode_Run
  */
__INLINE static void SYSCON_EnableAnalogPowerLines(SYSCON_PowerMode_Type PowerMode,
                                                   SYSCON_AnalogPowerLines_Type PowerLines)
{
    lpclib_assert(SYSCON_IS_POWER_MODE(PowerMode));

    if (PowerMode == SYSCON_PowerMode_Sleep) {
        lpclib_assert((PowerLines & ~SYSCON_PDSLEEPCFG_Mask) == 0);
        SYSCON->PDSLEEPCFG = (SYSCON->PDSLEEPCFG & ~(PowerLines)) | SYSCON_PDSLEEPCFG_Required;
    } else if (PowerMode == SYSCON_PowerMode_Awake) {
        lpclib_assert((PowerLines & ~SYSCON_PDAWAKECFG_Mask) == 0);
        SYSCON->PDAWAKECFG = (SYSCON->PDAWAKECFG & ~(PowerLines)) | SYSCON_PDAWAKECFG_Required;
    } else { /* SYSCON_PowerMode_Run */
        lpclib_assert((PowerLines & ~SYSCON_PDRUNCFG_Mask) == 0);
        SYSCON->PDRUNCFG = (SYSCON->PDRUNCFG & ~((uint16_t)PowerLines)) | SYSCON_PDRUNCFG_Required;
    }
}

/** @brief  Disable power to analog power blocks for the specified power mode.
  * @param  PowerMode  The power mode in which this setting should apply
  * @param  PowerLines The analog blocks to which power should be disabled
  * @return None.
  *
  * Valid PowerModes:
  * - SYSCON_PowerMode_Sleep
  * - SYSCON_PowerMode_Awake
  * - SYSCON_PowerMode_Run
  */
__INLINE static void SYSCON_DisableAnalogPowerLines(SYSCON_PowerMode_Type PowerMode,
                                                    SYSCON_AnalogPowerLines_Type PowerLines)
{
    lpclib_assert(SYSCON_IS_POWER_MODE(PowerMode));

    if (PowerMode == SYSCON_PowerMode_Sleep) {
        lpclib_assert((PowerLines & ~SYSCON_PDSLEEPCFG_Mask) == 0);
        SYSCON->PDSLEEPCFG |= (SYSCON_PDSLEEPCFG_Required | PowerLines);
    } else if (PowerMode == SYSCON_PowerMode_Awake) {
        lpclib_assert((PowerLines & ~SYSCON_PDAWAKECFG_Mask) == 0);
        SYSCON->PDAWAKECFG |= (SYSCON_PDAWAKECFG_Required | PowerLines);
    } else { /* SYSCON_PowerMode_Run */
        lpclib_assert((PowerLines & ~SYSCON_PDRUNCFG_Mask) == 0);
        SYSCON->PDRUNCFG |= (SYSCON_PDRUNCFG_Required | PowerLines);
    }
}

/** @brief  Get Enabled Power Settings for System Analog Power Blocks
  * @param  PowerMode  The power mode to get the power settings for
  * @return Bits specifying powered analog peripherals in given mode
  */
__INLINE static SYSCON_AnalogPowerLines_Type SYSCON_GetEnabledAnalogPowerLines(SYSCON_PowerMode_Type PowerMode)
{
    if (PowerMode == SYSCON_PowerMode_Sleep) {
        return ~(SYSCON->PDSLEEPCFG & SYSCON_PDSLEEPCFG_Mask);
    } else if (PowerMode == SYSCON_PowerMode_Awake) {
        return ~(SYSCON->PDAWAKECFG & SYSCON_PDAWAKECFG_Mask);
    } else { /* SYSCON_PowerMode_Run */
        return ~(SYSCON->PDRUNCFG & SYSCON_PDRUNCFG_Mask);
    }
}

/** @brief  Get the MCU's Device ID
  * @return The device's ID.
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

