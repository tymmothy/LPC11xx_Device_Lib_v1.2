/***************************************************************************//*
 * @file     system_lpc11xx.c
 * @brief    LPC11xx MCU Initialization Functions (Required for CMSIS)
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. June 2010
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

/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "lpc11xx.h"
#include "system_lpc11xx.h"

#include "lpc11xx/flash.h"
#include "lpc11xx/syscon.h"


/* File-Local Defines -------------------------------------------------------*/

/* By default the AHB divider is set to 1 (bus runs at system clock speed) */
#ifndef AHBCLKDIV_Val
# define AHBCLKDIV_Val 1
#endif

/* By default the Main clock divider is set to 1 (CPU at system clock speed) */
#ifndef MAINCLKDIV_Val
# define MAINCLKDIV_Val 1
#endif

/* If an HSE_Val (High Speed External Oscillator frequency) has been supplied,
 *   use that as the system oscillator input frequency.  Otherwise fall back
 *   to using the internal RC oscillator speed.
 */
#ifdef HSE_Val
# define MCUOSC_Val HSE_Val
#else
# define MCUOSC_Val IRC_Val
#endif


/* Static Variables ---------------------------------------------------------*/

/*! @brief Watchdog Oscillator Frequency Table */
static const uint32_t WDTOscFreqs[] = {
          0,  500000,  800000, 1100000,
    1400000, 1600000, 1800000, 2000000,
    2200000, 2400000, 2600000, 2700000,
    2900000, 3100000, 3200000, 3400000
};


/* Global Variables ---------------------------------------------------------*/

/*! @brief System Core Clock Frequency.  Required for CMSIS compliance. */
uint32_t SystemCoreClock = IRC_Val;

/*! @brief System High-Speed Bus Frequency */
uint32_t SystemAHBClock = IRC_Val;


/* Functions ----------------------------------------------------------------*/

/* System PLL Initialization is only compiled in if the expected CPU speed
 * differs from the input clock rate.  You can always use the SYSCON
 * PLL interface to change later.
 */

#if MCUOSC_Val != F_CPU
/** @brief Initialize system PLL
  *
  * @return None.
  *
  * Sets up the main MCU PLL.
  */
static inline void SysPLLInit(void)
{
    /* Set the PLL to use the configured clock source */
#ifdef HSE_Val
    SYSCON_SetSysPLLClockSource(SYSCON_SysPLLClockSource_SysOsc);
#else
    SYSCON_SetSysPLLClockSource(SYSCON_SysPLLClockSource_IRC);
#endif
    SYSCON_EnableSysPLLClockSourceUpdate();
    while (!SYSCON_SysPLLClockSourceIsUpdated());

    SYSCON_SetSysPLLMVal((F_CPU / MCUOSC_Val) - 1);

#if (F_CPU < 19500000)
    SYSCON_SetSysPLLPVal(SYSCON_SysPLLPVal_8);
#elif (F_CPU < 39000000)
    SYSCON_SetSysPLLPVal(SYSCON_SysPLLPVal_4);
#elif (F_CPU < 78000000)
    SYSCON_SetSysPLLPVal(SYSCON_SysPLLPVal_2);
#else
    SYSCON_SetSysPLLPVal(SYSCON_SysPLLPVal_1);
#endif

    /* Make sure the PLL is powered */
    SYSCON_EnableAnalogPowerLinesForMode(SYSCON_PowerMode_Run, SYSCON_AnalogPowerLine_SysPLL);
    while (!SYSCON_SysPLLIsLocked());

    /* Set the Main Clock to use the System PLL's Output as its source */
    SYSCON_SetMainClockSource(SYSCON_MainClockSource_SysPLLOut);
    SYSCON_EnableMainClockSourceUpdate();
    while (!SYSCON_MainClockSourceIsUpdated());

}
#endif


/** @brief Convert an (opaque) WDT Oscillator frequency type to a Hz value.
  * @param  [in]  oscVal   The Oscillator frequency type (opaque) to convert
  *
  * @return A numeric value that is the actual WDT oscillator speed.
  *
  * Sets up main microcontroller PLL
  */
uint32_t WDTOscValToFreq(SYSCON_WDTOscFreq_Type oscVal)
{
    return WDTOscFreqs[oscVal];
}


/** @brief Set the system & AHB clock variables based on current clock configs
  *
  * @return None.
  *
  * Sets SystemCoreClock, SystemAHBClock based on values in clock
  * configuration registers.
  *
  * REQUIRED for CMSIS compliance.
  */
void SystemCoreClockUpdate(void)
{
    uint8_t tmp;
    uint32_t clockSpeed;


    tmp = SYSCON_GetMainClockSource();

    if (tmp == SYSCON_MainClockSource_IRC) {
        clockSpeed = IRC_Val;
    } else if (tmp == SYSCON_MainClockSource_SysPLLIn) {
        clockSpeed = IRC_Val;
#ifdef HSE_Val
        tmp = SYSCON_GetSysPLLClockSource();
        if (tmp == SYSCON_SysPLLClockSource_SysOsc) {
            clockSpeed = HSE_Val;
        }
#endif
    } else if (tmp == SYSCON_MainClockSource_WDTOsc) {
        tmp = SYSCON_GetWDTOscFreq();
        clockSpeed = WDTOscValToFreq(tmp);
        clockSpeed /= (SYSCON_GetWDTOscDivider() + 1) * 2;
    } else { /* SYSCON_MainClockSource_SysPLLOut */
        tmp = SYSCON_GetSysPLLClockSource();
        if (tmp == SYSCON_SysPLLClockSource_IRC) {
            clockSpeed = IRC_Val * SYSCON_GetSysPLLMVal();
#ifdef HSE_Val
        } else { /* SYSCON_SysPLLClockSource_SysOsc*/
            clockSpeed = HSE_Val * SYSCON_GetSysPLLMVal();
#endif
        }
    }

    SystemCoreClock = clockSpeed / SYSCON_GetAHBClockDivider();

    if (SYSCON_GetAHBClockDivider() == 0) {
        SystemAHBClock= 0;
    } else {
        SystemAHBClock = SystemCoreClock / SYSCON_GetAHBClockDivider();
    }
}


/** @brief Initialize the system.
  *
  * @return None.
  *
  * Sets up necessary system clocks and basic hardware.
  *
  * REQUIRED for CMSIS compliance.
  */
void SystemInit(void) __attribute__((weak));
void SystemInit(void)
{
    int i;


    /* Initialize analog power configuration modes (Set to sane values) */
    SYSCON_InitAnalogPowerLines();

#if defined(__DEBUG_RAM)
    SYSMEMREMAP_Map(SYSMEMREMAP_Remap_RAM);
#elif defined(__DEBUG_FLASH)
    SYSMEMREMAP_Map(SYSMEMREMAP_Remap_FLASH);
#endif

    /* If using an external crystal / oscillator, enable the system
     * oscillator, set the frequency range & connect power
     */

#ifdef HSE_Val
    SYSCON_DisableSysOscBypass();
# if (HSE_Val) < 20000000
    SYSCON_SetSysOscFreqRange(SYSCON_SysOscFreqRange_1_20);
# else
    SYSCON_SetSysOscFreqRange(SYSCON_SysOscFreqRange_15_25);
# endif
    SYSCON_EnableAnalogPowerLinesForMode(SYSCON_PowerMode_Run, SYSCON_AnalogPowerLine_SysOsc);
#endif

    /* Brief delay to let the system stabilize a little */
    for (i = 0; i < 1000; i++) {
        __asm__ __volatile__(" nop\r\n");
    }

    /* Make sure we don't exceed flash specs when changing clock speed */
    FLASH_SetWaitStates(2);

#if MCUOSC_Val != F_CPU
    /* Configure & connect the system PLL, if desired */
    SysPLLInit();
#endif

    /* Update system frequency variables */
    SystemCoreClock = F_CPU;

    /* Set flash latency to maximum for running */
    if (SystemCoreClock < 20000000UL) {
        FLASH_SetWaitStates(0);
    } else if (SystemCoreClock < 40000000UL) {
        FLASH_SetWaitStates(1);
    }

    SYSCON_SetAHBClockDivider(AHBCLKDIV_Val);

    /* Update AHB frequency variable */
    SystemAHBClock = F_CPU / AHBCLKDIV_Val;

    /* Enable IOCON system clock */
    SYSCON_EnableAHBClockLines(SYSCON_AHBClockLine_IOCON);
}

