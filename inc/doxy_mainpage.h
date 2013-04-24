/**************************************************************************//**
 * @mainpage LPC11xx Device Library Version 2.0
 * @brief    Main Doxygen Documentation Page
 * @author   Tymm Twillman
 * @date     1. January 2012
 *****************************************************************************
 * @section Introduction
 *
 * This is a low-level library for using LPC11xx / LPC11Cxx / LPC11XXL
 * series microcontrollers.  It is meant to be used with the 2.x Cortex-M0
 * CMSIS files from ARM.
 *
 * @section Overview
 *
 * The library consists of a core MCU header file for low-level MCU definitions
 * (IRQ numbers, peripheral register layouts, peripheral bit definitions),
 * a system level header file + code (handling chip initialization,
 * libc startup, and MCU main clock interface), access-level header files
 * and code for interfacing with each of the LPC11xx on-chip peripherals,
 * and link scripts for linking programs for the assorted LPC11xx
 * microcontrollers.
 * @section Layout
 * <pre>
 * LPC11xx_Device_Lib_v1.2/
 *    Makefile      -- Top-level makefile for building docs or the library
 *
 *    docs/         -- Doxygen-generated documentation of the library
 *
 *    example/      -- Example firmware source for using assorted peripherals /
 *                      library features
 *
 *    inc/          -- Top-level include directory
 *      doxy_mainpage.h  -- Source of this documentation file
 *      lpc11xx/         -- Header files for lpc11xx peripherals & functions
 *        adc.h               -- Analog to Digital Converter interface
 *        crp.h               -- Code Read Protection interface
 *        ct16b.h             -- 16-bit Counter / Timer interface
 *        ct32b.h             -- 32-bit Counter / Timer interface
 *        flash.h             -- Flash Controller interface
 *        gpio.h              -- General Purpose I/O interface
 *        i2c.h               -- I2C Controller interface
 *        iap.h               -- Flash programming interface
 *        iocon.h             -- IO Configuration interface
 *        isr_vector.h        -- Interrupt Service Routine structure
 *        pmu.h               -- Power Management Unit interface
 *        ssp.h               -- Synchronous Serial Peripheral (/SPI) interface
 *        syscon.h            -- System Configuration Block interface
 *        uart.h              -- UART interface
 *        wdt.h               -- Watchdog Timer interface
 *      lpc11xx.h        -- Base header file for using lpc11xx microcontrollers
 *      lpclib_assert.h  -- Header file for "assert" debugging of the library
 *      system_lpc11xx.h -- Header file with interface to low-level chip
 *                           (power-up, clocking) functions
 *
 *    link_scripts/ -- Linker scripts
 *      (contains assorted link scripts for different CPU models)
 *
 *    lpc11xx.mk    -- Make include file for simple inclusion of the library
 *                      build into user projects
 *
 *    src/          -- 'C' source files
 *      Makefile         -- Make file for building the library objects
 *      lpc11xx_crp.c    -- Code Read Protection storage
 *      lpc11xx_crt0.c   -- CPU initialization / libc start-up code
 *      lpc11xx_iap.c    -- Flash programming functions
 *      lpc11xx_pll.c    -- PLL interface functions
 *      lpclib_assert.c  -- Assert function
 *      system_lpc11xx.c -- CMSIS-required system functions (SystemInit, SystemCoreClockUpdate)
 * </pre>
 *
 * @section Design Goals
 *
 * The library is an attempt to give a level of abstraction and ease of use
 * to the low-level hardware, while staying as light-weight as possible.
 * Efforts were made also to match CMSIS coding standards (thus the use
 * of CamelCase and excessive enums).  Some decisions were more difficult
 * than others (e.g. using uint32_t for bitmasks vs. typedefs that might provide
 * a more clear path to find the documentation for bit meanings).
 *
 *
 * @section ToDo To Do
 *
 * Across Files
 * - verify correctness of documentation, parameters.
 * - correct compilation issues
 *
 * adc.h
 * - Masks (multiple ADC inputs) abstraction ? ADC_ChannelMask_AD0 ...
 * - IT naming (IT??? Intr? Irq?)
 * - EnableInterruptMask (not channels)?
 * - Enable/Disable vs SetInterruptMask
 *
 * can.h
 * - Create
 *
 * canopen.h
 * ct16b.h
 *
 * ct32b.h
 * - Match up to ct16b.h
 *
 * flash.h
 *
 * gpio.h
 * - Masks (esp. wrt sense)... ...ForMaskedPins? ForPin? these are kinda gross.
 * - Multiple pins abstraction ?
 * - Interrupts (sense) abstraction ?
 * - Set/GetPinDirections ?
 *
 * i2c.h
 * - Monitor mode -> work on proper abstraction
 * - Make a state machine to handle I2C comms
 *
 * iap.h
 * iocon.h
 * - Capitalization stuff
 *
 * isr_vector.h
 * lpc11xx.h
 * lpclib_assert.h
 * pmu.h
 * ssp.h
 * syscon.h
 *
 * system_lpc11xx.c
 * - Set up separate PSP/MSP stack pointers???
 * system_lpc11xx.h
 *
 * uart.h
 * - IT naming (IT??? Intr? Irq?)
 *
 * power_profiles.h
 * - Complete
 *
 * wdt.h
 *
 * linker files
 * - change StartLogic refs to WAKEUP
 *****************************************************************************
 * @section License
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

/** @defgroup LPC11xx_Core LPC11xx Core Peripheral & Interrupt Definitions   */

/** @defgroup LPC11xx_System LPC11xx Microcontroller System Interface        */

/** @defgroup LPC_Peripheral_AbstractionLayer LPC Peripheral Abstraction Layer */
