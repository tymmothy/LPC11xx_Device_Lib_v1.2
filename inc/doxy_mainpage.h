/**************************************************************************//**
 * @mainpage LPC11xx Device Library Version 2.0
 * @brief    Main Doxygen Documentation Page
 * @author   Tymm Twillman
 * @date     1. January 2012
 *****************************************************************************
 * @section intro Introduction
 *
 * This is a low-level library for using LPC11xx / LPC11Cxx / LPC11XXL
 * series microcontrollers.  It is meant to be used with the 2.x Cortex-M0
 * CMSIS files from ARM.
 *
 * The library consists of a core MCU header file for low-level MCU definitions
 * (IRQ numbers, peripheral register layouts, peripheral bit definitions),
 * a system level header file + code (handling chip initialization,
 * libc startup, and MCU main clock interface), access-level header files
 * and code for interfacing with each of the LPC11xx on-chip peripherals,
 * and link scripts for linking programs for the assorted LPC11xx
 * microcontrollers.
 *
 * @section todo To Do
 *
 * Across Files
 * - doxygen prettification, improve doxygen layout
 * - find best sizes of integers for passing small bitmasks & ints around
 * - correct compilation issues
 *
 * adc.h
 * - Masks (multiple ADC inputs) abstraction ?
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
 * isr_vector.h
 *
 * lpc11xx.h
 * - Wakeup / Start IRQ numbering
 *
 * lpclib_assert.h
 * pmu.h
 * ssp.h
 *
 * syscon.h
 * - Analog power control bits (type naming, function naming, better docs)
 *
 * system_lpc11xx.c
 * - Set up separate PSP/MSP stack pointers???
 * system_lpc11xx.h
 *
 * uart.h
 * power_profiles.h
 *
 * wdt.h
 *****************************************************************************
 * @section License License
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
