/**************************************************************************//**
 * @mainpage LPC11xx Device Library Version 2.0
 * @author   Tymm Twillman
 * @date     9. February 2012
 *
 * @section intro Introduction
 *
 * This is a low-level library for using LPC11xx series microcontrollers.  It
 * is meant to be used with the 2.x Cortex-M0 CMSIS files from ARM.
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
 * - Across Files
 * -- doxygen prettification, improve doxygen layout
 * -- find best sizes of integers for passing small bitmasks & ints around
 * -- correct compilation issues
 *
 * - system_lpc11xx.c
 * -- Set up separate PSP/MSP stack pointers???
 *
 * - syscon.h
 * -- Analog power control bits (type naming, function naming, better docs)
 *
 *
 *****************************************************************************/

/** @defgroup LPC11xx_Device NXP LPC11xx Peripheral & Interrupt Definitions  */

/** @defgroup LPC11xx_System NXP LPC11xx Microcontroller System Interface    */

/** @defgroup LPC_Peripheral_Access_Layer LPC Peripheral Access Layer        */
