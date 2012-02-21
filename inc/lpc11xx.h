/**************************************************************************//**
 * @file     lpc11xx.h
 * @brief    Base header file for using NXP LPC11xx/LPC11Cxx microcontrollers.
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. January 2012
 ******************************************************************************
 * @section Overview
 * This file contains peripheral register & interrupt definitions for NXP
 * LPC11xx microcontrollers.
 *
 * Microcontrollers handled by this file:
 * - LPC1102
 * - LPC1111  (LPC1111/x01)
 * - LPC1111L (LPC1111/x02)
 * - LPC1112  (LPC1112/x01)
 * - LPC1112L (LPC1112/x02)
 * - LPC1113  (LPC1113/x01)
 * - LPC1113L (LPC1113/x02)
 * - LPC1114  (LPC1114/x01)
 * - LPC1114L (LPC1114/x02)
 * - LPC11C14
 * - LPC11C24
 *
 * The structure of this file:
 * - Basic core-level LPC11xx definitions (IRC clock, Cortex-M0 configuration)
 * - Interrupt numbering
 * - Importing core_cm0.h from ARM CMSIS package for Cortex-M0 interface
 * - Peripheral register types
 * - Peripheral register bit definitions
 * - Peripheral memory locations
 * - Peripheral instances
 *
 * @note
 * Not all peripherals are brought out on all chips; those with lower pin
 * counts for example may not have SSP1, and not all GPIO lines will be
 * available on chips with fewer pins.
 *
 * @note
 * For proper exporting of the full & correct peripheral interfaces:
 * - Define the symbol LPC11XX for base series chips (LPC1111/2/3/4).
 * - Define the symbol LPC11XXL for enhanced chips (power profiles, etc).
 * - Define the symbole LPC11CXX for CAN enabled chips.
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

#ifndef LPC11XX_H_
#define LPC11XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @addtogroup LPC11xx_Core LPC11xx Core Peripheral & Interrupt Definitions
  * @{
  */

/* Defines --------------------------------------------------------------------------------------*/

#define IRC_Val                 (12000000UL)    /*!< Speed of internal RC Oscillator (12 Mhz)    */


/** @defgroup LPC11xx_Cortex_M0_Configuration LPC11xx Cortex-M0 MCU Configuration
  * @{
  */

#define __NVIC_PRIO_BITS        (3)             /*!< Number of Priority Bits used by the NVIC    */
#define __Vendor_SysTickConfig  (0)             /*!< No vendor-specific SysTick configuration    */
#define __CM0_REV               (0x0000)        /*!< Cortex-M0 Core is Revision 0                */

/** @} */


/* IRQ Numbers ----------------------------------------------------------------------------------*/

/** @defgroup LPC11xx_IRQn LPC11xx IRQ Numbers
  * @{
  */

/*! @brief LPC11xx IRQ Numbers */
typedef enum {

/***** Cortex-M0 Exceptions & Interrupt Numbers ******************************/

/*  NMI_IRQn                 = -14,     Unimplemented on LPC11xx             */

    HardFault_IRQn           = -13,    /*!<  3 Cortex Hard Fault Interrupt   */
    SVC_IRQn                 = -5,     /*!< 11 Cortex-M0 SVC Interrupt       */
    PendSV_IRQn              = -2,     /*!< 14 Cortex-M0 Pending SV Int      */
    SysTick_IRQn             = -1,     /*!< 15 Cortex-M0 System Tick Int     */

/***** LPC11xx Specific Interrupt Numbers ************************************/

    START0_IRQn         = 0,      /*!< StartLogic0 (GPIO0.0) Interrupt  */
    StartLogic1_IRQn         = 1,      /*!< StartLogic1 (GPIO0.1) Interrupt  */
    StartLogic2_IRQn         = 2,      /*!< StartLogic2 (GPIO0.2) Interrupt  */
    StartLogic3_IRQn         = 3,      /*!< StartLogic3 (GPIO0.3) Interrupt  */
    StartLogic4_IRQn         = 4,      /*!< StartLogic4 (GPIO0.4) Interrupt  */
    StartLogic5_IRQn         = 5,      /*!< StartLogic5 (GPIO0.5) Interrupt  */
    StartLogic6_IRQn         = 6,      /*!< StartLogic6 (GPIO0.6) Interrupt  */
    StartLogic7_IRQn         = 7,      /*!< StartLogic7 (GPIO0.7) Interrupt  */
    StartLogic8_IRQn         = 8,      /*!< StartLogic8 (GPIO0.8) Interrupt  */
    StartLogic9_IRQn         = 9,      /*!< StartLogic9 (GPIO0.9) Interrupt  */
    StartLogic10_IRQn        = 10,     /*!< StartLogic10 (GPIO0.10) Int.     */
    StartLogic11_IRQn        = 11,     /*!< StartLogic11 (GPIO0.11) Int.     */
    StartLogic12_IRQn        = 12,     /*!< StartLogic12 (GPIO1.0) Interrupt */
    SSP1_IRQn                = 14,     /*!< SSP1 Peripheral Interrupt        */
    I2C0_IRQn                = 15,     /*!< I2C0 Peripheral Interrupt        */
    CT16B0_IRQn              = 16,     /*!< 16-bit Counter/Timer 0 Interrupt */
    CT16B1_IRQn              = 17,     /*!< 16-bit Counter/Timer 1 Interrupt */
    CT32B0_IRQn              = 18,     /*!< 32-bit Counter/Timer 0 Interrupt */
    CT32B1_IRQn              = 19,     /*!< 32-bit Counter/Timer 1 Interrupt */
    SSP0_IRQn                = 20,     /*!< SSP0 Peripheral Interrupt        */
    UART0_IRQn               = 21,     /*!< UART0 Peripheral Interrupt       */
    ADC0_IRQn                = 24,     /*!< A to D Converter 0 Interrupt     */
    WDT_IRQn                 = 25,     /*!< Watchdog Timer Interrupt         */
    BOD_IRQn                 = 26,     /*!< Brownout Detection Interrupt     */
    GPIO3_IRQn               = 28,     /*!< GPIO Port 3 Interrupt            */
    GPIO2_IRQn               = 29,     /*!< GPIO Port 2 Interrupt            */
    GPIO1_IRQn               = 30,     /*!< GPIO Port 1 Interrupt            */
    GPIO0_IRQn               = 31,     /*!< GPIO Port 0 Interrupt            */
} IRQn_Type;

/** @} */


/* Include for Core Cortex-M0 types, defines, etc. ----------------------------------------------*/

#include "core_cm0.h"


/**
  * @defgroup LPC11xx_Peripherals LPC11xx Peripherals
  * @{
  */

/* Peripheral Registers -------------------------------------------------------------------------*/

/** @defgroup I2C I2C: Inter-Integrated Circuit (Two Wire Interface)
  * @{
  */

/*! @brief I2C Register Map */
typedef struct {
    __IO    uint32_t    CONSET;        /*!< Offset: 0x000  Control Set Register                  */
    __I     uint32_t    STAT;          /*!< Offset: 0x004  Status Register                       */
    __IO    uint32_t    DAT;           /*!< Offset: 0x008  Data Register                         */
    __IO    uint32_t    ADR0;          /*!< Offset: 0x00c  Address Register 0                    */
    __IO    uint32_t    SCLH;          /*!< Offset: 0x010  SCL Duty Cycle High                   */
    __IO    uint32_t    SCLL;          /*!< Offset: 0x014  SCL Duty Cycle Low                    */
    __O     uint32_t    CONCLR;        /*!< Offset: 0x018  Control Clear Register                */
    __IO    uint32_t    MMCTRL;        /*!< Offset: 0x01c  Monitor Mode Control                  */
    __IO    uint32_t    ADR1;          /*!< Offset: 0x020  Address Register 1                    */
    __IO    uint32_t    ADR2;          /*!< Offset: 0x024  Address Register 2                    */
    __IO    uint32_t    ADR3;          /*!< Offset: 0x028  Address Register 3                    */
    __I     uint32_t    DATA_BUFFER;   /*!< Offset: 0x02c  Data Buffer Register                  */
    __IO    uint32_t    MASK0;         /*!< Offset: 0x030  Slave Address Mask Register 0         */
    __IO    uint32_t    MASK1;         /*!< Offset: 0x034  Slave Address Mask Register 1         */
    __IO    uint32_t    MASK2;         /*!< Offset: 0x038  Slave Address Mask Register 2         */
    __IO    uint32_t    MASK3;         /*!< Offset: 0x03c  Slave Address Mask Register 3         */
} I2C_Type;

/** @} */

/** @defgroup WDT WDT: Watchdog Timer
  * @{
  */

/*! @brief Watchdog Timer Register Map (LPC11XXL)*/
typedef struct {
    __IO    uint32_t    MOD;           /*!< Offset: 0x000  Watchdog Mode Register                */
    __IO    uint32_t    TC;            /*!< Offset: 0x004  Timer Constant Register               */
    __O     uint32_t    FEED;          /*!< Offset: 0x008  Feed Sequence Register                */
    __I     uint32_t    TV;            /*!< Offset: 0x00c  Timer Value Register                  */

#if defined(LPC11XXL)  /* L-Series parts have windowed WDT with the following added registers    */
            uint32_t      Reserved4[1];                                          /*!< (Reserved) */
    __IO    uint32_t    WARNINT;       /*!< Offset: 0x014  Watchdog Warn Interrupt Compare Value */
    __IO    uint32_t    WINDOW;        /*!< Offset: 0x018  Timer Window Register                 */
#endif

} WDT_Type;

/** @} */

/** @defgroup UART UART: Universal Asynchronous Receiver Transmitter
  * @{
  */

/*! @brief UART Register Map */
typedef struct {
    union {
        __I  uint32_t   RBR;           /*!< Offset: 0x000  Receive Buffer Register   (DLAB=0)    */
        __O  uint32_t   THR;           /*!< Offset: 0x000  Transmit Holding Register (DLAB=0)    */
        __IO uint32_t   DLL;           /*!< Offset: 0x000  Divisor Latch LSB         (DLAB=1)    */
    };

    union {
        __IO uint32_t   DLM;           /*!< Offset: 0x004  Divisor Latch MSB         (DLAB=1)    */
        __IO uint32_t   IER;           /*!< Offset: 0x004  Interrupt Enable Register (DLAB=0)    */
    };

    union {
        __I uint32_t    IIR;           /*!< Offset: 0x008  Interrupt ID Register                 */
        __O uint32_t    FCR;           /*!< Offset: 0x008  FIFO Control Register                 */
    };

    __IO    uint32_t    LCR;           /*!< Offset: 0x00c  Line Control Register                 */
    __IO    uint32_t    MCR;           /*!< Offset: 0x010  Modem Control Register                */
    __I     uint32_t    LSR;           /*!< Offset: 0x014  Line Status Register                  */
    __I     uint32_t    MSR;           /*!< Offset: 0x018  Modem Status Register                 */
    __IO    uint32_t    SCR;           /*!< Offset: 0x01C  Scratch Pad Register                  */
    __IO    uint32_t    ACR;           /*!< Offset: 0x020  Auto-Baud Control Register            */
            uint32_t      Reserved9[1];                                          /*!< (Reserved) */
    __IO    uint32_t    FDR;           /*!< Offset: 0x028  Fractional Divider Register           */
            uint32_t      Reserved11[1];                                         /*!< (Reserved) */
    __IO    uint32_t    TER;           /*!< Offset: 0x030  Transmit Enable Register              */
            uint32_t      Reserved13_18[6];                                      /*!< (Reserved) */
    __IO    uint32_t    RS485CTRL;     /*!< Offset: 0x04c  RS485 Mode Control                    */
    __IO    uint32_t    ADRMATCH;      /*!< Offset: 0x050  RS485 Address Match                   */
    __IO    uint32_t    RS485DLY;      /*!< Offset: 0x054  RS485 Direction Control Delay         */
    __I     uint32_t    FIFOLVL;       /*!< Offset: 0x058  FIFO Level Register (?)               */
} UART_Type;

/** @} */

/** @defgroup ADC ADC: Analog to Digital Converter
  * @{
  */

/*! @brief ADC Register Map */
typedef struct {
    __IO    uint32_t    CR;            /*!< Offset: 0x000  A/D Control Register                  */
    __IO    uint32_t    GDR;           /*!< Offset: 0x004  A/D Global Data Register              */
    uint32_t              Reserved2[1];                                          /*!< (Reserved) */
    __IO    uint32_t    INTEN;         /*!< Offset: 0x00c  Interrupt Enable Register             */
    __IO    uint32_t    DR0;           /*!< Offset: 0x010  Channel 0 Data Register               */
    __IO    uint32_t    DR1;           /*!< Offset: 0x014  Channel 1 Data Register               */
    __IO    uint32_t    DR2;           /*!< Offset: 0x018  Channel 2 Data Register               */
    __IO    uint32_t    DR3;           /*!< Offset: 0x01c  Channel 3 Data Register               */
    __IO    uint32_t    DR4;           /*!< Offset: 0x020  Channel 4 Data Register               */
    __IO    uint32_t    DR5;           /*!< Offset: 0x024  Channel 5 Data Register               */
    __IO    uint32_t    DR6;           /*!< Offset: 0x028  Channel 6 Data Register               */
    __IO    uint32_t    DR7;           /*!< Offset: 0x02c  Channel 7 Data Register               */
    __I     uint32_t    STAT;          /*!< Offset: 0x030  Status Register                       */
} ADC_Type;

/** @} */

/** @defgroup PMU PMU: Power Management Unit
  * @{
  */

/*! @brief PMU Register Map */
typedef struct {
    __IO    uint32_t    PCON;          /*!< Offset: 0x000  Power Control Register                */
    __IO    uint32_t    GPREG0;        /*!< Offset: 0x004  General Purpose Register 0            */
    __IO    uint32_t    GPREG1;        /*!< Offset: 0x008  General Purpose Register 1            */
    __IO    uint32_t    GPREG2;        /*!< Offset: 0x00c  General Purpose Register 2            */
    __IO    uint32_t    GPREG3;        /*!< Offset: 0x010  General Purpose Register 3            */
    __IO    uint32_t    GPREG4;        /*!< Offset: 0x014  GP Register 4 / Wakeup Hysteresis     */
} PMU_Type;

/** @} */

/** @defgroup FLASH FLASH: Flash Memory Configuration Block
  * @{
  */

/*! @brief FLASH Register Map */
typedef struct {
            uint32_t      Reserved0_3[4];                                        /*!< (Reserved) */
    __IO    uint32_t    FLASHCFG;      /*!< Offset: 0x010  Flash Latency Configuration           */
            uint32_t      Reserved5_7[3];                                        /*!< (Reserved) */
    __IO    uint32_t    FMSSTART;      /*!< Offset: 0x020  Flash Signature Start Address         */
    __IO    uint32_t    FMSEND;        /*!< Offset: 0x024  Flash Signature Stop Address          */
            uint32_t      Reserved10[1];                                         /*!< (Reserved) */
    __I     uint32_t    FMSW0;         /*!< Offset: 0x02c  Flash Signature Word 0                */
    __I     uint32_t    FMSW1;         /*!< Offset: 0x030  Flash Signature Word 1                */
    __I     uint32_t    FMSW2;         /*!< Offset: 0x034  Flash Signature Word 2                */
    __I     uint32_t    FMSW3;         /*!< Offset: 0x038  Flash Signature Word 3                */
            uint32_t      Reserved15_1015[1001];                                 /*!< (Reserved) */
    __I     uint32_t    FMSTAT;        /*!< Offset: 0xfe0  Flash Signature Generation Status Reg */
            uint32_t      Reserved1017[1];                                       /*!< (Reserved) */
    __O     uint32_t    FMSTATCLR;     /*!< Offset: 0xfe8  Flash Signature Gen Status Clear Reg  */
} FLASH_Type;

/** @} */

/** @defgroup SSP SSP: Synchronous Serial Peripheral
  * @{
  */

/*! @brief SSP Register Map */
typedef struct {
    __IO    uint32_t    CR0;           /*!< Offset: 0x000  Control Register 0                    */
    __IO    uint32_t    CR1;           /*!< Offset: 0x004  Control Register 1                    */
    __IO    uint32_t    DR;            /*!< Offset: 0x008  Data Register                         */
    __I     uint32_t    SR;            /*!< Offset: 0x00c  Status Register                       */
    __IO    uint32_t    CPSR;          /*!< Offset: 0x010  Clock Prescale Register               */
    __IO    uint32_t    IMSC;          /*!< Offset: 0x014  Interrupt Mask Set/Clear              */
    __IO    uint32_t    RIS;           /*!< Offset: 0x018  Raw Interrupt Status                  */
    __IO    uint32_t    MIS;           /*!< Offset: 0x01c  Masked Interrupt Status               */
    __IO    uint32_t    ICR;           /*!< Offset: 0x020  Interrupt Clear Register              */
} SSP_Type;

/** @} */

/** @defgroup    IOCON IOCON: IO Configuration Block
  * @{
  */

/*! @brief IOCON Register Map */
typedef struct {
    __IO    uint32_t    PIO2_6;        /*!< Offset: 0x000  PIO2_6 Configuration                  */
            uint32_t      Reserved1[1];                                          /*!< (Reserved) */
    __IO    uint32_t    PIO2_0;        /*!< Offset: 0x008  PIO2_0 Configuration                  */
    __IO    uint32_t    RESET_PIO0_0;  /*!< Offset: 0x00c  PIO0_0 Configuration                  */
    __IO    uint32_t    PIO0_1;        /*!< Offset: 0x010  PIO0_1 Configuration                  */
    __IO    uint32_t    PIO1_8;        /*!< Offset: 0x014  PIO1_8 Configuration                  */
            uint32_t      Reserved6[1];                                          /*!< (Reserved) */
    __IO    uint32_t    PIO0_2;        /*!< Offset: 0x018  PIO0_2 Configuration                  */
    __IO    uint32_t    PIO2_7;        /*!< Offset: 0x01c  PIO2_7 Configuration                  */
    __IO    uint32_t    PIO2_8;        /*!< Offset: 0x020  PIO2_8 Configuration                  */
    __IO    uint32_t    PIO2_1;        /*!< Offset: 0x024  PIO2_1 Configuration                  */
    __IO    uint32_t    PIO0_3;        /*!< Offset: 0x028  PIO0_3 Configuration                  */
    __IO    uint32_t    PIO0_4;        /*!< Offset: 0x02c  PIO0_4 Configuration                  */
    __IO    uint32_t    PIO0_5;        /*!< Offset: 0x030  PIO0_5 Configuration                  */
    __IO    uint32_t    PIO1_9;        /*!< Offset: 0x034  PIO1_9 Configuration                  */
    __IO    uint32_t    PIO3_4;        /*!< Offset: 0x038  PIO3_4 Configuration                  */
    __IO    uint32_t    PIO2_4;        /*!< Offset: 0x03c  PIO2_4 Configuration                  */
    __IO    uint32_t    PIO2_5;        /*!< Offset: 0x040  PIO2_5 Configuration                  */
    __IO    uint32_t    PIO3_5;        /*!< Offset: 0x044  PIO3_5 Configuration                  */
    __IO    uint32_t    PIO0_6;        /*!< Offset: 0x048  PIO0_6 Configuration                  */
    __IO    uint32_t    PIO0_7;        /*!< Offset: 0x04c  PIO0_7 Configuration                  */
    __IO    uint32_t    PIO2_9;        /*!< Offset: 0x050  PIO2_9 Configuration                  */
    __IO    uint32_t    PIO2_10;       /*!< Offset: 0x054  PIO2_10Configuration                  */
    __IO    uint32_t    PIO2_2;        /*!< Offset: 0x058  PIO2_2 Configuration                  */
    __IO    uint32_t    PIO0_8;        /*!< Offset: 0x05c  PIO0_8 Configuration                  */
    __IO    uint32_t    PIO0_9;        /*!< Offset: 0x060  PIO0_9 Configuration                  */
    __IO    uint32_t    SWCLK_PIO0_10; /*!< Offset: 0x064  PIO0_10Configuration                  */
    __IO    uint32_t    PIO1_10;       /*!< Offset: 0x068  PIO1_10Configuration                  */
    __IO    uint32_t    PIO2_11;       /*!< Offset: 0x06c  PIO2_11Configuration                  */
    __IO    uint32_t    R_PIO0_11;     /*!< Offset: 0x070  PIO0_11Configuration                  */
    __IO    uint32_t    R_PIO1_0;      /*!< Offset: 0x074  PIO1_0 Configuration                  */
    __IO    uint32_t    R_PIO1_1;      /*!< Offset: 0x078  PIO1_1 Configuration                  */
    __IO    uint32_t    R_PIO1_2;      /*!< Offset: 0x07c  PIO1_2 Configuration                  */
    __IO    uint32_t    PIO3_0;        /*!< Offset: 0x080  PIO3_0 Configuration                  */
    __IO    uint32_t    PIO3_1;        /*!< Offset: 0x084  PIO3_1 Configuration                  */
    __IO    uint32_t    PIO2_3;        /*!< Offset: 0x088  PIO2_3 Configuration                  */
    __IO    uint32_t    SWDIO_PIO1_3;  /*!< Offset: 0x08c  PIO1_3 Configuration                  */
    __IO    uint32_t    PIO1_4;        /*!< Offset: 0x090  PIO1_4 Configuration                  */
    __IO    uint32_t    PIO1_11;       /*!< Offset: 0x094  PIO1_11Configuration                  */
    __IO    uint32_t    PIO3_2;        /*!< Offset: 0x098  PIO3_2 Configuration                  */
    __IO    uint32_t    PIO1_5;        /*!< Offset: 0x09c  PIO1_5 Configuration                  */
    __IO    uint32_t    PIO1_6;        /*!< Offset: 0x0a0  PIO1_6 Configuration                  */
    __IO    uint32_t    PIO1_7;        /*!< Offset: 0x0a4  PIO1_7 Configuration                  */
    __IO    uint32_t    PIO3_3;        /*!< Offset: 0x0a8  PIO3_3 Configuration                  */
    __IO    uint32_t    SCK0_LOC;      /*!< Offset: 0x0ac  SSP 0 SCK Location                    */
    __IO    uint32_t    DSR0_LOC;      /*!< Offset: 0x0b0  UART 0 DSR Location                   */
    __IO    uint32_t    DCD0_LOC;      /*!< Offset: 0x0b4  UART 0 DCD Location                   */
    __IO    uint32_t    RI0_LOC;       /*!< Offset: 0x0b8  UART 0 RI Location                    */
} IOCON_Type;

/** @} */

/** @defgroup SYSCON SYSCON: System Configuration Block
  * @{
  */

/*! @brief SYSCON Register Map */
typedef struct {
    __IO    uint32_t    SYSMEMREMAP;   /*!< Offset: 0x000  System Memory Remap Control           */
    __IO    uint32_t    PRESETCTRL;    /*!< Offset: 0x004  Peripheral Reset Control              */
    __IO    uint32_t    SYSPLLCTRL;    /*!< Offset: 0x008  System PLL Control                    */
    __I     uint32_t    SYSPLLSTAT;    /*!< Offset: 0x00c  System PLL Status                     */
            uint32_t      Reserved4_7[4];                                        /*!< (Reserved) */
    __IO    uint32_t    SYSOSCCTRL;    /*!< Offset: 0x020  System Oscillator Control             */
    __IO    uint32_t    WDTOSCCTRL;    /*!< Offset: 0x024  Watchdog Timer Oscillator Control     */
    __IO    uint32_t    IRCCTRL;       /*!< Offset: 0x028  Internal RC Clock Trim                */
            uint32_t      Reserved11[1];                                         /*!< (Reserved) */
    __IO    uint32_t    SYSRESSTAT;    /*!< Offset: 0x030  System Reset Source Identification    */
            uint32_t      Reserved13_15[3];                                      /*!< (Reserved) */
    __IO    uint32_t    SYSPLLCLKSEL;  /*!< Offset: 0x040  System PLL Clock Source               */
    __IO    uint32_t    SYSPLLCLKUEN;  /*!< Offset: 0x044  System PLL Clock Enable               */
            uint32_t      Reserved18_27[10];                                     /*!< (Reserved) */
   __IO     uint32_t    MAINCLKSEL;    /*!< Offset: 0x070  Main Clock Source                     */
   __IO     uint32_t    MAINCLKUEN;    /*!< Offset: 0x074  Main Clock Enable                     */
   __IO     uint32_t    SYSAHBCLKDIV;  /*!< Offset: 0x078  AHB Clock Divider                     */
            uint32_t      Reserved31[1];                                         /*!< (Reserved) */
   __IO     uint32_t    SYSAHBCLKCTRL; /*!< Offset: 0x080  AHB Clock Control                     */
            uint32_t      Reserved33_36[4];                                      /*!< (Reserved) */
   __IO     uint32_t    SSP0CLKDIV;    /*!< Offset: 0x094  SSP0 Clock Divider                    */
   __IO     uint32_t    UART0CLKDIV;   /*!< Offset: 0x098  UART0 Clock Divider                   */
   __IO     uint32_t    SSP1CLKDIV;    /*!< Offset: 0x09c  SSP1 Clock Divider                    */
            uint32_t      Reserved40_51[12];                                     /*!< (Reserved) */
   __IO     uint32_t    WDTCLKSEL;     /*!< Offset: 0x0d0  Watchdog Clock Source                 */
   __IO     uint32_t    WDTCLKUEN;     /*!< Offset: 0x0d4  Watchdog Clock Enable                 */
   __IO     uint32_t    WDTCLKDIV;     /*!< Offset: 0x0d8  Watchdog Clock Divider                */
            uint32_t      Reserved55[1];                                         /*!< (Reserved) */
   __IO     uint32_t    CLKOUTCLKSEL;  /*!< Offset: 0x0e0  CLKOUT Clock Select                   */
   __IO     uint32_t    CLKOUTUEN;     /*!< Offset: 0x0e4  CLKOUT Enable                         */
   __IO     uint32_t    CLKOUTDIV;     /*!< Offset: 0x0e8  CLKOUT Divider                        */
            uint32_t      Reserved59_63[5];                                      /*!< (Reserved) */
   __I      uint32_t    PIOPORCAP0;    /*!< Offset: 0x100  Power On Reset Captured Pin Status 0  */
   __I      uint32_t    PIOPORCAP1;    /*!< Offset: 0x104  Power On Reset Captured Pin Status 1  */
            uint32_t      Reserved66_83[18];                                     /*!< (Reserved) */
   __IO     uint32_t    BODCTRL;       /*!< Offset: 0x150  Brownout Detetector Control           */
   __IO     uint32_t    SYSTCKCAL;     /*!< Offset: 0x154  SysTick Calibration Value             */
            uint32_t      Reserved86_127[42];                                    /*!< (Reserved) */
   __IO     uint32_t    STARTAPRP0;    /*!< Offset: 0x200  Start Logic Edges                     */
   __IO     uint32_t    STARTERP0;     /*!< Offset: 0x204  Start Logic Enables                   */
   __IO     uint32_t    STARTRSRP0CLR; /*!< Offset: 0x208  Start Logic Clear                     */
   __IO     uint32_t    STARTSRP0;     /*!< Offset: 0x20c  Start Logic Status                    */
            uint32_t      Reserved132_139[8];                                    /*!< (Reserved) */
   __IO     uint32_t    PDSLEEPCFG;    /*!< Offset: 0x230  Deep Sleep Mode Power Down Config     */
   __IO     uint32_t    PDAWAKECFG;    /*!< Offset: 0x234  Wakeup Mode Power Down Configuration  */
   __IO     uint32_t    PDRUNCFG;      /*!< Offset: 0x238  Running Mode Power Down Configuration */
            uint32_t      Reserved143_582[110];                                  /*!< (Reserved) */
   __I      uint32_t    DEVICE_ID;     /*!< Offset: 0x3f4  LPC Device ID                         */
} SYSCON_Type;

/** @} */

/** @defgroup CT16B CT16B: 16-Bit Counter / Timer
  * @{
  */

/*! @brief CT16B Register Map */
typedef struct {
    __IO    uint32_t    IR;            /*!< Offset: 0x000  Interrupt Register                    */
    __IO    uint32_t    TCR;           /*!< Offset: 0x004  Timer Control Register                */
    __IO    uint32_t    TC;            /*!< Offset: 0x008  Timer Counter Register                */
    __IO    uint32_t    PR;            /*!< Offset: 0x00c  Prescale Register                     */
    __IO    uint32_t    PC;            /*!< Offset: 0x010  Prescale Counter Register             */
    __IO    uint32_t    MCR;           /*!< Offset: 0x014  Match Control Register                */
    __IO    uint32_t    MR0;           /*!< Offset: 0x018  MAT0 Match Register                   */
    __IO    uint32_t    MR1;           /*!< Offset: 0x01c  MAT1 Match Register                   */
    __IO    uint32_t    MR2;           /*!< Offset: 0x020  MAT2 Match Register                   */
    __IO    uint32_t    MR3;           /*!< Offset: 0x024  MAT3 Match Register                   */
    __IO    uint32_t    CCR;           /*!< Offset: 0x028  Counter Control Register              */
    __I     uint32_t    CR0;           /*!< Offset: 0x02c  Capture Channel 0 Register            */
            uint32_t      Reserved12_14[3];                                      /*!< (Reserved) */
    __IO    uint32_t    EMR;           /*!< Offset: 0x03c  External Match Register               */
            uint32_t      Reserved16_27[12];                                     /*!< (Reserved) */
    __IO    uint32_t    CTCR;          /*!< Offset: 0x070  Count Control Register                */
    __IO    uint32_t    PWMC;          /*!< Offset: 0x074  PWM Control Register                  */
} CT16B_Type;

/** @} */

/** @defgroup CT32B CT32B: 32-Bit Counter / Timer
  * @{
  */

/*! @brief CT32B Register Map */
typedef struct {
    __IO    uint32_t    IR;            /*!< Offset: 0x000  Interrupt Register                    */
    __IO    uint32_t    TCR;           /*!< Offset: 0x004  Timer Control Register                */
    __IO    uint32_t    TC;            /*!< Offset: 0x008  Timer Counter Register                */
    __IO    uint32_t    PR;            /*!< Offset: 0x00c  Prescale Register                     */
    __IO    uint32_t    PC;            /*!< Offset: 0x010  Prescale Counter Register             */
    __IO    uint32_t    MCR;           /*!< Offset: 0x014  Match Control Register                */
    __IO    uint32_t    MR0;           /*!< Offset: 0x018  MAT0 Match Register                   */
    __IO    uint32_t    MR1;           /*!< Offset: 0x01c  MAT1 Match Register                   */
    __IO    uint32_t    MR2;           /*!< Offset: 0x020  MAT2 Match Register                   */
    __IO    uint32_t    MR3;           /*!< Offset: 0x024  MAT3 Match Register                   */
    __IO    uint32_t    CCR;           /*!< Offset: 0x028  Counter Control Register              */
    __I     uint32_t    CR0;           /*!< Offset: 0x02c  Capture Channel 0 Register            */
            uint32_t      Reserved12_14[3];                                      /*!< (Reserved) */
    __IO    uint32_t    EMR;           /*!< Offset: 0x03c  External Match Register               */
            uint32_t      Reserved16_27[12];                                     /*!< (Reserved) */
    __IO    uint32_t    CTCR;          /*!< Offset: 0x070  Count Control Register                */
    __IO    uint32_t    PWMC;          /*!< Offset: 0x074  PWM Control Register                  */
} CT32B_Type;

/** @} */

/** @defgroup GPIO GPIO: General Purpose IO
  * @{
  */

/*! @brief GPIO Register Map */
typedef struct {
    __IO    uint32_t    SELDATA[4095]; /*!< Offset: 0x0000  Address-Selected IO Data             */
    __IO    uint32_t    DATA;          /*!< Offset: 0x7ffc  All IO Pins Data                     */
    __IO    uint32_t      Reserved[4096];                                        /*!< (Reserved) */
    __IO    uint32_t    DIR;           /*!< Offset: 0x8000  Data Direction Register              */
    __IO    uint32_t    IS;            /*!< Offset: 0x8004  Interrupt Sense Register             */
    __IO    uint32_t    IBE;           /*!< Offset: 0x8008  Interrupt Both Edges Register        */
    __IO    uint32_t    IEV;           /*!< Offset: 0x800c  Interrupt Event Register             */
    __IO    uint32_t    IE;            /*!< Offset: 0x8010  Interrupt Mask Register              */
    __I     uint32_t    RIS;           /*!< Offset: 0x8014  Raw Interrupt Stat Register          */
    __I     uint32_t    MIS;           /*!< Offset: 0x8018  Masked Interrupt Status Register     */
    __O     uint32_t    IC;            /*!< Offset: 0x801c  Interrupt Clear Register             */
} GPIO_Type;

/** @} */


#if defined(LPC11CXX)

/** @defgroup CAN CAN: Controller Area Network
  * @{
  */

/*! @brief CAN Register Map */
typedef struct {
    __IO    uint32_t    CNTL;          /*!< Offset: 0x000  Control Register                      */
    __IO    uint32_t    STAT;          /*!< Offset: 0x004  Status Register                       */
    __O     uint32_t    EC;            /*!< Offset: 0x008  Error Counter Register                */
    __IO    uint32_t    BT;            /*!< Offset: 0x00c  Bit Timing Register                   */
    __O     uint32_t    INT;           /*!< Offset: 0x010  Interrupt Register                    */
    __IO    uint32_t    TEST;          /*!< Offset: 0x014  Test Register                         */
    __IO    uint32_t    BRPE;          /*!< Offset: 0x018  Baud Rate Prescaler Extension         */
            uint32_t      Reserved7[1];                                          /*!< (Reserved) */
    __IO    uint32_t    IF1_CMDREQ;    /*!< Offset: 0x020  Interface 1 Command Request Register  */
    __IO    uint32_t    IF1_CMDMSK;    /*!< Offset: 0x024  Interface 1 Command Mask Register     */
    __IO    uint32_t    IF1_IDMSK1;    /*!< Offset: 0x028  Interface 1 Identifier Mask 1 Register*/
    __IO    uint32_t    IF1_IDMSK2;    /*!< Offset: 0x02c  Interface 1 Identifier Mask 2 Register*/
    __IO    uint32_t    IF1_ARB1;      /*!< Offset: 0x030  Interface 1 Arbitration 1 Register    */
    __IO    uint32_t    IF1_ARB2;      /*!< Offset: 0x034  Interface 1 Arbitration 2 Register    */
    __IO    uint32_t    IF1_MCTRL;     /*!< Offset: 0x038  Interface 1 Message Control Register  */
    __IO    uint32_t    IF1_DA1;       /*!< Offset: 0x03c  Interface 1 Data A1 Register          */
    __IO    uint32_t    IF1_DA2;       /*!< Offset: 0x040  Interface 1 Data A2 Register          */
    __IO    uint32_t    IF1_DB1;       /*!< Offset: 0x044  Interface 1 Data B1 Register          */
    __IO    uint32_t    IF1_DB2;       /*!< Offset: 0x048  Interface 1 Data B2 Register          */
    __IO    uint32_t    IF1_DB2;       /*!< Offset: 0x04c  Interface 1 Data B2 Register          */
            uint32_t      Reserved20_23[4];                                      /*!< (Reserved) */
    __IO    uint32_t    IF2_CMDREQ;    /*!< Offset: 0x060  Interface 2 Command Request Register  */
    __IO    uint32_t    IF2_CMDMSK;    /*!< Offset: 0x064  Interface 2 Command Mask Register     */
    __IO    uint32_t    IF2_IDMSK1;    /*!< Offset: 0x068  Interface 2 Identifier Mask 1 Register*/
    __IO    uint32_t    IF2_IDMSK2;    /*!< Offset: 0x06c  Interface 2 Identifier Mask 2 Register*/
    __IO    uint32_t    IF2_ARB1;      /*!< Offset: 0x070  Interface 2 Arbitration 1 Register    */
    __IO    uint32_t    IF2_ARB2;      /*!< Offset: 0x074  Interface 2 Arbitration 2 Register    */
    __IO    uint32_t    IF2_MCTRL;     /*!< Offset: 0x078  Interface 2 Message Control Register  */
    __IO    uint32_t    IF2_DA1;       /*!< Offset: 0x07c  Interface 2 Data A1 Register          */
    __IO    uint32_t    IF2_DA2;       /*!< Offset: 0x080  Interface 2 Data A2 Register          */
    __IO    uint32_t    IF2_DB1;       /*!< Offset: 0x084  Interface 2 Data B1 Register          */
    __IO    uint32_t    IF2_DB2;       /*!< Offset: 0x088  Interface 2 Data B2 Register          */
    __IO    uint32_t    IF2_DB2;       /*!< Offset: 0x08c  Interface 2 Data B2 Register          */
            uint32_t      Reserved36_63[28];                                     /*!< (Reserved) */
    __O     uint32_t    TXREQ1;        /*!< Offset: 0x100  Transmission Request Register 1       */
    __O     uint32_t    TXREQ2;        /*!< Offset: 0x104  Transmission Request Register 2       */
            uint32_t      Reserved66_71[6];                                      /*!< (Reserved) */
    __O     uint32_t    ND1;           /*!< Offset: 0x120  New Data Register 1                   */
    __O     uint32_t    ND2;           /*!< Offset: 0x104  New Data Register 2                   */
            uint32_t      Reserved74_79[6];                                      /*!< (Reserved) */
    __O     uint32_t    IR1;           /*!< Offset: 0x140  Interrupt Pending Register 1          */
    __O     uint32_t    IR2;           /*!< Offset: 0x144  Interrupt Pending Register 2          */
            uint32_t      Reserved82_87[6];                                      /*!< (Reserved) */
    __O     uint32_t    MSGV1;         /*!< Offset: 0x160  Message Valid Register 1              */
    __O     uint32_t    MSGV2;         /*!< Offset: 0x164  Message Valid Register 2              */
            uint32_t      Reserved90_95[6];                                      /*!< (Reserved) */
    __IO    uint32_t    CLKDIV;        /*!< Offset: 0x180  Clock Divider                         */
} CAN_Type;

/** @} */

#endif /* #if defined(LPC11CXX) */


/* Peripheral Register Bit Definitions  -------------------------------------*/

/** @defgroup I2C_Register_Bit_Definitions I2C Register Bit Definitions
  * @ingroup  I2C
  * @{
  */

/** @defgroup I2C_CONSET_Bit_Definitions CONSET: Control Set Register
  * @{
  */

#define I2C_CONSET_Mask                (0x7c)              /*!< Usable bits in CONSET register   */

#define I2C_AA                         (1 << 2)            /*!< Assert Acknowledge Set Bit       */
#define I2C_SI                         (1 << 3)            /*!< I2C Interrupt Enable Bit         */
#define I2C_STO                        (1 << 4)            /*!< STOP Flag Set Bit                */
#define I2C_STA                        (1 << 5)            /*!< START Flag Set Bit               */
#define I2C_I2EN                       (1 << 6)            /*!< I2C Interface Enable Bit         */

/** @} */

/** @defgroup I2C_CONCLR_Bit_Definitions CONCLR: Control Clear Register
  * @{
  */

#define I2C_CONCLR_Mask                (0x6c)              /*!< Usable bits in CONCLR register   */

#define I2C_AAC                        (1 << 2)            /*!< Assert Acknowledge Clear Bit     */
#define I2C_SIC                        (1 << 3)            /*!< I2C Interrupt Disable Flag       */
#define I2C_STAC                       (1 << 5)            /*!< START Flag Clear Bit             */
#define I2C_I2ENC                      (1 << 6)            /*!< I2C Interface Disable Bit        */

/** @} */

/** @defgroup I2C_STAT_Bit_Definitions STAT: Status Register
  * @{
  */

#define I2C_STAT_Mask                  (0xf8)              /*!< Usable bits in STAT register     */

/** @} */

/** @defgroup I2C_DAT_Bit_Definitions DAT: Data Register
  * @{
  */

#define I2C_DAT_Mask                   (0xff)              /*!< Usable bits in DAT register      */

/** @} */

/** @defgroup I2C_ADR0-3_Bit_Definitions ADRn: Slave Address Register
  * @{
  */

#define I2C_ADR_Mask                   (0xff)              /*!< Usable bits in ADRn Registers    */

#define I2C_ADDR_Mask                  (0x7f << 1)         /*!< Address Mask (7 bits)            */
#define I2C_ADDR_Shift                 (1)                 /*!< Bit shift of ADDR Mask           */

#define I2C_GC                         (1 << 0)            /*!< Gen. Call Enable (Answer 0x00)   */

/** @} */

/** @defgroup I2C_MASK0-3_Bit_Definitions MASKn: Slave Address Mask Register
  * @{
  */

#define I2C_MASK_Mask                  (0xfe)              /*!< Usable bits in MASKn Registers   */

#define I2C_ADDRMASK_Mask              (0x7f << 1)         /*!< Address n Mask (7 bits)          */
#define I2C_ADDRMASK_Shift             (1)                 /*!< Bit shift of MASK Mask           */

/** @} */

/** @defgroup I2C_SCLH_Bit_Definitions SCLH: Duty Cycle Register High Half Word
  * @{
  */

#define I2C_SCLH_Mask                  (0xffff)            /*!< Count for SCL High Time          */

/** @} */

/** @defgroup I2C_SCLL_Bit_Definitions SCLL: Duty Cycle Register Low Half Word
  * @{
  */

#define I2C_SCLL_Mask                  (0xffff)            /*!< Count for SCL Low Time           */

/** @} */

/** @defgroup I2C_MMCTRL_Bit_Definitions MMCTRL: Monitor Mode Control Register
  * @{
  */

#define I2C_MMCTRL_Mask                (0x0b)              /*!< Usable Bits in MMCTRL Register   */

#define I2C_MM_ENA                     (1 << 0)            /*!< Monitor Mode Enable              */
#define I2C_ENA_SCL                    (1 << 1)            /*!< SCL Output Enable                */
#define I2C_MATCH_ALL                  (1 << 3)            /*!< Generate Int (in MM) on Any Addr */

/** @} */

/**
  * @}
  */


/** @defgroup WDT_Register_Bit_Definitions WDT Register Bit Definitions
  * @ingroup  WDT
  * @{
  */

/** @addtogroup WDT_MOD_Bit_Definitions MOD: Watchdog Timer Mode Register
  * @{
  */

#define WDT_MOD_Mask                   (0x0f)              /*!< Usable bits in MOD register      */

#define WDT_WDEN                       (1 << 0)            /*!< Watchdog Enable (Set Only)       */
#define WDT_WDRESET                    (1 << 1)            /*!< Watchdog Reset (Set Only)        */
#define WDT_WDTOF                      (1 << 2)            /*!< Watchdog Timeout Flag (SW clear) */
#define WDT_WDINT                      (1 << 3)            /*!< Watchdog Interrupt Flag (RO)     */

#if defined(LPC11XXL)  /* L-series parts have windowed WDT with extra features */
# define WDT_WDPROTECT                 (1 << 4)            /*!< Reload value protection mode     */
#endif

/** @} */

/** @addtogroup WDT_TC_Bit_Definitions TC: Watchdog Timer Constant Register
  * @{
  */

#define WDT_TC_Mask                    (0x00ffffffUL)      /*!< Usable bits in TC register       */

/** @} */

/** @addtogroup WDT_TV_Bit_Definitions TV: Watchdog Timer Value Register
  * @{
  */

#define WDT_TV_Mask                    (0x00ffffffUL)      /*!< Usable bits in TV register       */

/** @} */

#if defined(LPC11XXL)   /* L-series parts have windowed WDT with extra features
/** @addtogroup WDT_WARNINT_Bit_Definitions WARNINT: Watchdog Timer Warning Interrupt Register
  * @{
  */

#define WDT_WARNINT_Mask               (0x03ff)            /*!< Warning Interrupt Compare Value  */

/** @} */

/** @addtogroup WDT_WINDOW_Bit_Definitions WINDOW: Watchdog Timer Window Timer Register
  * @{
  */

#define WDT_WINDOW_Mask                (0x00ffffffUL)      /*!< Window Value                     */

/** @} */
#endif /* #if defined(LPC11XXL) */

/**
  * @}
  */


/** @defgroup UART_Register_Bit_Definitions UART Register Bit Definitions
  * @ingroup  UART
  * @{
  */

/** @defgroup UART_RBR_Bit_Definitions RBR: Receive Buffer Register
  * @{
  */

#define UART_RBR_Mask                  (0xff)              /*!< Usable bits in RBR register      */

/** @} */

/** @defgroup UART_DLL_Bit_Definitions DLL: Divisor Latch LSB Register
  * @{
  */

#define UART_DLLSB_Mask                (0xff)              /*!< Usable bits in DLLSB register    */

/** @} */

/** @defgroup UART_DLM_Bit_Definitions DLM: Divisor Latch MSB Register
  * @{
  */

#define UART_DLMSB_Mask                (0xff)              /*!< Usable bits in DLMSB register    */

/** @} */

/** @defgroup UART_IER_Bit_Definitions IER: Interrupt Enable Register
  * @{
  */

#define UART_IER_Mask                  (0x0307)            /*!< Usable Bits in IER Register      */

#define UART_RBR_INT_ENA               (1 << 0)            /*!< Rx Data Avail/Timeout IRQ Enable */
#define UART_THRE_INT_ENA              (1 << 1)            /*!< TX Holding Reg Empty IRQ Enable  */
#define UART_LINE_INT_ENA              (1 << 2)            /*!< RX Line Status IRQ Enable        */
#define UART_ABEO_INT_ENA              (1 << 8)            /*!< Auto Baud IRQ Enable             */
#define UART_ABTO_INT_ENA              (1 << 9)            /*!< Auto Baud Timeout IRQ Enable     */

/** @} */

/** @defgroup UART_IIR_Bit_Definitions IIR: Interrupt ID Register
  * @{
  */

#define UART_IIR_Mask                  (0x03cf)            /*!< Usable bits in IIR register    */

#define UART_INTID_Mask                (0x0f)              /*!< Mask for Standard Interrupt ID's */
#define UART_INTID_Shift               (0)                 /*!< Bit shift of INTID Mask          */
#define UART_INTID_NONE                (0x01)              /*!< Int. Status (0 == IT pending)    */
#define UART_INTID_RLS                 (0x06)              /*!< Receive Line Status Interrupt    */
#define UART_INTID_RDA                 (0x04)              /*!< Receive Data Available Interrupt */
#define UART_INTID_CTI                 (0x0c)              /*!< Character Time-out Interrupt     */
#define UART_INTID_THRE                (0x02)              /*!< Tx Holding Reg Empty Interrupt   */
#define UART_INTID_MODM                (0x00)              /*!< Modem Line Interrupt             */

#define UART_FIFO_Mask                 (0x03 << 6)         /*!< Copy of UFCR                     */
#define UART_FIFO_Shift                (6)                 /*!< Bit shift for FIFO Mask          */

#define UART_ABIT_Mask                 (0x03 << 8)         /*!< Mask for Auto Baud Interrupts    */
#define UART_ABIT_Shift                (8)                 /*!< Bit shift of ABIT Mask           */

#define UART_IT_ABEO                   (1 << 8)            /*!< End of Auto-Baud Interrupt       */
#define UART_IT_ABTO                   (1 << 9)            /*!< Auto-Baud Timeout Interrupt      */

/** @} */

/** @defgroup UART_FCR_Bit_Definitions FCR: FIFO Control Register (WO)
  * @{
  */

#define UART_FCR_Mask                  (0xc7)              /*!< Usable bits in FCR register      */

#define UART_RXTRIGLVL_Mask            (3 << 6)            /*!< Mask for RX Trigger Level        */
#define UART_RXTRIGLVL_Shift           (6)                 /*!< Bit shift of RXTRIGLVL Mask      */
#define UART_RXTRIGLVL_1               (0x00)              /*!< Trigger RX IRQ on 1B  in FIFO    */
#define UART_RXTRIGLVL_4               (0x01 << 6)         /*!< Trigger RX IRQ on 4B  in FIFO    */
#define UART_RXTRIGLVL_8               (0x02 << 6)         /*!< Trigger RX IRQ on 8B  in FIFO    */
#define UART_RXTRIGLVL_14              (0x03 << 6)         /*!< Trigger RX IRQ on 14B in FIFO    */

#define UART_FIFOEN                    (1 << 0)            /*!< Enable RX/TX FIFOS (NECESSARY)   */
#define UART_FIFO_RX_RESET             (1 << 1)            /*!< Clear RX FIFO (self-clearing)    */
#define UART_FIFO_TX_RESET             (1 << 2)            /*!< Clear TX FIFO (self-clearing)    */

/** @} */

/** @defgroup UART_LCR_Bit_Definitions LCR: Line Control Register
  * @{
  */

#define UART_LCR_Mask                  (0xff)              /*!< Usable bits in LCR register      */

#define UART_WORDLEN_Mask              (0x03)              /*!< Mask for Word Length             */
#define UART_WORDLEN_Shift             (0)                 /*!< Bit shift of WORDLEN Mask        */
#define UART_WORDLEN_5                 (0x00)              /*!< 5 bit characters                 */
#define UART_WORDLEN_6                 (0x01)              /*!< 6 bit characters                 */
#define UART_WORDLEN_7                 (0x02)              /*!< 7 bit characters                 */
#define UART_WORDLEN_8                 (0x03)              /*!< 8 bit characters                 */

#define UART_PARITY_Mask               (0x07 << 3)         /*!< Mask for Parity Type             */
#define UART_PARITY_Shift              (3)                 /*!< Bit shift of PARITY Mask         */
#define UART_PARITY_NONE               (0x00 << 3)         /*!< No Parity                        */
#define UART_PARITY_ODD                (0x01 << 3)         /*!< Odd Parity                       */
#define UART_PARITY_EVEN               (0x03 << 3)         /*!< Even Parity                      */
#define UART_PARITY_1                  (0x05 << 3)         /*!< Force Parity = 1                 */
#define UART_PARITY_0                  (0x07 << 3)         /*!< Force Parity = 0                 */

#define UART_2STOPBITS                 (1 << 2)            /*!< 2 stop bits                      */
#define UART_PAREN                     (1 << 3)            /*!< Enable Parity                    */
#define UART_BREAK                     (1 << 6)            /*!< Set BREAK on TX line             */
#define UART_DLAB                      (1 << 7)            /*!< Enable Access to Div Latch       */

/** @} */

/** @defgroup UART_MCR_Bit_Definitions MCR: Modem Control Register
  * @{
  */

#define UART_MCR_Mask                  (0xd3)              /*!< Usable bits in MCR register      */

#define UART_DTR                       (1 << 0)            /*!< Control of /DTR line             */
#define UART_RTS                       (1 << 1)            /*!< Control of /RTS line             */
#define UART_LOOPBACK                  (1 << 4)            /*!< Enable Loopback Mode             */
#define UART_RTSENA                    (1 << 6)            /*!< Enable Auto-RTS Flow Control     */
#define UART_CTSENA                    (1 << 7)            /*!< Enable Auto-CTS Flow Control     */

/** @} */

/** @defgroup UART_LSR_Bit_Definitions LSR: Line Status Register
  * @{
  */

#define UART_LSR_Mask                  (0xff)              /*!< Usable bits in LSR register      */

#define UART_RDR                       (1 << 0)            /*!< Receiver Data Ready              */
#define UART_OE                        (1 << 1)            /*!< Overrun Error                    */
#define UART_PE                        (1 << 2)            /*!< Parity Error                     */
#define UART_FE                        (1 << 3)            /*!< Framing Error                    */
#define UART_BI                        (1 << 4)            /*!< Break Interrupt                  */
#define UART_THRE                      (1 << 5)            /*!< Tx Holding Register Empty        */
#define UART_TEMT                      (1 << 6)            /*!< Transmitter Empty                */
#define UART_RXFE                      (1 << 7)            /*!< Error in RX FIFO                 */

/** @} */

/** @defgroup UART_MSR_Bit_Definitions MSR: Modem Status Register
  * @{
  */

#define UART_MSR_Mask                  (0xff)              /*!< Usable bits in MSR register      */

#define UART_DCTS                      (1 << 0)            /*!< Change in CTS                    */
#define UART_DDSR                      (1 << 1)            /*!< Change in DSR                    */
#define UART_TERI                      (1 << 2)            /*!< Low-to-high on RI                */
#define UART_DDCD                      (1 << 3)            /*!< Change on DCD                    */
#define UART_CTS                       (1 << 4)            /*!< Current CTS State (inverted)     */
#define UART_DSR                       (1 << 5)            /*!< Current DSR State (inverted)     */
#define UART_RI                        (1 << 6)            /*!< Current RI State (inverted)      */
#define UART_DCD                       (1 << 7)            /*!< Current DCD State (inverted)     */

/** @} */

/** @defgroup UART_ACR_Bit_Definitions ACR: Auto-baud Control Register
  * @{
  */

#define UART_ACR_Mask                  (0x0307)            /*!< Usable bits in ACR register      */

#define UART_AUTOBAUD                  (1 << 0)            /*!< Autobaud Running                 */
#define UART_MODE1                     (1 << 1)            /*!< Mode 1 Selected (0 = Mode 0)     */
#define UART_AUTORESTART               (1 << 2)            /*!< Autobaud Restart on Timeout      */
#define UART_ABEOIRQCLR                (1 << 8)            /*!< Clear ABEO Interrupt             */
#define UART_OBTOIRQCLR                (1 << 9)            /*!< Clear ABTO Interrupt             */

/** @} */

/** @defgroup UART_FDR_Bit_Definitions FDR: Fractional Divider Register
  * @{
  */

#define UART_FDR_Mask                  (0xff)              /*!< Usable bits in FDR register      */

#define UART_DIVADDVAL_Mask            (0x0f)              /*!< Fractional Divider Div. Val Mask */
#define UART_DIVADDVAL_Shift           (0)                 /*!< Bit shift of DIVADDVAL Mask      */

#define UART_MULVAL_Mask               (0x0f << 4)         /*!< Fract. Divider Mult. Val Mask    */
#define UART_MULVAL_Shift              (4)                 /*!< Bit shift of MULVAL Mask         */

/** @} */

/** @defgroup UART_TER_Bit_Definitions TER: Transmit Enable Register
  * @{
  */

#define UART_TXEN                      (1 << 7)            /*!< Enable Transmitter               */

/** @} */

/** @defgroup UART_RS485CTRL_Bit_Definitions RS485CTRL: RS485-Mode Control Register
  * @{
  */

#define UART_RS485CTRL_Mask            (0x3f)              /*!< Usable bits in RS485CTRL reg.    */

#define UART_NMMEN                     (1 << 0)            /*!< Enable RS/EIA485 Multidrop Mode  */
#define UART_RXDIS                     (1 << 1)            /*!< Disable Receiver                 */
#define UART_AADEN                     (1 << 2)            /*!< Enable RS/EIA485 Auto Addr Det.  */
#define UART_DIRSEL                    (1 << 3)            /*!< 0=RTS Dir Ctrl, 1=DTR Dir Ctrl   */
#define UART_DCTRL                     (1 << 4)            /*!< Enable Direction Control         */
#define UART_OINV                      (1 << 5)            /*!< Invert Polarity on Dir Ctrl Pin  */

/** @} */

/** @defgroup UART_RS485ADRMATCH_Bit_Definitions RS485ADRMATCH: RS485-Mode Address Match Register
  * @{
  */

#define UART_RS485ADRMATCH_Mask        (0xff)              /*!< Usable bits in RS485ADRMATCH reg */

/** @} */

/** @defgroup UART_RS485DLY_Bit_Definitions RS485DLY: RS485-Mode Delay Register
  * @{
  */

#define UART_RS485DLY_Mask             (0xff)              /*!< Usable bits in RS485DLY register */

/** @} */

/**
  * @}
  */


/** @defgroup ADC_Register_Bit_Definitions ADC Register Bit Definitions
  * @ingroup ADC
  * @{
  */

/** @defgroup ADC_CR0_Bit_Definitions CR0: Control Register 0
  * @{
  */

#define ADC_CR0_Mask                   (0x00ffffffUL)      /*!< Usable bits in CR0 register      */

#define ADC_SEL_Mask                   (0xffUL)            /*!< ADC Channel Selection Mask       */
#define ADC_SEL_Shift                  (0)                 /*!< Bit shift of SEL Mask            */
#define ADC_SEL_0                      (1 << 0)            /*!< Select AD0 for Conversion        */
#define ADC_SEL_1                      (1 << 1)            /*!< Select AD1 for Conversion        */
#define ADC_SEL_2                      (1 << 2)            /*!< Select AD2 for Conversion        */
#define ADC_SEL_3                      (1 << 3)            /*!< Select AD3 for Conversion        */
#define ADC_SEL_4                      (1 << 4)            /*!< Select AD4 for Conversion        */
#define ADC_SEL_5                      (1 << 5)            /*!< Select AD5 for Conversion        */
#define ADC_SEL_6                      (1 << 6)            /*!< Select AD6 for Conversion        */
#define ADC_SEL_7                      (1 << 7)            /*!< Select AD7 for Conversion        */

#define ADC_CLKDIV_Mask                (0xffUL << 8)       /*!< ADC Clock Divisor Mask           */
#define ADC_CLKDIV_Shift               (8)                 /*!< Bit shift of CLKDIV Mask         */

#define ADC_CLKS_Mask                  (0x07UL << 17)      /*!< ADC # Clocks / Burst Conversion  */
#define ADC_CLKS_Shift                 (17)                /*!< Bit shift of CLKS Mask           */

#define ADC_START_Mask                 (0x07UL << 24)      /*!< Start Mode Mask                  */
#define ADC_START_Shift                (24)                /*!< Bit shift of START Mask          */

#define ADC_BURST                      (1UL << 16)         /*!< Enable "Burst Mode" (HW scan)    */
#define ADC_EDGE                       (1UL << 27)         /*!< Set Edge for Capture Mode        */

/** @} */

/** @defgroup ADC_DR_Bit_Definitions DR: Data Registers (and GDR: Global Data Register)
  * @{
  */

#define ADC_GDR_Mask                   (0xc700ffffUL)      /*!< Usable bits in GDR register      */

#define ADC_V_VREF_Mask                (0xffffUL)          /*!< Result of ADC Conversion         */
#define ADC_V_VREF_Shift               (0)                 /*!< Bit shift of V_VREF Mask         */

#define ADC_CHN_Mask                   (0x07UL << 24)      /*!< Channel Result Came From         */
#define ADC_CHN_Shift                  (24)                /*!< Bit shift of CHN Mask            */

#define ADC_OVERRUN                    (1UL << 30)         /*!< Result Overwritten Flag          */
#define ADC_DONE                       (1UL << 31)         /*!< Conversion Done Flag             */

/** @} */

/** @defgroup ADC_STAT_Bit_Definitions STAT: Status Register
  * @{
  */

#define ADC_STAT_Mask                  (0x0001ffffUL)      /*!< Usable bits in STAT register     */

#define ADC_STATDONE_Mask              (0xffUL)            /*!< "DONE" Flags Mask                */
#define ADC_STATDONE_Shift             (0)                 /*!< Bit shift of STATDONE Mask       */
#define ADC_STATDONE_0                 (1UL << 0)          /*!< Channel 0 Done Flag              */
#define ADC_STATDONE_1                 (1UL << 1)          /*!< Channel 1 Done Flag              */
#define ADC_STATDONE_2                 (1UL << 2)          /*!< Channel 2 Done Flag              */
#define ADC_STATDONE_3                 (1UL << 3)          /*!< Channel 3 Done Flag              */
#define ADC_STATDONE_4                 (1UL << 4)          /*!< Channel 4 Done Flag              */
#define ADC_STATDONE_5                 (1UL << 5)          /*!< Channel 5 Done Flag              */
#define ADC_STATDONE_6                 (1UL << 6)          /*!< Channel 6 Done Flag              */
#define ADC_STATDONE_7                 (1UL << 7)          /*!< Channel 7 Done Flag              */

#define ADC_STATOVERRUN_Mask           (0xffUL << 8)       /*!< "OVERRUN" Flags Mask             */
#define ADC_STATOVERRUN_Shift          (8)                 /*!< Bit shift of STATOVERRUN Mask    */
#define ADC_STATOVERRUN_0              (1UL << 8)          /*!< Channel 0 Overrun Flag           */
#define ADC_STATOVERRUN_1              (1UL << 9)          /*!< Channel 1 Overrun Flag           */
#define ADC_STATOVERRUN_2              (1UL << 10)         /*!< Channel 2 Overrun Flag           */
#define ADC_STATOVERRUN_3              (1UL << 11)         /*!< Channel 3 Overrun Flag           */
#define ADC_STATOVERRUN_4              (1UL << 12)         /*!< Channel 4 Overrun Flag           */
#define ADC_STATOVERRUN_5              (1UL << 13)         /*!< Channel 5 Overrun Flag           */
#define ADC_STATOVERRUN_6              (1UL << 14)         /*!< Channel 6 Overrun Flag           */
#define ADC_STATOVERRUN_7              (1UL << 15)         /*!< Channel 7 Overrun Flag           */

#define ADC_ADINT                      (1UL << 16)         /*!< Interrupt Flag                   */

/** @} */

/** @defgroup ADC_INTEN_Bit_Definitions INTEN: Interrupt Enable Register
  * @{
  */

#define ADC_INTEN_Mask                 (0x01ff)            /*!< Usable bits in INTEN register    */

#define ADC_ADINTEN_Mask               (0xff)              /*!< Enable Channel Interrupts Mask   */
#define ADC_ADINTEN_Shift              (0)                 /*!< Bit shift of ADINTEN Mask        */
#define ADC_ADINTEN_0                  (1 << 0)            /*!< Enable Interrupt on Channel 0    */
#define ADC_ADINTEN_1                  (1 << 1)            /*!< Enable Interrupt on Channel 1    */
#define ADC_ADINTEN_2                  (1 << 2)            /*!< Enable Interrupt on Channel 2    */
#define ADC_ADINTEN_3                  (1 << 3)            /*!< Enable Interrupt on Channel 3    */
#define ADC_ADINTEN_4                  (1 << 4)            /*!< Enable Interrupt on Channel 4    */
#define ADC_ADINTEN_5                  (1 << 5)            /*!< Enable Interrupt on Channel 5    */
#define ADC_ADINTEN_6                  (1 << 6)            /*!< Enable Interrupt on Channel 6    */
#define ADC_ADINTEN_7                  (1 << 7)            /*!< Enable Interrupt on Channel 7    */

#define ADC_ADGINTEN                   (1 << 8)            /*!< Enable ADC Global Interrupt      */

/** @} */

/**
  * @}
  */


/** @defgroup PMU_Register_Bit_Definitions PMU Register Bit Definitions
  * @ingroup  PMU
  * @{
  */

/** @defgroup PMU_PCON_Bit_Definitions PCON: Power Configuration Register
  * @{
  */

#define PMU_PCON_Mask                  (0x902)             /*!< Usable bits in PMU PCON register */

#define PMU_DPDEN                      (1 << 1)            /*!< Enable Deep Power Down on WFI    */
#define PMU_SLEEPFLAG                  (1 << 8)            /*!< Power Down Mode Entered Flag     */
#define PMU_DPDFLAG                    (1 << 11)           /*!< Deep Power Down Mode Flag        */

/** @} */

/** @defgroup PMU_GPREG4_Bit_Definitions GPREG4: General Purpose Register 4 / Wakeup Hysteresis Control Register
  * @{
  */

#define PMU_WAKEUPHYS                  (1 << 10)           /*!< Enable Hysteresis on Wakeup Pin  */

/** @} */

/**
  * @}
  */


/** @defgroup FLASH_Register_Bit_Definitions FLASH Register Bit Definitions
  * @ingroup  FLASH
  * @{
  */

/** @defgroup FLASH_FLASHCFG_Bit_Definitions FLASHCFG: Flash Configuration Register
  * @{
  */

#define FLASH_FLASHTIM_Mask            (0x03)              /*!< Mask for Setting Flash Timing    */
#define FLASH_FLASHTIM_1               (0x00)              /*!< Flash Accesses take 1 Sys Clock  */
#define FLASH_FLASHTIM_2               (0x01)              /*!< Flash Accesses take 2 Sys Clocks */
#define FLASH_FLASHTIM_3               (0x02)              /*!< Flash Accesses take 3 Sys Clocks */

/** @} */

/** @defgroup FLASH_FMSSTART_Bit_Definitions FMSSTART: Flash Module Signature Start Register
  * @{
  */

#define FLASH_START_Mask               (0xffff)            /*!< Mask / Signature Gen Start Addr  */

/** @} */

/** @defgroup FLASH_FMSSTOP_Bit_Definitions FMSSTOP: Flash Module Signature Stop Register
  * @{
  */

#define FLASH_STOP_Mask                (0xffff)            /*!< Mask / Signature Gen Stop Addr   */
#define FLASH_SIG_START                (1 << 17)           /*!< Start Signature Generation Flag  */

/** @} */

/** @defgroup FLASH_FMSTAT_Bit_Definitions FMSTAT: Flash Module Status Register
  * @{
  */

#define FLASH_SIG_DONE                 (1 << 2)            /*!< Signature Generation Completed   */

/** @} */

/** @defgroup FLASH_FMSTATCLR_Bit_Definitions FMSTATCLR: Flash Module Status Register Clear
  * @{
  */

#define FLASH_SIG_DONE_CLR             (1 << 2)            /*!< Clear Signature Gen Complete Flg */

/** @} */


/**
  * @}
  */


/** @defgroup SSP_Register_Bit_Definitions SSP Register Bit Definitions
  * @ingroup  SSP
  * @{
  */

/** @defgroup SSP_CR0_Bit_Definitions CR0: Control Register 0
  * @{
  */

#define SSP_CR0_Mask                   (0xffff)            /*!< Usable bits in CR0 register      */

#define SSP_DSS_Mask                   (0x0f)              /*!< SSP Data Size Select Mask        */
#define SSP_DSS_Shift                  (0)                 /*!< Bit shift of DSS Mask            */
#define SSP_DSS_4                      (0x03)              /*!< 4-bit  Data Size                 */
#define SSP_DSS_5                      (0x04)              /*!< 5-bit  Data Size                 */
#define SSP_DSS_6                      (0x05)              /*!< 6-bit  Data Size                 */
#define SSP_DSS_7                      (0x06)              /*!< 7-bit  Data Size                 */
#define SSP_DSS_8                      (0x07)              /*!< 8-bit  Data Size                 */
#define SSP_DSS_9                      (0x08)              /*!< 9-bit  Data Size                 */
#define SSP_DSS_10                     (0x09)              /*!< 10-bit Data Size                 */
#define SSP_DSS_11                     (0x0a)              /*!< 11-bit Data Size                 */
#define SSP_DSS_12                     (0x0b)              /*!< 12-bit Data Size                 */
#define SSP_DSS_13                     (0x0c)              /*!< 13-bit Data Size                 */
#define SSP_DSS_14                     (0x0d)              /*!< 14-bit Data Size                 */
#define SSP_DSS_15                     (0x0e)              /*!< 15-bit Data Size                 */
#define SSP_DSS_16                     (0x0f)              /*!< 16-bit Data Size                 */

#define SSP_FRF_Mask                   (0x03 << 4)         /*!< Frame Format Bitmask             */
#define SSP_FRF_Shift                  (4)                 /*!< Bit shift of FRF Mask            */
#define SSP_FRF_SPI                    (0x00)              /*!< SPI Frame Format                 */
#define SSP_FRF_TI                     (0x01 << 4)         /*!< TI Frame Format                  */
#define SSP_FRF_MICROWIRE              (0x02 << 4)         /*!< Microwire Frame Format           */

/* Bit Frequency is PCLK / (CPSDVSR * (SCR + 1)) */

#define SSP_SCR_Mask                   (0xff << 8)         /*!< Serial Clock Data Rate Mask      */
#define SSP_SCR_Shift                  (8)                 /*!< Bit shift of SCR Mask            */

#define SSP_CPOL                       (1 << 6)            /*!< Clock Polarity Hi Btwn Frames    */
#define SSP_CPHA                       (1 << 7)            /*!< Latch Bits on 2nd Edge           */

/** @} */

/** @defgroup SSP_CR1_Bit_Definitions CR1: Control Register 1
  * @{
  */

#define SSP_CR1_Mask                   (0x0f)              /*!< Usable bits in CR1 register      */

#define SSP_CR1_MODE_Mask              (0x0c)              /*!< SSP Mode Mask                    */
#define SSP_CR1_MODE_Shift             (0)                 /*!< Bit shift of CR1_MODE Mask       */

#define SSP_LBM                        (1 << 0)            /*!< Loop Back Mode Enable            */
#define SSP_SSE                        (1 << 1)            /*!< SSP Peripheral Enable            */
#define SSP_MS                         (1 << 2)            /*!< Slave Mode Enable                */
#define SSP_SOD                        (1 << 3)            /*!< Disable Slave Mode Output        */

/** @} */

/** @defgroup SSP_SR_Bit_Definitions SR: Status Register
  * @{
  */

#define SSP_SR_Mask                    (0x1f)              /*!< Usable bits in SR register       */

#define SSP_TFE                        (1 << 0)            /*!< Transmit FIFO Empty Flag         */
#define SSP_TNF                        (1 << 1)            /*!< Transmit FIFO Not Full Flag      */
#define SSP_RNE                        (1 << 2)            /*!< Receive FIFO Not Empty Flag      */
#define SSP_RFF                        (1 << 3)            /*!< Receive FIFO Full Flag           */
#define SSP_BSY                        (1 << 4)            /*!< SPI Controller Busy Flag         */

/** @} */

/** @defgroup SSP_IMR_Bit_Definitions IMR: Interrupt Mask Set/Clear Register
  * @{
  */

#define SSP_IMR_Mask                   (0x0f)              /*!< Usable bits in IMR register      */

#define SSP_RORIM                      (1 << 0)            /*!< Receive Overrun IRQ Enable       */
#define SSP_RTIM                       (1 << 1)            /*!< Receive Timeout IRQ Enable       */
#define SSP_RXIM                       (1 << 2)            /*!< Rx FIFO > 1/2 Full IRQ Enable    */
#define SSP_TXIM                       (1 << 3)            /*!< Tx FIFO at least 1/2 Empty       */

/** @} */

/** @defgroup SSP_RIS_Bit_Definitions RIS: Raw Interrupt Status Register
  * @{
  */

#define SSP_RIS_Mask                   (0x0f)              /*!< Usable bits in RIS register     */

#define SSP_RORRIS                     (1 << 0)            /*!< Raw Receive Overrun IRQ          */
#define SSP_RTRIS                      (1 << 1)            /*!< Raw Receive Timeout IRQ          */
#define SSP_RXRIS                      (1 << 2)            /*!< Raw Rx FIFO > 1/2 Full IRQ       */
#define SSP_TXRIS                      (1 << 3)            /*!< Raw Tx FIFO at least 1/2         */

/** @} */

/** @defgroup SSP_MIR_Bit_Definitions MIR: Masked Interrupt Status Register
  * @{
  */

#define SSP_MIR_Mask                   (0x0f)              /*!< Usable bits in MIR register     */

#define SSP_RORMIS                     (1 << 0)            /*!< Rx Overrun Interrupt Pending     */
#define SSP_RTMIS                      (1 << 2)            /*!< Rx Timeout Interrupt Pending     */
#define SSP_RXMIS                      (1 << 3)            /*!< Rx Fifo Half Full Int Pending    */
#define SSP_TXMIS                      (1 << 4)            /*!< Tx Fifo Half Empty Int Pending   */

/** @} */

/** @defgroup SSP_ICR_Bit_Definitions ICR: Interrupt Clear Register
  * @{
  */

#define SSP_ICR_Mask                   (0x03)              /*!< Usable bits in ICR register     */

#define SSP_RORIC                      (1 << 0)            /*!< Clear Receive Overrun Interrupt  */
#define SSP_RTIC                       (1 << 1)            /*!< Clear Receive Timeout Interrupt  */

/** @} */

/**
  * @}
  */


/** @defgroup IOCON_Register_Bit_Definitions IOCON Register Bit Definitions
  * @ingroup  IOCON
  * @{
  */

/** @defgroup IOCON_PIOn_Bit_Definitions PIOn: IO Configuration Registers
  * @{
  */

#define IOCON_PIO_Mask                 (0x03bf)            /*!< Usable bits in PIO register      */

#define IOCON_FUNC_Mask                (0x07)              /*!< Pin Function Mask                */
#define IOCON_FUNC_Shift               (0)                 /*!< Bit shift of FUNC Mask           */

#define IOCON_MODE_Mask                (0x03 << 3)         /*!< Pin Mode Mask                    */
#define IOCON_MODE_Shift               (3)                 /*!< Bit shift of MODE Mask           */
#define IOCON_MODE_INACTIVE            (0x00)              /*!< No Pullup / Pulldown             */
#define IOCON_MODE_PULLDOWN            (0x01 << 3)         /*!< Pulldown Resistor Enabled        */
#define IOCON_MODE_PULLUP              (0x02 << 3)         /*!< Pullup Resistor Enabled          */
#define IOCON_MODE_REPEATER            (0x03 << 3)         /*!< Repeater Mode                    */

#define IOCON_I2C_Mask                 (0x0300)            /*!< Pin I2C Setting Mask             */
#define IOCON_I2C_Shift                (8)                 /*!< Bit shift of I2C Mask            */
#define IOCON_I2C_I2C                  (0x00)              /*!< Standard I2C                     */
#define IOCON_I2C_GPIO                 (0x01 << 8)         /*!< Standard GPIO                    */
#define IOCON_I2C_FASTPLUS             (0x02 << 8)         /*!< FastPlus I2C                     */

#define IOCON_HYS                      (1 << 5)            /*!< Enable Hysteresis                */
#define IOCON_AD                       (1 << 7)            /*!< Enable Analog Mode               */

#if defined(LPC11XXL)  /* L-series parts only */
# define IOCON_OD                      (1 << 10)           /*!< Enable pseudo Open-Drain Mode    */
#endif

/** @} */

/** @defgroup IOCON_LOC_Bit_Definitions SCK0_LOC/RI0_LOC/etc: Pin Location Configuration Registers
  * @{
  */

#define IOCON_LOC_Mask                 (0x03)              /*!< Pin Location Mask                */

/** @} */

/**
  * @}
  */


/** @defgroup SYSCON_Register_Bit_Definitions SYSCON Register Bit Definitions
  * @ingroup  SYSCON
  * @{
  */

/* Note: Naming here adds the register name since SYSCON combines so many
 * different functions
 */

/** @defgroup SYSCON_SYSMEMREMAP_Bit_Definitions SYSMEMREMAP: Memory Remapping Register
  * @{
  */

#define SYSCON_SYSMEMREMAP_MAP_Mask       (0x03)           /*!< Memory Remap Bit Mask            */
#define SYSCON_SYSMEMREMAP_MAP_BOOTLOADER (0x00)           /*!< Map Bootloader ROM into Sys Mem  */
#define SYSCON_SYSMEMREMAP_MAP_RAM        (0x01)           /*!< Map RAM into System Memory       */
#define SYSCON_SYSMEMREMAP_MAP_FLASH      (0x02)           /*!< Map FLASH into System Memory     */

/** @} */

/** @defgroup SYSCON_PRESETCTRL_Bit_Definitions PRESETCTRL: Peripheral Reset Control Register
  * @{
  */

#if defined(LPC11CXX)  /* CAN Parts only */
# define SYSCON_PRESETCTRL_RESET_Mask  (0x0f)              /*!< Peripheral Reset Mask            */
#else
# define SYSCON_PRESETCTRL_RESET_Mask  (0x07)              /*!< Peripheral Reset Mask            */
#endif

#define SYSCON_PRESETCTRL_SSP0_RST_N   (1 << 0)            /*!< Reset SSP0 Peripheral            */
#define SYSCON_PRESETCTRL_I2C0_RST_N   (1 << 1)            /*!< Reset I2C0 Peripheral            */
#define SYSCON_PRESETCTRL_SSP1_RST_N   (1 << 2)            /*!< Reset SSP1 Peripheral            */

#if defined(LPC11CXX)  /* CAN Parts only */
# define SYSCON_PRESETCTRL_CAN0_RST_N  (1 << 3)            /*!< Reset CAN0 Peripheral            */
#endif

/** @} */

/** @defgroup SYSCON_SYSPLLCTRL_Bit_Definitions SYSPLLCTRL: System PLL Control Register
  * @{
  */

#define SYSCON_SYSPLLCTRL_Mask         (0x7f)              /*!< Usable bits in SYSPLLCTRL reg.   */

#define SYSCON_SYSPLLCTRL_MSEL_Mask    (0x1f)              /*!< SysPLL Feedback Divider Mask     */
#define SYSCON_SYSPLLCTRL_MSEL_Shift   (0)                 /*!< Bit shift / SYSPLLCTRL_MSEL Mask */

#define SYSCON_SYSPLLCTRL_PSEL_Mask    (0x03 << 5)         /*!< SysPLL Post Divider Ratio Mask   */
#define SYSCON_SYSPLLCTRL_PSEL_Shift   (5)                 /*!< Bit shift / SYSPLLCTRL_PSEL Mask */
#define SYSCON_SYSPLLCTRL_PSEL_DIV2    (0x00 << 5)         /*!< Post-Divide by 2                 */
#define SYSCON_SYSPLLCTRL_PSEL_DIV4    (0x01 << 5)         /*!< Post-Divide by 4                 */
#define SYSCON_SYSPLLCTRL_PSEL_DIV8    (0x02 << 5)         /*!< Post-Divide by 8                 */
#define SYSCON_SYSPLLCTRL_PSEL_DIV16   (0x03 << 5)         /*!< Post-Divide by 16                */

/** @} */

/** @defgroup SYSCON_SYSPLLSTAT_Bit_Definitions SYSPLLSTAT: System PLL Status Register
  * @{
  */

#define SYSCON_SYSPLLSTAT_LOCKED       (1 << 0)            /*!< (RO) System PLL is Locked Flag   */

/** @} */

/** @defgroup SYSCON_IRCCTL_Bit_Definitions IRCCTL: Internal RC Clock Control Register
  * @{
  */

#define SYSCON_IRCCTL_TRIM_Mask        (0xff)              /*!< Usable bits in IRCCTL register   */

/** @} */

/** @defgroup SYSCON_SYSOSCCTRL_Bit_Definitions SYSOSCCTRL: System Oscillator Control Register
  * @{
  */

#define SYSCON_SYSOSCCTRL_Mask         (0x03)              /*!< Useable bits in SYSOSCCTRL reg.  */
#define SYSCON_SYSOSCCTRL_BYPASS       (1 << 0)            /*!< Bypass the System Oscillator     */
#define SYSCON_SYSOSCCTRL_FREQRANGE    (1 << 1)            /*!< External Xtal Frequency 15-25Mhz */

/** @} */

/** @defgroup SYSCON_WDTOSCCTL_Bit_Definitions WDTOSCCTL: Watchdog Timer Oscillator Control Register
  * @{
  */

#define SYSCON_WDTOSCCTL_Mask           (0x01ff)           /*!< Usable bits in WDTOSCCTL reg.    */

#define SYSCON_WDTOSCCTRL_DIV_Mask      (0x1f)             /*!< WDT Oscillator Divider Value     */
#define SYSCON_WDTOSCCTRL_DIV_Shift     (0)                /*!< Bit shift of WDTOSCCTRL_DIV Mask */

#define SYSCON_WDTOSCCTRL_FREQSEL_Mask  (0x01e0)           /*!< WDT Oscillator Frequency Mask    */
#define SYSCON_WDTOSCCTRL_FREQSEL_Shift (5)                /*!< Shift of WDTOSCCTRL_FREQSEL Mask */
#define SYSCON_WDTOSCCTRL_FREQSEL_0_5   (0x01 << 5)        /*!< WDT Oscillator 0.5 Mhz           */
#define SYSCON_WDTOSCCTRL_FREQSEL_0_8   (0x02 << 5)        /*!< WDT Oscillator 0.8 Mhz           */
#define SYSCON_WDTOSCCTRL_FREQSEL_1_1   (0x03 << 5)        /*!< WDT Oscillator 1.1 Mhz           */
#define SYSCON_WDTOSCCTRL_FREQSEL_1_4   (0x04 << 5)        /*!< WDT Oscillator 1.4 Mhz           */
#define SYSCON_WDTOSCCTRL_FREQSEL_1_6   (0x05 << 5)        /*!< WDT Oscillator 1.6 Mhz           */
#define SYSCON_WDTOSCCTRL_FREQSEL_1_8   (0x06 << 5)        /*!< WDT Oscillator 1.8 Mhz           */
#define SYSCON_WDTOSCCTRL_FREQSEL_2_0   (0x07 << 5)        /*!< WDT Oscillator 2.0 Mhz           */
#define SYSCON_WDTOSCCTRL_FREQSEL_2_2   (0x08 << 5)        /*!< WDT Oscillator 2.2 Mhz           */
#define SYSCON_WDTOSCCTRL_FREQSEL_2_4   (0x09 << 5)        /*!< WDT Oscillator 2.4 Mhz           */
#define SYSCON_WDTOSCCTRL_FREQSEL_2_6   (0x0a << 5)        /*!< WDT Oscillator 2.6 Mhz           */
#define SYSCON_WDTOSCCTRL_FREQSEL_2_7   (0x0b << 5)        /*!< WDT Oscillator 2.7 Mhz           */
#define SYSCON_WDTOSCCTRL_FREQSEL_2_9   (0x0c << 5)        /*!< WDT Oscillator 2.9 Mhz           */
#define SYSCON_WDTOSCCTRL_FREQSEL_3_1   (0x0d << 5)        /*!< WDT Oscillator 3.1 Mhz           */
#define SYSCON_WDTOSCCTRL_FREQSEL_3_2   (0x0e << 5)        /*!< WDT Oscillator 3.2 Mhz           */
#define SYSCON_WDTOSCCTRL_FREQSEL_3_4   (0x0f << 5)        /*!< WDT Oscillator 3.4 Mhz           */

/** @} */

/** @defgroup SYSCON_SYSRESSTAT_Bit_Definitions SYSRESSTAT: System Reset Source Status Register
  * @{
  */

#define SYSCON_SYSRESSTAT_STAT_Mask    (0x1f)              /*!< System Reset Source Mask         */
#define SYSCON_SYSRESSTAT_STAT_POR     (1 << 0)            /*!< Reset was from Power-On Reset    */
#define SYSCON_SYSRESSTAT_STAT_EXTRST  (1 << 1)            /*!< Reset was from External Reset    */
#define SYSCON_SYSRESSTAT_STAT_WDT     (1 << 2)            /*!< Reset was from Watchdog          */
#define SYSCON_SYSRESSTAT_STAT_BOD     (1 << 3)            /*!< Reset was from Brownout          */
#define SYSCON_SYSRESSTAT_STAT_SYSRST  (1 << 4)            /*!< Reset was from NVIC Reset        */

/** @} */

/** @defgroup SYSCON_SYSPLLCLKSEL_Bit_Definitions SYSPLLCLKSEL: System PLL Clock Select Register
  * @{
  */

#define SYSCON_SYSPLLCLKSEL_Mask       (0x03)              /*!< System PLL Clock Select Mask     */

#define SYSCON_SYSPLLCLKSEL_IRC        (0x00)              /*!< System PLL Fed from Internal RC  */
#define SYSCON_SYSPLLCLKSEL_SYSOSC     (0x01)              /*!< System PLL Fed from Sys Osc.     */

/** @} */

/** @defgroup SYSCON_SYSPLLUEN_Bit_Definitions SYSPLLUEN: System PLL Source Update Enable Register
  * @{
  */

#define SYSCON_SYSPLLUEN_ENA           (1 << 0)            /*!< Enable System PLL Source Update  */

/** @} */

/** @defgroup SYSCON_MAINCLKSEL_Bit_Definitions MAINCLKSEL: Main Clock Select Register
  * @{
  */

#define SYSCON_MAINCLKSEL_Mask         (0x03)              /*!< Main Clock Source Select Mask    */
#define SYSCON_MAINCLKSEL_IRC          (0x00)              /*!< Main Clock Fed from Internal RC  */
#define SYSCON_MAINCLKSEL_SYSPLLIN     (0x01)              /*!< Main Clock Fed from Sys PLL IN   */
#define SYSCON_MAINCLKSEL_WDTOSC       (0x02)              /*!< Main Clock Fed from WDT Osc.     */
#define SYSCON_MAINCLKSEL_SYSPLLOUT    (0x03)              /*!< Main Clock Fed from Sys PLL Out  */

/** @} */

/** @defgroup SYSCON_MAINCLKUEN_Bit_Definitions MAINCLKUEN: Main Clock Source Update Enable Register
  * @{
  */

#define SYSCON_MAINCLKUEN_ENA          (1 << 0)            /*!< Enable Update of Main Clock Src  */

/** @} */

/** @defgroup SYSCON_SYSAHBCLKDIV_Bit_Definitions SYSAHBCLKDIV: System AHB Clock Divider Register
  * @{
  */

#define SYSCON_SYSAHBCLKDIV_Mask       (0xff)              /*!< Usable bits in SYSAHBCLKDIV reg. */

/** @} */

/** @defgroup SYSCON_SYSAHBCLKCTRL_Bit_Definitions SYSAHBCLKCTRL: System AHB Clock Control Register
  * @{
  */

#define SYSCON_SYSAHBCLKCTRL_Mask       (0x0003ffffUL)     /*!< AHB Clock Control Mask           */
#define SYSCON_SYSAHBCLKCTRL_SYS        (1 << 0)           /*!< (RO) Main System Clock Enabled   */
#define SYSCON_SYSAHBCLKCTRL_ROM        (1 << 1)           /*!< Enable AHB clock to System ROM   */
#define SYSCON_SYSAHBCLKCTRL_RAM        (1 << 2)           /*!< Enable AHB clock to System RAM   */
#define SYSCON_SYSAHBCLKCTRL_FLASHREG   (1 << 3)           /*!< Enable AHB clock to Flash CTRL   */
#define SYSCON_SYSAHBCLKCTRL_FLASHARRAY (1 << 4)           /*!< Enable AHB clock to system FLASH */
#define SYSCON_SYSAHBCLKCTRL_I2C0       (1 << 5)           /*!< Enable AHB clock to I2C0 Periph. */
#define SYSCON_SYSAHBCLKCTRL_GPIO       (1 << 6)           /*!< Enable AHB clock to GPIO Periph. */
#define SYSCON_SYSAHBCLKCTRL_CT16B0     (1 << 7)           /*!< Enable AHB clock to 16b Timer 0  */
#define SYSCON_SYSAHBCLKCTRL_CT16B1     (1 << 8)           /*!< Enable AHB clock to 16b Timer 1  */
#define SYSCON_SYSAHBCLKCTRL_CT32B0     (1 << 9)           /*!< Enable AHB clock to 32b Timer 0  */
#define SYSCON_SYSAHBCLKCTRL_CT32B1     (1 << 10)          /*!< Enable AHB clock to 32b Timer 1  */
#define SYSCON_SYSAHBCLKCTRL_SSP0       (1 << 11)          /*!< Enable AHB clock to SSP0 Periph. */
#define SYSCON_SYSAHBCLKCTRL_UART0      (1 << 12)          /*!< Enable AHB clock to UART0 Periph */
#define SYSCON_SYSAHBCLKCTRL_ADC0       (1 << 13)          /*!< Enable AHB clock to ADC0         */
#define SYSCON_SYSAHBCLKCTRL_WDT        (1 << 15)          /*!< Enable AHB clock to Watchdog Tmr */
#define SYSCON_SYSAHBCLKCTRL_IOCON      (1 << 16)          /*!< Enable AHB clock to IO Config    */
#define SYSCON_SYSAHBCLKCTRL_CAN0       (1 << 17)          /*!< Enable AHB clock to CAN Periph.  */
#define SYSCON_SYSAHBCLKCTRL_SSP1       (1 << 18)          /*!< Enable AHB clock to SSP1 Periph. */

/** @} */

/** @defgroup SYSCON_SSP0CLKDIV_Bit_Definitions SSP0CLKDIV: SSP0 Clock Divider Register
  * @{
  */

#define SYSCON_SSP0CLKDIV_Mask         (0xff)              /*!< Usable bits in SSP0CLKDIV reg.   */

/** @} */

/** @defgroup SYSCON_UART0CLKDIV_Bit_Definitions UART0CLKDIV: UART 0 Clock Divider Register
  * @{
  */

#define SYSCON_UART0CLKDIV_Mask        (0xff)              /*!< Usable bits in UART0CLKDIV reg.  */

/** @} */

/** @defgroup SYSCON_SSP1CLKDIV_Bit_Definitions SSP1CLKDIV: SSP1 Clock Divider Register
  * @{
  */

#define SYSCON_SSP1CLKDIV_Mask         (0xff)              /*!< Usable bits in SSP1CLKDIV reg.   */

/** @} */

/** @defgroup SYSCON_WDTCLKSEL_Bit_Definitions WDTCLKSEL: Watchdog Timer Clock Select Register
  * @{
  */

#define SYSCON_WDTCLKSEL_Mask          (0x03)              /*!< WDT Clock Selection Mask         */
#define SYSCON_WDTCLKSEL_IRC           (0x00)              /*!< WDT Clock Fed From Internal RC   */
#define SYSCON_WDTCLKSEL_MAINCLK       (0x01)              /*!< WDT Clock Fed from Main Clock    */
#define SYSCON_WDTCLKSEL_WDTOSC        (0x02)              /*!< WDT Clock Fed from WDT Osc.      */

/** @} */

/** @defgroup SYSCON_WDTCLKUEN_Bit_Definitions WDTCLKUEN: Watchdog Timer Clock Source Update Enable Register
  * @{
  */

#define SYSCON_WDTCLKUEN_ENA           (1 << 0)            /*!< Enable Update of WDT Clock Src.  */

/** @} */

/** @defgroup SYSCON_WDTCLKDIV_Bit_Definitions WDTCLKDIV: Watchdog Timer Clock Divider Register
  * @{
  */

#define SYSCON_WDTCLKDIV_Mask          (0xff)              /*!< Usable bits in WDTCLKDIV reg.    */

/** @} */

/** @defgroup SYSCON_CLKOUTSEL_Bit_Definitions CLKOUTSEL: Clock Out Select Register
  * @{
  */

#define SYSCON_CLKOUTSEL_Mask          (0x03)              /*!< CLKOUT Clock Source Mask         */
#define SYSCON_CLKOUTSEL_IRC           (0x00)              /*!< CLKOUT Clock Fed from Int. RC    */
#define SYSCON_CLKOUTSEL_SYSOSC        (0x01)              /*!< CLKOUT Clock Fed from System Osc.*/
#define SYSCON_CLKOUTSEL_WDTOSC        (0x02)              /*!< CLKOUT Clock Fed from WDT Osc.   */
#define SYSCON_CLKOUTSEL_MAINCLK       (0x03)              /*!< CLKOUT Clock Fed from Main Clock */

/** @} */

/** @defgroup SYSCON_CLKOUTUEN_Bit_Definitions CLKOUTUEN: Clock Out Source Update Enable Register
  * @{
  */

#define SYSCON_CLKOUTUEN_ENA           (1 << 0)            /*!< Enable Update of CLKOUT Source   */

/** @} */

/** @defgroup SYSCON_CLKOUTDIV_Bit_Definitions CLKOUTDIV: Clock Out Divider Register
  * @{
  */

#define SYSCON_CLKOUTDIV_Mask          (0xff)              /*!< Usable bits in CLKOUTDIV         */

/** @} */

/** @defgroup SYSCON_PIOPORCAP0_Bit_Definitions PIOPORCAP0: Programmable IO Power On Reset Capture Register 0
  * @{
  */

#define SYSCON_PIOPORCAP0_CAP_Mask     (0xffffffffUL)      /*!< Power-on Capture 0 bitmask       */
#define SYSCON_PIOPORCAP0_CAP_PIO0_0   (1 << 0)            /*!< Power-on Capt level of PIO0_0    */
#define SYSCON_PIOPORCAP0_CAP_PIO0_1   (1 << 1)            /*!< Power-on Capt level of PIO0_1    */
#define SYSCON_PIOPORCAP0_CAP_PIO0_2   (1 << 2)            /*!< Power-on Capt level of PIO0_2    */
#define SYSCON_PIOPORCAP0_CAP_PIO0_3   (1 << 3)            /*!< Power-on Capt level of PIO0_3    */
#define SYSCON_PIOPORCAP0_CAP_PIO0_4   (1 << 4)            /*!< Power-on Capt level of PIO0_4    */
#define SYSCON_PIOPORCAP0_CAP_PIO0_5   (1 << 5)            /*!< Power-on Capt level of PIO0_5    */
#define SYSCON_PIOPORCAP0_CAP_PIO0_6   (1 << 6)            /*!< Power-on Capt level of PIO0_6    */
#define SYSCON_PIOPORCAP0_CAP_PIO0_7   (1 << 7)            /*!< Power-on Capt level of PIO0_7    */
#define SYSCON_PIOPORCAP0_CAP_PIO0_8   (1 << 8)            /*!< Power-on Capt level of PIO0_8    */
#define SYSCON_PIOPORCAP0_CAP_PIO0_9   (1 << 9)            /*!< Power-on Capt level of PIO0_9    */
#define SYSCON_PIOPORCAP0_CAP_PIO0_10  (1 << 10)           /*!< Power-on Capt level of PIO0_10   */
#define SYSCON_PIOPORCAP0_CAP_PIO0_11  (1 << 11)           /*!< Power-on Capt level of PIO0_11   */
#define SYSCON_PIOPORCAP0_CAP_PIO1_0   (1 << 12)           /*!< Power-on Capt level of PIO1_0    */
#define SYSCON_PIOPORCAP0_CAP_PIO1_1   (1 << 13)           /*!< Power-on Capt level of PIO1_1    */
#define SYSCON_PIOPORCAP0_CAP_PIO1_2   (1 << 14)           /*!< Power-on Capt level of PIO1_2    */
#define SYSCON_PIOPORCAP0_CAP_PIO1_3   (1 << 15)           /*!< Power-on Capt level of PIO1_3    */
#define SYSCON_PIOPORCAP0_CAP_PIO1_4   (1 << 16)           /*!< Power-on Capt level of PIO1_4    */
#define SYSCON_PIOPORCAP0_CAP_PIO1_5   (1 << 17)           /*!< Power-on Capt level of PIO1_5    */
#define SYSCON_PIOPORCAP0_CAP_PIO1_6   (1 << 18)           /*!< Power-on Capt level of PIO1_6    */
#define SYSCON_PIOPORCAP0_CAP_PIO1_7   (1 << 19)           /*!< Power-on Capt level of PIO1_7    */
#define SYSCON_PIOPORCAP0_CAP_PIO1_8   (1 << 20)           /*!< Power-on Capt level of PIO1_8    */
#define SYSCON_PIOPORCAP0_CAP_PIO1_9   (1 << 21)           /*!< Power-on Capt level of PIO1_9    */
#define SYSCON_PIOPORCAP0_CAP_PIO1_10  (1 << 22)           /*!< Power-on Capt level of PIO1_10   */
#define SYSCON_PIOPORCAP0_CAP_PIO1_11  (1 << 23)           /*!< Power-on Capt level of PIO1_11   */
#define SYSCON_PIOPORCAP0_CAP_PIO2_0   (1 << 24)           /*!< Power-on Capt level of PIO2_0    */
#define SYSCON_PIOPORCAP0_CAP_PIO2_1   (1 << 25)           /*!< Power-on Capt level of PIO2_1    */
#define SYSCON_PIOPORCAP0_CAP_PIO2_2   (1 << 26)           /*!< Power-on Capt level of PIO2_2    */
#define SYSCON_PIOPORCAP0_CAP_PIO2_3   (1 << 27)           /*!< Power-on Capt level of PIO2_3    */
#define SYSCON_PIOPORCAP0_CAP_PIO2_4   (1 << 28)           /*!< Power-on Capt level of PIO2_4    */
#define SYSCON_PIOPORCAP0_CAP_PIO2_5   (1 << 29)           /*!< Power-on Capt level of PIO2_5    */
#define SYSCON_PIOPORCAP0_CAP_PIO2_6   (1 << 30)           /*!< Power-on Capt level of PIO2_6    */
#define SYSCON_PIOPORCAP0_CAP_PIO2_7   (1 << 31)           /*!< Power-on Capt level of PIO2_7    */

/** @} */

/** @defgroup SYSCON_PIOPORCAP1_Bit_Definitions PIOPORCAP1: Programmable IO Power On Reset Capture Register 1
  * @{
  */

#define SYSCON_PIOPORCAP1_CAP_Mask     (0x0000003fUL)      /*!< Power-on Capture 1 bitmask       */
#define SYSCON_PIOPORCAP1_CAP_PIO2_8   (1 << 0)            /*!< Power-on Capt level of PIO2_8    */
#define SYSCON_PIOPORCAP1_CAP_PIO2_9   (1 << 1)            /*!< Power-on Capt level of PIO2_9    */
#define SYSCON_PIOPORCAP1_CAP_PIO2_10  (1 << 2)            /*!< Power-on Capt level of PIO2_10   */
#define SYSCON_PIOPORCAP1_CAP_PIO2_11  (1 << 3)            /*!< Power-on Capt level of PIO2_11   */
#define SYSCON_PIOPORCAP1_CAP_PIO3_0   (1 << 4)            /*!< Power-on Capt level of PIO3_0    */
#define SYSCON_PIOPORCAP1_CAP_PIO3_1   (1 << 5)            /*!< Power-on Capt level of PIO3_1    */
#define SYSCON_PIOPORCAP1_CAP_PIO3_2   (1 << 6)            /*!< Power-on Capt level of PIO3_2    */
#define SYSCON_PIOPORCAP1_CAP_PIO3_3   (1 << 7)            /*!< Power-on Capt level of PIO3_3    */
#define SYSCON_PIOPORCAP1_CAP_PIO3_4   (1 << 8)            /*!< Power-on Capt level of PIO3_4    */
#define SYSCON_PIOPORCAP1_CAP_PIO3_5   (1 << 9)            /*!< Power-on Capt level of PIO3_5    */

/** @} */

/** @defgroup SYSCON_BODCTRL_Bit_Definitions BODCTRL: Brown-Out Detector Control Register
  * @{
  */

#define SYSCON_BODCTRL_Mask            (0x1f)              /*!< Usable bits in BODCTRL           */

#define SYSCON_BODCTRL_BODRSTLEV_Mask  (0x03)              /*!< BOD Reset Level Mask             */
#define SYSCON_BODCTRL_BODRSTLEV_Shift (0)                 /*!< Shift of BODCTRL_BODRSTLEV Mask  */
#define SYSCON_BODCTRL_BODRSTLEV_1V46  (0x00)              /*!< BOD Reset @ 1.46V                */
#define SYSCON_BODCTRL_BODRSTLEV_2V06  (0x01)              /*!< BOD Reset @ 2.06V                */
#define SYSCON_BODCTRL_BODRSTLEV_2V35  (0x02)              /*!< BOD Reset @ 2.35V                */
#define SYSCON_BODCTRL_BODRSTLEV_2V71  (0x03)              /*!< BOD Reset @ 2.71V                */

#define SYSCON_BODCTRL_BODINTVAL_Mask  (0x0c)              /*!< BOD Interrupt Level Mask         */
#define SYSCON_BODCTRL_BODINTVAL_Shift (2)                 /*!< Shift of BODCTRL_BODINTVAL Mask  */
#define SYSCON_BODCTRL_BODINTVAL_1V65  (0x00)              /*!< BOD Interrupt @ 1.65V            */
#define SYSCON_BODCTRL_BODINTVAL_2V22  (0x01 << 2)         /*!< BOD Interrupt @ 2.22V            */
#define SYSCON_BODCTRL_BODINTVAL_2V52  (0x02 << 2)         /*!< BOD Interrupt @ 2.52V            */
#define SYSCON_BODCTRL_BODINTVAL_2V80  (0x03 << 2)         /*!< BOD Interrupt @ 2.80V            */

#define SYSCON_BODCTRL_BODRSTENA       (1 << 4)            /*!< Enable BOD Reset                 */

/** @} */

/** @defgroup SYSCON_SYSTCKCAL_Bit_Definitions SYSTCKCAL: SysTick Calibration Register
  * @{
  */

#define SYSCON_SYSTCKCAL_CAL_Mask      (0x03ffffffUL)      /*!< Systick Calibration Value Mask   */

/** @} */

/** @defgroup SYSCON_STARTAPRP0_Bit_Definitions STARTAPRP0: Start-Logic Edge Control Register 0
  * @{
  */

#define SYSCON_STARTAPRP0_Mask         (0x1fff)            /*!< Start-Logic Edge Control bitmask */
#define SYSCON_STARTAPRP0_APRPIO0_0    (1 << 0)            /*!< Start Logic Edge Ctrl / PIO0_0   */
#define SYSCON_STARTAPRPO_APRPIO0_1    (1 << 1)            /*!< Start Logic Edge Ctrl / PIO0_1   */
#define SYSCON_STARTAPRPO_APRPIO0_2    (1 << 2)            /*!< Start Logic Edge Ctrl / PIO0_2   */
#define SYSCON_STARTAPRPO_APRPIO0_3    (1 << 3)            /*!< Start Logic Edge Ctrl / PIO0_3   */
#define SYSCON_STARTAPRPO_APRPIO0_4    (1 << 4)            /*!< Start Logic Edge Ctrl / PIO0_4   */
#define SYSCON_STARTAPRPO_APRPIO0_5    (1 << 5)            /*!< Start Logic Edge Ctrl / PIO0_5   */
#define SYSCON_STARTAPRPO_APRPIO0_6    (1 << 6)            /*!< Start Logic Edge Ctrl / PIO0_6   */
#define SYSCON_STARTAPRPO_APRPIO0_7    (1 << 7)            /*!< Start Logic Edge Ctrl / PIO0_7   */
#define SYSCON_STARTAPRPO_APRPIO0_8    (1 << 8)            /*!< Start Logic Edge Ctrl / PIO0_8   */
#define SYSCON_STARTAPRPO_APRPIO0_9    (1 << 9)            /*!< Start Logic Edge Ctrl / PIO0_9   */
#define SYSCON_STARTAPRPO_APRPIO0_10   (1 << 10)           /*!< Start Logic Edge Ctrl / PIO0_10  */
#define SYSCON_STARTAPRPO_APRPIO0_11   (1 << 11)           /*!< Start Logic Edge Ctrl / PIO0_11  */
#define SYSCON_STARTAPRPO_APRPIO1_0    (1 << 12)           /*!< Start Logic Edge Ctrl / PIO1_0   */

/** @} */

/** @defgroup SYSCON_STARTERP0_Bit_Definitions STARTERP0: Start-Logic Enable Control Register 0
  * @{
  */

#define SYSCON_STARTEPRP0_Mask         (0x1fff)            /*!< Start-Logic Enable bitmask       */
#define SYSCON_STARTERP0_ERPIO0_0      (1 << 0)            /*!< Start Logic Enable for PIO0_0    */
#define SYSCON_STARTERP0_ERPIO0_1      (1 << 1)            /*!< Start Logic Enable for PIO0_1    */
#define SYSCON_STARTERP0_ERPIO0_2      (1 << 2)            /*!< Start Logic Enable for PIO0_2    */
#define SYSCON_STARTERP0_ERPIO0_3      (1 << 3)            /*!< Start Logic Enable for PIO0_3    */
#define SYSCON_STARTERP0_ERPIO0_4      (1 << 4)            /*!< Start Logic Enable for PIO0_4    */
#define SYSCON_STARTERP0_ERPIO0_5      (1 << 5)            /*!< Start Logic Enable for PIO0_5    */
#define SYSCON_STARTERP0_ERPIO0_6      (1 << 6)            /*!< Start Logic Enable for PIO0_6    */
#define SYSCON_STARTERP0_ERPIO0_7      (1 << 7)            /*!< Start Logic Enable for PIO0_7    */
#define SYSCON_STARTERP0_ERPIO0_8      (1 << 8)            /*!< Start Logic Enable for PIO0_8    */
#define SYSCON_STARTERP0_ERPIO0_9      (1 << 9)            /*!< Start Logic Enable for PIO0_9    */
#define SYSCON_STARTERP0_ERPIO0_10     (1 << 10)           /*!< Start Logic Enable for PIO0_10   */
#define SYSCON_STARTERP0_ERPIO0_11     (1 << 11)           /*!< Start Logic Enable for PIO0_11   */
#define SYSCON_STARTERP0_ERPIO1_0      (1 << 12)           /*!< Start Logic Enable for PIO1_0    */

/** @} */

/** @defgroup SYSCON_STARTRSRP0CLR_Bit_Definitions STARTRSRP0CLR: Start-Logic Clear Control Register 0
  * @{
  */

#define SYSCON_STARTRSRP0CLR_Mask       (0x1fff)           /*!< Start-Logic Clear bitmask        */
#define SYSCON_STARTRSRP0CLR_RSRPIO0_0  (1 << 0)           /*!< Start Logic Clear for PIO0_0     */
#define SYSCON_STARTRSRP0CLR_RSRPIO0_1  (1 << 1)           /*!< Start Logic Clear for PIO0_1     */
#define SYSCON_STARTRSRP0CLR_RSRPIO0_2  (1 << 2)           /*!< Start Logic Clear for PIO0_2     */
#define SYSCON_STARTRSRP0CLR_RSRPIO0_3  (1 << 3)           /*!< Start Logic Clear for PIO0_3     */
#define SYSCON_STARTRSRP0CLR_RSRPIO0_4  (1 << 4)           /*!< Start Logic Clear for PIO0_4     */
#define SYSCON_STARTRSRP0CLR_RSRPIO0_5  (1 << 5)           /*!< Start Logic Clear for PIO0_5     */
#define SYSCON_STARTRSRP0CLR_RSRPIO0_6  (1 << 6)           /*!< Start Logic Clear for PIO0_6     */
#define SYSCON_STARTRSRP0CLR_RSRPIO0_7  (1 << 7)           /*!< Start Logic Clear for PIO0_7     */
#define SYSCON_STARTRSRP0CLR_RSRPIO0_8  (1 << 8)           /*!< Start Logic Clear for PIO0_8     */
#define SYSCON_STARTRSRP0CLR_RSRPIO0_9  (1 << 9)           /*!< Start Logic Clear for PIO0_9     */
#define SYSCON_STARTRSRP0CLR_RSRPIO0_10 (1 << 10)          /*!< Start Logic Clear for PIO0_10    */
#define SYSCON_STARTRSRP0CLR_RSRPIO0_11 (1 << 11)          /*!< Start Logic Clear for PIO0_11    */
#define SYSCON_STARTRSRP0CLR_RSRPIO1_0  (1 << 12)          /*!< Start Logic Clear for PIO1_0     */

/** @} */

/** @defgroup SYSCON_STARTSRP0_Bit_Definitions STARTSRP0: Start-Logic Status Register 0
  * @{
  */

#define SYSCON_STARTSRP0_Mask          (0x1fff)            /*!< Start-Logic Status bitmask      */
#define SYSCON_STARTSRP0_SRPIO0_0      (1 << 0)            /*!< Start Logic Status for PIO0_0    */
#define SYSCON_STARTSRP0_SRPIO0_1      (1 << 1)            /*!< Start Logic Status for PIO0_1    */
#define SYSCON_STARTSRP0_SRPIO0_2      (1 << 2)            /*!< Start Logic Status for PIO0_2    */
#define SYSCON_STARTSRP0_SRPIO0_3      (1 << 3)            /*!< Start Logic Status for PIO0_3    */
#define SYSCON_STARTSRP0_SRPIO0_4      (1 << 4)            /*!< Start Logic Status for PIO0_4    */
#define SYSCON_STARTSRP0_SRPIO0_5      (1 << 5)            /*!< Start Logic Status for PIO0_5    */
#define SYSCON_STARTSRP0_SRPIO0_6      (1 << 6)            /*!< Start Logic Status for PIO0_6    */
#define SYSCON_STARTSRP0_SRPIO0_7      (1 << 7)            /*!< Start Logic Status for PIO0_7    */
#define SYSCON_STARTSRP0_SRPIO0_8      (1 << 8)            /*!< Start Logic Status for PIO0_8    */
#define SYSCON_STARTSRP0_SRPIO0_9      (1 << 9)            /*!< Start Logic Status for PIO0_9    */
#define SYSCON_STARTSRP0_SRPIO0_10     (1 << 10)           /*!< Start Logic Status for PIO0_10   */
#define SYSCON_STARTSRP0_SRPIO0_11     (1 << 11)           /*!< Start Logic Status for PIO0_11   */
#define SYSCON_STARTSRP0_SRPIO1_0      (1 << 12)           /*!< Start Logic Status for PIO0_12   */

/** @} */

/** @defgroup SYSCON_PDSLEEPCFG_Bit_Definitions PDSLEEPCFG: Power-Down Sleep Configuration Register
  * @{
  */

#define SYSCON_PDSLEEPCFG_Mask         (0x48)              /*!< Sleep Mode PowerDown Config Mask */
#define SYSCON_PDSLEEPCFG_Required     (0x000018b7UL)      /*!< Required Bits for PDSLEEPCFG     */
#define SYSCON_PDSLEEPCFG_BOD_PD       (1 << 3)            /*!< Power Down BOD on Sleep          */
#define SYSCON_PDSLEEPCFG_WDTOSC_PD    (1 << 6)            /*!< Power Down WDT Osc. on Sleep     */

/* Other bits must be left alone */

/** @} */

/** @defgroup SYSCON_PDAWAKECFG_Bit_Definitions PDAWAKECFG: Power-Down Awake Configuration Register
  * @{
  */

#define SYSCON_PDAWAKECFG_Mask         (0x0f)              /*!< Awake Mode PowerDown Config Mask */
#define SYSCON_PDAWAKECFG_Required     (0x00000ed00UL)     /*!< Required Bits for PDAWAKECFG     */
#define SYSCON_PDAWAKECFG_IRCOUT_PD    (1 << 0)            /*!< Power Down IRC Out in Wake Mode  */
#define SYSCON_PDAWAKECFG_IRC_PD       (1 << 1)            /*!< Power Down IRC Clk in Wake Mode  */
#define SYSCON_PDAWAKECFG_FLASH_PD     (1 << 2)            /*!< Power Down Flash in Wake Mode    */
#define SYSCON_PDAWAKECFG_BOD_PD       (1 << 3)            /*!< Power Down BOD in Wake Mode      */
#define SYSCON_PDAWAKECFG_ADC0_PD      (1 << 4)            /*!< Power Down AtoD 0 in Wake Mode   */
#define SYSCON_PDAWAKECFG_SYSOSC_PD    (1 << 5)            /*!< Power Down Sys Osc. in Wake Mode */
#define SYSCON_PDAWAKECFG_WDTOSC_PD    (1 << 6)            /*!< Power Down WDT Osc. in Wake Mode */
#define SYSCON_PDAWAKECFG_SYSPLL_PD    (1 << 7)            /*!< Power Down Sys PLL in Wake Mode  */

/** @} */

/** @defgroup SYSCON_PDRUNCFG_Bit_Definitions PDRUNCFG: Power-Down Running Configuration Register
  * @{
  */

#define SYSCON_PDRUNCFG_Mask           (0x0f)              /*!< Run Mode Power-Down Config Mask  */
#define SYSCON_PDRUNCFG_Required       (0x00000ed00UL)     /*!< Required Bits for PDRUNCFG       */
#define SYSCON_IRCOUT_PD               (1 << 0)            /*!< Power down IRC Out in Run Mode   */
#define SYSCON_IRC_PD                  (1 << 1)            /*!< Power down IRC clock in Run Mode */
#define SYSCON_FLASH_PD                (1 << 2)            /*!< Power down FLASH in Run Mode     */
#define SYSCON_BOD_PD                  (1 << 3)            /*!< Power down BOD in Run Mode       */
#define SYSCON_PDRUNCFG_ADC0_PD        (1 << 4)            /*!< Power down AtoD 0 in Run Mode    */
#define SYSCON_PDRUNCFG_SYSOSC_PD      (1 << 5)            /*!< Power down Sys Osc in Run Mode   */
#define SYSCON_PDRUNCFG_WDTOSC_PD      (1 << 6)            /*!< Power down WDT Osc in Run Mode   */
#define SYSCON_PDRUNCFG_PLL_PD         (1 << 7)            /*!< Power down Sys PLL in Run Mode   */

/** @} */

/**
  * @}
  */


/** @defgroup CT16B_Register_Bit_Definitions CT16B Register Bit Definitions
  * @ingroup  CT16B
  * @{
  */

/** @defgroup CT16B_IR_Bit_Definitions IR: Interrupt Register
  * @{
  */

#define CT16B_IR_Mask                  (0x1f)              /*!< Usable bits in IR                */

#define CT16B_IR_MR0                   (1 << 0)            /*!< Match Channel 0 Interrupt Flag   */
#define CT16B_IR_MR1                   (1 << 1)            /*!< Match Channel 1 Interrupt Flag   */
#define CT16B_IR_MR2                   (1 << 2)            /*!< Match Channel 2 Interrupt Flag   */
#define CT16B_IR_MR3                   (1 << 3)            /*!< Match Channel 3 Interrupt Flag   */
#define CT16B_IR_CR0                   (1 << 4)            /*!< Capture Channel 0 Interrupt Flag */

/** @} */

/** @defgroup CT16B_TCR_Bit_Definitions TCR: Control Register
  * @{
  */

#define CT16B_TCR_Mask                 (0x03)              /*!< Usable bits in TCR               */

#define CT16B_CE                       (1 << 0)            /*!< Counter Enable                   */
#define CT16B_CR                       (1 << 1)            /*!< Counter Reset                    */

/** @} */

/** @defgroup CT16B_TC_Bit_Definitions TC: Timer Counter Register
  * @{
  */

#define CT16B_TC_Mask                  (0xffff)            /*!< Mask for Timer Counter Count     */

/** @} */

/** @defgroup CT16B_PR_Bit_Definitions PR: Prescale Register
  * @{
  */

#define CT16B_PR_Mask                  (0xffff)            /*!< Mask for Prescale Register Value */

/** @} */

/** @defgroup CT16B_PC_Bit_Definitions PC: Prescale Counter Register
  * @{
  */

#define CT16B_PC_Mask                  (0xffff)            /*!< Mask for Prescale Counter Value  */

/** @} */

/** @defgroup CT16B_MCR_Bit_Definitions MCR: Match Control Register
  * @{
  */

#define CT16B_MCR_Mask                 (0x0fff)            /*!< Usable bits in MCRn              */

#define CT16B_MR0I                     (1 << 0)            /*!< Interrupt when MR0 matches TC    */
#define CT16B_MR0R                     (1 << 1)            /*!< Reset if MR0 matches TC          */
#define CT16B_MR0S                     (1 << 2)            /*!< Stop if MR0 matches TC           */
#define CT16B_MR1I                     (1 << 3)            /*!< Interrupt when MR1 matches TC    */
#define CT16B_MR1R                     (1 << 4)            /*!< Reset if MR1 matches TC          */
#define CT16B_MR1S                     (1 << 5)            /*!< Stop if MR1 matches TC           */
#define CT16B_MR2I                     (1 << 6)            /*!< Interrupt when MR2 matches TC    */
#define CT16B_MR2R                     (1 << 7)            /*!< Reset if MR2 matches TC          */
#define CT16B_MR2S                     (1 << 8)            /*!< Stop if MR2 matches TC           */
#define CT16B_MR3I                     (1 << 9)            /*!< Interrupt when MR3 matches TC    */
#define CT16B_MR3R                     (1 << 10)           /*!< Reset if MR3 matches TC          */
#define CT16B_MR3S                     (1 << 11)           /*!< Stop if MR3 matches TC           */

/** @} */

/** @defgroup CT16B_CCR_Bit_Definitions CCR: Capture Control Register
  * @{
  */

#define CT16B_CCR_Mask                 (0x07)              /*!< Usable bits in CCR              */

#define CT16B_CAP0RE                   (1 << 0)            /*!< Capture on CAP0 rising edge     */
#define CT16B_CAP0FE                   (1 << 1)            /*!< Capture on CAP0 falling edge    */
#define CT16B_CAP0I                    (1 << 2)            /*!< Enable Interrupt on Capt Event  */

/** @} */

/** @defgroup CT16B_CR0_Bit_Definitions CR0: Capture Register 0
  * @{
  */

#define CT16B_CR0_Mask                 (0xffff)            /*!< Mask for Capture Value           */

/** @} */

/** @defgroup CT16B_EMR_Bit_Definitions EMR: External Match Register
  * @{
  */

#define CT16B_EMR_Mask                 (0x0fff)            /*!< Usable bits in EMR               */

#define CT16B_EM0                      (1 << 0)            /*!< Status of MAT0 Pin               */
#define CT16B_EM1                      (1 << 1)            /*!< Status of MAT1 Pin               */
#define CT16B_EM2                      (1 << 2)            /*!< Status of MAT2 Pin               */
#define CT16B_EM3                      (1 << 3)            /*!< Status of MAT3 Pin               */

#define CT16B_EMC0_Mask                (0x03 << 4)         /*!< MAT0 Match Control Bits          */
#define CT16B_EMC0_Shift               (4)                 /*!< Bit shift of EMC0 Mask           */

#define CT16B_EMC1_Mask                (0x03 << 6)         /*!< MAT1 Match Control Bits          */
#define CT16B_EMC1_Shift               (6)                 /*!< Bit shift of EMC1 Mask           */

#define CT16B_EMC2_Mask                (0x03 << 8)         /*!< MAT2 Match Control Bits          */
#define CT16B_EMC2_Shift               (8)                 /*!< Bit shift of EMC2 Mask           */

#define CT16B_EMC3_Mask                (0x03 << 10)        /*!< MAT3 Match Control Bits          */
#define CT16B_EMC3_Shift               (10)                /*!< Bit shift of EMC3 Mask           */

/** @} */

/** @defgroup CT16B_CTCR_Bit_Definitions CTCR: Timer Count Control Registe
  * @{
  */

#define CT16B_CTCR_Mask                (0x0f)              /*!< Usable bits in CTCR              */

#define CT16B_MODE_Mask                (0x03)              /*!< Count/Timer Mode Mask            */
#define CT16B_MODE_Shift               (0)                 /*!< Bit shift of MODE Mask           */
#define CT16B_MODE_TIMER               (0x00)              /*!< Timer Mode                       */
#define CT16B_MODE_COUNT_RISE          (0x01)              /*!< Count Rising Edges               */
#define CT16B_MODE_COUNT_FALL          (0x02)              /*!< Count Falling Edges              */
#define CT16B_MODE_COUNT_BOTH          (0x03)              /*!< Count Both Edges                 */

#define CT16B_COUNT_INPUT_Mask         (0x03 << 2)         /*!< Counter Mode Input Channel       */
#define CT16B_COUNT_INPUT_Shift        (2)                 /*!< Bit shift of COUNT_INPUT Mask    */
#define CT16B_COUNT_INPUT_CAP0         (0x00)              /*!< Counter Input CAP0 Pin           */

/** @} */

/** @defgroup CT16B_PWMC_Bit_Definitions PWMC: PWM Control Register
  * @{
  */

#define CT16B_PWMC_Mask                (0x0f)              /*!< Ueable Bits in PWMC              */

#define CT16B_PWM0E                    (1 << 0)            /*!< Enable PWM0                      */
#define CT16B_PWM1E                    (1 << 1)            /*!< Enable PWM1                      */
#define CT16B_PWM2E                    (1 << 2)            /*!< Enable PWM2                      */
#define CT16B_PWM3E                    (1 << 3)            /*!< Enable PWM3                      */

/** @} */

/**
  * @}
  */


/** @defgroup CT32B_Register_Bit_Definitions CT32B Register Bit Definitions
  * @ingroup  CT32B
  * @{
  */

/** @defgroup CT32B_IR_Bit_Definitions IR: Interrupt Register
  * @{
  */

#define CT32B_IR_Mask                  (0x1f)              /*!< Usable bits in IR                */

#define CT32B_IT_MR0                   (1 << 0)            /*!< Match Channel 0 Interrupt Flag   */
#define CT32B_IT_MR1                   (1 << 1)            /*!< Match Channel 1 Interrupt Flag   */
#define CT32B_IT_MR2                   (1 << 2)            /*!< Match Channel 2 Interrupt Flag   */
#define CT32B_IT_MR3                   (1 << 3)            /*!< Match Channel 3 Interrupt Flag   */
#define CT32B_IT_CR0                   (1 << 4)            /*!< Capture Channel 0 Interrupt Flag */

/** @} */

/** @defgroup CT32B_TCR_Bit_Definitions TCR: Control Register
  * @{
  */

#define CT32B_TCR_Mask                 (0x03)              /*!< Usable bits in TCR               */

#define CT32B_CE                       (1 << 0)            /*!< Counter Enable                   */
#define CT32B_CR                       (1 << 1)            /*!< Counter Reset                    */

/** @} */

/** @defgroup CT32B_TC_Bit_Definitions TC: Timer Counter Register
  * @{
  */

#define CT32B_TC_Mask                  (0xffffffffUL)      /*!< Mask for Timer Ctr Count         */

/** @} */

/** @defgroup CT32B_PR_Bit_Definitions PR: Prescale Register
  * @{
  */

#define CT32B_PR_Mask                  (0xffffffffUL)      /*!< Mask for Prescale Reg Value      */

/** @} */

/** @defgroup CT32B_PC_Bit_Definitions PC: Prescale Counter Register
  * @{
  */

#define CT32B_PC_Mask                  (0xffffffffUL)      /*!< Mask for Prescale Ctr Value      */

/** @} */

/** @defgroup CT32B_MCR_Bit_Definitions MCR: Match Control Register
  * @{
  */

#define CT32B_MCR_Mask                 (0x0fff)            /*!< Usable bits in MCRn              */

#define CT32B_MR0I                     (1 << 0)            /*!< Interrupt when MR0 matches TC    */
#define CT32B_MR0R                     (1 << 1)            /*!< Reset if MR0 matches TC          */
#define CT32B_MR0S                     (1 << 2)            /*!< Stop if MR0 matches TC           */
#define CT32B_MR1I                     (1 << 3)            /*!< Interrupt when MR1 matches TC    */
#define CT32B_MR1R                     (1 << 4)            /*!< Reset if MR1 matches TC          */
#define CT32B_MR1S                     (1 << 5)            /*!< Stop if MR1 matches TC           */
#define CT32B_MR2I                     (1 << 6)            /*!< Interrupt when MR2 matches TC    */
#define CT32B_MR2R                     (1 << 7)            /*!< Reset if MR2 matches TC          */
#define CT32B_MR2S                     (1 << 8)            /*!< Stop if MR2 matches TC           */
#define CT32B_MR3I                     (1 << 9)            /*!< Interrupt when MR3 matches TC    */
#define CT32B_MR3R                     (1 << 10)           /*!< Reset if MR3 matches TC          */
#define CT32B_MR3S                     (1 << 11)           /*!< Stop if MR3 matches TC           */

/** @} */

/** @defgroup CT32B_CCR_Bit_Definitions CCR: Capture Control Register
  * @{
  */

#define CT32B_CCR_Mask                 (0x07)              /*!< Usable bits in CCR               */

#define CT32B_CAP0RE                   (1 << 0)            /*!< Capture on CAP0 rising edge      */
#define CT32B_CAP0FE                   (1 << 1)            /*!< Capture on CAP0 falling edge     */
#define CT32B_CAP0I                    (1 << 2)            /*!< Enable Interrupt on Capt. Event  */

/** @} */

/** @defgroup CT32B_CR0_Bit_Definitions CR0: Capture Register 0
  * @{
  */

#define CT32B_CR0_Mask                 (0xffffffffUL)      /*!< Mask for Capture Value           */

/** @} */

/** @defgroup CT32B_EMR_Bit_Definitions EMR: External Match Register
  * @{
  */

#define CT32B_EMR_Mask                 (0x0fff)            /*!< Usable bits in EMR               */

#define CT32B_EM0                      (1 << 0)            /*!< Status of MAT0 Pin               */
#define CT32B_EM1                      (1 << 1)            /*!< Status of MAT1 Pin               */
#define CT32B_EM2                      (1 << 2)            /*!< Status of MAT2 Pin               */
#define CT32B_EM3                      (1 << 3)            /*!< Status of MAT3 Pin               */

#define CT32B_EMC0_Mask                (0x03 << 4)         /*!< MAT0 Match Control Bits          */
#define CT32B_EMC0_Shift               (4)                 /*!< Bit shift of EMC0 Mask           */
#define CT32B_EMC1_Mask                (0x03 << 6)         /*!< MAT1 Match Control Bits          */
#define CT32B_EMC1_Shift               (6)                 /*!< Bit shift of EMC1 Mask           */
#define CT32B_EMC2_Mask                (0x03 << 8)         /*!< MAT2 Match Control Bits          */
#define CT32B_EMC2_Shift               (8)                 /*!< Bit shift of EMC2 Mask           */
#define CT32B_EMC3_Mask                (0x03 << 10)        /*!< MAT3 Match Control Bits          */
#define CT32B_EMC3_Shift               (10)                /*!< Bit shift of EMC3 Mask           */

/** @} */

/** @defgroup CT32B_CTCR_Bit_Definitions CTCR: Timer Count Control Register
  * @{
  */

#define CT32B_CTCR_Mask                (0x0f)              /*!< Usable bits in CTCR              */

#define CT32B_MODE_Mask                (0x03)              /*!< CT32B Count/Timer Mode Mask      */
#define CT32B_MODE_Shift               (0)                 /*!< Bit shift of MODE Mask           */
#define CT32B_MODE_TIMER               (0x00)              /*!< Timer Mode                       */
#define CT32B_MODE_COUNT_RISE          (0x01)              /*!< Count Rising Edges               */
#define CT32B_MODE_COUNT_FALL          (0x02)              /*!< Count Falling Edges              */
#define CT32B_MODE_COUNT_BOTH          (0x03)              /*!< Count Both Edges                 */

#define CT32B_COUNT_INPUT_Mask         (0x03 << 2)         /*!< Counter Mode Input Channel       */
#define CT32B_COUNT_INPUT_Shift        (2)                 /*!< Bit shift of COUNT_INPUT Mask    */
#define CT32B_COUNT_INPUT_CAP0         (0x00)              /*!< Count CAP0 Transitions           */

/** @} */

/** @defgroup CT32B_PWMC_Bit_Definitions PWMC: PWM Control Register
  * @{
  */

#define CT32B_PWMC_Mask                (0x0f)              /*!< Usable bits in PWMC              */

#define CT32B_PWM0E                    (1 << 0)            /*!< Enable PWM0                      */
#define CT32B_PWM1E                    (1 << 1)            /*!< Enable PWM1                      */
#define CT32B_PWM2E                    (1 << 2)            /*!< Enable PWM2                      */
#define CT32B_PWM3E                    (1 << 3)            /*!< Enable PWM3                      */

/** @} */

/**
  * @}
  */


/** @defgroup GPIO_Register_Bit_Definitions GPIO Register Bit Definitions
  * @ingroup  GPIO
  * @{
  */

/** @defgroup GPIO_DATA_Bit_Definitions DATAn: Data Registers
  * @{
  */

#define GPIO_DATA_Mask                 (0x0fff)            /*!< Bitmask of all GPIO data bits    */
#define GPIO_D0                        (1 << 0)            /*!< Pin 0  Data                      */
#define GPIO_D1                        (1 << 1)            /*!< Pin 1  Data                      */
#define GPIO_D2                        (1 << 2)            /*!< Pin 2  Data                      */
#define GPIO_D3                        (1 << 3)            /*!< Pin 3  Data                      */
#define GPIO_D4                        (1 << 4)            /*!< Pin 4  Data                      */
#define GPIO_D5                        (1 << 5)            /*!< Pin 5  Data                      */
#define GPIO_D6                        (1 << 6)            /*!< Pin 6  Data                      */
#define GPIO_D7                        (1 << 7)            /*!< Pin 7  Data                      */
#define GPIO_D8                        (1 << 8)            /*!< Pin 8  Data                      */
#define GPIO_D9                        (1 << 9)            /*!< Pin 9  Data                      */
#define GPIO_D10                       (1 << 10)           /*!< Pin 10 Data                      */
#define GPIO_D11                       (1 << 11)           /*!< Pin 11 Data                      */

/** @} */

/** @defgroup GPIO_DIR_Bit_Definitions DIR: Direction Register
  * @{
  */

#define GPIO_DIR_Mask                  (0x0fff)            /*!< Bitmask of all GPIO dir bits     */
#define GPIO_DIR0                      (1 << 0)            /*!< Pin 0  Direction (0=IN / 1=OUT)  */
#define GPIO_DIR1                      (1 << 1)            /*!< Pin 1  Direction (0=IN / 1=OUT)  */
#define GPIO_DIR2                      (1 << 2)            /*!< Pin 2  Direction (0=IN / 1=OUT)  */
#define GPIO_DIR3                      (1 << 3)            /*!< Pin 3  Direction (0=IN / 1=OUT)  */
#define GPIO_DIR4                      (1 << 4)            /*!< Pin 4  Direction (0=IN / 1=OUT)  */
#define GPIO_DIR5                      (1 << 5)            /*!< Pin 5  Direction (0=IN / 1=OUT)  */
#define GPIO_DIR6                      (1 << 6)            /*!< Pin 6  Direction (0=IN / 1=OUT)  */
#define GPIO_DIR7                      (1 << 7)            /*!< Pin 7  Direction (0=IN / 1=OUT)  */
#define GPIO_DIR8                      (1 << 8)            /*!< Pin 8  Direction (0=IN / 1=OUT)  */
#define GPIO_DIR9                      (1 << 9)            /*!< Pin 9  Direction (0=IN / 1=OUT)  */
#define GPIO_DIR10                     (1 << 10)           /*!< Pin 10 Direction (0=IN / 1=OUT)  */
#define GPIO_DIR11                     (1 << 11)           /*!< Pin 11 Direction (0=IN / 1=OUT)  */

/** @} */

/** @defgroup GPIO_ISENSE_Bit_Definitions ISENSE: Interrupt Sense Control Register
  * @{
  */

#define GPIO_IS_Mask                   (0x0fff)            /*!< Bitmask of all GPIO int senses   */
#define GPIO_IS0                       (1 << 0)            /*!< Pin 0  Int Sense (0=Edge/1=Lvl)  */
#define GPIO_IS1                       (1 << 1)            /*!< Pin 1  Int Sense (0=Edge/1=Lvl)  */
#define GPIO_IS2                       (1 << 2)            /*!< Pin 2  Int Sense (0=Edge/1=Lvl)  */
#define GPIO_IS3                       (1 << 3)            /*!< Pin 3  Int Sense (0=Edge/1=Lvl)  */
#define GPIO_IS4                       (1 << 4)            /*!< Pin 4  Int Sense (0=Edge/1=Lvl)  */
#define GPIO_IS5                       (1 << 5)            /*!< Pin 5  Int Sense (0=Edge/1=Lvl)  */
#define GPIO_IS6                       (1 << 6)            /*!< Pin 6  Int Sense (0=Edge/1=Lvl)  */
#define GPIO_IS7                       (1 << 7)            /*!< Pin 7  Int Sense (0=Edge/1=Lvl)  */
#define GPIO_IS8                       (1 << 8)            /*!< Pin 8  Int Sense (0=Edge/1=Lvl)  */
#define GPIO_IS9                       (1 << 9)            /*!< Pin 9  Int Sense (0=Edge/1=Lvl)  */
#define GPIO_IS10                      (1 << 10)           /*!< Pin 10 Int Sense (0=Edge/1=Lvl)  */
#define GPIO_IS11                      (1 << 11)           /*!< Pin 11 Int Sense (0=Edge/1=Lvl)  */

/** @} */

/** @defgroup GPIO_IBE_Bit_Definitions IBE: Interrupt Both Edges Register
  * @{
  */

#define GPIO_IBE_Mask                  (0x0fff)            /*!< Bitmask of all "both edges" ints */
#define GPIO_IBE0                      (1 << 0)            /*!< Pin 0  Int Both Edges (1 = Both) */
#define GPIO_IBE1                      (1 << 1)            /*!< Pin 1  Int Both Edges (1 = Both) */
#define GPIO_IBE2                      (1 << 2)            /*!< Pin 2  Int Both Edges (1 = Both) */
#define GPIO_IBE3                      (1 << 3)            /*!< Pin 3  Int Both Edges (1 = Both) */
#define GPIO_IBE4                      (1 << 4)            /*!< Pin 4  Int Both Edges (1 = Both) */
#define GPIO_IBE5                      (1 << 5)            /*!< Pin 5  Int Both Edges (1 = Both) */
#define GPIO_IBE6                      (1 << 6)            /*!< Pin 6  Int Both Edges (1 = Both) */
#define GPIO_IBE7                      (1 << 7)            /*!< Pin 7  Int Both Edges (1 = Both) */
#define GPIO_IBE8                      (1 << 8)            /*!< Pin 8  Int Both Edges (1 = Both) */
#define GPIO_IBE9                      (1 << 9)            /*!< Pin 9  Int Both Edges (1 = Both) */
#define GPIO_IBE10                     (1 << 10)           /*!< Pin 10 Int Both Edges (1 = Both) */
#define GPIO_IBE11                     (1 << 11)           /*!< Pin 11 Int Both Edges (1 = Both) */

/** @} */

/** @defgroup GPIO_IEV_Bit_Definitions IEV: Interrupt Event Level Register
  * @{
  */

#define GPIO_IEV_Mask                  (0x0fff)            /*!< Bitmask of all GPIO event levels */
#define GPIO_IEV0                      (1 << 0)            /*!< Pin 0  Int Event Level (0=L,1=H) */
#define GPIO_IEV1                      (1 << 1)            /*!< Pin 1  Int Event Level (0=L,1=H) */
#define GPIO_IEV2                      (1 << 2)            /*!< Pin 2  Int Event Level (0=L,1=H) */
#define GPIO_IEV3                      (1 << 3)            /*!< Pin 3  Int Event Level (0=L,1=H) */
#define GPIO_IEV4                      (1 << 4)            /*!< Pin 4  Int Event Level (0=L,1=H) */
#define GPIO_IEV5                      (1 << 5)            /*!< Pin 5  Int Event Level (0=L,1=H) */
#define GPIO_IEV6                      (1 << 6)            /*!< Pin 6  Int Event Level (0=L,1=H) */
#define GPIO_IEV7                      (1 << 7)            /*!< Pin 7  Int Event Level (0=L,1=H) */
#define GPIO_IEV8                      (1 << 8)            /*!< Pin 8  Int Event Level (0=L,1=H) */
#define GPIO_IEV9                      (1 << 9)            /*!< Pin 9  Int Event Level (0=L,1=H) */
#define GPIO_IEV10                     (1 << 10)           /*!< Pin 10 Int Event Level (0=L,1=H) */
#define GPIO_IEV11                     (1 << 11)           /*!< Pin 11 Int Event Level (0=L,1=H) */

/** @} */

/** @defgroup GPIO_IE_and_MIS_Bit_Definitions (IE, MIS): Interrupt Enable, Interrupt Mask Registers
  * @{
  */

#define GPIO_MASK_Mask                 (0x0fff)            /*!< Bitmask of all GPIO interrupts   */
#define GPIO_MASK0                     (1 << 0)            /*!< Pin 0  Interrupt Mask(ed)        */
#define GPIO_MASK1                     (1 << 1)            /*!< Pin 1  Interrupt Mask(ed)        */
#define GPIO_MASK2                     (1 << 2)            /*!< Pin 2  Interrupt Mask(ed)        */
#define GPIO_MASK3                     (1 << 3)            /*!< Pin 3  Interrupt Mask(ed)        */
#define GPIO_MASK4                     (1 << 4)            /*!< Pin 4  Interrupt Mask(ed)        */
#define GPIO_MASK5                     (1 << 5)            /*!< Pin 5  Interrupt Mask(ed)        */
#define GPIO_MASK6                     (1 << 6)            /*!< Pin 6  Interrupt Mask(ed)        */
#define GPIO_MASK7                     (1 << 7)            /*!< Pin 7  Interrupt Mask(ed)        */
#define GPIO_MASK8                     (1 << 8)            /*!< Pin 8  Interrupt Mask(ed)        */
#define GPIO_MASK9                     (1 << 9)            /*!< Pin 9  Interrupt Mask(ed)        */
#define GPIO_MASK10                    (1 << 10)           /*!< Pin 10 Interrupt Mask(ed)        */
#define GPIO_MASK11                    (1 << 11)           /*!< Pin 11 Interrupt Mask(ed)        */

/** @} */

/** @defgroup GPIO_IRS_Bit_Definitions IRS: Interrupt Raw Status Register
  * @{
  */

#define GPIO_IRS_Mask                  (0x0fff)            /*!< Bitmask of all GPIO raw ints     */
#define GPIO_RAWST0                    (1 << 0)            /*!< Pin 0  Raw Int Status            */
#define GPIO_RAWST1                    (1 << 1)            /*!< Pin 1  Raw Int Status            */
#define GPIO_RAWST2                    (1 << 2)            /*!< Pin 2  Raw Int Status            */
#define GPIO_RAWST3                    (1 << 3)            /*!< Pin 3  Raw Int Status            */
#define GPIO_RAWST4                    (1 << 4)            /*!< Pin 4  Raw Int Status            */
#define GPIO_RAWST5                    (1 << 5)            /*!< Pin 5  Raw Int Status            */
#define GPIO_RAWST6                    (1 << 6)            /*!< Pin 6  Raw Int Status            */
#define GPIO_RAWST7                    (1 << 7)            /*!< Pin 7  Raw Int Status            */
#define GPIO_RAWST8                    (1 << 8)            /*!< Pin 8  Raw Int Status            */
#define GPIO_RAWST9                    (1 << 9)            /*!< Pin 9  Raw Int Status            */
#define GPIO_RAWST10                   (1 << 10)           /*!< Pin 10 Raw Int Status            */
#define GPIO_RAWST11                   (1 << 11)           /*!< Pin 11 Raw Int Status            */

/** @} */

/** @defgroup GPIO_IC_Bit_Definitions IC: Interrupt Clear Register
  * @{
  */

#define GPIO_CLR_Mask                  (0x0fff)            /*!< Bitmask of all GPIO int clears   */
#define GPIO_CLR0                      (1 << 0)            /*!< Pin 0  Clear Interrupt           */
#define GPIO_CLR1                      (1 << 1)            /*!< Pin 1  Clear Interrupt           */
#define GPIO_CLR2                      (1 << 2)            /*!< Pin 2  Clear Interrupt           */
#define GPIO_CLR3                      (1 << 3)            /*!< Pin 3  Clear Interrupt           */
#define GPIO_CLR4                      (1 << 4)            /*!< Pin 4  Clear Interrupt           */
#define GPIO_CLR5                      (1 << 5)            /*!< Pin 5  Clear Interrupt           */
#define GPIO_CLR6                      (1 << 6)            /*!< Pin 6  Clear Interrupt           */
#define GPIO_CLR7                      (1 << 7)            /*!< Pin 7  Clear Interrupt           */
#define GPIO_CLR8                      (1 << 8)            /*!< Pin 8  Clear Interrupt           */
#define GPIO_CLR9                      (1 << 9)            /*!< Pin 9  Clear Interrupt           */
#define GPIO_CLR10                     (1 << 10)           /*!< Pin 10 Clear Interrupt           */
#define GPIO_CLR11                     (1 << 11)           /*!< Pin 11 Clear Interrupt           */

/** @} */

/**
  * @}
  */


#if defined(LPC11CXX)

/** @defgroup CAN_Register_Bit_Definitions CAN Register Bit Definitions
  * @ingroup  CAN
  * @{
  */

/** @defgroup CAN_CNTL_Bit_Definitions CNTL: CAN Control Register
  * @{
  */

#define CAN_CNTL_Mask                  (0x00ef)            /*!< Usable bits in CNTL register     */

#define CAN_INIT                       (1 << 0)            /*!< Initialization is Started        */
#define CAN_IE                         (1 << 1)            /*!< CAN Module Interrupt Enable      */
#define CAN_SIE                        (1 << 2)            /*!< Status Change Interrupt Enable   */
#define CAN_EIE                        (1 << 3)            /*!< Error Interrupt Enable           */
#define CAN_DAR                        (1 << 5)            /*!< Disable Auto-Retransmit          */
#define CAN_CCE                        (1 << 6)            /*!< Configuration Change Enable      */
#define CAN_TEST                       (1 << 7)            /*!< TEST Mode Enable                 */

/** @} */

/** @defgroup CAN_STAT_Bit_Definitions STAT: CAN Status Register
  * @{
  */

#define CAN_STAT_Mask                  (0x00ff)            /*!< Usable bits in STATUS register   */

#define CAN_LEC_Mask                   (0x0007)            /*!< "Last Error Code" bitmask        */
#define CAN_LEC_Shift                  (0)                 /*!< Bit shift of LEC mask            */

#define CAN_LEC_NO_ERROR               (0x00)              /*!< No Error.                        */
#define CAN_LEC_STUFF_ERROR            (0x01)              /*!< Bit Stuff Error                  */
#define CAN_LEC_FORM_ERROR             (0x02)              /*!< Bad Message Format               */
#define CAN_LEC_ACK_ERROR              (0x03)              /*!< Message was not Ack'd            */
#define CAN_LEC_BIT1_ERROR             (0x04)              /*!< Tx'd recessive 1 bit forced low  */
#define CAN_LEC_BIT0_ERROR             (0x05)              /*!< Tx'd dominant 0 bit forced high  */
#define CAN_LEC_CRC_ERROR              (0x06)              /*!< Bad CRC Checksum                 */

#define CAN_TXOK                       (1 << 3)            /*!< Tx was Successful (reset by cpu) */
#define CAN_RXOK                       (1 << 4)            /*!< Rx'd Successfully (reset by cpu) */
#define CAN_EPASS                      (1 << 5)            /*!< Error Passive State         (RO) */
#define CAN_EWARN                      (1 << 6)            /*!< At least 1 err ctr >= 96    (RO) */
#define CAN_BOFF                       (1 << 7)            /*!< Ctrlr is in BUSOFF state    (RO) */

/** @} */

/** @defgroup CAN_EC_Bit_Definitions EC: CAN Error Counter Register (RO)
  * @{
  */

#define CAN_EC_Mask                    (0xffff)            /*!< Usable bits in EC register       */

#define CAN_TEC_Mask                   (0x00ff)            /*!< Transmit Error Count bitmask     */
#define CAN_TEC_Shift                  (0)                 /*!< Bit shift of TEC mask            */

#define CAN_REC_Mask                   (0x007f << 8)       /*!< Receive Error Count bitmask      */
#define CAN_REC_Shift                  (8)                 /*!< Bit shift of REC mask            */

#define CAN_RP                         (1 << 15)           /*!< Receive Error Passive            */

/** @} */

/** @defgroup CAN_BT_Bit_Definitions BT: CAN Bit Timing Register
  * @{
  */

#define CAN_BT_Mask                    (0x7fff)            /*!< Useable bits in BT register      */

#define CAN_BRP_Mask                   (0x002f)            /*!< Baud Rate Prescaler Bitmask      */
#define CAN_BRP_Shift                  (0)                 /*!< Bit shift of BRP mask            */

#define CAN_SJW_Mask                   (0x0003 << 6)       /*!< Resync Jump Width Bitmask        */
#define CAN_SJW_Shift                  (6)                 /*!< Bit shift of SJW mask            */

#define CAN_TSEG1_Mask                 (0x000f << 8)       /*!< Time Segmt Before Sample Point   */
#define CAN_TSEG1_Shift                (8)                 /*!< Bit shift of TSEG1 mask          */

#define CAN_TSEG2_Mask                 (0x0007 << 12)      /*!< Time Segmt After Sample Point    */
#define CAN_TSEG2_Shift                (12)                 /*!< Bit shift of TSEG2 mask         */

/** @} */

/** @defgroup CAN_INT_Bit_Definitions INT: CAN Interrupt Register (RO)
  * @{
  */

#define CAN_INTID_Mask                 (0xffff)            /*!< CAN Interrupt ID Bitmask         */
#define CAN_INTID_NONE                 (0x0000)            /*!< No Interrupt Pending             */
#define CAN_INTID_MESSAGE_1            (0x0001)            /*!< Interrupt from Message Object 1  */
#define CAN_INTID_MESSAGE_2            (0x0002)            /*!< Interrupt from Message Object 2  */
#define CAN_INTID_MESSAGE_3            (0x0003)            /*!< Interrupt from Message Object 3  */
#define CAN_INTID_MESSAGE_4            (0x0004)            /*!< Interrupt from Message Object 4  */
#define CAN_INTID_MESSAGE_5            (0x0005)            /*!< Interrupt from Message Object 5  */
#define CAN_INTID_MESSAGE_6            (0x0006)            /*!< Interrupt from Message Object 6  */
#define CAN_INTID_MESSAGE_7            (0x0007)            /*!< Interrupt from Message Object 7  */
#define CAN_INTID_MESSAGE_8            (0x0008)            /*!< Interrupt from Message Object 8  */
#define CAN_INTID_MESSAGE_9            (0x0009)            /*!< Interrupt from Message Object 9  */
#define CAN_INTID_MESSAGE_10           (0x000a)            /*!< Interrupt from Message Object 10 */
#define CAN_INTID_MESSAGE_11           (0x000b)            /*!< Interrupt from Message Object 11 */
#define CAN_INTID_MESSAGE_12           (0x000c)            /*!< Interrupt from Message Object 12 */
#define CAN_INTID_MESSAGE_13           (0x000d)            /*!< Interrupt from Message Object 13 */
#define CAN_INTID_MESSAGE_14           (0x000e)            /*!< Interrupt from Message Object 14 */
#define CAN_INTID_MESSAGE_15           (0x000f)            /*!< Interrupt from Message Object 15 */
#define CAN_INTID_MESSAGE_16           (0x0010)            /*!< Interrupt from Message Object 16 */
#define CAN_INTID_MESSAGE_17           (0x0011)            /*!< Interrupt from Message Object 17 */
#define CAN_INTID_MESSAGE_18           (0x0012)            /*!< Interrupt from Message Object 18 */
#define CAN_INTID_MESSAGE_19           (0x0013)            /*!< Interrupt from Message Object 19 */
#define CAN_INTID_MESSAGE_20           (0x0014)            /*!< Interrupt from Message Object 20 */
#define CAN_INTID_MESSAGE_21           (0x0015)            /*!< Interrupt from Message Object 21 */
#define CAN_INTID_MESSAGE_22           (0x0016)            /*!< Interrupt from Message Object 22 */
#define CAN_INTID_MESSAGE_23           (0x0017)            /*!< Interrupt from Message Object 23 */
#define CAN_INTID_MESSAGE_24           (0x0018)            /*!< Interrupt from Message Object 24 */
#define CAN_INTID_MESSAGE_25           (0x0019)            /*!< Interrupt from Message Object 25 */
#define CAN_INTID_MESSAGE_26           (0x001a)            /*!< Interrupt from Message Object 26 */
#define CAN_INTID_MESSAGE_27           (0x001b)            /*!< Interrupt from Message Object 27 */
#define CAN_INTID_MESSAGE_28           (0x001c)            /*!< Interrupt from Message Object 28 */
#define CAN_INTID_MESSAGE_29           (0x001d)            /*!< Interrupt from Message Object 29 */
#define CAN_INTID_MESSAGE_30           (0x001e)            /*!< Interrupt from Message Object 30 */
#define CAN_INTID_MESSAGE_31           (0x001f)            /*!< Interrupt from Message Object 31 */
#define CAN_INTID_MESSAGE_32           (0x0020)            /*!< Interrupt from Message Object 32 */
#define CAN_INTID_STATUS               (0x8000)            /*!< Status Interrupt                 */

/** @} */

/** @defgroup CAN_TEST_Bit_Definitions TEST: CAN Test Register
  * @{
  */

#define CAN_TEST_Mask                  (0x000c)            /*!< Usable bits in TEST register     */

#define CAN_TX_Mask                    (0x0003 << 5)       /*!< TX Line control bitmask          */
#define CAN_TX_Shift                   (5)                 /*!< Bit shift of TX mask             */
#define CAN_TX_NORMAL                  (0x00 << 5)         /*!< TXD line controlled by CAN ctlr  */
#define CAN_TX_MONITOR                 (0x01 << 5)         /*!< TXD line can be monitored        */
#define CAN_TX_LOW                     (0x02 << 5)         /*!< Force TXD low (dominant)         */
#define CAN_TX_HIGH                    (0x03 << 5)         /*!< Force TXD high (recessive)       */

#define CAN_BASIC                      (1 << 2)            /*!< Enable basic mode (IF1/TX,IF2/RX)*/
#define CAN_SILENT                     (1 << 3)            /*!< Force silent operation           */
#define CAN_LBACK                      (1 << 4)            /*!< Enable Loopback mode             */
#define CAN_RX                         (1 << 7)            /*!< Monitor value of RX pin (RO)     */

/** @} */

/** @defgroup CAN_BRPE_Bit_Definitions BRPE: CAN Baud Rate Prescaler Extension Register
  * @{
  */

#define CAN_BRPE_Mask                  (0x0007)            /*!< Baudrate Prescaler Extension bits*/

/** @} */

/** @defgroup CAN_IF_CMDREQ_Bit_Definitions IFxCMDREQ: Message Interface Cmd. Request Reg.
  * @{
  */

#define CAN_IF_CMDREQ_Mask             (0x103f)            /*!< Usable bits in IFxCMDREQ regs    */

#define CAN_IF_MN_Mask                 (0x003f)            /*!< Message Number Bitmask           */
#define CAN_IF_MN_Shift                (0)                 /*!< Bit shift of IFxMN mask          */

#define CAN_BUSY                       (1 << 15)           /*!< CMDREQ Register Busy Flag (RO)   */

/** @} */

/** @defgroup CAN_IF_CMDMSK_Bit_Definitions IFxCMDMSK: Message Interface Command Mask Reg.
  * @{
  */

#define CAN_IF_CMDMSK_Mask             (0x00ff)            /*!< Usable bits in IFxCMDMSK regs    */

#define CAN_IF_DATA_B                  (1 << 0)            /*!< xfer Data Bytes 4-7 to/frm msg   */
#define CAN_IF_DATA_A                  (1 << 1)            /*!< xfer Data Bytes 0-3 to/frm msg   */
#define CAN_IF_TXRQST                  (1 << 2)            /*!< Request Transmission   (WR mode) */
#define CAN_IF_TXRQST                  (1 << 2)            /*!< Clear NEWDAT bit       (RD mode) */
#define CAN_IF_CLRINTPND               (1 << 3)            /*!< Clear Interrupt Pending (RD mode)*/
#define CAN_IF_CTRL                    (1 << 4)            /*!< xfer Ctrl Bits to/from message   */
#define CAN_IF_ARB                     (1 << 5)            /*!< xfer Arbitration bits to/frm msg */
#define CAN_IF_MASK                    (1 << 6)            /*!< xfer ID MASK bits to/from msg    */
#define CAN_IF_WR                      (1 << 7)            /*!< Write Mode                       */

/** @} */

/** @defgroup CAN_IF_MASK1_Bit_Definitions IFxMASK1: Message Interface ID Mask Reg. 1
  * @{
  */

#define CAN_IF_MASK1_Mask              (0xffff)            /*!< Usable bits in IFnMASK1 regs     */

#define CAN_IF_IDMASK1_Mask            (0xffff)            /*!< Bits for ID Mask bits[0:15]      */
#define CAN_IF_IDMASK1_Shift           (0)                 /*!< Bit shift of IFxIDMASK1 mask     */

/** @} */

/** @defgroup CAN_IF_MASK2_Bit_Definitions IFxMASK2: Message Interface ID Mask Register 2
  * @{
  */

#define CAN_IF_MASK2_Mask              (0xdfff)            /*!< Usable bits in IFxMASK2 regs     */

#define CAN_IF_IDMASK2_Mask            (0x0fff)            /*!< Bits for ID Mask bits[28:16]     */
#define CAN_IF_IDMASK2_Shift           (0)                 /*!< Bit shift of IFxIDMASK2 mask     */

#define CAN_IF_MDIR                    (1 << 14)           /*!< DIR bit used for accept filtering*/
#define CAN_IF_MXTD                    (1 << 15)           /*!< XTD bit used for accept filtering*/

/** @} */

/** @defgroup CAN_IF_ARB1_Bit_Definitions IFxARB1: Message Interface Arbitration 1 Reg.
  * @{
  */

#define CAN_IF_ARB1_Mask               (0xffff)            /*!< Usable bits in IFxARB1 registers */

#define CAN_IF_ID1_Mask                (0xffff)            /*!< CAN ID Bits 0-15                 */
#define CAN_IF_ID1_Shift               (0)                 /*!< Bit shift of IFxID1 mask         */

/** @} */

/** @defgroup CAN_IF_ARB2_Bit_Definitions IFxARB2: Message Interface Arbitration 2 Registers
  * @{
  */

#define CAN_IF_ARB2_Mask               (0xffff)            /*!< Usable bits in IFxARB2 registers */

#define CAN_IF_ID2_Mask                (0x0fff)            /*!< CAN ID Bits[16:28]               */
#define CAN_IF_ID2_Shift               (0)                 /*!< Bit shift of IFxID2 mask         */

#define CAN_IF_DIR                     (1 << 13)           /*!< Message Direction is TX          */
#define CAN_IF_XTD                     (1 << 14)           /*!< Extended Identifier (29b vs 11b) */
#define CAN_IF_MSGVAL                  (1 << 15)           /*!< Message Valid (reset by CPU)     */

/** @} */

/** @defgroup CAN_IF_MCTRL_Bit_Definitions IFxMCTRL: Message Interface Message Control Reg
  * @{
  */

#define CAN_IF_MCTRL_Mask              (0xff8f)            /*!< Usable bits in IFxMCTRL regs     */

#define CAN_IF_DLC_Mask                (0x000f)            /*!< Data Length Code of message      */
#define CAN_IF_DLC_Shift               (0)                 /*!< Bit shift of IFxDLC mask         */

#define CAN_IF_EOB                     (1 << 7)            /*!< End of Buffer (single/last obj.) */
#define CAN_IF_TXRQST                  (1 << 8)            /*!< Object waiting to be Tx'd        */
#define CAN_IF_RMTEN                   (1 << 9)            /*!< Remote Enable (TXRQST set on rx) */
#define CAN_IF_RXIE                    (1 << 10)           /*!< Rx Interrupt Enable              */
#define CAN_IF_TXIE                    (1 << 11)           /*!< Tx Interrupt Enable              */
#define CAN_IF_UMASK                   (1 << 12)           /*!< Ena MSK[28:0], MXTD, MDIR filters*/
#define CAN_IF_INTPND                  (1 << 13)           /*!< Interrupt is Pending on msg obj  */
#define CAN_IF_MSGLST                  (1 << 14)           /*!< Message Lost (overwrote a msg)   */
#define CAN_IF_NEWDAT                  (1 << 15)           /*!< New Data Written to msg object   */

/** @} */

/** @defgroup CAN_IF_DA1_Bit_Definitions IFxDA1: Message Interface Data A1 Registers
  * @{
  */

#define CAN_IF_DA1_Mask                (0xffff)            /*!< Usable bits in IFxDA1 registers  */

#define CAN_IF_DATA0_Mask              (0xff)              /*!< Data Byte 0 Bitmask              */
#define CAN_IF_DATA0_Shift             (0)                 /*!< Bit shift of IFxDATA0 mask       */

#define CAN_IF_DATA1_Mask              (0xff << 8)         /*!< Data Byte 1 Bitmask              */
#define CAN_IF_DATA1_Shift             (8)                 /*!< Bit shift of IFxDATA1 mask       */

/** @} */

/** @defgroup CAN_IF_DA2_Bit_Definitions IFxDA2: Message Interface Data A2 Registers
  * @{
  */

#define CAN_IF_DB2_Mask                (0xffff)            /*!< Usable bits in IFxDA2 registers  */

#define CAN_IF_DATA2_Mask              (0xff)              /*!< Data Byte 2 Bitmask              */
#define CAN_IF_DATA2_Shift             (0)                 /*!< Bit shift of IFxDATA2 mask       */

#define CAN_IF_DATA3_Mask              (0xff << 8)         /*!< Data Byte 3 Bitmask              */
#define CAN_IF_DATA3_Shift             (8)                 /*!< Bit shift of IFxDATA3 mask       */

/** @} */

/** @defgroup CAN_IF_DB1_Bit_Definitions IFxDB1: Message Interface Data B1 Registers
  * @{
  */

#define CAN_IF_DB1_Mask                (0xffff)            /*!< Usable bits in IFxDB1 registers  */

#define CAN_IF_DATA4_Mask              (0xff)              /*!< Data Byte 4 Bitmask              */
#define CAN_IF_DATA4_Shift             (0)                 /*!< Bit shift of IFxDATA4 mask       */

#define CAN_IF_DATA5_Mask              (0xff << 8)         /*!< Data Byte 5 Bitmask              */
#define CAN_IF_DATA5_Shift             (8)                 /*!< Bit shift of IFxDATA5 mask       */


/** @} */

/** @defgroup CAN_IF_DB2_Bit_Definitions IFxDB2: Message Interface Data B2 Registers
  * @{
  */

#define CAN_DB2_Mask                   (0xffff)            /*!< Usable bits in IFxDB2 registers  */

#define CAN_DATA6_Mask                 (0xff)              /*!< Data Byte 6 Bitmask              */
#define CAN_DATA6_Shift                (0)                 /*!< Bit shift of IFxDATA6 mask       */

#define CAN_DATA7_Mask                 (0xff << 8)         /*!< Data Byte 7 Bitmask              */
#define CAN_DATA7_Shift                (8)                 /*!< Bit shift of IFxDATA7 mask       */

/** @} */

/** @defgroup CAN_TXREQ1_Bit_Definitions TXREQ1: Transmit Request Register 1 (RO)
  * @{
  */

#define CAN_TXREQ1_Mask                (0xffff)            /*!< Usable bits in TXREQ1 register   */

#define CAN_TXRQST_1                   (1 << 0)            /*!< Message Object 1 Tx Pending      */
#define CAN_TXRQST_2                   (1 << 1)            /*!< Message Object 2 Tx Pending      */
#define CAN_TXRQST_3                   (1 << 2)            /*!< Message Object 3 Tx Pending      */
#define CAN_TXRQST_4                   (1 << 3)            /*!< Message Object 4 Tx Pending      */
#define CAN_TXRQST_5                   (1 << 4)            /*!< Message Object 5 Tx Pending      */
#define CAN_TXRQST_6                   (1 << 5)            /*!< Message Object 6 Tx Pending      */
#define CAN_TXRQST_7                   (1 << 6)            /*!< Message Object 7 Tx Pending      */
#define CAN_TXRQST_8                   (1 << 7)            /*!< Message Object 8 Tx Pending      */
#define CAN_TXRQST_9                   (1 << 8)            /*!< Message Object 9 Tx Pending      */
#define CAN_TXRQST_10                  (1 << 9)            /*!< Message Object 10 Tx Pending     */
#define CAN_TXRQST_11                  (1 << 10)           /*!< Message Object 11 Tx Pending     */
#define CAN_TXRQST_12                  (1 << 11)           /*!< Message Object 12 Tx Pending     */
#define CAN_TXRQST_13                  (1 << 12)           /*!< Message Object 13 Tx Pending     */
#define CAN_TXRQST_14                  (1 << 13)           /*!< Message Object 14 Tx Pending     */
#define CAN_TXRQST_15                  (1 << 14)           /*!< Message Object 15 Tx Pending     */
#define CAN_TXRQST_16                  (1 << 15)           /*!< Message Object 16 Tx Pending     */

/** @} */

/** @defgroup CAN_TXREQ2_Bit_Definitions TXREQ2: Transmit Request Register 2 (RO)
  * @{
  */

#define CAN_TXREQ2_Mask                (0xffff)            /*!< Usable bits in TXREQ2 register   */

#define CAN_TXRQST_17                  (1 << 0)            /*!< Message Object 17 Tx Pending     */
#define CAN_TXRQST_18                  (1 << 1)            /*!< Message Object 18 Tx Pending     */
#define CAN_TXRQST_19                  (1 << 2)            /*!< Message Object 19 Tx Pending     */
#define CAN_TXRQST_20                  (1 << 3)            /*!< Message Object 20 Tx Pending     */
#define CAN_TXRQST_21                  (1 << 4)            /*!< Message Object 21 Tx Pending     */
#define CAN_TXRQST_22                  (1 << 5)            /*!< Message Object 22 Tx Pending     */
#define CAN_TXRQST_23                  (1 << 6)            /*!< Message Object 23 Tx Pending     */
#define CAN_TXRQST_24                  (1 << 7)            /*!< Message Object 24 Tx Pending     */
#define CAN_TXRQST_25                  (1 << 8)            /*!< Message Object 25 Tx Pending     */
#define CAN_TXRQST_26                  (1 << 9)            /*!< Message Object 26 Tx Pending     */
#define CAN_TXRQST_27                  (1 << 10)           /*!< Message Object 27 Tx Pending     */
#define CAN_TXRQST_28                  (1 << 11)           /*!< Message Object 28 Tx Pending     */
#define CAN_TXRQST_29                  (1 << 12)           /*!< Message Object 29 Tx Pending     */
#define CAN_TXRQST_30                  (1 << 13)           /*!< Message Object 30 Tx Pending     */
#define CAN_TXRQST_31                  (1 << 14)           /*!< Message Object 31 Tx Pending     */
#define CAN_TXRQST_32                  (1 << 15)           /*!< Message Object 32 Tx Pending     */

/** @} */

/** @defgroup CAN_ND1_Bit_Definitions ND1: New Data Register 1 (RO)
  * @{
  */

#define CAN_ND1_Mask                   (0xffff)            /*!< Usable bits in ND1 register      */

#define CAN_NEWDAT_1                   (1 << 0)            /*!< Message Object 1 Has New Data    */
#define CAN_NEWDAT_2                   (1 << 1)            /*!< Message Object 2 Has New Data    */
#define CAN_NEWDAT_3                   (1 << 2)            /*!< Message Object 3 Has New Data    */
#define CAN_NEWDAT_4                   (1 << 3)            /*!< Message Object 4 Has New Data    */
#define CAN_NEWDAT_5                   (1 << 4)            /*!< Message Object 5 Has New Data    */
#define CAN_NEWDAT_6                   (1 << 5)            /*!< Message Object 6 Has New Data    */
#define CAN_NEWDAT_7                   (1 << 6)            /*!< Message Object 7 Has New Data    */
#define CAN_NEWDAT_8                   (1 << 7)            /*!< Message Object 8 Has New Data    */
#define CAN_NEWDAT_9                   (1 << 8)            /*!< Message Object 9 Has New Data    */
#define CAN_NEWDAT_10                  (1 << 9)            /*!< Message Object 10 Has New Data   */
#define CAN_NEWDAT_11                  (1 << 10)           /*!< Message Object 11 Has New Data   */
#define CAN_NEWDAT_12                  (1 << 11)           /*!< Message Object 12 Has New Data   */
#define CAN_NEWDAT_13                  (1 << 12)           /*!< Message Object 13 Has New Data   */
#define CAN_NEWDAT_14                  (1 << 13)           /*!< Message Object 14 Has New Data   */
#define CAN_NEWDAT_15                  (1 << 14)           /*!< Message Object 15 Has New Data   */
#define CAN_NEWDAT_16                  (1 << 15)           /*!< Message Object 16 Has New Data   */

/** @} */

/** @defgroup CAN_ND2_Bit_Definitions ND2: New Data Register 2 (RO)
  * @{
  */

#define CAN_ND2_Mask                   (0xffff)            /*!< Useable Bits in ND2 register     */

#define CAN_NEWDAT_17                  (1 << 0)            /*!< Message Object 17 Has New Data   */
#define CAN_NEWDAT_18                  (1 << 1)            /*!< Message Object 18 Has New Data   */
#define CAN_NEWDAT_19                  (1 << 2)            /*!< Message Object 19 Has New Data   */
#define CAN_NEWDAT_20                  (1 << 3)            /*!< Message Object 20 Has New Data   */
#define CAN_NEWDAT_21                  (1 << 4)            /*!< Message Object 21 Has New Data   */
#define CAN_NEWDAT_22                  (1 << 5)            /*!< Message Object 22 Has New Data   */
#define CAN_NEWDAT_23                  (1 << 6)            /*!< Message Object 23 Has New Data   */
#define CAN_NEWDAT_24                  (1 << 7)            /*!< Message Object 24 Has New Data   */
#define CAN_NEWDAT_25                  (1 << 8)            /*!< Message Object 25 Has New Data   */
#define CAN_NEWDAT_26                  (1 << 9)            /*!< Message Object 26 Has New Data   */
#define CAN_NEWDAT_27                  (1 << 10)           /*!< Message Object 27 Has New Data   */
#define CAN_NEWDAT_28                  (1 << 11)           /*!< Message Object 28 Has New Data   */
#define CAN_NEWDAT_29                  (1 << 12)           /*!< Message Object 29 Has New Data   */
#define CAN_NEWDAT_30                  (1 << 13)           /*!< Message Object 30 Has New Data   */
#define CAN_NEWDAT_31                  (1 << 14)           /*!< Message Object 31 Has New Data   */
#define CAN_NEWDAT_32                  (1 << 15)           /*!< Message Object 32 Has New Data   */

/** @} */

/** @defgroup CAN_IR1_Bit_Definitions IR1: Interrupt Pending Register 1 (RO)
  * @{
  */

#define CAN_IR1_Mask                   (0xffff)            /*!< Usable Bits in IR1 register      */

#define CAN_INTPND_1                   (1 << 0)            /*!< Message Object 1 Int. Pending    */
#define CAN_INTPND_2                   (1 << 1)            /*!< Message Object 2 Int. Pending    */
#define CAN_INTPND_3                   (1 << 2)            /*!< Message Object 3 Int. Pending    */
#define CAN_INTPND_4                   (1 << 3)            /*!< Message Object 4 Int. Pending    */
#define CAN_INTPND_5                   (1 << 4)            /*!< Message Object 5 Int. Pending    */
#define CAN_INTPND_6                   (1 << 5)            /*!< Message Object 6 Int. Pending    */
#define CAN_INTPND_7                   (1 << 6)            /*!< Message Object 7 Int. Pending    */
#define CAN_INTPND_8                   (1 << 7)            /*!< Message Object 8 Int. Pending    */
#define CAN_INTPND_9                   (1 << 8)            /*!< Message Object 9 Int. Pending    */
#define CAN_INTPND_10                  (1 << 9)            /*!< Message Object 10 Int. Pending   */
#define CAN_INTPND_11                  (1 << 10)           /*!< Message Object 11 Int. Pending   */
#define CAN_INTPND_12                  (1 << 11)           /*!< Message Object 12 Int. Pending   */
#define CAN_INTPND_13                  (1 << 12)           /*!< Message Object 13 Int. Pending   */
#define CAN_INTPND_14                  (1 << 13)           /*!< Message Object 14 Int. Pending   */
#define CAN_INTPND_15                  (1 << 14)           /*!< Message Object 15 Int. Pending   */
#define CAN_INTPND_16                  (1 << 15)           /*!< Message Object 16 Int. Pending   */

/** @} */

/** @defgroup CAN_IR2_Bit_Definitions IR2: Interrupt Pending Register 2 (RO)
  * @{
  */

#define CAN_IR2_Mask                   (0xffff)            /*!< Usable bits in IR2 register      */

#define CAN_INTPND_17                  (1 << 0)            /*!< Message Object 17 Int. Pending   */
#define CAN_INTPND_18                  (1 << 1)            /*!< Message Object 18 Int. Pending   */
#define CAN_INTPND_19                  (1 << 2)            /*!< Message Object 19 Int. Pending   */
#define CAN_INTPND_20                  (1 << 3)            /*!< Message Object 20 Int. Pending   */
#define CAN_INTPND_21                  (1 << 4)            /*!< Message Object 21 Int. Pending   */
#define CAN_INTPND_22                  (1 << 5)            /*!< Message Object 22 Int. Pending   */
#define CAN_INTPND_23                  (1 << 6)            /*!< Message Object 23 Int. Pending   */
#define CAN_INTPND_24                  (1 << 7)            /*!< Message Object 24 Int. Pending   */
#define CAN_INTPND_25                  (1 << 8)            /*!< Message Object 25 Int. Pending   */
#define CAN_INTPND_26                  (1 << 9)            /*!< Message Object 26 Int. Pending   */
#define CAN_INTPND_27                  (1 << 10)           /*!< Message Object 27 Int. Pending   */
#define CAN_INTPND_28                  (1 << 11)           /*!< Message Object 28 Int. Pending   */
#define CAN_INTPND_29                  (1 << 12)           /*!< Message Object 29 Int. Pending   */
#define CAN_INTPND_30                  (1 << 13)           /*!< Message Object 30 Int. Pending   */
#define CAN_INTPND_31                  (1 << 14)           /*!< Message Object 31 Int. Pending   */
#define CAN_INTPND_32                  (1 << 15)           /*!< Message Object 32 Int. Pending   */

/** @} */

/** @defgroup CAN_MSGV1_Bit_Definitions MSGV2: Message Valid Register 2 (RO)
  * @{
  */

#define CAN_MSGV1_Mask                 (0xffff)            /*!< Usable bits in MSGV1 register    */

#define CAN_MSGVAL_1                   (1 << 0)            /*!< Message Object 1 Valid           */
#define CAN_MSGVAL_2                   (1 << 1)            /*!< Message Object 2 Valid           */
#define CAN_MSGVAL_3                   (1 << 2)            /*!< Message Object 3 Valid           */
#define CAN_MSGVAL_4                   (1 << 3)            /*!< Message Object 4 Valid           */
#define CAN_MSGVAL_5                   (1 << 4)            /*!< Message Object 5 Valid           */
#define CAN_MSGVAL_6                   (1 << 5)            /*!< Message Object 6 Valid           */
#define CAN_MSGVAL_7                   (1 << 6)            /*!< Message Object 7 Valid           */
#define CAN_MSGVAL_8                   (1 << 7)            /*!< Message Object 8 Valid           */
#define CAN_MSGVAL_9                   (1 << 8)            /*!< Message Object 9 Valid           */
#define CAN_MSGVAL_10                  (1 << 9)            /*!< Message Object 10 Valid          */
#define CAN_MSGVAL_11                  (1 << 10)           /*!< Message Object 11 Valid          */
#define CAN_MSGVAL_12                  (1 << 11)           /*!< Message Object 12 Valid          */
#define CAN_MSGVAL_13                  (1 << 12)           /*!< Message Object 13 Valid          */
#define CAN_MSGVAL_14                  (1 << 13)           /*!< Message Object 14 Valid          */
#define CAN_MSGVAL_15                  (1 << 14)           /*!< Message Object 15 Valid          */
#define CAN_MSGVAL_16                  (1 << 15)           /*!< Message Object 16 Valid          */

/** @} */

/** @defgroup CAN_MSGV2_Bit_Definitions MSGV2: Message Valid Register 2 (RO)
  * @{
  */

#define CAN_MSGV2_Mask                 (0xffff)            /*!< Usable bits in MSGV2 register    */

#define CAN_MSGVAL_17                  (1 << 0)            /*!< Message Object 17 Valid          */
#define CAN_MSGVAL_18                  (1 << 1)            /*!< Message Object 18 Valid          */
#define CAN_MSGVAL_19                  (1 << 2)            /*!< Message Object 19 Valid          */
#define CAN_MSGVAL_20                  (1 << 3)            /*!< Message Object 20 Valid          */
#define CAN_MSGVAL_21                  (1 << 4)            /*!< Message Object 21 Valid          */
#define CAN_MSGVAL_22                  (1 << 5)            /*!< Message Object 22 Valid          */
#define CAN_MSGVAL_23                  (1 << 6)            /*!< Message Object 23 Valid          */
#define CAN_MSGVAL_24                  (1 << 7)            /*!< Message Object 24 Valid          */
#define CAN_MSGVAL_25                  (1 << 8)            /*!< Message Object 25 Valid          */
#define CAN_MSGVAL_26                  (1 << 9)            /*!< Message Object 26 Valid          */
#define CAN_MSGVAL_27                  (1 << 10)           /*!< Message Object 27 Valid          */
#define CAN_MSGVAL_28                  (1 << 11)           /*!< Message Object 28 Valid          */
#define CAN_MSGVAL_29                  (1 << 12)           /*!< Message Object 29 Valid          */
#define CAN_MSGVAL_30                  (1 << 13)           /*!< Message Object 30 Valid          */
#define CAN_MSGVAL_31                  (1 << 14)           /*!< Message Object 31 Valid          */
#define CAN_MSGVAL_32                  (1 << 15)           /*!< Message Object 32 Valid          */

/** @} */

/** @defgroup CAN_CLKDIV_Bit_Definitions CLKDIV: Clock Divider Register
  *
  * Note: Divisor is (Register Value) + 1; e.g. writing 1 means PCLK / 2
  * @{
  */

#define CAN_CLKDIV_Mask                (0x0007)            /*!< Usable bits in CLKDIV register   */

/** @} */

/**
  * @}
  */

#endif /* #if defined(LPC11CXX) */


/* Peripheral Memory Locations ------------------------------------------------------------------*/

/**
  * @defgroup LPC11xx_Peripheral_Addresses LPC11xx Peripheral base memory addresses
  * @{
  */

#define I2C0_BASE       (0x40000000UL)                     /*!< I2C Controller 0                 */
#define WDT_BASE        (0x40004000UL)                     /*!< Watchdog Timer                   */
#define UART0_BASE      (0x40008000UL)                     /*!< UART Peripheral 0                */
#define ADC0_BASE       (0x4001c000UL)                     /*!< Analog to Digital Converter 0    */
#define PMU_BASE        (0x40038000UL)                     /*!< Power Management Unit            */
#define FLASH_BASE      (0x4003c000UL)                     /*!< Flash Configuration Block        */
#define SSP0_BASE       (0x40040000UL)                     /*!< Synch Serial Peripheral 0        */
#define IOCON_BASE      (0x40044000UL)                     /*!< IO Configuration Block           */
#define SYSCON_BASE     (0x40048000UL)                     /*!< System Configuration Block       */
#define SSP1_BASE       (0x40058000UL)                     /*!< Synch Serial Peripheral 1        */
#define CT16B0_BASE     (0x4000c000UL)                     /*!< 16-bit Counter / Timer 0         */
#define CT16B1_BASE     (0x40010000UL)                     /*!< 16-bit Counter / Timer 1         */
#define CT32B0_BASE     (0x40014000UL)                     /*!< 32-bit Counter / Timer 0         */
#define CT32B1_BASE     (0x40018000UL)                     /*!< 32-bit Counter / Timer 1         */
#define GPIO0_BASE      (0x50000000UL)                     /*!< General Purpose IO Port 0        */
#define GPIO1_BASE      (0x50010000UL)                     /*!< General Purpose IO Port 1        */
#define GPIO2_BASE      (0x50020000UL)                     /*!< General Purpose IO Port 2        */
#define GPIO3_BASE      (0x50030000UL)                     /*!< General Purpose IO Port 3        */

#if defined(LPC11CXX)  /* CAN enabled parts */
# define CAN0_BASE      (0x40050000UL)                     /*!< Controller Area Network          */
#endif /* #if defined(LPC11CXX) */

/* Defined in core_cm0.h
#define SYSTICK_BASE    (0xe000e010UL)
#define NVIC_BASE       (0xe000e100UL)
#define SCB_BASE        (0xe000ed00UL)
*/

/**
  * @}
  */


/* Exported Variables ---------------------------------------------------------------------------*/

/**
  * @defgroup LPC11xx_Peripheral_Instances LPC11xx Peripheral Instances
  * @{
  */

#define I2C0            ((I2C_Type *)I2C0_BASE)            /*!< I2C Controller 0                 */
#define WDT             ((WDT_Type *)WDT_BASE)             /*!< Watchdog Timer                   */
#define UART0           ((UART_Type *)UART0_BASE)          /*!< UART Peripheral 0                */
#define ADC0            ((ADC_Type *)ADC0_BASE)            /*!< Analog to Digital Converter 0    */
#define PMU             ((PMU_Type *)PMU_BASE)             /*!< Power Management Unit            */
#define FLASH           ((FLASH_Type *)FLASH_BASE)         /*!< Flash Configuration Block        */
#define SSP0            ((SSP_Type *)SSP0_BASE)            /*!< Synch Serial Peripheral 0        */
#define IOCON           ((IOCON_Type *)IOCON_BASE)         /*!< IO Configuration Block           */
#define SYSCON          ((SYSCON_Type *)SYSCON_BASE)       /*!< System Configuration Block       */
#define SSP1            ((SSP_Type *)SSP1_BASE)            /*!< Synch Serial Peripheral 1        */
#define CT16B0          ((CT16B_Type *)CT16B0_BASE)        /*!< 16-bit Counter / Timer 0         */
#define CT16B1          ((CT16B_Type *)CT16B1_BASE)        /*!< 16-bit Counter / Timer 1         */
#define CT32B0          ((CT32B_Type *)CT32B0_BASE)        /*!< 32-bit Counter / Timer 0         */
#define CT32B1          ((CT32B_Type *)CT32B1_BASE)        /*!< 32-bit Counter / Timer 1         */
#define GPIO0           ((GPIO_Type *)GPIO0_BASE)          /*!< General Purpose IO Port 0        */
#define GPIO1           ((GPIO_Type *)GPIO1_BASE)          /*!< General Purpose IO Port 1        */
#define GPIO2           ((GPIO_Type *)GPIO2_BASE)          /*!< General Purpose IO Port 2        */
#define GPIO3           ((GPIO_Type *)GPIO3_BASE)          /*!< General Purpose IO Port 3        */

#if defined(LPC11CXX)
# define CAN0           ((CAN_Type *)CAN0_BASE)            /*!< Controller Area Network          */
#endif /* #if defined(LPC11CXX) */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
};
#endif

#endif /* #ifndef LPC11XX_H_ */
