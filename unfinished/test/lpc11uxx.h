/******************************************************************************
 * @file:    lpc11uxx.h
 * @purpose: Header File for using LPC11Uxx USB-enabled Cortex-M0 CPUs
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    1. June 2010
 *-----------------------------------------------------------------------------
 *
 * This ties together different files for core Cortex-M0 registers &
 *  peripherals for using LPC11Uxx USB microcontrollers.
 *
 *****************************************************************************/

#ifndef LPC11UXX_H_
#define LPC11UXX_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Defines --------------------------------------------------------------------------------------*/

/*! Speed of internal RC Oscillator (12 Mhz)        */
#define IRC_Val                        (12000000UL)

/*! Number of Priority Bits used by the NVIC        */
#define __NVIC_PRIO_BITS               (2)


/* IRQ Numbers ----------------------------------------------------------------------------------*/

/** @addtogroup IRQn
  * @{
  */

typedef enum {

/***** Cortex-M0 Exceptions & Interrupt Numbers ******************************/
    NMI_IRQn                 = -14,    /*!<  2 Non Maskable Interrupt        */
    HardFault_IRQn           = -13,    /*!<  3 Cortex Hard Fault Interrupt   */
    SVC_IRQn                 = -5,     /*!< 11 Cortex-M0 SVC Interrupt       */
    PendSV_IRQn              = -2,     /*!< 14 Cortex-M0 Pending SV Int      */
    SysTick_IRQn             = -1,     /*!< 15 Cortex-M0 System Tick Int     */

/***** LPC11xx Specific Interrupt Numbers ************************************/

    GPIO_PinInt0_IRQn        = 0,      /*!< GPIO Pin Interrupt 0             */
    GPIO_PinInt1_IRQn        = 1,      /*!< GPIO Pin Interrupt 1             */
    GPIO_PinInt2_IRQn        = 2,      /*!< GPIO Pin Interrupt 2             */
    GPIO_PinInt3_IRQn        = 3,      /*!< GPIO Pin Interrupt 3             */
    GPIO_PinInt4_IRQn        = 4,      /*!< GPIO Pin Interrupt 4             */
    GPIO_PinInt5_IRQn        = 5,      /*!< GPIO Pin Interrupt 5             */
    GPIO_PinInt6_IRQn        = 6,      /*!< GPIO Pin Interrupt 6             */
    GPIO_PinInt7_IRQn        = 7,      /*!< GPIO Pin Interrupt 7             */
    GPIO_GROUP0_IRQn         = 8,      /*!< GPIO Grouped Interrupt 0         */
    GPIO_GROUP1_IRQn         = 9,      /*!< GPIO Grouped Interrupt 1         */
    SSP1_IRQn                = 14,     /*!< SSP1 Peripheral Interrupt        */
    I2C_IRQn                 = 15,     /*!< I2C Peripheral Interrupt         */
    CT16B0_IRQn              = 16,     /*!< 16-bit Counter/Timer 0 Interrupt */
    CT16B1_IRQn              = 17,     /*!< 16-bit Counter/Timer 1 Interrupt */
    CT32B0_IRQn              = 18,     /*!< 32-bit Counter/Timer 0 Interrupt */
    CT32B1_IRQn              = 19,     /*!< 32-bit Counter/Timer 1 Interrupt */
    SSP0_IRQn                = 20,     /*!< SSP0 Peripheral Interrupt        */
    USART_IRQn               = 21,     /*!< USART Peripheral Interrupt       */
    USB_IRQ                  = 22,     /*!< USB IRQ Interrupt                */
    USB_FIQ                  = 23,     /*!< USB FIQ Interrupt                */
    ADC_IRQn                 = 24,     /*!< A to D Converter Interrupt       */
    WWDT_IRQn                = 25,     /*!< Windowed Watchdog Timer Int.     */
    BOD_IRQn                 = 26,     /*!< Brownout Detection Interrupt     */
    USB_WAKEUP_IRQn          = 30,     /*!< USB Wakeup Interrupt             */
} IRQn_Type;

/**
  * @}
  */


/* Include for Core Cortex-M0 types, defines, etc. ----------------------------------------------*/

#include "core_cm0.h"


/* Peripheral Registers -------------------------------------------------------------------------*/

/** @addtogroup I2C
  *
  * I2C Controller
  *
  * @{
  */

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

/**
  * @}
  */


/** @addtogroup WWDT
  *
  * Windowed Watchdog Timer
  *
  * @{
  */

typedef struct {
    __IO    uint32_t    MOD;           /*!< Offset: 0x000  Watchdog Mode Register                */
    __IO    uint32_t    TC;            /*!< Offset: 0x004  Timer Constant Register               */
    __O     uint32_t    FEED;          /*!< Offset: 0x008  Feed Sequence Register                */
    __I     uint32_t    TV;            /*!< Offset: 0x00c  Timer Value Register                  */
    __IO    uint32_t    CLKSEL;        /*!< Offset: 0x010  Clock Select Register                 */
    __IO    uint32_t    WARNINT;       /*!< Offset: 0x014  Warning Interrupt Compare Value       */
    __IO    uint32_t    WINDOW;        /*!< Offset: 0x018  Window Compare Value                  */
} WWDT_Type;

/**
  * @}
  */


/** @addtogroup USART
  *
  * USART
  *
  * @{
  */

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
    __IO    uint32_t    ICR;           /*!< Offset: 0x024  IrDA Control Register                 */
    __IO    uint32_t    FDR;           /*!< Offset: 0x028  Fractional Divider Register           */
    __IO    uint32_t    OSR;           /*!< Offset: 0x02C  Oversampling Register                 */
    __IO    uint32_t    TER;           /*!< Offset: 0x030  Transmit Enable Register              */
            uint32_t      Reserved13_15[3];
    __IO    uint32_t    HDEN;          /*!< Offset: 0x040  Half Duplex Enable Register           */
            uint32_t      Reserved17_18[2];
    __IO    uint32_t    SCICTRL;       /*!< Offset: 0x048  Smart Card Interface Control          */
    __IO    uint32_t    RS485CTRL;     /*!< Offset: 0x04c  RS485 Mode Control                    */
    __IO    uint32_t    ADRMATCH;      /*!< Offset: 0x050  RS485 Address Match                   */
    __IO    uint32_t    RS485DLY;      /*!< Offset: 0x054  RS485 Direction Control Delay         */
    __I     uint32_t    SYNCCTRL;      /*!< Offset: 0x058  Synchronous Mode Control              */
} USART_Type;

/**
  * @}
  */


/** @addtogroup ADC
  *
  * Analog to Digital Converter Register Layout
  *
  * @{
  */

typedef struct {
    __IO    uint32_t    CR;            /*!< Offset: 0x000  A/D Control Register                  */
    __IO    uint32_t    GDR;           /*!< Offset: 0x004  A/D Global Data Register              */
    uint32_t              Reserved2[1];
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

/**
  * @}
  */


/** @addtogroup PMU
  *
  * Power Control Block
  *
  * @{
  */

typedef struct {
    __IO    uint32_t    PCON;          /*!< Offset: 0x000  Power Control Register                */
    __IO    uint32_t    GPREG0;        /*!< Offset: 0x004  General Purpose Register 0            */
    __IO    uint32_t    GPREG1;        /*!< Offset: 0x008  General Purpose Register 1            */
    __IO    uint32_t    GPREG2;        /*!< Offset: 0x00c  General Purpose Register 2            */
    __IO    uint32_t    GPREG3;        /*!< Offset: 0x010  General Purpose Register 3            */
    __IO    uint32_t    GPREG4;        /*!< Offset: 0x014  GP Register 4 / Wakeup Hysteresis     */
} PMU_Type;

/**
  * @}
  */


/** @addtogroup FLASH
  *
  * Flash Configuration Block
  *
  * @{
  */

typedef struct {
            uint32_t      Reserved0_3[4];
    __IO    uint32_t    FLASHCFG;      /*!< Offset: 0x010  Flash Latency Configuration           */
            uint32_t      Reserved5_7[3];
    __IO    uint32_t    FMSSTART;      /*!< Offset: 0x020  Signature Start Address Register      */
    __IO    uint32_t    FMSSTOP;       /*!< Offset: 0x024  Signature Stop Address Register       */
            uint32_t      Reserved10[1];
    _I      uint32_t    FMSW0;         /*!< Offset: 0x02c  Signature Word 0                      */
    _I      uint32_t    FMSW1;         /*!< Offset: 0x030  Signature Word 1                      */
    _I      uint32_t    FMSW2;         /*!< Offset: 0x034  Signature Word 2                      */
    _I      uint32_t    FMSW3;         /*!< Offset: 0x038  Signature Word 3                      */
            uint32_t      Reserved15_1015[1001];
    _I      uint32_t    FMSTAT;        /*!< Offset: 0xfe0  Signature Generation Status Register  */
            uint32_t      Reserved1017[1];
    _O      uint32_t    FMSTATCLR;     /*!< Offset: 0xfe8  Signature Generation Status Clear Reg */
} FLASH_Type;

/**
  * @}
  */


/** @addtogroup SSP
  *
  * Synchronous Serial Peripheral
  *
  * @{
  */

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

/**
  * @}
  */


/** @addtogroup IOCON
  *
  * IO Configuration Block
  *
  * @{
  */

typedef struct {
    __IO    uint32_t    RESET_PIO0_0;  /*!< Offset: 0x000  PIO0_0 Configuration                  */
    __IO    uint32_t    PIO0_1;        /*!< Offset: 0x004  PIO0_1 Configuration                  */
    __IO    uint32_t    PIO0_2;        /*!< Offset: 0x008  PIO0_2 Configuration                  */
    __IO    uint32_t    PIO0_3;        /*!< Offset: 0x00c  PIO0_3 Configuration                  */
    __IO    uint32_t    PIO0_4;        /*!< Offset: 0x010  PIO0_4 Configuration                  */
    __IO    uint32_t    PIO0_5;        /*!< Offset: 0x014  PIO0_5 Configuration                  */
    __IO    uint32_t    PIO0_6;        /*!< Offset: 0x018  PIO0_6 Configuration                  */
    __IO    uint32_t    PIO0_7;        /*!< Offset: 0x01c  PIO0_7 Configuration                  */
    __IO    uint32_t    PIO0_8;        /*!< Offset: 0x020  PIO0_8 Configuration                  */
    __IO    uint32_t    PIO0_9;        /*!< Offset: 0x024  PIO0_9 Configuration                  */
    __IO    uint32_t    SWCLK_PIO0_10; /*!< Offset: 0x028  PIO0_10 Configuration                 */
    __IO    uint32_t    TDI_PIO0_11;   /*!< Offset: 0x02c  PIO0_11 Configuration                 */
    __IO    uint32_t    TMS_PIO0_12;   /*!< Offset: 0x030  PIO0_12 Configuration                 */
    __IO    uint32_t    TDO_PIO0_13;   /*!< Offset: 0x034  PIO0_13 Configuration                 */
    __IO    uint32_t    TRST_PIO0_14;  /*!< Offset: 0x038  PIO0_14 Configuration                 */
    __IO    uint32_t    SWDIO_PIO0_15; /*!< Offset: 0x03c  PIO0_15 Configuration                 */
    __IO    uint32_t    PIO0_16;       /*!< Offset: 0x040  PIO0_16 Configuration                 */
    __IO    uint32_t    PIO0_17;       /*!< Offset: 0x044  PIO0_17 Configuration                 */
    __IO    uint32_t    PIO0_18;       /*!< Offset: 0x048  PIO0_18 Configuration                 */
    __IO    uint32_t    PIO0_19;       /*!< Offset: 0x04c  PIO0_19 Configuration                 */
    __IO    uint32_t    PIO0_20;       /*!< Offset: 0x050  PIO0_20 Configuration                 */
    __IO    uint32_t    PIO0_21;       /*!< Offset: 0x054  PIO0_21 Configuration                 */
    __IO    uint32_t    PIO0_22;       /*!< Offset: 0x058  PIO0_22 Configuration                 */
    __IO    uint32_t    PIO0_23;       /*!< Offset: 0x05c  PIO0_23 Configuration                 */
            uint32_t      Reserved24_28[5];
    __IO    uint32_t    PIO1_5;        /*!< Offset: 0x074  PIO1_5 Configuration                  */
            uint32_t      Reserved26_32[7];
    __IO    uint32_t    PIO1_13;       /*!< Offset: 0x094  PIO1_13 Configuration                 */
    __IO    uint32_t    PIO1_14;       /*!< Offset: 0x098  PIO1_14 Configuration                 */
    __IO    uint32_t    PIO1_15;       /*!< Offset: 0x09c  PIO1_15 Configuration                 */
    __IO    uint32_t    PIO1_16;       /*!< Offset: 0x0a0  PIO1_16 Configuration                 */
            uint32_t      Reserved37_38[2];
    __IO    uint32_t    PIO1_19;       /*!< Offset: 0x0ac  PIO1_19 Configuration                 */
    __IO    uint32_t    PIO1_20;       /*!< Offset: 0x0b0  PIO1_20 Configuration                 */
    __IO    uint32_t    PIO1_21;       /*!< Offset: 0x0b4  PIO1_21 Configuration                 */
    __IO    uint32_t    PIO1_22;       /*!< Offset: 0x0b8  PIO1_22 Configuration                 */
    __IO    uint32_t    PIO1_23;       /*!< Offset: 0x0bc  PIO1_23 Configuration                 */
    __IO    uint32_t    PIO1_24;       /*!< Offset: 0x0c0  PIO1_24 Configuration                 */
    __IO    uint32_t    PIO1_25;       /*!< Offset: 0x0c4  PIO1_25 Configuration                 */
    __IO    uint32_t    PIO1_26;       /*!< Offset: 0x0c8  PIO1_26 Configuration                 */
    __IO    uint32_t    PIO1_27;       /*!< Offset: 0x0cc  PIO1_27 Configuration                 */
    __IO    uint32_t    PIO1_28;       /*!< Offset: 0x0d0  PIO1_28 Configuration                 */
    __IO    uint32_t    PIO1_29;       /*!< Offset: 0x0d4  PIO1_29 Configuration                 */
            uint32_t      Reserved54[1];
    __IO    uint32_t    PIO1_31;       /*!< Offset: 0x0dc  PIO1_31 Configuration                 */
} IOCON_Type;

/**
  * @}
  */


/** @addtogroup SYSCON
  *
  * System Configuration Block
  *
  * @{
  */

typedef struct {
    __IO    uint32_t    SYSMEMREMAP;   /*!< Offset: 0x000  System Memory Remap Control           */
    __IO    uint32_t    PRESETCTRL;    /*!< Offset: 0x004  Peripheral Reset Control              */
    __IO    uint32_t    SYSPLLCTRL;    /*!< Offset: 0x008  System PLL Control                    */
    __I     uint32_t    SYSPLLSTAT;    /*!< Offset: 0x00c  System PLL Status                     */
    __IO    uint32_t    USBPLLCTRL;    /*!< Offset: 0x010  USB PLL Control                       */
    __I     uint32_t    USBPLLSTAT;    /*!< Offset: 0x014  USB PLL Status                        */
            uint32_t      Reserved6_7[2];
    __IO    uint32_t    SYSOSCCTRL;    /*!< Offset: 0x020  System Oscillator Control             */
    __IO    uint32_t    WWDTOSCCTRL;   /*!< Offset: 0x024  Watchdog Timer Oscillator Control     */
            uint32_t      Reserved10_11[2];
    __IO    uint32_t    SYSRESSTAT;    /*!< Offset: 0x030  System Reset Source Identification    */
            uint32_t      Reserved13_15[3];
    __IO    uint32_t    SYSPLLCLKSEL;  /*!< Offset: 0x040  System PLL Clock Source               */
    __IO    uint32_t    SYSPLLCLKUEN;  /*!< Offset: 0x044  System PLL Clock Update Enable        */
    __IO    uint32_t    USBPLLCLKSEL;  /*!< Offset: 0x048  USB PLL Clock Source                  */
    __IO    uint32_t    USBPLLCLKUEN;  /*!< Offset: 0x04c  USB PLL Clock Update Enable           */
            uint32_t      Reserved20_27[8];
   __IO     uint32_t    MAINCLKSEL;    /*!< Offset: 0x070  Main Clock Source                     */
   __IO     uint32_t    MAINCLKUEN;    /*!< Offset: 0x074  Main Clock Enable                     */
   __IO     uint32_t    SYSAHBCLKDIV;  /*!< Offset: 0x078  AHB Clock Divider                     */
            uint32_t      Reserved31[1];
   __IO     uint32_t    SYSAHBCLKCTRL; /*!< Offset: 0x080  AHB Clock Control                     */
            uint32_t      Reserved33_36[4];
   __IO     uint32_t    SSP0CLKDIV;    /*!< Offset: 0x094  SSP0 Clock Divider                    */
   __IO     uint32_t    UARTCLKDIV;    /*!< Offset: 0x098  USART Clock Divider                   */
   __IO     uint32_t    SSP1CLKDIV;    /*!< Offset: 0x09c  SSP1 Clock Divider                    */
            uint32_t      Reserved40_47[8];
   __IO     uint32_t    USBCLKSEL;     /*!< Offset: 0x0c0  USB Clock Source                      */
   __IO     uint32_t    USBCLKUEN;     /*!< Offset: 0x0c4  USB Clock Update Enable               */
   __IO     uint32_t    USBCLKDIV;     /*!< Offset: 0x0c8  USB Clock Divider                     */
            uint32_t      Reserved51_55[5];
   __IO     uint32_t    CLKOUTCLKSEL;  /*!< Offset: 0x0e0  CLKOUT Clock Select                   */
   __IO     uint32_t    CLKOUTUEN;     /*!< Offset: 0x0e4  CLKOUT Update Enable                  */
   __IO     uint32_t    CLKOUTDIV;     /*!< Offset: 0x0e8  CLKOUT Divider                        */
            uint32_t      Reserved59_63[5];
   __I      uint32_t    PIOPORCAP0;    /*!< Offset: 0x100  Power On Reset Captured Pin Status 0  */
   __I      uint32_t    PIOPORCAP1;    /*!< Offset: 0x104  Power On Reset Captured Pin Status 1  */
            uint32_t      Reserved66_83[18];
   __IO     uint32_t    BODCTRL;       /*!< Offset: 0x150  Brownout Detetector Control           */
   __IO     uint32_t    SYSTCKCAL;     /*!< Offset: 0x154  SysTick Calibration Value             */
            uint32_t      Reserved86_91[6];
   __IO     uint32_t    IRQLATENCY;    /*!< Offset: 0x170  IRQ Delay                             */
   __IO     uint32_t    NMISRC;        /*!< Offset: 0x174  NMI Source Control                    */
   __IO     uint32_t    PINTSEL0;      /*!< Offset: 0x178  GPIO Pin Interrupt Select Register 0  */
   __IO     uint32_t    PINTSEL1;      /*!< Offset: 0x17c  GPIO Pin Interrupt Select Register 1  */
   __IO     uint32_t    PINTSEL2;      /*!< Offset: 0x180  GPIO Pin Interrupt Select Register 2  */
   __IO     uint32_t    PINTSEL3;      /*!< Offset: 0x184  GPIO Pin Interrupt Select Register 3  */
   __IO     uint32_t    PINTSEL4;      /*!< Offset: 0x188  GPIO Pin Interrupt Select Register 4  */
   __IO     uint32_t    PINTSEL5;      /*!< Offset: 0x18c  GPIO Pin Interrupt Select Register 5  */
   __IO     uint32_t    PINTSEL6;      /*!< Offset: 0x190  GPIO Pin Interrupt Select Register 6  */
   __IO     uint32_t    PINTSEL7;      /*!< Offset: 0x194  GPIO Pin Interrupt Select Register 7  */
   __IO     uint32_t    USBCLKCTRL;    /*!< Offset: 0x198  USB Clock Control                     */
   __I      uint32_t    USBCLKSTAT;    /*!< Offset: 0x19c  USB Clock Status                      */
            uint32_t      Reserved104_128[25];
   __IO     uint32_t    STARTERP0;     /*!< Offset: 0x204  Start Logic Enable Register 0         */
            uint32_t      Reserved130_132[3];
   __IO     uint32_t    STARTERP1;     /*!< Offset: 0x214  Start Logic Enable Register 1         */
            uint32_t      Reserved134_139[6];
   __IO     uint32_t    PDSLEEPCFG;    /*!< Offset: 0x230  Deep Sleep Mode Power Down Config     */
   __IO     uint32_t    PDAWAKECFG;    /*!< Offset: 0x234  Wakeup Mode Power Down Configuration  */
   __IO     uint32_t    PDRUNCFG;      /*!< Offset: 0x238  Running Mode Power Down Configuration */
            uint32_t      Reserved143_582[110];
   __I      uint32_t    DEVICE_ID;     /*!< Offset: 0x3f4  LPC Device ID                         */
} SYSCON_Type;

/**
  * @}
  */


/** @addtogroup CT16B
  *
  * 16-bit Counter / Timer
  *
  * @{
  */

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
            uint32_t      Reserved12_14[3];
    __IO    uint32_t    EMR;           /*!< Offset: 0x03c  External Match Register               */
            uint32_t      Reserved16_27[12];
    __IO    uint32_t    CTCR;          /*!< Offset: 0x070  Count Control Register                */
    __IO    uint32_t    PWMC;          /*!< Offset: 0x074  PWM Control Register                  */
} CT16B_Type;

/**
  * @}
  */


/** @addtogroup CT32B
  *
  * 32-bit Counter / Timer
  *
  * @{
  */

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
            uint32_t      Reserved12_14[3];
    __IO    uint32_t    EMR;           /*!< Offset: 0x03c  External Match Register               */
            uint32_t      Reserved16_27[12];
    __IO    uint32_t    CTCR;          /*!< Offset: 0x070  Count Control Register                */
    __IO    uint32_t    PWMC;          /*!< Offset: 0x074  PWM Control Register                  */
} CT32B_Type;

/**
  * @}
  */


/** @addtogroup GPIO Pin Interrupt Control Type
  *
  * @{
  */

typedef struct {
    __IO    uint32_t    ISEL;           /*!< Offset: 0x000  Pin Interrupt Mode Register          */
    __IO    uint32_t    IENR;           /*!< Offset: 0x004  Pin Interrupt Enable (Rising)        */
    __O     uint32_t    SIENR;          /*!< Offset: 0x008  Set Pin Interrupt Enable (Rising)    */
    __O     uint32_t    CIENR;          /*!< Offset: 0x00c  Clear Pin Interrupt Enable (Rising)  */
    __IO    uint32_t    IENF;           /*!< Offset: 0x010  Pin Int Enable Falling/Active Level  */
    __O     uint32_t    SIENF;          /*!< Offset: 0x014  Set Pin Int Ena Falling/Active Level */
    __O     uint32_t    CIENF;          /*!< Offset: 0x018  Clear Pin Int Ena Falling/Act Level  */
    __IO    uint32_t    RISE;           /*!< Offset: 0x01c  Pin Interrupt Rising Edge Register   */
    __IO    uint32_t    FALL;           /*!< Offset: 0x020  Pin Interrupt Falling Edge Register  */
    __IO    uint32_t    IST;            /*!< Offset: 0x024  Pin Interrupt Status Register        */
} GPIO_PININT_Type;

/**
  * @}
  */


/** @addtogroup GPIO Group Interrupt Control Type
  *
  * @{
  */

typedef struct {
    __IO    uint32_t    CTRL;          /*!< Offset: 0x000  GPIO Grouped Interrupt Control Reg    */
            uint32_t      Reserved1_7[7];
    __IO    uint32_t    PORT_POL0;     /*!< Offset: 0x020  GPIO Grouped Int. Port 0 Polarity     */
    __IO    uint32_t    PORT_POL1;     /*!< Offset: 0x024  GPIO Grouped Int. Port 1 Polarity     */
            uint32_t      Reserved10_15[6];
    __IO    uint32_t    PORT_POL0;     /*!< Offset: 0x040  GPIO Grouped Int. Port 0 Polarity     */
    __IO    uint32_t    PORT_POL1;     /*!< Offset: 0x044  GPIO Grouped Int. Port 1 Polarity     */
} GPIO_GROUPINT_Type;

/**
  * @}
  */


/** @addtogroup GPIO Type
  *
  * @{
  */

typedef struct {
    __IO    uint8_t     B0[32];        /*!< Offset: 0x0000 GPIO Byte Pin Registers Port 0        */
    __IO    uint8_t     B1[32];        /*!< Offset: 0x0020 GPIO Byte Pin Registers Port 1        */
            uint32_t      Reserved16_1023[1008];
    __IO    uint32_t    W0[32];        /*!< Offset: 0x1000 GPIO Word Pin Registers Port 0        */
    __IO    uint32_t    W1[32];        /*!< Offset: 0x1080 GPIO Word Pin Registers Port 1        */
            uint32_t      Reserved1088_2047[960]
    __IO    uint32_t    DIR0;          /*!< Offset: 0x2000 GPIO Directions Port 0                */
    __IO    uint32_t    DIR1;          /*!< Offset: 0x2004 GPIO Directions Port 1                */
            uint32_t      Reserved2050_2079[30];
    __IO    uint32_t    MASK0;         /*!< Offset: 0x2080 GPIO Port 0 MPORT Mask                */
    __IO    uint32_t    MASK1;         /*!< Offset: 0x2084 GPIO Port 1 MPORT Mask               */
            uint32_t      Reserved2082_2111[30];
    __IO    uint32_t    PORT0;         /*!< Offset: 0x2100 GPIO Port 0 Pin Control / States      */
    __IO    uint32_t    PORT1;         /*!< Offset: 0x2104 GPIO Port 0 Pin Control / States      */
            uint32_t      Reserved2114_2143[30];
    __IO    uint32_t    MPORT0;        /*!< Offset: 0x2180 GPIO Masked Port 0                    */
    __IO    uint32_t    MPORT1;        /*!< Offset: 0x2184 GPIO Masked Port 1                    */
            uint32_t      Reserved2146_2175[30];
    __IO    uint32_t    SET0;          /*!< Offset: 0x2200 GPIO Port 0 Set Bits                  */
    __IO    uint32_t    SET1;          /*!< Offset: 0x2204 GPIO Port 1 Set Bits                  */
            uint32_t      Reserved2178_2207[30];
    __O     uint32_t    CLR0;          /*!< Offset: 0x2280 GPIO Port 0 Clear Bits                */
    __O     uint32_t    CLR1;          /*!< Offset: 0x2284 GPIO Port 1 Clear Bits                */
            uint32_t      Reserved2210_2239[30];
    __O     uint32_t    NOT0;          /*!< Offset: 0x2300 GPIO Port 0 Invert Bits               */
    __O     uint32_t    NOT1;          /*!< Offset: 0x2304 GPIO Port 1 Invert Bits               */
} GPIO_Type;

/**
  * @}
  */


/** @addtogroup USBD
  *
  * USB Device Interface
  *
  * @{
  */

typedef struct {
    __IO    uint32_t    DEVCMDSTAT;    /*!< Offset: 0x000  USB Device Command / Status Register  */
    __IO    uint32_t    INFO;          /*!< Offset: 0x004  USB Info Register                     */
    __IO    uint32_t    EPLISTSTART;   /*!< Offset: 0x008  USB EP Cmd/Status List Start Address  */
    __IO    uint32_t    DATABUFSTART;  /*!< Offset: 0x00c  USB Data Buffer Start Address         */
            uint32_t      Reserved4[1];
    __IO    uint32_t    EPSKIP;        /*!< Offset: 0x014  USB Endpoint Skip                     */
    __IO    uint32_t    EPINUSE;       /*!< Offset: 0x018  USB Endpoint Buffer In Use            */
    __IO    uint32_t    EPBUFCFG;      /*!< Offset: 0x01c  USB Endpoint Buffer Config Register   */
    __IO    uint32_t    INTSTAT;       /*!< Offset: 0x020  USB Interrupt Status Register         */
    __IO    uint32_t    INTEN;         /*!< Offset: 0x024  USB Interrupt Enable Register         */
    __IO    uint32_t    INTSETSTAT;    /*!< Offset: 0x028  USB Set Interrupt Status Register     */
    __IO    uint32_t    INTROUTING;    /*!< Offset: 0x02c  USB Interrupt Routing Register        */
            uint32_t      Reserved12[1];
    __I     uint32_t    EPTOGGLE;      /*!< Offset: 0x034  USB Endpoint Toggle Register          */
} USBD_Type;

/**
  * @}
  */


/* Peripheral Register Bit Definitions  -------------------------------------*/

/** @addtogroup I2C
  *
  * I2C Controller
  *
  * @{
  */

/** @defgroup I2C_CONSET_Bit_Definitions (I2CxCONSET) I2C Control Set Register
  * @{
  */

#define I2C_CONSET_Mask                (0x7e)              /*!< Useable Bits in CONSET Register  */
#define I2C_CONSET_Shift               (0x00)

#define I2C_AA                         (1 << 2)            /*!< Assert Acknowledge Set Bit       */
#define I2C_SI                         (1 << 3)            /*!< I2C Interrupt Enable Bit         */
#define I2C_STO                        (1 << 4)            /*!< STOP Flag Set Bit                */
#define I2C_STA                        (1 << 5)            /*!< START Flag Set Bit               */
#define I2C_I2EN                       (1 << 6)            /*!< I2C Interface Enable Bit         */

/**
  * @}
  */

/** @defgroup I2C_CONCLR_Bit_Definitions (I2CxCONCLR) I2C Control Clear Register
  *
  * @{
  */

#define I2C_CONCLR_Mask                (0x6e)              /*!< Useable Bits in CONCLR Register  */
#define I2C_CONCLR_Shift               (0x00)

#define I2C_AAC                        (1 << 2)            /*!< Assert Acknowledge Clear Bit     */
#define I2C_SIC                        (1 << 3)            /*!< I2C Interrupt Disable Flag       */
#define I2C_STAC                       (1 << 5)            /*!< START Flag Clear Bit             */
#define I2C_I2ENC                      (1 << 6)            /*!< I2C Interface Disable Bit        */

/**
  * @}
  */

/** @defgroup I2C_STAT_Bit_Definitions (I2CxSTAT) I2C Status Register
  *
  * @{
  */

#define I2C_STAT_Mask                  (0xf8)              /*!< I2C Data Bits Mask               */
#define I2C_STAT_Shift                 (0)

/**
  * @}
  */

/** @defgroup I2C_DAT_Bit_Definitions (I2CxDAT) I2C Data Register
  *
  * @{
  */

#define I2C_DAT_Mask                   (0xff)              /*!< I2C Data Bits Mask               */
#define I2C_DAT_Shift                  (0)

/**
  * @}
  */

/** @defgroup I2C_ADR{0,1,2,3}_Bit_Definitions (I2CxADRn) I2C Slave Address Register
  *
  * @{
  */

#define I2C_ADDR_Mask                  (0x7f << 1)         /*!< Address 0 Mask (7 bits)          */
#define I2C_ADDR_Shift                 (1)

#define I2C_GC                         (1 << 0)            /*!< Gen. Call Enable (Answer 0x00)   */

/**
  * @}
  */

/** @defgroup I2C_MASK{0,1,2,3}_Bit_Definitions (I2CxMASKn) I2C Slave Address Mask Register
  *
  * @{
  */

#define I2C_MASK_Mask                  (0x7f << 1)         /*!< Address 0 Mask (7 bits)          */
#define I2C_MASK_Shift                 (1)

/**
  * @}
  */

/** @defgroup I2C_SCLH_Bit_Definitions (I2CxSCLH) I2C Duty Cycle Register High Half Word
  *
  * @{
  */

#define I2C_SCLH_Mask                  (0xffff)            /*!< Count for SCL High Time          */
#define I2C_SCLH_Shift                 (0)

/**
  * @}
  */

/** @defgroup I2C_SCLL_Bit_Definitions (I2CxSCLL) I2C Duty Cycle Register Low Half Word
  *
  * @{
  */

#define I2C_SCLL_Mask                  (0xffff)            /*!< Count for SCL Low Time           */
#define I2C_SCLL_Shift                 (0)

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup I2C_MMCTRL_Bit_Definitions (I2CxMMCTRL) I2C Monitor Mode Control Register
  *
  * @{
  */

#define I2C_MMCTRL_Mask                (0x0b)              /*!< Useable Bits in MMCTRL Register  */
#define I2C_MMCTRL_Shift               (0x00)

#define I2C_MM_ENA                     (1 << 0)            /*!< Monitor Mode Enable              */
#define I2C_ENA_SCL                    (1 << 1)            /*!< SCL Output Enable                */
#define I2C_MATCH_ALL                  (1 << 3)            /*!< Generate Int (in MM) on Any Addr */

/**
  * @}
  */

/**
  * @}
  */


/** @addtogroup WWDT
  *
  * Windowed Watchdog Timer
  *
  * @{
  */

/** @addtogroup WWDT_MOD_Bit_Definitions (WWDTMOD) Windowed Watchdog Timer Mode Register
  * @{
  */

#define WDT_MOD_Mask                   (0x3f)              /*!< Watchdog Mode Useable Bit Mask   */
#define WDT_MOD_Shift                  (0)

#define WDT_WDEN                       (1 << 0)            /*!< Watchdog Enable (Set Only)       */
#define WDT_WDRESET                    (1 << 1)            /*!< Watchdog Reset (Set Only)        */
#define WDT_WDTOF                      (1 << 2)            /*!< Watchdog Timeout Flag (SW clear) */
#define WDT_WDINT                      (1 << 3)            /*!< Watchdog Interrupt Flag (RO)     */
#define WWDT_WDPROTECT                 (1 << 4)            /*!< Watchdog Update Mode             */
#define WWDT_LOCK                      (1 << 5)            /*!< Prevent Disabling Clock Source   */

/**
  * @}
  */

/** @addtogroup WWDT_TC_Bit_Definitions (WWDTTC) Windowed Watchdog Timer Constant Register
  * @{
  */

#define WWDT_TC_Mask                   (0x0fff)            /*!< WWDT Timer Constant Useable Bits */
#define WWDT_TC_Shift                  (0)

/**
  * @}
  */

/** @addtogroup WWDT_TV_Bit_Definitions (WWDTTV) Windowed Watchdog Timer Value Register
  * @{
  */

#define WWDT_TV_Mask                   (0x0fff)            /*!< Timer Value Useable Bits         */
#define WWDT_TV_Shift                  (0)

/**
  * @}
  */

/** @addtogroup WWDT_CLKSEL_Bit_Definitions (WWDTCLKSEL) Windowed Watchdog Clock Select Register
  * @{
  */

#define WWDT_CLKSEL_Mask               (0x80000001)        /*!< Clock Select Useable Bits        */
#define WWDT_CLKSEL_Shift              (0)

#define WWDT_CLKSEL_WDOSC              (1UL << 0)          /*!< Select WD Osc (vs. IRC)          */
#define WWDT_CLKSEL_LOCK               (1UL << 31)         /*!< Prevent changing reg when 1      */

/**
  * @}
  */

/** @addtogroup WWDT_WARNINT_Bit_Definitions (WWDTWARNINT) Windowed Watchdog Warning Interrupt Register
  * @{
  */

#define WWDT_WARNINT_Mask              (0x3f)              /*!< Warning Interrupt Compare Value  */
#define WWDT_WARNINT_Shift             (0)

/**
  * @}
  */

/** @addtogroup WWDT_WINDOW_Bit_Definitions (WWDTWINDOW) Windowed Watchdog Window Register
  * @{
  */

#define WWDT_WINDOW_Mask               (0x00ffffffUL)      /*!< Max WWDTV Value when Feeding     */
#define WWDT_WINDOW_Shift              (0)

/**
  * @}
  */

/**
  * @}
  */


/** @addtogroup USART
  *
  * USART Peripheral
  *
  * @{
  */

/** @defgroup USART_RBR_Bit_Definitions (UxRBR) USART Receive Buffer Register
  * @{
  */

#define USART_RBR_Mask                 (0xff)              /*!< Receiver Buffer Register Mask    */
#define USART_RBR_Shift                (0)

/**
  * @}
  */

/** @defgroup USART_DLL_Bit_Definitions (UxDLL) USART Divisor Latch LSB Register
  *
  * @{
  */

#define USART_DLLSB_Mask               (0xff)              /*!< Divisor Latch LSB Mask           */
#define USART_DLLSB_Shift              (0)

/**
  * @}
  */

/** @defgroup USART_DLM_Bit_Definitions (UxDLM) USART Divisor Latch MSB Register
  *
  * @{
  */

#define USART_DLMSB_Mask               (0xff)             /*!< Divisor Latch MSB Mask           */
#define USART_DLMSB_Shift              (0)

/**
  * @}
  */

/** @defgroup USART_IER_Bit_Definitions (UxIER) USART Interrupt Enable Register
  *
  * @{
  */

#define USART_IER_Mask                 (0x030f)            /*!< Useable Bits in IER Register     */
#define USART_IER_Shift                (0)

#define USART_RBR_INT_ENA              (1 << 0)            /*!< Rx Data Avail/Timeout IRQ Enable */
#define USART_THRE_INT_ENA             (1 << 1)            /*!< TX Holding Reg Empty IRQ Enable  */
#define USART_LINE_INT_ENA             (1 << 2)            /*!< RX Line Status IRQ Enable        */
#define USART_MODEM_INT_ENA            (1 << 3)            /*!< Modem Status IRQ Enable          */
#define USART_ABEO_INT_ENA             (1 << 8)            /*!< Auto Baud IRQ Enable             */
#define USART_ABTO_INT_ENA             (1 << 9)            /*!< Auto Baud Timeout IRQ Enable     */

/**
  * @}
  */

/** @defgroup USART_IIR_Bit_Definitions (UxIIR) USART Interrupt ID Register
  *
  * @{
  */

#define USART_IIR_Mask                 (0x03cf)           /*!< Useable Bits in IIR Register      */
#define USART_IIR_Shift                (0)

#define USART_INTID_Mask               (0x0f)              /*!< Mask for Standard Interrupt ID's */
#define USART_INTID_Shift              (0)
#define USART_INTID_NONE               (0x01)              /*!< Int. Status (0 == IT pending)    */
#define USART_INTID_RLS                (0x06)              /*!< Receive Line Status Interrupt    */
#define USART_INTID_RDA                (0x04)              /*!< Receive Data Available Interrupt */
#define USART_INTID_CTI                (0x0c)              /*!< Character Time-out Interrupt     */
#define USART_INTID_THRE               (0x02)              /*!< Tx Holding Reg Empty Interrupt   */
#define USART_INTID_MODM               (0x00)              /*!< Modem Line Interrupt             */

#define USART_FIFO_Mask                (0x03 << 6)         /*!< Copy of UFCR                     */
#define USART_FIFO_Shift               (6)                 /*!< Bit Shift for Fifo Bits          */

#define USART_ABIT_Mask                (0x03 << 8)         /*!< Mask for Auto Baud Interrupts    */
#define USART_ABIT_Shift               (8)

#define USART_IT_ABEO                  (1 << 8)            /*!< End of Auto-Baud Interrupt       */
#define USART_IT_ABTO                  (1 << 9)            /*!< Auto-Baud Timeout Interrupt      */

/**
  * @}
  */

/** @defgroup USART_FCR_Bit_Definitions (UxFCR) USART FIFO Control Register (WO)
  *
  * @{
  */

#define USART_FCR_Mask                 (0xc7)             /*!< Useable Bits in FCR Register      */
#define USART_FCR_Shift                (0)

#define USART_RXTRIGLVL_Mask           (3 << 6)            /*!< Mask for RX Trigger Level        */
#define USART_RXTRIGLVL_Shift          (6)
#define USART_RXTRIGLVL_1              (0x00)              /*!< Trigger RX IRQ on 1B  in FIFO    */
#define USART_RXTRIGLVL_4              (0x01 << 6)         /*!< Trigger RX IRQ on 4B  in FIFO    */
#define USART_RXTRIGLVL_8              (0x02 << 6)         /*!< Trigger RX IRQ on 8B  in FIFO    */
#define USART_RXTRIGLVL_14             (0x03 << 6)         /*!< Trigger RX IRQ on 14B in FIFO    */

#define USART_FIFOEN                   (1 << 0)            /*!< Enable RX/TX FIFOS (NECESSARY)   */
#define USART_FIFO_RX_RESET            (1 << 1)            /*!< Clear RX FIFO (self-clearing)    */
#define USART_FIFO_TX_RESET            (1 << 2)            /*!< Clear TX FIFO (self-clearing)    */

/**
  * @}
  */

/** @defgroup USART_LCR_Bit_Definitions (UxLCR) USART Line Control Register
  *
  * @{
  */

#define USART_LCR_Mask                 (0xff)              /*!< Useable Bits in LCR Register     */
#define USART_LCR_Shift                (0)

#define USART_WORDLEN_Mask             (0x03)              /*!< Mask for Word Length             */
#define USART_WORDLEN_Shift            (0)
#define USART_WORDLEN_5                (0x00)              /*!< 5 bit characters                 */
#define USART_WORDLEN_6                (0x01)              /*!< 6 bit characters                 */
#define USART_WORDLEN_7                (0x02)              /*!< 7 bit characters                 */
#define USART_WORDLEN_8                (0x03)              /*!< 8 bit characters                 */

#define USART_PARITY_Mask              (0x07 << 3)         /*!< Mask for Parity Type             */
#define USART_PARITY_Shift             (3)
#define USART_PARITY_NONE              (0x00 << 3)         /*!< No Parity                        */
#define USART_PARITY_ODD               (0x01 << 3)         /*!< Odd Parity                       */
#define USART_PARITY_EVEN              (0x03 << 3)         /*!< Even Parity                      */
#define USART_PARITY_1                 (0x05 << 3)         /*!< Force Parity = 1                 */
#define USART_PARITY_0                 (0x07 << 3)         /*!< Force Parity = 0                 */

#define USART_2STOPBITS                (1 << 2)            /*!< 2 stop bits                      */
#define USART_PAREN                    (1 << 3)            /*!< Enable Parity                    */
#define USART_BREAK                    (1 << 6)            /*!< Set BREAK on TX line             */
#define USART_DLAB                     (1 << 7)            /*!< Enable Access to Div Latch       */

/**
  * @}
  */

/** @defgroup USART_MCR_Bit_Definitions (UxMCR) USART Modem Control Register
  *
  * @{
  */

#define USART_MCR_Mask                 (0xd3)              /*!< Useable Bits in MCR Register     */
#define USART_MCR_Shift                (0)

#define USART_DTR                      (1 << 0)            /*!< Control of /DTR line             */
#define USART_RTS                      (1 << 1)            /*!< Control of /RTS line             */
#define USART_LOOPBACK                 (1 << 4)            /*!< Enable Loopback Mode             */
#define USART_RTSENA                   (1 << 6)            /*!< Enable Auto-RTS Flow Control     */
#define USART_CTSENA                   (1 << 7)            /*!< Enable Auto-CTS Flow Control     */

/**
  * @}
  */

/** @defgroup USART_LSR_Bit_Definitions (UxLSR) USART Line Status Register
  *
  * @{
  */

#define USART_LSR_Mask                 (0x1ff)             /*!< Useable Bits in LSR Register     */
#define USART_LSR_Shift                (0)

#define USART_RDR                      (1 << 0)            /*!< Receiver Data Ready              */
#define USART_OE                       (1 << 1)            /*!< Overrun Error                    */
#define USART_PE                       (1 << 2)            /*!< Parity Error                     */
#define USART_FE                       (1 << 3)            /*!< Framing Error                    */
#define USART_BI                       (1 << 4)            /*!< Break Interrupt                  */
#define USART_THRE                     (1 << 5)            /*!< Tx Holding Register Empty        */
#define USART_TEMT                     (1 << 6)            /*!< Transmitter Empty                */
#define USART_RXFE                     (1 << 7)            /*!< Error in RX FIFO                 */
#define USART_TXERR                    (1 << 8)            /*!< Tx Error (smartcard use)         */

/**
  * @}
  */

/** @defgroup USART_MSR_Bit_Definitions (UxMSR) USART Modem Status Register
  *
  * @{
  */

#define USART_MSR_Mask                 (0xff)              /*!< Useable Bits in MSR Register     */
#define USART_MSR_Shift                (0)

#define USART_DCTS                     (1 << 0)            /*!< Change in CTS                    */
#define USART_DDSR                     (1 << 1)            /*!< Change in DSR                    */
#define USART_TERI                     (1 << 2)            /*!< Low-to-high on RI                */
#define USART_DDCD                     (1 << 3)            /*!< Change on DCD                    */
#define USART_CTS                      (1 << 4)            /*!< Current CTS State (inverted)     */
#define USART_DSR                      (1 << 5)            /*!< Current DSR State (inverted)     */
#define USART_RI                       (1 << 6)            /*!< Current RI State (inverted)      */
#define USART_DCD                      (1 << 7)            /*!< Current DCD State (inverted)     */

/**
  * @}
  */

/** @defgroup USART_ACR_Bit_Definitions (UxACR) USART Auto-baud Control Register
  *
  * @{
  */

#define USART_ACR_Mask                 (0x0307)            /*!< Useable Bits in ACR Register     */
#define USART_ACR_Shift                (0)

#define USART_AUTOBAUD                 (1 << 0)            /*!< Autobaud Running                 */
#define USART_MODE1                    (1 << 1)            /*!< Mode 1 Selected (0 = Mode 0)     */
#define USART_AUTORESTART              (1 << 2)            /*!< Autobaud Restart on Timeout      */
#define USART_ABEOIRQCLR               (1 << 8)            /*!< Clear ABEO Interrupt             */
#define USART_OBTOIRQCLR               (1 << 9)            /*!< Clear ABTO Interrupt             */

/**
  * @}
  */

/** @defgroup USART_ICR_Bit_Definitions (UxICR) USART IrDA Control Register
  *
  * @{
  */

#define USART_ICR_Mask                 (0x03f)             /*!< Useable Bits in ICR Register     */
#define USART_ICR_Shift                (0)

/* Note: datasheet for lpc11u1x contradicts itself on these. */
#define USART_PULSEDIV_Mask            (0x07 << 3)         /*!< Pulse Width Setting Mask         */
#define USART_PULSEDIV_Shift           (3)
#define USART_PULSEDIV_3_16_Baud       (0x00)              /*!< Pulse Width 3/(16 x baud rate)   */
#define USART_PULSEDIV_2PCLK           (0x01)              /*!< Pulse Width 2 x Tpclk            */
#define USART_PULSEDIV_4PCLK           (0x02)              /*!< Pulse Width 4 x Tpclk            */
#define USART_PULSEDIV_8PCLK           (0x03)              /*!< Pulse Width 8 x Tpclk            */
#define USART_PULSEDIV_16PCLK          (0x04)              /*!< Pulse Width 16 x Tpclk           */
#define USART_PULSEDIV_32PCLK          (0x05)              /*!< Pulse Width 32 x Tpclk           */
#define USART_PULSEDIV_64PCLK          (0x06)              /*!< Pulse Width 64 x Tpclk           */
#define USART_PULSEDIV_128PCLK         (0x07)              /*!< Pulse Width 128 x Tpclk          */

#define USART_IRDAEN                   (1 << 0)            /*!< Autobaud Running                 */
#define USART_IRDAINV                  (1 << 1)            /*!< Mode 1 Selected (0 = Mode 0)     */
#define USART_FIXPULSEEN               (1 << 2)            /*!< Autobaud Restart on Timeout      */
#define USART_OBTOIRQCLR               (1 << 9)            /*!< Clear ABTO Interrupt             */

/**
  * @}
  */

/** @defgroup USART_FDR_Bit_Definitions (UxFDR) Uart Fractional Divider Register
  *
  * @{
  */

#define USART_FDR_Mask                 (0xff)              /*!< Useable Bits in FDR Register    */
#define USART_FDR_Shift                (0)

#define USART_DIVADDVAL_Mask           (0x0f)              /*!< Fractional Divider Div. Val Mask */
#define USART_DIVADDVAL_Shift          (0)

#define USART_MULVAL_Mask              (0x0f << 4)         /*!< Fract. Divider Mult. Val Mask    */
#define USART_MULVAL_Shift             (4)

/**
  * @}
  */

/** @defgroup USART_TER_Bit_Definitions (UxTER) USART Transmit Enable Register
  *
  * @{
  */

#define USART_TER_Mask                 (1 << 7)            /*!< Useable Bits in TER Register     */
#define USART_TER_Shift                (0)

#define USART_TXEN                     (1 << 7)            /*!< Enable Transmitter               */

/**
  * @}
  */

/** @defgroup USART_HDR_Bit_Definitions (UxHDR) USART Half Duplex Enable Register
  *
  * @{
  */

#define USART_HDR_Mask                 (1 << 7)            /*!< Useable Bits in HDR Register     */
#define USART_HDR_Shift                (0)

#define USART_HDEN                     (1 << 7)            /*!< Enable Half Duplex               */

/**
  * @}
  */

/** @defgroup USART_SCICTRL_Bit_Definitions (UxSCICTRL) USART Smart Card Interface Control Register
  *
  * @{
  */

#define USART_SCICTRL_Mask             (0xff7e)            /*!< Useable Bits in SCICTRL Register */
#define USART_SCICTRL_Shift            (0)

#define USART_TXRETRY_Mask             (0x07 << 5)         /*!< Smart Card Max Retransmits Mask  */
#define USART_TXRETRY_Shift            (5)
#define USART_TXRETRY_0                (0x00)              /*!< Max 0 Retries Before TX Err set  */
#define USART_TXRETRY_1                (0x01)              /*!< Max 1 Retries Before TX Err set  */
#define USART_TXRETRY_2                (0x02)              /*!< Max 2 Retries Before TX Err set  */
#define USART_TXRETRY_3                (0x03)              /*!< Max 3 Retries Before TX Err set  */
#define USART_TXRETRY_4                (0x04)              /*!< Max 4 Retries Before TX Err set  */
#define USART_TXRETRY_5                (0x05)              /*!< Max 5 Retries Before TX Err set  */
#define USART_TXRETRY_6                (0x06)              /*!< Max 6 Retries Before TX Err set  */
#define USART_TXRETRY_7                (0x07)              /*!< Max 7 Retries Before TX Err set  */

#define USART_XTRAGUARD_Mask           (0xff << 8)         /*!< Extra Guard Time Mask            */
#define USART_XTRAGUARD_Shift          (8)

#define USART_SCIEN                    (1 << 0)            /*!< Smart Card Interface Enable      */
#define USART_NACKDIS                  (1 << 1)            /*!< Smart Card NACK Response Disable */
#define USART_PROTSEL                  (1 << 2)            /*!< Smart Card Protocol Select       */
#define USART
#define USART_HDEN                     (1 << 7)            /*!< Enable Half Duplex               */

/**
  * @}
  */

/** @defgroup USART_RS485CTRL_Bit_Definitions (UxRS485CTRL) USART RS485-Mode Control Register
  *
  * @{
  */

#define USART_RS485CTRL_Mask           (0x3f)              /*!< Useable Bits in RS485CTRL Reg.   */
#define USART_RS485CTRL_Shift          (0)

#define USART_NMMEN                    (1 << 0)            /*!< Enable RS/EIA485 Multidrop Mode  */
#define USART_RXDIS                    (1 << 1)            /*!< Disable Receiver                 */
#define USART_AADEN                    (1 << 2)            /*!< Enable RS/EIA485 Auto Addr Det.  */
#define USART_DIRSEL                   (1 << 3)            /*!< 0=RTS Dir Ctrl, 1=DTR Dir Ctrl   */
#define USART_DCTRL                    (1 << 4)            /*!< Enable Direction Control         */
#define USART_OINV                     (1 << 5)            /*!< Invert Polarity on Dir Ctrl Pin  */

/**
  * @}
  */

/** @defgroup USART_RS485ADRMATCH_Bit_Definitions (UxRS485ADRMATCH) USART RS485-Mode Address Match Register
  *
  * @{
  */

#define USART_RS485ADRMATCH_Mask       (0xff)              /*!< RS485 Address Match Value        */
#define USART_RS485ADRMATCH_Shift      (0)

/**
  * @}
  */

/** @defgroup USART_RS485DLY_Bit_Definitions (UxRS485DLY) USART RS485-Mode Delay Register
  *
  * @{
  */

#define USART_DLY_Mask                 (0xff)              /*!< Direction Control Delay Value    */
#define USART_DLY_Shift                (0)

/**
  * @}
  */

/**
  * @}
  */


/** @addtogroup ADC
  *
  * Analog to Digital Converter
  *
  * @{
  */

/** @defgroup ADC_CR0_Bit_Definitions (ADCxCR) ADC Control Register 0
  *
  * @{
  */

#define ADC_CR0_Mask                   (0x00ffffffUL)      /*!< Useable Bits in ADC CR0 Register */
#define ADC_CR0_Shift                  (0)

#define ADC_SEL_Mask                   (0xffUL)            /*!< ADC Channel Selection Mask       */
#define ADC_SEL_Shift                  (0)
#define ADC_SEL_0                      (1 << 0)            /*!< Select AD0 for Conversion        */
#define ADC_SEL_1                      (1 << 1)            /*!< Select AD1 for Conversion        */
#define ADC_SEL_2                      (1 << 2)            /*!< Select AD2 for Conversion        */
#define ADC_SEL_3                      (1 << 3)            /*!< Select AD3 for Conversion        */
#define ADC_SEL_4                      (1 << 4)            /*!< Select AD4 for Conversion        */
#define ADC_SEL_5                      (1 << 5)            /*!< Select AD5 for Conversion        */
#define ADC_SEL_6                      (1 << 6)            /*!< Select AD6 for Conversion        */
#define ADC_SEL_7                      (1 << 7)            /*!< Select AD7 for Conversion        */

#define ADC_CLKDIV_Mask                (0xffUL << 8)       /*!< ADC Clock Divisor Mask           */
#define ADC_CLKDIV_Shift               (8)

#define ADC_CLKS_Mask                  (0x07UL << 17)      /*!< ADC # Clocks / Burst Conversion  */
#define ADC_CLKS_Shift                 (17)

#define ADC_START_Mask                 (0x07UL << 24)      /*!< Start Mode Mask                  */
#define ADC_START_Shift                (24)

#define ADC_BURST                      (1UL << 16)         /*!< Enable "Burst Mode" (HW scan)    */
#define ADC_EDGE                       (1UL << 27)         /*!< Set Edge for Capture Mode        */

/**
  * @}
  */

/** @defgroup ADC_DR_Bit_Definitions, ADC_GDR_Bit_Definitions (ADCxDR, ADCxGDR) ADC Data Registers, ADC Global Data Register
  *
  * @{
  */

#define ADC_GDR_Mask                   (0xc700ffffUL)      /*!< Useable Bits in ADC GDR Reg      */
#define ADC_GDR_Shift                  (0)

#define ADC_V_VREF_Mask                (0xffffUL)          /*!< Result of ADC Conversion         */
#define ADC_V_VREF_Shift               (0)

#define ADC_CHN_Mask                   (0x07UL << 24)      /*!< Channel Result Came From         */
#define ADC_CHN_Shift                  (24)

#define ADC_OVERRUN                    (1UL << 30)         /*!< Result Overwritten Flag          */
#define ADC_DONE                       (1UL << 31)         /*!< Conversion Done Flag             */

/**
  * @}
  */

/** @defgroup ADC_STAT_Bit_Definitions (ADCxSTAT) ADC Status Register
  *
  * @{
  */

#define ADC_STAT_Mask                  (0x0001ffffUL)      /*!< Useable Bits in ADC STAT Reg.    */
#define ADC_STAT_Shift                 (0)

#define ADC_STATDONE_Mask              (0xffUL)            /*!< "DONE" Flags Mask                */
#define ADC_STATDONE_Shift             (0)
#define ADC_STATDONE_0                 (1UL << 0)          /*!< Channel 0 Done Flag              */
#define ADC_STATDONE_1                 (1UL << 1)          /*!< Channel 1 Done Flag              */
#define ADC_STATDONE_2                 (1UL << 2)          /*!< Channel 2 Done Flag              */
#define ADC_STATDONE_3                 (1UL << 3)          /*!< Channel 3 Done Flag              */
#define ADC_STATDONE_4                 (1UL << 4)          /*!< Channel 4 Done Flag              */
#define ADC_STATDONE_5                 (1UL << 5)          /*!< Channel 5 Done Flag              */
#define ADC_STATDONE_6                 (1UL << 6)          /*!< Channel 6 Done Flag              */
#define ADC_STATDONE_7                 (1UL << 7)          /*!< Channel 7 Done Flag              */

#define ADC_STATOVERRUN_Mask           (0xffUL << 8)       /*!< "OVERRUN" Flags Mask             */
#define ADC_STATOVERRUN_Shift          (8)
#define ADC_STATOVERRUN_0              (1UL << 8)          /*!< Channel 0 Overrun Flag           */
#define ADC_STATOVERRUN_1              (1UL << 9)          /*!< Channel 1 Overrun Flag           */
#define ADC_STATOVERRUN_2              (1UL << 10)         /*!< Channel 2 Overrun Flag           */
#define ADC_STATOVERRUN_3              (1UL << 11)         /*!< Channel 3 Overrun Flag           */
#define ADC_STATOVERRUN_4              (1UL << 12)         /*!< Channel 4 Overrun Flag           */
#define ADC_STATOVERRUN_5              (1UL << 13)         /*!< Channel 5 Overrun Flag           */
#define ADC_STATOVERRUN_6              (1UL << 14)         /*!< Channel 6 Overrun Flag           */
#define ADC_STATOVERRUN_7              (1UL << 15)         /*!< Channel 7 Overrun Flag           */

#define ADC_ADINT                      (1UL << 16)         /*!< Interrupt Flag                   */

/**
  * @}
  */

/** @defgroup ADC_INTEN_Bit_Definitions (ADCxINTEN) ADC Interrupt Enable Register
  *
  * @{
  */

#define ADC_INTEN_Mask                 (0x01ff)            /*!< Useable Bits in ADC INTEN Reg    */
#define ADC_INTEN_Shift                (0)

#define ADC_ADINTEN_Mask               (0xff)              /*!< Enable Channel Interrupts Mask   */
#define ADC_ADINTEN_Shift              (0)
#define ADC_ADINTEN_0                  (1 << 0)            /*!< Enable Interrupt on Channel 0    */
#define ADC_ADINTEN_1                  (1 << 1)            /*!< Enable Interrupt on Channel 1    */
#define ADC_ADINTEN_2                  (1 << 2)            /*!< Enable Interrupt on Channel 2    */
#define ADC_ADINTEN_3                  (1 << 3)            /*!< Enable Interrupt on Channel 3    */
#define ADC_ADINTEN_4                  (1 << 4)            /*!< Enable Interrupt on Channel 4    */
#define ADC_ADINTEN_5                  (1 << 5)            /*!< Enable Interrupt on Channel 5    */
#define ADC_ADINTEN_6                  (1 << 6)            /*!< Enable Interrupt on Channel 6    */
#define ADC_ADINTEN_7                  (1 << 7)            /*!< Enable Interrupt on Channel 7    */

#define ADC_ADGINTEN                   (1 << 8)            /*!< Enable ADC Global Interrupt      */

/**
  * @}
  */

/**
  * @}
  */


/** @addtogroup PMU
  *
  * Power Mangement Unit
  *
  * @{
  */

/** @defgroup PMU_PCON_Bit_Definitions (PCON) Power Configuration Register
  *
  * @{
  */

#define PMU_PCON_Mask                  (0x0903)            /*!< Useable Bits in PMU PCON Reg.    */
#define PMU_PCON_Shift                 (0)

#define PMU_PM_Mask                    (0x03)              /*!< Power Mode Bits                  */
#define PMU_PM_Shift                   (0)

#define PMU_PM_Default                 (0x00)              /*!< Part in Active or Sleep Mode     */
#define PMU_PM_DeepSleep               (0x01)              /*!< WFI enters Deep Sleep Mode       */
#define PMU_PM_PowerDown               (0x02)              /*!< WFI enters Power Down Mode       */
#define PMU_PM_DeepPowerDown           (0x03)              /*!< WFI enters Deep Power Down Mode  */

#define PMU_DPDEN                      (1 << 1)            /*!< Enable Deep Power Down on WFI    */
#define PMU_SLEEPFLAG                  (1 << 8)            /*!< Power Down Mode Entered Flag     */
#define PMU_DPDFLAG                    (1 << 11)           /*!< Deep Power Down Mode Flag        */

/**
  * @}
  */

/** @defgroup PMU_GPREG4_Bit_Definitions (GPREG4) General Purpose Register 4 / Wakeup Hysteresis Control Register
  *
  * @{
  */

#define PMU_WAKEUPHYS                  (1 << 10)           /*!< Enable Hysteresis on Wakeup Pin  */

/**
  * @}
  */

/**
  * @}
  */


/** @addtogroup FLASH
  *
  * (FLASHCFG) Flash Configuration Register
  *
  * @{
  */

/** @defgroup FLASH_FLASHCFG_Bit_Definitions
  * @{
  */

#define FLASH_FLASHTIM_Mask            (0x03)              /*!< Mask for Setting Flash Timing    */
#define FLASH_FLASHTIM_Shift           (0)
#define FLASH_FLASHTIM_1               (0x00)              /*!< Flash Accesses take 1 Sys Clock  */
#define FLASH_FLASHTIM_2               (0x01)              /*!< Flash Accesses take 2 Sys Clocks */
#define FLASH_FLASHTIM_3               (0x02)              /*!< Flash Accesses take 3 Sys Clocks */

/**
  * @}
  */

/**
  * @}
  */


/** @addtogroup SSP
  *
  * Synchronous Serial Peripheral
  *
  * @{
  */

/** @defgroup SSP_CR0_Bit_Definitions (SSPxCR0) SSP Control Register 0
  *
  * @{
  */

#define SSP_CR0_Mask                   (0xffff)            /*!< Useable Bits in SSP CR0 Register */
#define SSP_CR0_Shift                  (0)

#define SSP_DSS_Mask                   (0x0f)              /*!< SSP Data Size Select Mask        */
#define SSP_DSS_Shift                  (0)
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
#define SSP_FRF_Shift                  (4)
#define SSP_FRF_SPI                    (0x00)              /*!< SPI Frame Format                 */
#define SSP_FRF_TI                     (0x01 << 4)         /*!< TI Frame Format                  */
#define SSP_FRF_MICROWIRE              (0x02 << 4)         /*!< Microwire Frame Format           */

/* Bit Frequency is PCLK / (CPSDVSR * (SCR + 1)) */
#define SSP_SCR_Mask                   (0xff << 8)         /*!< Serial Clock Data Rate Mask      */
#define SSP_SCR_Shift                  (8)

#define SSP_CPOL                       (1 << 6)            /*!< Clock Polarity Hi Btwn Frames    */
#define SSP_CPHA                       (1 << 7)            /*!< Latch Bits on 2nd Edge           */

/**
  * @}
  */

/** @defgroup SSP_CR1_Bit_Definitions (SSPxCR1) SSP Control Register 1
  *
  * @{
  */

#define SSP_CR1_Mask                   (0x0f)              /*!< Useable Bits in SSP CR1          */
#define SSP_CR1_Shift                  (0)

#define SSP_CR1_MODE_Mask              (0x0c)              /*!< SSP Mode Mask                    */
#define SSP_CR1_MODE_Shift             (0)

#define SSP_LBM                        (1 << 0)            /*!< Loop Back Mode Enable            */
#define SSP_SSE                        (1 << 1)            /*!< SSP Peripheral Enable            */
#define SSP_MS                         (1 << 2)            /*!< Slave Mode Enable                */
#define SSP_SOD                        (1 << 3)            /*!< Disable Slave Mode Output        */

/**
  * @}
  */

/** @defgroup SSP_SR_Bit_Definitions (SSPxSR) SSP Status Register
  *
  * @{
  */

#define SSP_SR_Mask                    (0x1f)              /*!< Useable Bits in SSP SR Register  */
#define SSP_SR_Shift                   (0)

#define SSP_TFE                        (1 << 0)            /*!< Transmit FIFO Empty Flag         */
#define SSP_TNF                        (1 << 1)            /*!< Transmit FIFO Not Full Flag      */
#define SSP_RNE                        (1 << 2)            /*!< Receive FIFO Not Empty Flag      */
#define SSP_RFF                        (1 << 3)            /*!< Receive FIFO Full Flag           */
#define SSP_BSY                        (1 << 4)            /*!< SPI Controller Busy Flag         */

/**
  * @}
  */

/** @defgroup SSP_IMR_Bit_Definitions (SSPxIMR) SSP Interrupt Mask Set/Clear Register
  *
  * @{
  */

#define SSP_IMR_Mask                   (0x0f)              /*!< Useable Bits in SSP IMR Register */
#define SSP_IMR_Shift                  (0)

#define SSP_RORIM                      (1 << 0)            /*!< Receive Overrun IRQ Enable       */
#define SSP_RTIM                       (1 << 1)            /*!< Receive Timeout IRQ Enable       */
#define SSP_RXIM                       (1 << 2)            /*!< Rx FIFO > 1/2 Full IRQ Enable    */
#define SSP_TXIM                       (1 << 3)            /*!< Tx FIFO at least 1/2 Empty       */

/**
  * @}
  */

/** @defgroup SSP_RIS_Bit_Definitions (SSPxRIS) SSP Raw Interrupt Status Register
  *
  * @{
  */

#define SSP_RIS_Mask                   (0x0f)              /*!< Useable Bits in SSP RIS Register */
#define SSP_RIS_Shift                  (0)

#define SSP_RORRIS                     (1 << 0)            /*!< Raw Receive Overrun IRQ          */
#define SSP_RTRIS                      (1 << 1)            /*!< Raw Receive Timeout IRQ          */
#define SSP_RXRIS                      (1 << 2)            /*!< Raw Rx FIFO > 1/2 Full IRQ       */
#define SSP_TXRIS                      (1 << 3)            /*!< Raw Tx FIFO at least 1/2         */

/**
  * @}
  */

/** @defgroup SSP_MIR_Bit_Definitions (SSPxMIR) SSP Masked Interrupt Status Register
  *
  * @{
  */

#define SSP_MIR_Mask                   (0x0f)              /*!< Useable Bits in SSP MIR Register */
#define SSP_MIR_Shift                  (0)

#define SSP_RORMIS                     (1 << 0)            /*!< Rx Overrun Interrupt Pending     */
#define SSP_RTMIS                      (1 << 2)            /*!< Rx Timeout Interrupt Pending     */
#define SSP_RXMIS                      (1 << 3)            /*!< Rx Fifo Half Full Int Pending    */
#define SSP_TXMIS                      (1 << 4)            /*!< Tx Fifo Half Empty Int Pending   */

/**
  * @}
  */

/** @defgroup SSP_ICR_Bit_Definitions (SSPxICR) SSP Interrupt Clear Register
  *
  * @{
  */

#define SSP_ICR_Mask                   (0x03)              /*!< Useable Bits in SSP ICR Register */
#define SSP_ICR_Shift                  (0)

#define SSP_RORIC                      (1 << 0)            /*!< Clear Receive Overrun Interrupt  */
#define SSP_RTIC                       (1 << 1)            /*!< Clear Receive Timeout Interrupt  */

/**
  * @}
  */

/**
  * @}
  */


/** @addtogroup IOCON
  *
  * IO Configuration Block
  *
  * @{
  */

/** @defgroup IOCON_PIOn_Bit_Definitions (IOCONPIOn) IO Configuration Registers
  *
  * @{
  */

#define IOCON_PIO_Mask                 (0x03bf)            /*!< Useable Bits in IOCON PIO Regs   */
#define IOCON_PIO_Shift                (0)

#define IOCON_FUNC_Mask                (0x07)              /*!< Pin Function Mask                */
#define IOCON_FUNC_Shift               (0)

#define IOCON_MODE_Mask                (0x03 << 3)         /*!< Pin Mode Mask                    */
#define IOCON_MODE_Shift               (3)
#define IOCON_MODE_INACTIVE            (0x00)              /*!< No Pullup / Pulldown             */
#define IOCON_MODE_PULLDOWN            (0x01 << 3)         /*!< Pulldown Resistor Enabled        */
#define IOCON_MODE_PULLUP              (0x02 << 3)         /*!< Pullup Resistor Enabled          */
#define IOCON_MODE_REPEATER            (0x03 << 3)         /*!< Repeater Mode                    */

#define IOCON_I2C_Mask                 (0x0300)            /*!< Pin I2C Setting Mask             */
#define IOCON_I2C_Shift                (8)
#define IOCON_I2C_I2C                  (0x00)              /*!< Standard I2C                     */
#define IOCON_I2C_GPIO                 (0x01 << 8)         /*!< Standard GPIO                    */
#define IOCON_I2C_FASTPLUS             (0x02 << 8)         /*!< FastPlus I2C                     */

#define IOCON_HYS                      (1 << 5)            /*!< Enable Hysteresis                */
#define IOCON_AD                       (1 << 7)            /*!< Enable Analog Mode               */

/**
  * @}
  */

/** @defgroup IOCON_LOC_Bit_Definitions (SCK_LOC, RI_LOC, etc.) Pin Location Configuration Registers
  *
  * @{
  */

#define IOCON_LOC_Mask                 (0x03)              /*!< Pin Location Mask                */
#define IOCON_LOC_Shift                (0)

/**
  * @}
  */

/**
  * @}
  */


/** @addtogroup SYSCON
  *
  * System Configuration Block
  *
  *  Naming here adds the register name since SYSCON combines so many
  *  different functions
  *
  * @{
  */

/** @defgroup SYSCON_SYSMEMREMAP_Bit_Definitions (SYSMEMREMAP) Memory Remapping Register
  *
  * @{
  */

#define SYSCON_SYSMEMREMAP_MAP_Mask       (0x03)           /*!< Memory Remap Bit Mask            */
#define SYSCON_SYSMEMREMAP_MAP_Shift      (0)
#define SYSCON_SYSMEMREMAP_MAP_BOOTLOADER (0x00)           /*!< Map Bootloader ROM into Sys Mem  */
#define SYSCON_SYSMEMREMAP_MAP_RAM        (0x01)           /*!< Map RAM into System Memory       */
#define SYSCON_SYSMEMREMAP_MAP_FLASH      (0x02)           /*!< Map FLASH into System Memory     */

/**
  * @}
  */

/** @defgroup SYSCON_PRESETCTRL_Bit_Definitions (PRESETCTRL) Peripheral Reset Control Register
  *
  * @{
  */

#define SYSCON_PRESETCTRL_RESET_Mask   (0x0f)              /*!< Peripheral Reset Mask            */
#define SYSCON_PRESETCTRL_RESET_Shift  (0)
#define SYSCON_PRESETCTRL_SSP0_RST_N   (1 << 0)            /*!< Reset SSP0 Peripheral            */
#define SYSCON_PRESETCTRL_I2C_RST_N    (1 << 1)            /*!< Reset I2C Peripheral             */
#define SYSCON_PRESETCTRL_SSP1_RST_N   (1 << 2)            /*!< Reset SSP1 Peripheral            */
#define SYSCON_PRESETCTRL_CAN_RST_N    (1 << 3)            /*!< Reset CAN Peripheral             */

/**
  * @}
  */

/** @defgroup SYSCON_SYSPLLCTRL_Bit_Definitions (SYSPLLCTRL) System PLL Control Register
  *
  * @{
  */

#define SYSCON_SYSPLLCTRL_Mask         (0x7f)              /*!< SYSPLLCTRL Useable bits          */
#define SYSCON_SYSPLLCTRL_Shift        (0)

#define SYSCON_SYSPLLCTRL_MSEL_Mask    (0x1f)              /*!< SysPLL Feedback Divider Mask     */
#define SYSCON_SYSPLLCTRL_MSEL_Shift   (0)

#define SYSCON_SYSPLLCTRL_PSEL_Mask    (0x03 << 5)         /*!< SysPLL Post Divider Ratio Mask   */
#define SYSCON_SYSPLLCTRL_PSEL_Shift   (5)
#define SYSCON_SYSPLLCTRL_PSEL_DIV2    (0x00 << 5)         /*!< Post-Divide by 2                 */
#define SYSCON_SYSPLLCTRL_PSEL_DIV4    (0x01 << 5)         /*!< Post-Divide by 4                 */
#define SYSCON_SYSPLLCTRL_PSEL_DIV8    (0x02 << 5)         /*!< Post-Divide by 8                 */
#define SYSCON_SYSPLLCTRL_PSEL_DIV16   (0x03 << 5)         /*!< Post-Divide by 16                */

/**
  * @}
  */

/** @defgroup SYSCON_SYSPLLSTAT_Bit_Definitions (SYSPLLSTAT) System PLL Status Register
  *
  * @{
  */

#define SYSCON_SYSPLLSTAT_LOCKED       (1 << 0)            /*!< (RO) System PLL is Locked Flag   */

/**
  * @}
  */

/** @defgroup SYSCON_IRCCTL_Bit_Definitions (IRCCTL) Internal RC Clock Control Register
  *
  * @{
  */

#define SYSCON_IRCCTL_TRIM_Mask        (0xff)              /*!< Internal RC Clock Trim Value     */
#define SYSCON_IRCCTL_TRIM_Shift       (0)

/**
  * @}
  */

/** @defgroup SYSCON_SYSOSCCTRL_Bit_Definitions (SYSOSCCTRL) System Oscillator Control Register
  *
  * @{
  */

#define SYSCON_SYSOSCCTRL_Mask         (0x03)              /*!< Useable SYSOSCCTRL Bits          */
#define SYSCON_SYSOSCCTRL_Shift        (0)

#define SYSCON_SYSOSCCTRL_BYPASS       (1 << 0)            /*!< Bypass the System Oscillator     */
#define SYSCON_SYSOSCCTRL_FREQRANGE    (1 << 1)            /*!< External Xtal Frequency 15-25Mhz */

/**
  * @}
  */

/** @defgroup SYSCON_WDTOSCCTL_Bit_Definitions (WDTOSCCTL) Watchdog Timer Oscillator Control Register
  *
  * @{
  */

#define SYSCON_WDTOSCCTL_Mask           (0x01ff)           /*!< WDTOSCCTL Useable Bits           */
#define SYSCON_WDTOSCCTL_Shift          (0)

#define SYSCON_WDTOSCCTRL_DIV_Mask      (0x1f)             /*!< WDT Oscillator Divider Value     */
#define SYSCON_WDTOSCCTRL_DIV_Shift     (0)

#define SYSCON_WDTOSCCTRL_FREQSEL_Mask  (0x01e0)           /*!< WDT Oscillator Frequency Mask    */
#define SYSCON_WDTOSCCTRL_FREQSEL_Shift (5)
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

/**
  * @}
  */

/** @defgroup SYSCON_SYSRESSTAT_Bit_Definitions (SYSRESSTAT) System Reset Source Status Register
  *
  * @{
  */

#define SYSCON_SYSRESSTAT_STAT_Mask    (0x1f)              /*!< System Reset Source Mask         */
#define SYSCON_SYSRESSTAT_STAT_Shift   (0)
#define SYSCON_SYSRESSTAT_STAT_POR     (1 << 0)            /*!< Reset was from Power-On Reset    */
#define SYSCON_SYSRESSTAT_STAT_EXTRST  (1 << 1)            /*!< Reset was from External Reset    */
#define SYSCON_SYSRESSTAT_STAT_WDT     (1 << 2)            /*!< Reset was from Watchdog          */
#define SYSCON_SYSRESSTAT_STAT_BOD     (1 << 3)            /*!< Reset was from Brownout          */
#define SYSCON_SYSRESSTAT_STAT_SYSRST  (1 << 4)            /*!< Reset was from NVIC Reset        */

/**
  * @}
  */

/** @defgroup SYSCON_SYSPLLCLKSEL_Bit_Definitions (SYSPLLCLKSEL) System PLL Clock Select Register
  *
  * @{
  */

#define SYSCON_SYSPLLCLKSEL_Mask       (0x03)              /*!< System PLL Clock Select Mask     */
#define SYSCON_SYSPLLCLKSEL_Shift      (0)

#define SYSCON_SYSPLLCLKSEL_IRC        (0x00)              /*!< System PLL Fed from Internal RC  */
#define SYSCON_SYSPLLCLKSEL_SYSOSC     (0x01)              /*!< System PLL Fed from Sys Osc.     */

/**
  * @}
  */

/** @defgroup SYSCON_SYSPLLUEN_Bit_Definitions (SYSPLLUEN) System PLL Source Update Enable Register
  *
  * @{
  */

#define SYSCON_SYSPLLUEN_ENA           (1 << 0)            /*!< Enable System PLL Source Update  */

/**
  * @}
  */

/** @defgroup SYSCON_MAINCLKSEL_Bit_Definitions (MAINCLKSEL) Main Clock Select Register
  *
  * @{
  */

#define SYSCON_MAINCLKSEL_Mask         (0x03)              /*!< Main Clock Source Select Mask    */
#define SYSCON_MAINCLKSEL_Shift        (0)

#define SYSCON_MAINCLKSEL_IRC          (0x00)              /*!< Main Clock Fed from Internal RC  */
#define SYSCON_MAINCLKSEL_SYSPLLIN     (0x01)              /*!< Main Clock Fed from Sys PLL IN   */
#define SYSCON_MAINCLKSEL_WDTOSC       (0x02)              /*!< Main Clock Fed from WDT Osc.     */
#define SYSCON_MAINCLKSEL_SYSPLLOUT    (0x03)              /*!< Main Clock Fed from Sys PLL Out  */

/**
  * @}
  */

/** @defgroup SYSCON_MAINCLKUEN_Bit_Definitions (MAINCLKUEN) Main Clock Source Update Enable Register
  *
  * @{
  */

#define SYSCON_MAINCLKUEN_ENA          (1 << 0)            /*!< Enable Update of Main Clock Src  */

/**
  * @}
  */

/** @defgroup SYSCON_SYSAHBCLKDIV_Bit_Definitions (SYSAHBCLKDIV) System AHB Clock Divider Register
  *
  * @{
  */

#define SYSCON_SYSAHBCLKDIV_Mask       (0xff)              /*!< AHB Clock Divider Value          */
#define SYSCON_SYSAHBCLKDIV_Shift      (0)

/**
  * @}
  */

/** @defgroup SYSCON_SYSAHBCLKCTRL_Bit_Definitions (SYSAHBCLKCTRL) System AHB Clock Control Register
  *
  * @{
  */

#define SYSCON_SYSAHBCLKCTRL_Mask       (0x0003ffffUL)     /*!< AHB Clock Control                */
#define SYSCON_SYSAHBCLKCTRL_Shift      (0)
#define SYSCON_SYSAHBCLKCTRL_SYS        (1 << 0)           /*!< (RO) Main System Clock Enabled   */
#define SYSCON_SYSAHBCLKCTRL_ROM        (1 << 1)           /*!< Enable AHB clock to System ROM   */
#define SYSCON_SYSAHBCLKCTRL_RAM        (1 << 2)           /*!< Enable AHB clock to System RAM   */
#define SYSCON_SYSAHBCLKCTRL_FLASHREG   (1 << 3)           /*!< Enable AHB clock to Flash CTRL   */
#define SYSCON_SYSAHBCLKCTRL_FLASHARRAY (1 << 4)           /*!< Enable AHB clock to system FLASH */
#define SYSCON_SYSAHBCLKCTRL_I2C        (1 << 5)           /*!< Enable AHB clock to I2C Periph.  */
#define SYSCON_SYSAHBCLKCTRL_GPIO       (1 << 6)           /*!< Enable AHB clock to GPIO Periph. */
#define SYSCON_SYSAHBCLKCTRL_CT16B0     (1 << 7)           /*!< Enable AHB clock to 16b Timer 0  */
#define SYSCON_SYSAHBCLKCTRL_CT16B1     (1 << 8)           /*!< Enable AHB clock to 16b Timer 1  */
#define SYSCON_SYSAHBCLKCTRL_CT32B0     (1 << 9)           /*!< Enable AHB clock to 32b Timer 0  */
#define SYSCON_SYSAHBCLKCTRL_CT32B1     (1 << 10)          /*!< Enable AHB clock to 32b Timer 1  */
#define SYSCON_SYSAHBCLKCTRL_SSP0       (1 << 11)          /*!< Enable AHB clock to SSP0 Periph. */
#define SYSCON_SYSAHBCLKCTRL_USART      (1 << 12)          /*!< Enable AHB clock to USART Perip. */
#define SYSCON_SYSAHBCLKCTRL_ADC        (1 << 13)          /*!< Enable AHB clock to ADC          */
#define SYSCON_SYSAHBCLKCTRL_WDT        (1 << 15)          /*!< Enable AHB clock to Watchdog Tmr */
#define SYSCON_SYSAHBCLKCTRL_IOCON      (1 << 16)          /*!< Enable AHB clock to IO Config    */
#define SYSCON_SYSAHBCLKCTRL_CAN        (1 << 17)          /*!< Enable AHB clock to CAN Periph.  */
#define SYSCON_SYSAHBCLKCTRL_SSP1       (1 << 18)          /*!< Enable AHB clock to SSP1 Periph. */

/**
  * @}
  */

/** @defgroup SYSCON_SSP0CLKDIV_Bit_Definitions (SSP0CLKDIV) SSP0 Clock Divider Register
  *
  * @{
  */

#define SYSCON_SSP0CLKDIV_Mask         (0xff)              /*!< SSP0 Clock Divider Value         */
#define SYSCON_SSP0CLKDIV_Shift        (0)

/**
  * @}
  */

/** @defgroup SYSCON_USARTCLKDIV_Bit_Definitions (USARTCLKDIV) USART Clock Divider Register
  *
  * @{
  */

#define SYSCON_USARTCLKDIV_Mask        (0xff)              /*!< USART Clock Divider Value        */
#define SYSCON_USARTCLKDIV_Shift       (0)

/**
  * @}
  */

/** @defgroup SYSCON_SSP1CLKDIV_Bit_Definitions (SSP1CLKDIV) SSP1 Clock Divider Register
  *
  * @{
  */

#define SYSCON_SSP1CLKDIV_Mask         (0xff)              /*!< SSP1 Clock Divider Value         */
#define SYSCON_SSP1CLKDIV_Shift        (0)

/**
  * @}
  */

/** @defgroup SYSCON_WDTCLKSEL_Bit_Definitions (WDTCLKSEL) Watchdog Timer Clock Select Register
  *
  * @{
  */

#define SYSCON_WDTCLKSEL_Mask          (0x03)              /*!< WDT Clock Selection Mask         */
#define SYSCON_WDTCLKSEL_Shift         (0)

#define SYSCON_WDTCLKSEL_IRC           (0x00)              /*!< WDT Clock Fed From Internal RC   */
#define SYSCON_WDTCLKSEL_MAINCLK       (0x01)              /*!< WDT Clock Fed from Main Clock    */
#define SYSCON_WDTCLKSEL_WDTOSC        (0x02)              /*!< WDT Clock Fed from WDT Osc.      */

/**
  * @}
  */

/** @defgroup SYSCON_WDTCLKUEN_Bit_Definitions (WDTCLKUEN) Watchdog Timer Clock Source Update Enable Register
  *
  * @{
  */

#define SYSCON_WDTCLKUEN_ENA           (1 << 0)            /*!< Enable Update of WDT Clock Src.  */

/**
  * @}
  */

/** @defgroup SYSCON_WDTCLKDIV_Bit_Definitions (WDTCLKDIV) Watchdog Timer Clock Divider Register
  *
  * @{
  */

#define SYSCON_WDTCLKDIV_Mask          (0xff)              /*!< WDT Clock Divider Value          */
#define SYSCON_WDTCLKDIV_Shift         (0)

/**
  * @}
  */

/** @defgroup SYSCON_CLKOUTSEL_Bit_Definitions (CLKOUTSEL) Clock Out Select Register
  *
  * @{
  */

#define SYSCON_CLKOUTSEL_Mask          (0x03)              /*!< CLKOUT Clock Source Mask         */
#define SYSCON_CLKOUTSEL_Shift         (0)

#define SYSCON_CLKOUTSEL_IRC           (0x00)              /*!< CLKOUT Clock Fed from Int. RC    */
#define SYSCON_CLKOUTSEL_SYSOSC        (0x01)              /*!< CLKOUT Clock Fed from System Osc.*/
#define SYSCON_CLKOUTSEL_WDTOSC        (0x02)              /*!< CLKOUT Clock Fed from WDT Osc.   */
#define SYSCON_CLKOUTSEL_MAINCLK       (0x03)              /*!< CLKOUT Clock Fed from Main Clock */

/**
  * @}
  */

/** @defgroup SYSCON_CLKOUTUEN_Bit_Definitions (CLKOUTUEN) Clock Out Source Update Enable Register
  *
  * @{
  */

#define SYSCON_CLKOUTUEN_ENA           (1 << 0)            /*!< Enable Update of CLKOUT Source   */

/**
  * @}
  */

/** @defgroup SYSCON_CLKOUTDIV_Bit_Definitions (CLKOUTDIV) Clock Out Divider Register
  *
  * @{
  */

#define SYSCON_CLKOUTDIV_Mask          (0xff)              /*!< CLKOUT Divider Value Mask        */
#define SYSCON_CLKOUTDIV_Shift         (0)

/**
  * @}
  */

/** @defgroup SYSCON_PIOPORCAP0_Bit_Definitions (PIOPORCAP0) Programmable IO Power On Reset Capture Register 0
  *
  * @{
  */

#define SYSCON_PIOPORCAP0_CAP_Mask     (0xffffffffUL)      /*!< Useable Bits in PIOPORCAP0       */
#define SYSCON_PIOPORCAP0_CAP_Shift    (0)

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

/**
  * @}
  */

/** @defgroup SYSCON_PIOPORCAP1_Bit_Definitions (PIOPORCAP1) Programmable IO Power On Reset Capture Register 1
  *
  * @{
  */

#define SYSCON_PIOPORCAP1_CAP_Mask     (0x0000003fUL)      /*!< Useable Bits in PIOPORCAP1       */
#define SYSCON_PIOPORCAP1_CAP_Shift    (0)

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

/**
  * @}
  */

/** @defgroup SYSCON_BODCTRL_Bit_Definitions (BODCTRL) Brown-Out Detector Control Register
  *
  * @{
  */

#define SYSCON_BODCTRL_Mask            (0x1f)              /*!< BODCTRL Useable Bits             */
#define SYSCON_BODCTRL_Shift           (0)

#define SYSCON_BODCTRL_BODRSTLEV_Mask  (0x03)              /*!< BOD Reset Level Mask             */
#define SYSCON_BODCTRL_BODRSTLEV_Shift (0)
#define SYSCON_BODCTRL_BODRSTLEV_1V46  (0x00)              /*!< BOD Reset @ 1.46V                */
#define SYSCON_BODCTRL_BODRSTLEV_2V06  (0x01)              /*!< BOD Reset @ 2.06V                */
#define SYSCON_BODCTRL_BODRSTLEV_2V35  (0x02)              /*!< BOD Reset @ 2.35V                */
#define SYSCON_BODCTRL_BODRSTLEV_2V71  (0x03)              /*!< BOD Reset @ 2.71V                */

#define SYSCON_BODCTRL_BODINTVAL_Mask  (0x0c)              /*!< BOD Interrupt Level Mask         */
#define SYSCON_BODCTRL_BODINTVAL_Shift (2)
#define SYSCON_BODCTRL_BODINTVAL_1V65  (0x00)              /*!< BOD Interrupt @ 1.65V            */
#define SYSCON_BODCTRL_BODINTVAL_2V22  (0x01 << 2)         /*!< BOD Interrupt @ 2.22V            */
#define SYSCON_BODCTRL_BODINTVAL_2V52  (0x02 << 2)         /*!< BOD Interrupt @ 2.52V            */
#define SYSCON_BODCTRL_BODINTVAL_2V80  (0x03 << 2)         /*!< BOD Interrupt @ 2.80V            */

#define SYSCON_BODCTRL_BODRSTENA       (1 << 4)            /*!< Enable BOD Reset                 */

/**
  * @}
  */

/** @defgroup SYSCON_SYSTCKCAL_Bit_Definitions (SYSTCKCAL) SysTick Calibration Register
  *
  * @{
  */

#define SYSCON_SYSTCKCAL_CAL_Mask      (0x03ffffffUL)      /*!< Systick Calibration Value Mask   */
#define SYSCON_SYSTCKCAL_CAL_Shift     (0)

/**
  * @}
  */

/** @defgroup SYSCON_STARTAPRP0_Bit_Definitions (STARTAPRP0) Start-Logic Edge Control Register 0
  *
  * @{
  */

#define SYSCON_STARTAPRP0_Mask         (0x0fff)            /*!< Useable Bits in STARTAPRP0       */
#define SYSCON_STARTAPRP0_Shift        (0)
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

/**
  * @}
  */

/** @defgroup SYSCON_STARTERP0_Bit_Definitions (STARTERP0) Start-Logic Enable Control Register 0
  *
  * @{
  */

#define SYSCON_STARTEPRP0_Mask         (0x0fff)            /*!< Useable Bits in STARTEPRP0       */
#define SYSCON_STARTEPRP0_Shift        (0)
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

/**
  * @}
  */

/** @defgroup SYSCON_STARTRSRP0CLR_Bit_Definitions (STARTRSRP0CLR) Start-Logic Clear Control Register 0
  *
  * @{
  */
#define SYSCON_STARTRSRP0CLR_Mask       (0x0fff)           /*!< Useable Bits in STARTRSRP0CLR    */
#define SYSCON_STARTRSRP0CLR_Shift      (0)
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

/**
  * @}
  */

/** @defgroup SYSCON_STARTSRP0_Bit_Definitions (STARTSRP0) Start-Logic Status Register 0
  *
  * @{
  */

#define SYSCON_STARTSRP0_Mask          (0x0fff)            /*!< Useable Bits in STARTSRP0        */
#define SYSCON_STARTSRP0_Shift         (0)
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

/**
  * @}
  */

/** @defgroup SYSCON_PDSLEEPCFG_Bit_Definitions (PDSLEEPCFG) Power-Down Sleep Configuration Register
  *
  * @{
  */

#define SYSCON_PDSLEEPCFG_Mask         (0x48)              /*!< Sleep Mode PowerDown Config Mask */
#define SYSCON_PDSLEEPCFG_Shift        (0)
#define SYSCON_PDSLEEPCFG_Required     (0x000018b7UL)      /*!< Required Bits for PDSLEEPCFG     */
#define SYSCON_PDSLEEPCFG_BOD_PD       (1 << 3)            /*!< Power Down BOD on Sleep          */
#define SYSCON_PDSLEEPCFG_WDTOSC_PD    (1 << 6)            /*!< Power Down WDT Osc. on Sleep     */

/* Other bits must be left alone */

/**
  * @}
  */

/** @defgroup SYSCON_PDAWAKECFG_Bit_Definitions (PDAWAKECFG) Power-Down Awake Configuration Register
  *
  * @{
  */

#define SYSCON_PDAWAKECFG_Mask         (0x0f)              /*!< Awake Mode PowerDown Config Mask */
#define SYSCON_PDAWAKECFG_Shift        (0)
#define SYSCON_PDAWAKECFG_Required     (0x00000ed00UL)     /*!< Required Bits for PDAWAKECFG     */
#define SYSCON_PDAWAKECFG_IRCOUT_PD    (1 << 0)            /*!< Power Down IRC Out in Wake Mode  */
#define SYSCON_PDAWAKECFG_IRC_PD       (1 << 1)            /*!< Power Down IRC Clk in Wake Mode  */
#define SYSCON_PDAWAKECFG_FLASH_PD     (1 << 2)            /*!< Power Down Flash in Wake Mode    */
#define SYSCON_PDAWAKECFG_BOD_PD       (1 << 3)            /*!< Power Down BOD in Wake Mode      */
#define SYSCON_PDAWAKECFG_ADC_PD       (1 << 4)            /*!< Power Down AtoD in Wake Mode     */
#define SYSCON_PDAWAKECFG_SYSOSC_PD    (1 << 5)            /*!< Power Down Sys Osc. in Wake Mode */
#define SYSCON_PDAWAKECFG_WDTOSC_PD    (1 << 6)            /*!< Power Down WDT Osc. in Wake Mode */
#define SYSCON_PDAWAKECFG_SYSPLL_PD    (1 << 7)            /*!< Power Down Sys PLL in Wake Mode  */

/**
  * @}
  */

/** @defgroup SYSCON_PDRUNCFG_Bit_Definitions (PDRUNCFG) Power-Down Running Configuration Register
  *
  * @{
  */

#define SYSCON_PDRUNCFG_Mask           (0x0f)              /*!< Run Mode Power-Down Config Mask  */
#define SYSCON_PDRUNCFG_Shift          (0)
#define SYSCON_PDRUNCFG_Required       (0x00000ed00UL)     /*!< Required Bits for PDRUNCFG       */
#define SYSCON_IRCOUT_PD               (1 << 0)            /*!< Power down IRC Out in Run Mode   */
#define SYSCON_IRC_PD                  (1 << 1)            /*!< Power down IRC clock in Run Mode */
#define SYSCON_FLASH_PD                (1 << 2)            /*!< Power down FLASH in Run Mode     */
#define SYSCON_BOD_PD                  (1 << 3)            /*!< Power down BOD in Run Mode       */
#define SYSCON_PDRUNCFG_ADC_PD         (1 << 4)            /*!< Power down AtoD in Run Mode      */
#define SYSCON_PDRUNCFG_SYSOSC_PD      (1 << 5)            /*!< Power down Sys Osc in Run Mode   */
#define SYSCON_PDRUNCFG_WDTOSC_PD      (1 << 6)            /*!< Power down WDT Osc in Run Mode   */
#define SYSCON_PDRUNCFG_PLL_PD         (1 << 7)            /*!< Power down Sys PLL in Run Mode   */

/**
  * @}
  */

/**
  * @}
  */


/** @addtogroup CT16B
  *
  * 16-bit Counter/Timer
  *
  * @{
  */

/** @defgroup CT16B_IR_Bit_Definitions (TxIR) 16-Bit Counter / Timer Interrupt Register
  *
  * @{
  */

#define CT16B_IR_Mask                  (0x1f)              /*!< Useable Bits in IR               */
#define CT16B_IR_Shift                 (0)

#define CT16B_IR_MR0                   (1 << 0)            /*!< Match Channel 0 Interrupt Flag   */
#define CT16B_IR_MR1                   (1 << 1)            /*!< Match Channel 1 Interrupt Flag   */
#define CT16B_IR_MR2                   (1 << 2)            /*!< Match Channel 2 Interrupt Flag   */
#define CT16B_IR_MR3                   (1 << 3)            /*!< Match Channel 3 Interrupt Flag   */
#define CT16B_IR_CR0                   (1 << 4)            /*!< Capture Channel 0 Interrupt Flag */

/**
  * @}
  */

/** @defgroup CT16B_TCR_Bit_Definitions (TxTCR) 16-Bit Counter / Timer Control Register
  *
  * @{
  */

#define CT16B_TCR_Mask                 (0x03)              /*!< Useable Bits in TCR              */
#define CT16B_TCR_Shift                (0)

#define CT16B_CE                       (1 << 0)            /*!< Counter Enable                   */
#define CT16B_CR                       (1 << 1)            /*!< Counter Reset                    */

/**
  * @}
  */

/** @defgroup CT16B_TC_Bit_Definitions (TxTC) 16-Bit Counter / Timer Timer Counter Register
  *
  * @{
  */

#define CT16B_TC_Mask                  (0xffff)            /*!< Mask for Timer Counter Count     */
#define CT16B_TC_Shift                 (0)

/**
  * @}
  */

/** @defgroup CT16B_PR_Bit_Definitions (TxPR) 16-Bit Counter / Timer Prescale Register
  *
  * @{
  */

#define CT16B_PR_Mask                  (0xffff)            /*!< Mask for Prescale Register Value */
#define CT16B_PR_Shift                 (0)

/**
  * @}
  */

/** @defgroup CT16B_PC_Bit_Definitions (TxPC) 16-Bit Counter / Timer Prescale Counter Register
  *
  * @{
  */

#define CT16B_PC_Mask                  (0xffff)            /*!< Mask for Prescale Counter Value  */
#define CT16B_PC_Shift                 (0)

/**
  * @}
  */

/** @defgroup CT16B_MCR_Bit_Definitions (TxMCR) 16-Bit Counter / Timer Match Control Register
  *
  * @{
  */

#define CT16B_MCR_Mask                 (0x0fff)            /*!< Useable Bits in MCRn             */
#define CT16B_MCR_Shift                (0)

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

/**
  * @}
  */

/** @defgroup CT16B_CCR_Bit_Definitions (TxCCR) 16-Bit Counter / Timer Capture Control Register
  *
  * @{
  */

#define CT16B_CCR_Mask                 (0x07)              /*!< Useable Bits in CCR             */
#define CT16B_CCR_Shift                (0)

#define CT16B_CAP0RE                   (1 << 0)            /*!< Capture on CAP0 rising edge     */
#define CT16B_CAP0FE                   (1 << 1)            /*!< Capture on CAP0 falling edge    */
#define CT16B_CAP0I                    (1 << 2)            /*!< Enable Interrupt on Capt Event  */

/**
  * @}
  */

/** @defgroup CT16B_CR0_Bit_Definitions (TxCR0) 16-Bit Counter / Timer Capture Register 0
  *
  * @{
  */

#define CT16B_CR0_Mask                 (0xffff)            /*!< Mask for Capture Value           */
#define CT16B_CR0_Shift                (0)

/**
  * @}
  */

/** @defgroup CT16B_EMR_Bit_Definitions (TxEMR) 16-Bit Counter / Timer External Match Register
  *
  * @{
  */

#define CT16B_EMR_Mask                 (0x0fff)            /*!< Useable Bits in EMR              */
#define CT16B_EMR_Shift                (0)

#define CT16B_EM0                      (1 << 0)            /*!< Status of MAT0 Pin               */
#define CT16B_EM1                      (1 << 1)            /*!< Status of MAT1 Pin               */
#define CT16B_EM2                      (1 << 2)            /*!< Status of MAT2 Pin               */
#define CT16B_EM3                      (1 << 3)            /*!< Status of MAT3 Pin               */

#define CT16B_EMC0_Mask                (0x03 << 4)         /*!< MAT0 Match Control Bits          */
#define CT16B_EMC0_Shift               (4)

#define CT16B_EMC1_Mask                (0x03 << 6)         /*!< MAT1 Match Control Bits          */
#define CT16B_EMC1_Shift               (6)

#define CT16B_EMC2_Mask                (0x03 << 8)         /*!< MAT2 Match Control Bits          */
#define CT16B_EMC2_Shift               (8)

#define CT16B_EMC3_Mask                (0x03 << 10)        /*!< MAT3 Match Control Bits          */
#define CT16B_EMC3_Shift               (10)

/**
  * @}
  */

/** @defgroup CT16B_CTCR_Bit_Definitions (TxCTCR) 16-Bit Counter / Timer Timer Count Control Registe
  *
  * @{
  */

#define CT16B_CTCR_Mask                (0x0f)              /*!< Useable Bits in CTCR             */
#define CT16B_CTCR_Shift               (0)

#define CT16B_MODE_Mask                (0x03)              /*!< Count/Timer Mode Mask            */
#define CT16B_MODE_Shift               (0)
#define CT16B_MODE_TIMER               (0x00)              /*!< Timer Mode                       */
#define CT16B_MODE_COUNT_RISE          (0x01)              /*!< Count Rising Edges               */
#define CT16B_MODE_COUNT_FALL          (0x02)              /*!< Count Falling Edges              */
#define CT16B_MODE_COUNT_BOTH          (0x03)              /*!< Count Both Edges                 */

#define CT16B_COUNT_INPUT_Mask         (0x03 << 2)         /*!< Counter Mode Input Channel       */
#define CT16B_COUNT_INPUT_Shift        (2)
#define CT16B_COUNT_INPUT_CAP0         (0x00)              /*!< Counter Input CAP0 Pin           */


/**
  * @}
  */

/** @defgroup CT16B_PWMC_Bit_Definitions (TxPWMC) 16-Bit Counter / Timer PWM Control Register
  *
  * @{
  */

#define CT16B_PWMC_Mask                (0x0f)              /*!< Useable Bits in PWMC             */
#define CT16B_PWMC_Shift               (0)

#define CT16B_PWM0E                    (1 << 0)            /*!< Enable PWM0                      */
#define CT16B_PWM1E                    (1 << 1)            /*!< Enable PWM1                      */
#define CT16B_PWM2E                    (1 << 2)            /*!< Enable PWM2                      */
#define CT16B_PWM3E                    (1 << 3)            /*!< Enable PWM3                      */

/**
  * @}
  */

/**
  * @}
  */


/** @addtogroup CT32B
  *
  * 32-bit Counter / Timer
  *
  * @{
  */

/** @defgroup CT32B_IR_Bit_Definitions (TxIR) 32-Bit Counter / Timer Interrupt Register
  *
  * @{
  */

#define CT32B_IR_Mask                  (0x1f)              /*!< Useable Bits in IR               */
#define CT32B_IR_Shift                 (0)

#define CT32B_IT_MR0                   (1 << 0)            /*!< Match Channel 0 Interrupt Flag   */
#define CT32B_IT_MR1                   (1 << 1)            /*!< Match Channel 1 Interrupt Flag   */
#define CT32B_IT_MR2                   (1 << 2)            /*!< Match Channel 2 Interrupt Flag   */
#define CT32B_IT_MR3                   (1 << 3)            /*!< Match Channel 3 Interrupt Flag   */
#define CT32B_IT_CR0                   (1 << 4)            /*!< Capture Channel 0 Interrupt Flag */

/**
  * @}
  */

/** @defgroup CT32B_TCR_Bit_Definitions (TxTCR) 32-Bit Counter / Timer Control Register
  *
  * @{
  */

#define CT32B_TCR_Mask                 (0x03)              /*!< Useable Bits in TCR              */
#define CT32B_TCR_Shift                (0)

#define CT32B_CE                       (1 << 0)            /*!< Counter Enable                   */
#define CT32B_CR                       (1 << 1)            /*!< Counter Reset                    */

/**
  * @}
  */

/** @defgroup CT32B_TC_Bit_Definitions (TxTC) 32-Bit Counter / Timer Timer Counter Register
  *
  * @{
  */

#define CT32B_TC_Mask                  (0xffffffffUL)      /*!< Mask for Timer Ctr Count         */
#define CT32B_TC_Shift                 (0)

/**
  * @}
  */

/** @defgroup CT32B_PR_Bit_Definitions (TxPR) 32-Bit Counter / Timer Prescale Register
  *
  * @{
  */

#define CT32B_PR_Mask                  (0xffffffffUL)      /*!< Mask for Prescale Reg Value      */
#define CT32B_PR_Shift                 (0)

/**
  * @}
  */

/** @defgroup CT32B_PC_Bit_Definitions (TxPC) 32-Bit Counter / Timer Prescale Counter Register
  *
  * @{
  */

#define CT32B_PC_Mask                  (0xffffffffUL)      /*!< Mask for Prescale Ctr Value      */
#define CT32B_PC_Shift                 (0)

/**
  * @}
  */

/** @defgroup CT32B_MCR_Bit_Definitions (TxMCR) 32-Bit Counter / Timer Match Control Register
  *
  * @{
  */

#define CT32B_MCR_Mask                 (0x0fff)            /*!< Useable Bits in MCRn             */
#define CT32B_MCR_Shift                (0)

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

/**
  * @}
  */

/** @defgroup CT32B_CCR_Bit_Definitions (TxCCR) 32-Bit Counter / Timer Capture Control Register
  *
  * @{
  */

#define CT32B_CCR_Mask                 (0x07)              /*!< Useable Bits in CCR              */
#define CT32B_CCR_Shift                (0)

#define CT32B_CAP0RE                   (1 << 0)            /*!< Capture on CAP0 rising edge      */
#define CT32B_CAP0FE                   (1 << 1)            /*!< Capture on CAP0 falling edge     */
#define CT32B_CAP0I                    (1 << 2)            /*!< Enable Interrupt on Capt. Event  */

/**
  * @}
  */

/** @defgroup CT32B_CR0_Bit_Definitions (TxCR0) 32-Bit Counter / Timer Capture Register 0
  *
  * @{
  */

#define CT32B_CR0_Mask                 (0xffffffffUL)      /*!< Mask for Capture Value           */
#define CT32B_CR0_Shift                (0)

/**
  * @}
  */

/** @defgroup CT32B_EMR_Bit_Definitions (TxEMR) 32-Bit Counter / Timer External Match Register
  *
  * @{
  */

#define CT32B_EMR_Mask                 (0x0fff)            /*!< Useable Bits in EMR              */
#define CT32B_EMR_Shift                (0)

#define CT32B_EM0                      (1 << 0)            /*!< Status of MAT0 Pin               */
#define CT32B_EM1                      (1 << 1)            /*!< Status of MAT1 Pin               */
#define CT32B_EM2                      (1 << 2)            /*!< Status of MAT2 Pin               */
#define CT32B_EM3                      (1 << 3)            /*!< Status of MAT3 Pin               */

#define CT32B_EMC0_Mask                (0x03 << 4)         /*!< MAT0 Match Control Bits          */
#define CT32B_EMC0_Shift               (4)
#define CT32B_EMC1_Mask                (0x03 << 6)         /*!< MAT1 Match Control Bits          */
#define CT32B_EMC1_Shift               (6)
#define CT32B_EMC2_Mask                (0x03 << 8)         /*!< MAT2 Match Control Bits          */
#define CT32B_EMC2_Shift               (8)
#define CT32B_EMC3_Mask                (0x03 << 10)        /*!< MAT3 Match Control Bits          */
#define CT32B_EMC3_Shift               (10)

/**
  * @}
  */

/** @defgroup CT32B_CTCR_Bit_Definitions (TxCTCR) 32-Bit Counter / Timer Timer Count Control Register
  *
  * @{
  */

#define CT32B_CTCR_Mask                (0x0f)              /*!< Useable Bits in CTCR             */
#define CT32B_CTCR_Shift               (0)

#define CT32B_MODE_Mask                (0x03)              /*!< CT32B Count/Timer Mode Mask      */
#define CT32B_MODE_Shift               (0)
#define CT32B_MODE_TIMER               (0x00)              /*!< Timer Mode                       */
#define CT32B_MODE_COUNT_RISE          (0x01)              /*!< Count Rising Edges               */
#define CT32B_MODE_COUNT_FALL          (0x02)              /*!< Count Falling Edges              */
#define CT32B_MODE_COUNT_BOTH          (0x03)              /*!< Count Both Edges                 */

#define CT32B_COUNT_INPUT_Mask         (0x03 << 2)         /*!< Counter Mode Input Channel       */
#define CT32B_COUNT_INPUT_Shift        (2)
#define CT32B_COUNT_INPUT_CAP0         (0x00)              /*!< Count CAP0 Transitions           */

/**
  * @}
  */

/** @defgroup CT32B_PWMC_Bit_Definitions (TxPWMC) 32-Bit Counter / Timer PWM Control Register
  *
  * @{
  */

#define CT32B_PWMC_Mask                (0x0f)              /*!< Useable Bits in PWMC             */
#define CT32B_PWMC_Shift               (0)

#define CT32B_PWM0E                    (1 << 0)            /*!< Enable PWM0                      */
#define CT32B_PWM1E                    (1 << 1)            /*!< Enable PWM1                      */
#define CT32B_PWM2E                    (1 << 2)            /*!< Enable PWM2                      */
#define CT32B_PWM3E                    (1 << 3)            /*!< Enable PWM3                      */

/**
  * @}
  */

/**
  * @}
  */


/** @addtogroup GPIO
  *
  * General Purpose IO Ports
  *
  * @{
  */

#error GPIO BITS NEED TO BE DEFINED


/**
  * @}
  */


/** @addtogroup USBD
  *
  * USB Device Interface
  *
  * @{
  */

/** @defgroup USBD_DEVCMDSTAT_Bit_Definitions (DEVCMDSTAT) USB Device Command / Status Register
  *
  * @{
  */

#define USBD_DEVCMDSTAT_Mask           (0x1703f3ffUL)      /*!< Useable Bits in DEVCMDSTAT       */
#define USBD_DEVCMDSTAT_Shift          (0)

#define USBD_DEV_ADDR_Mask             (0x7f)              /*!< USB Device Address Bits          */
#define USBD_DEV_ADDR_Shift            (0)

#define USBD_DEV_EN                    (1 << 7)            /*!< USB Device Enable                */
#define USBD_SETUP                     (1 << 8)            /*!< USB Device Setup Token Received  */
#define USBD_PLL_ON                    (1 << 9)            /*!< USB PLL Clock Always On          */
#define USBD_INTONNAK_AO               (1 << 12)           /*!< Int. on NAK for Int/Bulk Out     */
#define USBD_INTONNAK_AI               (1 << 13)           /*!< Int. on NAK for Int/Bulk In      */
#define USBD_INTONNAK_CO               (1 << 14)           /*!< Int. on NAK for Control Out      */
#define USBD_INTONNAK_CI               (1 << 15)           /*!< Int. on NAK for Control In       */
#define USBD_DCON                      (1 << 16)           /*!< USB Device Status Connect        */
#define USBD_DSUS                      (1 << 17)           /*!< USB Device Status Suspend        */
#define USBD_DCON_C                    (1 << 24)           /*!< USB Device Status Connect Change */
#define USBD_DSUS_C                    (1 << 25)           /*!< USB Device Status Suspend Change */
#define USBD_DRES_C                    (1 << 26)           /*!< USB Device Status Reset Change   */
#define USBD_VBUSDEBOUNCED             (1 << 28)           /*!< Vbus Detected                    */

/**
  * @}
  */

/** @defgroup USBD_INFO_Bit_Definitions (INFO) USB Info Register
  *
  * @{
  */

#define USBD_INFO_Mask                 (0x7fff)            /*!< Useable Bits in INFO Register    */
#define USBD_INFO_Shift                (0)

#define USBD_FRAME_NR_Mask             (0x07ff)            /*!< Frame Number Bits                */
#define USBD_FRAME_NR_Shift            (0)

#define USBD_ERR_CODE_Mask             (0x0f << 11)        /*!< Error Code Bits                  */
#define USBD_ERR_CODE_Shift            (11)

#define USBD_ERR_CODE_None             (0x00)              /*!< No Error                         */
#define USBD_ERR_CODE_PIDEncoding      (0x01)              /*!< PID Encoding Error               */
#define USBD_ERR_CODE_PIDUnknown       (0x02)              /*!< PID Unknown Error                */
#define USBD_ERR_CODE_PacketUnexpected (0x03)              /*!< Packet Unexpected Error          */
#define USBD_ERR_CODE_TokenCRC         (0x04)              /*!< Token CRC Error                  */
#define USBD_ERR_CODE_DataCRC          (0x05)              /*!< Data CRC Error                   */
#define USBD_ERR_CODE_Timeout          (0x06)              /*!< Timeout Error                    */
#define USBD_ERR_CODE_Babble           (0x07)              /*!< Babble                           */
#define USBD_ERR_CODE_TruncatedEOP     (0x08)              /*!< Truncated EOP                    */
#define USBD_ERR_CODE_NAK              (0x09)              /*!< NAK Sent/Received                */
#define USBD_ERR_CODE_SentStall        (0x0a)              /*!< Sent STALL                       */
#define USBD_ERR_CODE_Overrun          (0x0b)              /*!< Overrun                          */
#define USBD_ERR_CODE_SentEmpty        (0x0c)              /*!< Sent Empty Packet                */
#define USBD_ERR_CODE_BitStuff         (0x0d)              /*!< Bit Stuff Error                  */
#define USBD_ERR_CODE_Sync             (0x0e)              /*!< Sync Error                       */
#define USBD_ERR_CODE_WrongDataToggle  (0x0f)              /*!< Wrong Data Toggle                */

/**
  * @}
  */

/** @defgroup USBD_EPLISTSTART_Bit_Definitions (EPLISTSTART) USB EP Command / Status List Start
  *
  * NOTE: Address must start on 256 byte boundary
  *
  * @{
  */

#define USBD_EP_LIST_Mask              (0xffffff00UL)      /*!< Start Addr of USB EP Cmd/Status  */
#define USBD_EP_LIST_Shift             (0)

/**
  * @}
  */

/** @defgroup USBD_DATABUFSTART_Bit_Definitions (DATABUFSTART) USB Data Buffer Start AHB Address
  *
  * @{
  */

#define USBD_DA_BUF_Mask               (0xffc00000UL)      /*!< Start Addr of USB Buff Ptr Page  */
#define USBD_DA_BUF_Shift              (0)

/**
  * @}
  */

/** @defgroup USBD_EPSKIP_Bit_Definitions (EPSKIP) USB Endpoint Skip Register
  *
  * @{
  */

#define USBD_SKIP_Mask                 (0x3fffffffUL)      /*!< Endpoint Skip  */
#define USBD_SKIP_Shift                (0)

/**
  * @}
  */

/** @defgroup USBD_EPINUSE_Bit_Definitions (EPINUSE) USB Endpoint In Use Register
  *
  * Bits 2 - 9:
  *   0 -> Endpoint is using buffer 0
  *   1 -> Endpoint is using buffer 1
  *
  * @{
  */

#define USBD_BUF_Mask                  (0x03fc)            /*!< Buffer In Use Flags              */
#define USBD_BUF_Shift                 (0)

#define USBD_EP1OUT_BUF1               (1 << 2)            /*!< EP1 OUT Using Buffer 1           */
#define USBD_EP1IN_BUF1                (1 << 3)            /*!< EP1 IN  Using Buffer 1           */
#define USBD_EP2OUT_BUF1               (1 << 4)            /*!< EP2 OUT Using Buffer 1           */
#define USBD_EP2IN_BUF1                (1 << 5)            /*!< EP2 IN  Using Buffer 1           */
#define USBD_EP3OUT_BUF1               (1 << 6)            /*!< EP3 OUT Using Buffer 1           */
#define USBD_EP3IN_BUF1                (1 << 7)            /*!< EP3 IN  Using Buffer 1           */
#define USBD_EP4OUT_BUF1               (1 << 8)            /*!< EP4 OUT Using Buffer 1           */
#define USBD_EP4IN_BUF1                (1 << 9)            /*!< EP4 IN  Using Buffer 1           */

/**
  * @}
  */

/** @defgroup USBD_EPBUFCFG_Bit_Definitions (EPBUFCFG) USB Endpoint Buffer Configuration Register
  *
  * @{
  */

#define USBD_BUF_SB_Mask               (0x03fc)            /*!< Endpoint Double Buffering Flags  */
#define USBD_BUF_SB_Shift              (0)

#define USBD_EP1OUT_DBUF               (1 << 2)            /*!< Double Buffer EP1 OUT            */
#define USBD_EP1IN_DBUF                (1 << 3)            /*!< Double Buffer EP1 IN             */
#define USBD_EP2OUT_DBUF               (1 << 4)            /*!< Double Buffer EP2 OUT            */
#define USBD_EP2IN_DBUF                (1 << 5)            /*!< Double Buffer EP2 IN             */
#define USBD_EP3OUT_DBUF               (1 << 6)            /*!< Double Buffer EP3 OUT            */
#define USBD_EP3IN_DBUF                (1 << 7)            /*!< Double Buffer EP3 IN             */
#define USBD_EP4OUT_DBUF               (1 << 8)            /*!< Double Buffer EP4 OUT            */
#define USBD_EP4IN_DBUF                (1 << 9)            /*!< Double Buffer EP4 IN             */

/**
  * @}
  */

/** @defgroup USBD_INTSTAT_Bit_Definitions (INTSTAT) USB Interrupt Status Register
  *
  * @{
  */

#define USBD_INTSTAT_Mask              (0xc00001ffUL)      /*!< Interrupt Status Useable Bits    */
#define USBD_INTSTAT_Shift             (0)

#define USBD_EP0OUT                    (1 << 0)            /*!< EP0 OUT Interrupt                */
#define USBD_EP0IN                     (1 << 1)            /*!< EP0 IN Interrupt                 */
#define USBD_EP1OUT                    (1 << 2)            /*!< EP1 OUT Interrupt                */
#define USBD_EP1IN                     (1 << 3)            /*!< EP1 IN Interrupt                 */
#define USBD_EP2OUT                    (1 << 4)            /*!< EP2 OUT Interrupt                */
#define USBD_EP2IN                     (1 << 5)            /*!< EP2 IN Interrupt                 */
#define USBD_EP3OUT                    (1 << 6)            /*!< EP3 OUT Interrupt                */
#define USBD_EP3IN                     (1 << 7)            /*!< EP3 IN Interrupt                 */
#define USBD_EP4OUT                    (1 << 8)            /*!< EP4 OUT Interrupt                */
#define USBD_EP4IN                     (1 << 9)            /*!< EP4 IN Interrupt                 */
#define USBD_FRAME_INT                 (1 << 30)           /*!< Frame Interrupt (every msec)     */
#define USBD_DEV_INT                   (1 << 31)           /*!< Device Status Changed            */

/**
  * @}
  */

/** @defgroup USBD_INTEN_Bit_Definitions (INTEN) USB Interrupt Enable Register
  *
  * @{
  */

#define USBD_INTEN_Mask                (0xc00001ffUL)      /*!< Interrupt Enable Useable Bits    */
#define USBD_INTEN_Shift               (0)

#define USBD_EP0OUT_INT_EN             (1 << 0)            /*!< EP0 OUT Interrupt Enable         */
#define USBD_EP0IN_INT_EN              (1 << 1)            /*!< EP0 IN Interrupt Enable          */
#define USBD_EP1OUT_INT_EN             (1 << 2)            /*!< EP1 OUT Interrupt Enable         */
#define USBD_EP1IN_INT_EN              (1 << 3)            /*!< EP1 IN Interrupt Enable          */
#define USBD_EP2OUT_INT_EN             (1 << 4)            /*!< EP2 OUT Interrupt Enable         */
#define USBD_EP2IN_INT_EN              (1 << 5)            /*!< EP2 IN Interrupt Enable          */
#define USBD_EP3OUT_INT_EN             (1 << 6)            /*!< EP3 OUT Interrupt Enable         */
#define USBD_EP3IN_INT_EN              (1 << 7)            /*!< EP3 IN Interrupt Enable          */
#define USBD_EP4OUT_INT_EN             (1 << 8)            /*!< EP4 OUT Interrupt Enable         */
#define USBD_EP4IN_INT_EN              (1 << 9)            /*!< EP4 IN Interrupt Enable          */
#define USBD_FRAME_INT_EN              (1 << 30)           /*!< Frame Interrupt Enable           */
#define USBD_DEV_INT_EN                (1 << 31)           /*!< Device Status Changed Int Enable */

/**
  * @}
  */

/** @defgroup USBD_INTSETSTAT_Bit_Definitions (INTSETSTAT) USB Set Interrupt Status Register
  *
  * @{
  */

#define USBD_INTSETSTAT_Mask           (0xc00001ffUL)      /*!< Set Interrupt Stat. Useable Bits */
#define USBD_INTSETSTAT_Shift          (0)

#define USBD_EP0OUT_SET_INT            (1 << 0)            /*!< Set EP0 OUT Interrupt            */
#define USBD_EP0IN_SET_INT             (1 << 1)            /*!< Set EP0 IN Interrupt             */
#define USBD_EP1OUT_SET_INT            (1 << 2)            /*!< Set EP1 OUT Interrupt            */
#define USBD_EP1IN_SET_INT             (1 << 3)            /*!< Set EP1 IN Interrupt             */
#define USBD_EP2OUT_SET_INT            (1 << 4)            /*!< Set EP2 OUT Interrupt            */
#define USBD_EP2IN_SET_INT             (1 << 5)            /*!< Set EP2 IN Interrupt             */
#define USBD_EP3OUT_SET_INT            (1 << 6)            /*!< Set EP3 OUT Interrupt            */
#define USBD_EP3IN_SET_INT             (1 << 7)            /*!< Set EP3 IN Interrupt             */
#define USBD_EP4OUT_SET_INT            (1 << 8)            /*!< Set EP4 OUT Interrupt            */
#define USBD_EP4IN_SET_INT             (1 << 9)            /*!< Set EP4 IN Interrupt             */
#define USBD_FRAME_SET_INT             (1 << 30)           /*!< Set Frame Interrupt              */
#define USBD_DEV_SET_INT               (1 << 31)           /*!< Set Device Status Changed Int.   */

/**
  * @}
  */

/** @defgroup USBD_INTROUTING_Bit_Definitions (INTROUTING) USB Interrupt Routing Register
  *
  * Sets interrupt as IRQ/FIQ
  * @{
  */

#define USBD_INTROUTING_Mask           (0xc00001ffUL)      /*!< Interrrupt Routing Useable Bits  */
#define USBD_INTROUTING_Shift          (0)

#define USBD_EP0OUT_ROUTE_FIQ          (1 << 0)            /*!< Route EP0 OUT Interrupt to FIQ   */
#define USBD_EP0IN_ROUTE_FIQ           (1 << 1)            /*!< Route EP0 IN Interrupt to FIQ    */
#define USBD_EP1OUT_ROUTE_FIQ          (1 << 2)            /*!< Route EP1 OUT Interrupt to FIQ   */
#define USBD_EP1IN_ROUTE_FIQ           (1 << 3)            /*!< Route EP1 IN Interrupt to FIQ    */
#define USBD_EP2OUT_ROUTE_FIQ          (1 << 4)            /*!< Route EP2 OUT Interrupt to FIQ   */
#define USBD_EP2IN_ROUTE_FIQ           (1 << 5)            /*!< Route EP2 IN Interrupt to FIQ    */
#define USBD_EP3OUT_ROUTE_FIQ          (1 << 6)            /*!< Route EP3 OUT Interrupt to FIQ   */
#define USBD_EP3IN_ROUTE_FIQ           (1 << 7)            /*!< Route EP3 IN Interrupt to FIQ    */
#define USBD_EP4OUT_ROUTE_FIQ          (1 << 8)            /*!< Route EP4 OUT Interrupt to FIQ   */
#define USBD_EP4IN_ROUTE_FIQ           (1 << 9)            /*!< Route EP4 IN Interrupt to FIQ    */
#define USBD_FRAME_ROUTE_FIQ           (1 << 30)           /*!< Route Frame Interrupt to FIQ     */
#define USBD_DEV_ROUTE_FIQ             (1 << 31)           /*!< Route Dev Stat changed Int to FIQ*/

/**
  * @}
  */

/** @defgroup USBD_EPTOGGLE_Bit_Definitions (EPTOGGLE) USB Endpoint Toggle Register
  *
  * Sets interrupt as IRQ/FIQ
  * @{
  */

#define USBD_EPTOGGLE_Mask             (0x01ffUL)          /*!< Endpoint Data Toggle Useable Bits*/
#define USBD_EPTOGGLE_Shift            (0)

#define USBD_EP0OUT_TOGGLE             (1 << 0)            /*!< EP0 OUT Data Toggle              */
#define USBD_EP0IN_TOGGLE              (1 << 1)            /*!< EP0 IN Data Toggle               */
#define USBD_EP1OUT_TOGGLE             (1 << 2)            /*!< EP1 OUT Data Toggle              */
#define USBD_EP1IN_TOGGLE              (1 << 3)            /*!< EP1 IN Data Toggle               */
#define USBD_EP2OUT_TOGGLE             (1 << 4)            /*!< EP2 OUT Data Toggle              */
#define USBD_EP2IN_TOGGLE              (1 << 5)            /*!< EP2 IN Data Toggle               */
#define USBD_EP3OUT_TOGGLE             (1 << 6)            /*!< EP3 OUT Data Toggle              */
#define USBD_EP3IN_TOGGLE              (1 << 7)            /*!< EP3 IN Data Toggle               */
#define USBD_EP4OUT_TOGGLE             (1 << 8)            /*!< EP4 OUT Data Toggle              */
#define USBD_EP4IN_TOGGLE              (1 << 9)            /*!< EP4 IN Data Toggle               */
/**
  * @}
  */

/**
  * @}
  */

/* Peripheral Memory Locations ------------------------------------------------------------------*/

/** @defgroup LPC11xx_Peripheral_Memory_Locations
  * @{
  *
  * Base memory addresses for peripherals
  */

#define I2C_BASE        (0x40000000UL)                     /*!< I2C Controller                   */
#define WWDT_BASE       (0x40004000UL)                     /*!< Watchdog Timer                   */
#define USART_BASE      (0x40008000UL)                     /*!< USART                            */
#define ADC_BASE        (0x4001c000UL)                     /*!< Analog to Digital Converter      */
#define PMU_BASE        (0x40038000UL)                     /*!< Power Control                    */
#define FLASH_BASE      (0x4003c000UL)                     /*!< FLASH memory controller          */
#define SSP0_BASE       (0x40040000UL)                     /*!< Synch Serial Peripheral 0        */
#define IOCON_BASE      (0x40044000UL)                     /*!< IO Configuration Controller      */
#define SYSCON_BASE     (0x40048000UL)                     /*!< System Control                   */
#define SSP1_BASE       (0x40058000UL)                     /*!< Synch Serial Peripheral 1        */
#define USBD_BASE       (0x40080000UL)                     /*!< USB Device Peripheral            */
#define CT16B0_BASE     (0x4000c000UL)                     /*!< 16-bit Timer/Counter 0           */
#define CT16B1_BASE     (0x40010000UL)                     /*!< 16-bit Timer/Counter 1           */
#define CT32B0_BASE     (0x40014000UL)                     /*!< 32-bit Timer/Counter 0           */
#define CT32B1_BASE     (0x40018000UL)                     /*!< 32-bit Timer/Counter 1           */
#define GPIO0_BASE      (0x50000000UL)                     /*!< GPIO Port 0                      */
#define GPIO1_BASE      (0x50010000UL)                     /*!< GPIO Port 1                      */
#define GPIO2_BASE      (0x50020000UL)                     /*!< GPIO Port 2                      */
#define GPIO3_BASE      (0x50030000UL)                     /*!< GPIO Port 3                      */

/* Defined in core_cm0.h
#define SYSTICK_BASE    (0xe000e010UL)
#define NVIC_BASE       (0xe000e100UL)
#define SCB_BASE        (0xe000ed00UL)
*/

/**
  * @}
  */


/* Exported Variables ---------------------------------------------------------------------------*/

/** @defgroup LPC11Uxx_Peripheral_Instances
  * @{
  */

#define I2C             ((I2C_Type *)I2C_BASE)             /*!< I2C Peripheral                   */
#define WWDT            ((WWDT_Type *)WWDT_BASE)           /*!< Windowed Watchdog Timer          */
#define USART           ((USART_Type *)USART_BASE)         /*!< USART Peripheral                 */
#define ADC             ((ADC_Type *)ADC_BASE)             /*!< A to D Converter                 */
#define PMU             ((PMU_Type *)PMU_BASE)             /*!< Power Management Unit            */
#define FLASH           ((FLASH_Type *)FLASH_BASE)         /*!< Flash Configuration Block        */
#define SSP0            ((SSP_Type *)SSP0_BASE)            /*!< Synch Serial Peripheral 0        */
#define IOCON           ((IOCON_Type *)IOCON_BASE)         /*!< IO Configuration Block           */
#define SYSCON          ((SYSCON_Type *)SYSCON_BASE)       /*!< System Configuration Block       */
#define SSP1            ((SSP_Type *)SSP1_BASE)            /*!< Synch Serial Peripheral 1        */
#define USBD            ((USBD_Type *)USBD_BASE)           /*!< USB Device Peripheral            */
#define CT16B0          ((CT16B_Type *)CT16B0_BASE)        /*!< 16-bit Counter / Timer 0         */
#define CT16B1          ((CT16B_Type *)CT16B1_BASE)        /*!< 16-bit Counter / Timer 1         */
#define CT32B0          ((CT32B_Type *)CT32B0_BASE)        /*!< 32-bit Counter / Timer 0         */
#define CT32B1          ((CT32B_Type *)CT32B1_BASE)        /*!< 32-bit Counter / Timer 1         */
#define GPIO0           ((GPIO_Type *)GPIO0_BASE)          /*!< General Purpose IO Port 0        */
#define GPIO1           ((GPIO_Type *)GPIO1_BASE)          /*!< General Purpose IO Port 1        */
#define GPIO2           ((GPIO_Type *)GPIO2_BASE)          /*!< General Purpose IO Port 2        */
#define GPIO3           ((GPIO_Type *)GPIO3_BASE)          /*!< General Purpose IO Port 3        */

/**
  * @}
  */

#ifdef __cplusplus
};
#endif

#endif /* #ifndef LPC11UXX_H_ */
