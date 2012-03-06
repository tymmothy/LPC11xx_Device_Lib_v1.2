/** ***************************************************************************
 * @file     isr_vector.h
 * @brief    ISR/Exception Handler declarations for NXP LPC11xx MCUs
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. January 2012
 ******************************************************************************
 * @section Overview
 * This file declares a type for working with LPC11xx Interrupt Service
 * Routine vectors.  This is primarily useful for run-time configurable
 * ISR vectors in RAM, but can find other uses as well.
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

#ifndef NXP_LPC_ISR_VECTOR_H_
#define NXP_LPC_ISR_VECTOR_H_

#ifdef __cplusplus
extern "C" {
#endif


/**
  * @ingroup LPC11xx_System LPC11xx Microcontroller System Interface
  * @{
  */

/* Types --------------------------------------------------------------------*/

/** @defgroup LPC11xx_ISR_Vectors LPC11xx ISR Vectors
  * @{
  */

/** @brief Structure holding ISR Vectors for system handlers/IRQ handlers.
  *
  * Useful for declaring RAM-based ISR vectors, etc.
  */
typedef struct isr_vector {
    void *Stack_Addr;                               /*!< Stack Address @ Reset        */
    void (* Reset_Handler_Vect)(void);              /*!< System Reset Handler         */
    void (* NMI_Handler_Vect)(void);                /*!< Unused on LPC11xx            */
    void (* HardFault_Handler_Vect)(void);          /*!< Hard Fault Handler           */
    void (* Reserved4)(void);                       /*!<   (unused)                   */
    void (* Reserved5)(void);                       /*!<   (unused)                   */
    void (* Reserved6)(void);                       /*!<   (unused)                   */
    void *Checksum;                                 /*!< Vector Checksum Value        */
    void (* Reserved8)(void);                       /*!<   (unused)                   */
    void (* Reserved9)(void);                       /*!<   (unused)                   */
    void (* Reserved10)(void);                      /*!<   (unused)                   */
    void (* SVC_Handler_Vect)(void);                /*!< Supervisor Call Handler      */
    void (* Reserved12)(void);                      /*!<   (unused)                   */
    void (* Reserved13)(void);                      /*!<   (unused)                   */
    void (* PendSV_Handler_Vect)(void);             /*!< Pend Supervisor Call Handler */
    void (* SysTick_Handler_Vect)(void);            /*!< System Tick Timer Handler    */
    void (* WAKEUP0_IRQHandler_Vect)(void);         /*!< GPIO0.0  Wakeup IRQ Handler  */
    void (* WAKEUP1_IRQHandler_Vect)(void);         /*!< GPIO0.1  Wakeup IRQ Handler  */
    void (* WAKEUP2_IRQHandler_Vect)(void);         /*!< GPIO0.2  Wakeup IRQ Handler  */
    void (* WAKEUP3_IRQHandler_Vect)(void);         /*!< GPIO0.3  Wakeup IRQ Handler  */
    void (* WAKEUP4_IRQHandler_Vect)(void);         /*!< GPIO0.4  Wakeup IRQ Handler  */
    void (* WAKEUP5_IRQHandler_Vect)(void);         /*!< GPIO0.5  Wakeup IRQ Handler  */
    void (* WAKEUP6_IRQHandler_Vect)(void);         /*!< GPIO0.6  Wakeup IRQ Handler  */
    void (* WAKEUP7_IRQHandler_Vect)(void);         /*!< GPIO0.7  Wakeup IRQ Handler  */
    void (* WAKEUP8_IRQHandler_Vect)(void);         /*!< GPIO0.8  Wakeup IRQ Handler  */
    void (* WAKEUP9_IRQHandler_Vect)(void);         /*!< GPIO0.9  Wakeup IRQ Handler  */
    void (* WAKEUP10_IRQHandler_Vect)(void);        /*!< GPIO0.10 Wakeup IRQ Handler  */
    void (* WAKEUP11_IRQHandler_Vect)(void);        /*!< GPIO0.11 Wakeup IRQ Handler  */
    void (* WAKEUP12_IRQHandler_Vect)(void);        /*!< GPIO1.0  Wakeup IRQ Handler  */
#if defined(LPC11CXX)  /* CAN parts only */
    void (* CAN_IRQHandler_Vect)(void);             /*!< CAN IRQ Handler              */
#else
    void (* Reserved29)(void);                      /*!<   (unused)                   */
#endif /* ! #if defined(LPC11CXX) */
    void (* SSP1_IRQHandler_Vect)(void);            /*!< SSP1 IRQ Handler             */
    void (* I2C0_IRQHandler_Vect)(void);            /*!< I2C IRQ Handler              */
    void (* CT16B0_IRQHandler_Vect)(void);          /*!< 16-Bit Timer 0 IRQ Handler   */
    void (* CT16B1_IRQHandler_Vect)(void);          /*!< 16-Bit Timer 1 IRQ Handler   */
    void (* CT32B0_IRQHandler_Vect)(void);          /*!< 32-Bit Timer 0 IRQ Handler   */
    void (* CT32B1_IRQHandler_Vect)(void);          /*!< 32-Bit Timer 1 IRQ Handler   */
    void (* SSP0_IRQHandler_Vect)(void);            /*!< SSP0 IRQ Handler             */
    void (* UART0_IRQHandler_Vect)(void);           /*!< UART IRQ Handler             */
    void (* Reserved38)(void);                      /*!<   (unused)                   */
    void (* Reserved39)(void);                      /*!<   (unused)                   */
    void (* ADC0_IRQHandler_Vect)(void);            /*!< ADC IRQ Handler              */
    void (* WDT_IRQHandler_Vect)(void);             /*!< Watchdot Timer IRQ Handler   */
    void (* BOD_IRQHandler_Vect)(void);             /*!< Brownout Det. IRQ Handler    */
    void (* Reserved43)(void);                      /*!<   (unused)                   */
    void (* GPIO3_IRQHandler_Vect)(void);           /*!< GPIO3 IRQ Handler            */
    void (* GPIO2_IRQHandler_Vect)(void);           /*!< GPIO2 IRQ Handler            */
    void (* GPIO1_IRQHandler_Vect)(void);           /*!< GPIO1 IRQ Handler            */
    void (* GPIO0_IRQHandler_Vect)(void);           /*!< GPIO0 IRQ Handler            */
} ISRVector_Type;

/** @} */

/* System Exception / Interrupt Handlers ------------------------------------*/

/**
  * @defgroup LPC11xx_ExceptionHandlers LPC11xx Exception Handlers
  * @{
  */

/*! @brief System Reset Handler */
extern void Reset_Handler(void);

/* Not Implemented on LPC11xx: Nonmaskable Interrupt Handler
extern void NMI_Handler(void);
*/

/*! @brief Undefined Exception Handler */
extern void UndefinedException_Handler(void);

/*! @brief Hard Fault Handler */
extern void HardFault_Handler(void);

/*! @brief Supervisor Call Handler */
extern void SVC_Handler(void);

/*! @brief Pend Supervisor Call Handler */
extern void PendSV_Handler(void);

/*! @brief System Tick Timer Handler */
extern void SysTick_Handler(void);

/**
  * @}
  */

/** @defgroup LPC11xx_IRQHandlers LPC11xx Interrupt Handlers
  * @{
  */

/*! @brief Wakeup input 0 ("Start Logic" on GPIO0.0) IRQ handler */
extern void WAKEUP0_IRQHandler(void);

/*! @brief Wakeup Input 1 ("Start Logic" on GPIO 0.1) IRQ Handler */
extern void WAKEUP1_IRQHandler(void);

/*! @brief Wakeup Input 2 ("Start Logic" on GPIO0.2) IRQ Handler */
extern void WAKEUP2_IRQHandler(void);

/*! @brief Wakeup Input 3 ("Start Logic" on GPIO0.3) IRQ Handler */
extern void WAKEUP3_IRQHandler(void);

/*! @brief Wakeup Input 4 ("Start Logic" on GPIO0.4) IRQ Handler */
extern void WAKEUP4_IRQHandler(void);

/*! @brief Wakeup Input 5 ("Start Logic" on GPIO0.5) IRQ Handler */
extern void WAKEUP5_IRQHandler(void);

/*! @brief Wakeup Input 6 ("Start Logic" on GPIO0.6) IRQ Handler */
extern void WAKEUP6_IRQHandler(void);

/*! @brief Wakeup Input 7 ("Start Logic" on GPIO0.7) IRQ Handler */
extern void WAKEUP7_IRQHandler(void);

/*! @brief Wakeup Input 8 ("Start Logic" on GPIO0.8) IRQ Handler */
extern void WAKEUP8_IRQHandler(void);

/*! @brief Wakeup Input 9 ("Start Logic" on GPIO 0.9) IRQ Handler */
extern void WAKEUP9_IRQHandler(void);

/*! @brief Wakeup Input 10 ("Start Logic" on GPIO0.10) IRQ Handler */
extern void WAKEUP10_IRQHandler(void);

/*! @brief Wakeup Input 11 ("Start Logic" on GPIO0.11) IRQ Handler */
extern void WAKEUP11_IRQHandler(void);

/*! @brief Wakeup Input 12 ("Start Logic" on GPIO1.0) IRQ Handler */
extern void WAKEUP12_IRQHandler(void);

#if defined(LPC11CXX)  /* CAN parts only */
extern void CAN_IRQHandler(void);
#endif

/*! @brief Synchronous Serial Peripheral 1 IRQ Handler */
extern void SSP1_IRQHandler(void);

/*! @brief I2C Controller 0 IRQ Handler */
extern void I2C0_IRQHandler(void);

/*! @brief 16-Bit Timer/Counter 0 IRQ Handler */
extern void CT16B0_IRQHandler(void);

/*! @brief 16-Bit Timer/Counter 1 IRQ Handler */
extern void CT16B1_IRQHandler(void);

/*! @brief 32-Bit Timer/Counter 0 IRQ Handler */
extern void CT32B0_IRQHandler(void);

/*! @brief 32-Bit Timer/Counter 1 IRQ Handler */
extern void CT32B1_IRQHandler(void);

/*! @brief Synchronous Serial Peripheral 0 IRQ Handler */
extern void SSP0_IRQHandler(void);

/*! @brief UART 0 IRQ Handler */
extern void UART0_IRQHandler(void);

/*! @brief Analog-to-Digital Converter 0 IRQ Handler */
extern void ADC0_IRQHandler(void);

/*! @brief Watchdog Timer IRQ Handler */
extern void WDT_IRQHandler(void);

/*! @brief Brownout Detector IRQ Handler */
extern void BOD_IRQHandler(void);

/*! @brief GPIO Port 3 IRQ Handler */
extern void GPIO3_IRQHandler(void);

/*! @brief GPIO Port 2 IRQ Handler */
extern void GPIO2_IRQHandler(void);

/*! @brief GPIO Port 1 IRQ Handler */
extern void GPIO1_IRQHandler(void);

/*! @brief GPIO Port 0 IRQ Handler */
extern void GPIO0_IRQHandler(void);

/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
};
#endif

#endif /* #ifndef NXP_LPC_ISR_VECTOR_H_ */

