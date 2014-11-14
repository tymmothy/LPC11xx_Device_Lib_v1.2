/**************************************************************************//**
 * @file     uart.h
 * @brief    UART Interface Header for NXP LPC Microcontrollers
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. January 2012
 ******************************************************************************
 * @section Overview
 * This file gives a basic interface to NXP LPC microcontroller
 * UARTs.  It abstracts such things as setting baud rate, word sizes,
 * parity, etc., as well as simplifying use of UART interrupts
 * and reading/writing data to/from the UART.
 *
 * @note
 * This file does not handle the following necessary steps for UART use:
 * - The UART's (AHB or APB/VPB) clock line must be configured & enabled.
 * - IO Pins must be configured for UART use
 * - For interrupt use, an interrupt handler must be declared and
 *   the UART's interrupt line must be enabled in the microcontroller's
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
 ******************************************************************************
 *
 * NOTE: Pin Modes need to be configured in the PINCONFIG/IOCON block
 *       separately.
 *
 *****************************************************************************/

#ifndef NXP_LPC_UART_H_
#define NXP_LPC_UART_H_


#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include "lpc11xx.h"
#include "lpclib_assert.h"


/**
  * @defgroup UART_AbstractionLayer UART (Asynchronous Serial) Abstraction Layer
  * @ingroup  LPC_Peripheral_AbstractionLayer
  * @{
  */

/* Types & Type-Related Definitions -----------------------------------------*/

/**
  * @defgroup UART_Types UART Interface Types and Type-Related Definitions
  * @{
  */

/** @defgroup UART_InterruptConfigBits UART Interrupt Configuration Bits
  * @{
  */

/*
 * These 16550-based UARTs have an annoying disconnect between enabling
 *  interrupts and identification of pending interrupts...
 */

#define UART_Interrupt_Mask            (0x0307)            /*!< Mask of valid int config bits    */

#define UART_Interrupt_RxData          (0x0001)            /*!< Rx'd Data Interrupt Enable       */
#define UART_Interrupt_TxData          (0x0002)            /*!< Tx'd Data Interrupt Enable       */
#define UART_Interrupt_RxLineStatus    (0x0004)            /*!< Receiver Line Status Int Enable  */
#define UART_Interrupt_AutobaudEnd     (0x0100)            /*!< End of Auto Baud Int Enable      */
#define UART_Interrupt_AutobaudTimeout (0x0200)            /*!< Auto Baud Timeout Int Enable     */

/** @} */

/** @defgroup UART_InterruptIDs UART Pending Interrupt ID Values
  * @{
  */

/*! @brief UART Interrupt ID values (read from UART to determine cause of interrupt) */
typedef enum {
    UART_InterruptID_None              = 0x01,             /*!< No Pending Interrupts            */
    UART_InterruptID_ModemStatus       = 0x00,             /*!< Modem Status Interrupt Pending   */
    UART_InterruptID_TxEmpty           = 0x02,             /*!< Transmitter Empty Int Pending    */
    UART_InterruptID_RxDataAvailable   = 0x04,             /*!< Rx'd Data Available Int Pending  */
    UART_InterruptID_RxLineStatus      = 0x06,             /*!< Receiver Line Status Pending Int */
    UART_InterruptID_CharacterTimeOut  = 0x0c,             /*!< Rx'd Char Timeout Int Pending    */
} UART_InterruptID_Type;

/** @} */

/** @defgroup UART_Line_Status_Flags UART Line Status Flags
  * @{
  */

#define UART_LineStatus_Mask           (0x00ff)            /*!< Mask of all line status bits     */

#define UART_LineStatus_RxData         (1 << 0)            /*!< Received Data Ready Flag         */
#define UART_LineStatus_RxOverrun      (1 << 1)            /*!< Rx'd Data Overrun Occurred Flag  */
#define UART_LineStatus_ParityError    (1 << 2)            /*!< Rx Next char had Parity Error    */
#define UART_LineStatus_FramingError   (1 << 3)            /*!< Rx Next char had Framing Error   */
#define UART_LineStatus_Break          (1 << 4)            /*!< Rx next char had BREAK condition */
#define UART_LineStatus_TxReady        (1 << 5)            /*!< Tx FIFO Ready for Data Flag      */
#define UART_LineStatus_TxEmpty        (1 << 6)            /*!< Tx FIFO Empty Flag               */
#define UART_LineStatus_RxFIFO         (1 << 8)            /*!< Rx FIFO Contains Error Flag      */

/** @} */


/** @defgroup UART_Modem_Status_Bits UART Modem Status Bits
  * @{
  */

#define UART_ModemStatus_Mask          (0x00ff)            /*!< Mask of all modem status bits    */

#define UART_ModemStatus_CTSChange     (1 << 0)            /*!< CTS changed flag                 */
#define UART_ModemStatus_DSRChange     (1 << 1)            /*!< DSR changed flag                 */
#define UART_ModemStatus_RIEdge        (1 << 2)            /*!< RI edge detected flag            */
#define UART_ModemStatus_DCDChange     (1 << 3)            /*!< DCD changed flag                 */
#define UART_ModemStatus_CTSVal        (1 << 4)            /*!< CTS changed flag                 */
#define UART_ModemStatus_DSRVal        (1 << 5)            /*!< DSR changed flag                 */
#define UART_ModemStatus_RIVal         (1 << 6)            /*!< Current RI pin state             */
#define UART_ModemStatus_DCDVal        (1 << 7)            /*!< Current DCD pin state            */

/** @} */

/** @defgroup UART_Word_Length UART Word Length Configurations
  * @{
  */

/*! @brief UART Word Length Configuration Settings */
typedef enum {
    UART_WordLength_5b                 = 0x00,             /*!< Use 5-bit words                  */
    UART_WordLength_6b                 = 0x01,             /*!< Use 6-bit words                  */
    UART_WordLength_7b                 = 0x02,             /*!< Use 7-bit words                  */
    UART_WordLength_8b                 = 0x03              /*!< Use 8-bit words                  */
} UART_WordLength_Type;

/*! @brief Macro to test whether parameter is a valid Word Length value */
#define UART_IS_WORDLENGTH(WORDLENGTH) (((WORDLENGTH) >= UART_WordLength_5b) \
                                    && ((WORDLENGTH) <= UART_WordLength_8b))

/** @} */

/** @defgroup UART_Stop_Bits UART Stop Bit Configurations
  * @{
  */

/*! UART Stop Bit Configuration Settings */
typedef enum {
    UART_StopBits_1                    = 0x00,             /*!< Use 1 stop bit                   */
    UART_StopBits_2                    = 0x04              /*!< Use 2 stop bits                  */
} UART_StopBits_Type;

/*! @brief Macro to test whether parameter is a valid Stop Bits configuration value */
#define UART_IS_STOPBITS(STOPBITS) (((STOPBITS) == UART_StopBits_1) \
                                 || ((STOPBITS) == UART_StopBits_2))

/** @} */

/** @defgroup UART_Parity UART Parity Configurations
  * @{
  */

/*! @brief UART Parity Configuration Settings */
typedef enum {
    UART_Parity_None                   = 0x00,             /*!< No parity bit                    */
    UART_Parity_Odd                    = 0x08,             /*!< Odd parity                       */
    UART_Parity_Even                   = 0x18,             /*!< Even parity                      */
    UART_Parity_One                    = 0x28,             /*!< Force parity bit to 1            */
    UART_Parity_Zero                   = 0x38              /*!< Force parity bit to 0            */
} UART_Parity_Type;

/*! @brief Macro to test whether parameter is a valid Parity configuration value */
#define UART_IS_PARITY(PARITY) (((PARITY) == UART_Parity_None) || ((PARITY) == UART_Parity_Odd) \
                             || ((PARITY) == UART_Parity_Even) || ((PARITY) == UART_Parity_One) \
                             || ((PARITY) == UART_Parity_Zero))

/** @} */

/** @defgroup UART_RxFIFO_Trigger UART Rx FIFO Interrupt Trigger Counts
  * @{
  */

/*! @brief UART Number of Bytes in FIFO to Trigger Interrupt */
typedef enum {
    UART_RxFifoTrigger_1               = 0x00,             /*!< 1  byte  in Rx FIFO triggers IRQ */
    UART_RxFifoTrigger_4               = 0x40,             /*!< 4  bytes in Rx FIFO triggers IRQ */
    UART_RxFifoTrigger_8               = 0x80,             /*!< 8  bytes in Rx FIFO triggers IRQ */
    UART_RxFifoTrigger_14              = 0xc0,             /*!< 14 bytes in Rx FIFO triggers IRQ */
} UART_RxFifoTrigger_Type;

/*! @brief Macro to test whether parameter is a valid Rx FIFO Interrupt Trigger value */
#define UART_IS_RXFIFOTRIGGER(TRIGGER) (((TRIGGER) == UART_RxFifoTrigger_1) \
                                     || ((TRIGGER) == UART_RxFifoTrigger_4) \
                                     || ((TRIGGER) == UART_RxFifoTrigger_8) \
                                     || ((TRIGGER) == UART_RxFifoTrigger_14))

/** @} */

/** @defgroup UART_AutobaudModes UART Autobaud Modes
  * @{
  */

/*! @brief UART autobaud modes */
typedef enum {
    UART_AutobaudMode_0 = 0x00,                            /*!< Use start bit + LSB for autobaud */
    UART_AutobaudMode_1 = 0x02,                            /*!< Use start bit only for autobaud  */
} UART_AutobaudMode_Type;

/*! @brief Macro to test whether parameter is a valid Autobaud mode */
#define UART_IS_AUTOBAUDMODE(MODE) (((MODE) == UART_AutobaudMode_0) \
                                 || ((MODE) == UART_AutobaudMode_1))

/** @} */

/** @defgroup UART_AutobaudITs UART Autobaud Interrupts
  * @{
  */

#define UART_AutobaudIT_Mask     (0x0300)           /*!< Mask of all autobaud int bits    */

#define UART_AutobaudIT_Complete (1 << 8)           /*!< Autobaud completed interrupt     */
#define UART_AutobaudIT_Timeout  (1 << 9)           /*!< Autobaud timed out interrupt     */

/*! @brief Type for passing UART autobaud interrupt pending bits */
typedef uint32_t UART_AutobaudIT_Type;

/** @} */

/** @defgroup UART_RS485DirControlPins UART RS485 Direction Control Pins
  * @{
  */

/*! @brief UART RS485 direction control pins */
typedef enum {
    UART_RS485DirControlPin_RTS = 0x00,                    /*!< Use RTS for direction control    */
    UART_RS485DirControlPin_DTR = 0x08,                    /*!< Use DTR for direction control    */
} UART_RS485DirControlPin_Type;

/*! @brief Macro to test whether parameter is a valid RS485 direction control pin */
#define UART_IS_RS485DIRCONTROLPIN(CTRL) (((CTRL) == UART_RS485DirControlPin_RTS) \
                                       || ((CTRL) == UART_RS485DirControlPin_DTR))

/** @} */

/** @defgroup UART_RS485DirControlPolarity UART RS485 Direction Control Polarities
  * @{
  */

/*! @brief UART RS485 direction control polarities */
typedef enum {
    UART_RS485DirControlPolarity_Low  = 0x00,               /*!< DTR/RTS goes low when data sent  */
    UART_RS485DirControlPolarity_High = 0x20,               /*!< DTR/RTS goes high when data sent */
} UART_RS485DirControlPolarity_Type;

/*! @brief Macro to test whether parameter is a valid RS485 direction control pin */
#define UART_IS_RS485DIRCONTROLPOLARITY(POL) (((POL) == UART_RS485DirControlPolarity_Low) \
                                           || ((POL) == UART_RS485DirControlPolarity_High))

/** @} */

/**
  * @}
  */


/* UART Inline Functions ----------------------------------------------------*/

/** @defgroup UART_InlineFunctions UART Interface Inline Functions
  * @{
  */

/** @brief Get the last byte that was received by an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  The received byte.
  */
__INLINE static uint8_t UART_Recv(UART_Type *uart)
{
    return uart->RBR;
}

/** @brief Send a byte via an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  b            The byte to send
  */
__INLINE static void UART_Send(UART_Type *uart, uint8_t b)
{
    uart->THR = b;
}

/** @brief Enable specified interrupts on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  it_mask      A bitmask of the interrupts to enable
  */
__INLINE static void UART_EnableInterrupts(UART_Type *uart, uint32_t it_mask)
{
    lpclib_assert((it_mask & ~UART_Interrupt_Mask) == 0);

    uart->IER |= it_mask;
}

/** @brief Disable specified interrupts on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  it_mask      A bitmask of the interrupts to disable
  */
__INLINE static void UART_DisableInterrupts(UART_Type *uart, uint32_t it_mask)
{
    lpclib_assert((it_mask & ~UART_Interrupt_Mask) == 0);

    uart->IER &= ~it_mask;
}

/** @brief Get a bitmask of interrupts that are enabled on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  A bitmask of enabled UART interrupts.
  */
__INLINE static uint32_t UART_GetEnabledInterruptMask(UART_Type *uart)
{
    return uart->IER;
}

/** @brief Get the ID of the highest pending interrupt on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  The ID of the highest pending interrupt on the UART.
  */
__INLINE static UART_InterruptID_Type UART_GetPendingInterruptID(UART_Type *uart)
{
    return uart->IIR & UART_INTID_Mask;
}

/** @brief Set the word length for bytes sent/received via an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  length       The new UART word length setting
  */
__INLINE static void UART_SetWordLength(UART_Type *uart, UART_WordLength_Type length)
{
    lpclib_assert(UART_IS_WORDLENGTH(length));

    uart->LCR = (uart->LCR & ~UART_WORDLEN_Mask) | (length);
}

/** @brief Get the current word length setting for bytes sent/received via an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  The current UART word length setting.
  */
__INLINE static UART_WordLength_Type UART_GetWordLength(UART_Type *uart)
{
    return uart->LCR & UART_WORDLEN_Mask;
}

/** @brief Set the stop bits for bytes sent/received via an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  stop_bits    The new stop bits setting
  */
__INLINE static void UART_SetStopBits(UART_Type *uart, UART_StopBits_Type stop_bits)
{
    lpclib_assert(UART_IS_STOPBITS(stop_bits));

    uart->LCR = (uart->LCR & ~UART_2STOPBITS) | stop_bits;
}

/** @brief Get the current stop bits setting for bytes sent/received via an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  The current stop bits setting.
  */
__INLINE static UART_StopBits_Type UART_GetStopBits(UART_Type *uart)
{
    return (uart->LCR & UART_2STOPBITS);
}

/** @brief Set the parity type for bytes sent/received via an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  parity_type  The new parity setting
  */
__INLINE static void UART_SetParity(UART_Type *uart, UART_Parity_Type parity_type)
{
    lpclib_assert(UART_IS_PARITY(parity_type));

    uart->LCR = (uart->LCR & ~UART_PARITY_Mask) | parity_type;
}

/** @brief Get the current parity setting for bytes sent/received via an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  The current parity setting.
  */
__INLINE static UART_Parity_Type UART_GetParity(UART_Type *uart)
{
    return (uart->LCR & UART_PARITY_Mask);
}

/** @brief Get the bitmask of UART line status bits.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  A bitmask of line status bits (bits 0-8 used).
  */
__INLINE static uint32_t UART_GetLineStatus(UART_Type *uart)
{
    return uart->LSR;
}

/** @brief Get the bitmask of UART modem status bits.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  A bitmask of modem status bits (bits 0-7 used).
  */
__INLINE static uint32_t UART_GetModemStatus(UART_Type *uart)
{
    return uart->MSR;
}

/** @brief Enable an UART's transmitter.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_EnableTx(UART_Type *uart)
{
    uart->TER = UART_TXEN;
}

/** @brief Disable an UART's transmitter.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_DisableTx(UART_Type *uart)
{
    uart->TER = 0;
}

/** @brief Test whether an UART's transmitter is enabled.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  1 if the transmitter is enabled, 0 otherwise.
  */
__INLINE static unsigned int UART_TxIsEnabled(UART_Type *uart)
{
    return (uart->TER & UART_TXEN) ? 1:0;
}

/** @brief Begin signaling BREAK on an UART's TX line.
  * @param[in]  uart         A pointer to the UART instance
  *
  * @note
  * BREAK is signaled by holding the TX line in a '1' state beyond
  * the length of a frame (generally at least 15 bit periods).  This
  * function begins the signaling; timing must be done in software.
  */
__INLINE static void UART_BeginBreak(UART_Type *uart)
{
    uart->LCR |= UART_BREAK;
}

/** @brief End signaling BREAK on an UART's TX line.
  * @param[in]  uart         A pointer to the UART instance
  *
  * @sa UART_BeginBreak
  */
__INLINE static void UART_EndBreak(UART_Type *uart)
{
    uart->LCR &= ~UART_BREAK;
}

/** @brief Test whether an UART is currently signaling BREAK.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  1 if the UART is signaling BREAK, 0 otherwise.
  *
  * @sa UART_BeginBreak
  */
__INLINE static unsigned int UART_IsInBreak(UART_Type *uart)
{
    return (uart->LCR & UART_BREAK) ? 1:0;
}

/** @brief Enable loopback mode on an UART.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_EnableLoopback(UART_Type *uart)
{
    uart->MCR |= UART_LOOPBACK;
}

/** @brief Disable loopback mode on an UART.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_DisableLoopback(UART_Type *uart)
{
    uart->MCR &= ~UART_LOOPBACK;
}

/** @brief Test whether loopback mode is enabled on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  1 if loopback mode is enabled, 0 otherwise.
  */
__INLINE static unsigned int UART_LoopbackIsEnabled(UART_Type *uart)
{
    return (uart->MCR & UART_LOOPBACK) ? 1:0;
}

/** @brief Set the state of an UART's DTR line.
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  state        The new DTR state (non-zero value to set, zero to clear)
  */
__INLINE static void UART_SetDTR(UART_Type *uart, unsigned int state)
{
    if (state) {
        uart->MCR |= UART_DTR;
    } else {
        uart->MCR &= UART_DTR;
    }
}

/** @brief Get the current state of an UART's DTR Line.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  1 if in DTR is set, 0 otherwise.
  */
__INLINE static unsigned int UART_DTRIsSet(UART_Type *uart)
{
    return (uart->MCR & UART_DTR) ? 1:0;
}

/** @brief Set the state of an UART's RTS line.
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  state        The new RTS state (non-zero value to set, zero to clear)
  */
__INLINE static void UART_SetRTS(UART_Type *uart, unsigned int state)
{
    if (state) {
        uart->MCR |= UART_RTS;
    } else {
        uart->MCR &= UART_RTS;
    }
}

/** @brief Get the current state of an UART's RTS line.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  1 if in RTS is set, 0 otherwise.
  */
__INLINE static uint8_t UART_GetRTS(UART_Type *uart)
{
    return (uart->MCR & UART_RTS) ? 1:0;
}

/** @brief Enable automatic RTS flow control on an UART.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_AutoRTSEnable(UART_Type *uart)
{
    uart->MCR |= UART_RTSENA;
}

/** @brief Disable automatic RTS flow control on an UART.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_AutoRTSDisable(UART_Type *uart)
{
    uart->MCR &= ~UART_RTSENA;
}

/** @brief Test whether automatic RTS flow control is enabled on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  1 if automatic RTS flow control is enabled, 0 otherwise.
  */
__INLINE static uint8_t UART_AutoRTSIsEnabled(UART_Type *uart)
{
    return (uart->MCR & UART_RTSENA) ? 1:0;
}

/** @brief Enable automatic CTS flow control on an UART.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_AutoCTSEnable(UART_Type *uart)
{
    uart->MCR |= UART_CTSENA;
}

/** @brief Disable automatic CTS flow control on an UART.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_AutoCTSDisable(UART_Type *uart)
{
    uart->MCR &= ~UART_CTSENA;
}

/** @brief Test whether automatic CTS flow control is enabled on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  1 if automatic CTS flow control is enabled, 0 otherwise.
  */
__INLINE static uint8_t UART_AutoCTSIsEnabled(UART_Type *uart)
{
    return (uart->MCR & UART_CTSENA) ? 1:0;
}

/** @brief Set the number of bytes in an UART's Rx FIFO that will trigger an interrupt.
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  trigger      The new FIFO trigger setting
  */
__INLINE static void UART_SetRxFifoTrigger(UART_Type *uart, UART_RxFifoTrigger_Type trigger)
{
    lpclib_assert(UART_IS_RXFIFOTRIGGER(trigger));

    uart->FCR = trigger | UART_FIFOEN;
}

/** @brief Flush an UART's Receive FIFO.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_FlushRxFifo(UART_Type *uart)
{
    uart->FCR = (uart->IIR & UART_FIFO_Mask)
                 | UART_FIFOEN | UART_FIFO_RX_RESET;
}

/** @brief Flush an UART's Transmit FIFO.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_FlushTxFifo(UART_Type *uart)
{
    uart->FCR = (uart->IIR & UART_FIFO_Mask)
                 | UART_FIFOEN | UART_FIFO_TX_RESET;
}

/** @brief Flush an UART's (Rx & Tx) FIFOs.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_FlushFifos(UART_Type *uart)
{
    uart->FCR = (uart->IIR & UART_FIFO_Mask)
                 | UART_FIFOEN | UART_FIFO_TX_RESET | UART_FIFO_RX_RESET;
}

/** @brief Enable an UART's FIFOs.
  * @param[in]  uart         A pointer to the UART instance
  *
  * @note
  * FIFOs should be enabled during normal use (according to the datasheet,
  * enabling the FIFOs is necessary for proper operation).
  */
__INLINE static void UART_EnableFifos(UART_Type *uart)
{
    uart->FCR = UART_FIFOEN;
}

/** @brief Disable an UART's FIFOs.
  * @param[in]  uart         A pointer to the UART instance
  *
  * @note
  * FIFOs should not be disabled during normal use.
  *
  * Disabling FIFOS will:
  * - Clear the RX Trigger, setting it back to 1 character.
  * - Flush the FIFOs.
  */
__INLINE static void UART_DisableFifos(UART_Type *uart)
{
    uart->FCR = 0;
}

/** @brief Set an UART's baud rate divisor.
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  divisor      The number of prescaled UART clocks per bit (16 bits)
  *
  * @note
  * A divisor of 0 is treated as a divisor of 1.
  */
__INLINE static void UART_SetDivisor(UART_Type *uart, uint16_t divisor)
{
    uart->LCR |= UART_DLAB;
    uart->DLL = divisor & 0xff;
    uart->DLM = divisor >> 8;
    uart->LCR &= ~UART_DLAB;
}

/** @brief Get an UART's current baud rate divisor.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  The configured # of prescaled UART clocks per bit.
  */
__INLINE static uint16_t UART_GetDivisor(UART_Type *uart)
{
    uint16_t Divisor;


    uart->LCR |= UART_DLAB;
    Divisor = ((uart->DLM & 0xff) << 8) | (uart->DLL & 0xff);
    uart->LCR &= ~UART_DLAB;

    return Divisor;
}

/** @brief Start autobaud on an UART.
  * @param[in]  uart         A pointer to the UART instance
  *
  * @note
  * Autobaud running flag will automatically clear when autobaud has
  * completed.
  */
__INLINE static void UART_BeginAutobaud(UART_Type *uart)
{
    uart->ACR |= UART_AUTOBAUD;
}

/** @brief End autobaud on an UART.
  * @param[in]  uart         A pointer to the UART instance
  *
  * @sa UART_BeginAutobaud
  */
__INLINE static void UART_EndAutobaud(UART_Type *uart)
{
    uart->ACR &= ~UART_AUTOBAUD;
}

/** @brief Test whether an UART's autobaud is active.
  * @param[in]  uart         A pointer to the UART instance
  *
  * @sa UART_BeginAutobaud
  */
__INLINE static int UART_AutobaudIsRunning(UART_Type *uart)
{
    return (uart->ACR & UART_AUTOBAUD) ? 1:0;
}

/** @brief Set an UART's autobaud mode.
  * @param[in]  uart         A pointer to the UART instance
  * @param mode         The new autobaud mode.
  *
  * @sa UART_BeginAutobaud
  */
__INLINE static void UART_SetAutobaudMode(UART_Type *uart, UART_AutobaudMode_Type mode)
{
    lpclib_assert(UART_IS_AUTOBAUDMODE(mode));

    uart->ACR = (uart->ACR & ~UART_MODE1) | mode;
}

/** @brief Get an UART's current autobaud mode.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  The current autobaud mode of the UART
  *
  * @sa UART_BeginAutobaud
  */
__INLINE static UART_AutobaudMode_Type UART_GetAutobaudMode(UART_Type *uart)
{
    return uart->ACR & UART_MODE1;
}

/** @brief Enable autobaud auto-restart on an UART.
  * @param[in]  uart         A pointer to the UART instance
  *
  * @sa UART_BeginAutobaud
  *
  * @note
  * Autobaud autorestart causes the autobaud function to re-start
  * in case of timeout.
  */
__INLINE static void UART_EnableAutobaudAutoRestart(UART_Type *uart)
{
    uart->ACR |= UART_AUTORESTART;
}

/** @brief Disable autobaud auto-restart on an UART.
  * @param[in]  uart         A pointer to the UART instance
  *
  * @sa UART_BeginAutobaud
  * @sa UART_EnableAutobaudAutoRestart
  */
__INLINE static void UART_DisableAutobaudAutoRestart(UART_Type *uart)
{
    uart->ACR &= ~UART_AUTORESTART;
}

/** @brief Get an UART's current autobaud mode.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  The current autobaud mode of the UART
  *
  * @sa UART_BeginAutobaud
  * @sa UART_EnableAutobaudAutoRestart
  */
__INLINE static unsigned int UART_AutobaudAutoRestartIsEnabled(UART_Type *uart)
{
    return (uart->ACR & UART_AUTORESTART) ? 1:0;
}

/** @brief Get a bitmask of pending autobaud interrupts on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  A bitmask of pending autobaud interrupts.
  */
__INLINE static UART_AutobaudIT_Type UART_GetPendingAutobaudITs(UART_Type *uart)
{
    return uart->IIR & (UART_IT_ABEO | UART_IT_ABTO);
}

/** @brief Clear pending autobaud interrupts on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  abit_mask    A bitmask of autobaud interrupts to clear
  */
__INLINE static void UART_ClearPendingAutobaudITs(UART_Type *uart, UART_AutobaudIT_Type abit_mask)
{
    lpclib_assert((abit_mask & ~UART_AutobaudIT_Mask) == 0);

    uart->ACR |= abit_mask;
}

/** @brief Set an UART's fractional divider
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  div          The divider portion of the fract. divider (0 <= Div; disabled if 0)
  * @param[in]  mult         The multiplier portion of the fract. divider (0 < Div <= Mult <= 15)
  *
  * @note
  * - The fractional divider is a UART clock prescaler that is applied before the baud divider.
  * - If the divider is 0, the fractional divider is disabled.
  * - The multiplier must be greater than or equal to the divider.
  * - The multiplier must be >= 1 even if the fractional divider is unused.
  * - Final baud rate is PCLK / (16 * (256 * DLM + DLL) * (1 + (Div / Mult))).
  * - Div/Mult range is 1/15 to 14/15.
  * - DLM/DLL total must be >= 3 when fractional divider is used.
  */
__INLINE static void UART_SetFractionalDivider(UART_Type *uart, unsigned int div, unsigned int mult)
{
    lpclib_assert(div <= 14);
    lpclib_assert((mult > 0) && (mult <= 15));
    lpclib_assert(mult > div);

    uart->FDR = (mult << UART_MULVAL_Shift) | div;
}

/** @brief Get the divider portion of an UART's fractional divider value.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  The divider portion of the UART's fractional divider value.
  *
  * @sa UART_SetFractionalDivider
  */
__INLINE static unsigned int UART_GetFractionalDividerDiv(UART_Type *uart)
{
    return uart->FDR & UART_DIVADDVAL_Mask;
}

/** @brief Get the multiplier portion of an UART's fractional divider value.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  The multiplier portion of the UART's fractional divider value.
  *
  * @sa UART_SetFractionalDivider
  */
__INLINE static unsigned int UART_GetFractionalDividerMult(UART_Type *uart)
{
    return (uart->FDR & UART_MULVAL_Mask) >> UART_MULVAL_Shift;
}

/** @brief Disable an UART's fractional divider.
  * @param[in]  uart         A pointer to the UART instance
  *
  * @note
  * Sets the fractional divider's multiplier to 1, the divider to 0.
  */
__INLINE static void UART_DisableFractionalDivider(UART_Type *uart)
{
    uart->FDR = (1 << UART_MULVAL_Shift);
}

/** @brief Enable RS485 normal multidrop mode on an UART.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_EnableRS485NormalMultidropMode(UART_Type *uart)
{
    uart->RS485CTRL |= UART_NMMEN;
}

/** @brief Disable RS485 normal multidrop mode on an UART.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_DisableRS485NormalMultidropMode(UART_Type *uart)
{
    uart->RS485CTRL &= ~UART_NMMEN;
}

/** @brief Test whether RS485 normal multidrop mode is enabled on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  1 if normal multidrop mode is enabled, 0 otherwise.
  */
__INLINE static unsigned int UART_RS485NormalMultidropModeIsEnabled(UART_Type *uart)
{
    return (uart->RS485CTRL & UART_NMMEN) ? 1:0;
}

/** @brief Enable the receiver in RS485 mode on an UART.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_EnableRS485Rx(UART_Type *uart)
{
    /* The receiver is enabled when the bit is cleared. */
    uart->RS485CTRL &= ~UART_RXDIS;
}

/** @brief Disable the receiver in RS485 mode on an UART.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_DisableRS485Rx(UART_Type *uart)
{
    /* Receiver is disabled when the bit is set. */
    uart->RS485CTRL |= UART_RXDIS;
}

/** @brief Test whether the receiver is enabled on an UART in RS485 mode.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  1 if the receiver is enabled, 0 otherwise.
  *
  * @note
  * The receiver is enabled when the bit is cleared.
  */
__INLINE static unsigned int UART_RS485RxIsEnabled(UART_Type *uart)
{
    return (uart->RS485CTRL & UART_RXDIS) ? 0:1;
}

/** @brief Enable auto address detect in RS485 mode on an UART.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_EnableRS485AutoAddressDetect(UART_Type *uart)
{
    uart->RS485CTRL |= UART_AADEN;
}

/** @brief Disable auto address detect in RS485 mode on an UART.
  * @param[in]  uart         A pointer to the UART instance
  */
__INLINE static void UART_DisableRS485AutoAddressDetect(UART_Type *uart)
{
    uart->RS485CTRL &= ~UART_AADEN;
}

/** @brief Test whether auto address detect is enabled on an UART in RS485 mode.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  1 if auto auto address detect is enabled, 0 otherwise.
  */
__INLINE static unsigned int UART_RS485AutoAddressDetectIsEnabled(UART_Type *uart)
{
    return (uart->RS485CTRL & UART_AADEN) ? 1:0;
}

/** @brief Set the pin used for direction control in RS485 mode on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  pin          The pin to use for RS485 direction control.
  */
__INLINE static void UART_SetRS485DirControlPin(UART_Type *uart,
                                                UART_RS485DirControlPin_Type pin)
{
    lpclib_assert(UART_IS_RS485DIRCONTROLPIN(pin));

    uart->RS485CTRL = (uart->RS485CTRL & ~UART_DIRSEL) | pin;
}

/** @brief Get the pin used for direction control in RS485 mode on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  The pin used for RS485 direction control.
  */
__INLINE static UART_RS485DirControlPin_Type UART_GetRS485DirControlPin(UART_Type *uart)
{
    return (uart->RS485CTRL & UART_DIRSEL);
}

/** @brief Enable RS485 automatic direction control on an UART.
  * @param[in]  uart         A pointer to the UART instance
  *
  * @sa UART_SetRS485DirControlPin
  */
__INLINE static void UART_EnableRS485AutoDirControl(UART_Type *uart)
{
    uart->RS485CTRL |= UART_DCTRL;
}

/** @brief Disable RS485 automatic direction control on an UART.
  * @param[in]  uart         A pointer to the UART instance
  *
  * @sa UART_SetRS485DirControlPin
  */
__INLINE static void UART_DisableRS485AutoDirControl(UART_Type *uart)
{
    uart->RS485CTRL &= ~UART_DCTRL;
}

/** @brief Test whether RS485 automatic direction control is enabled on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  1 if automatic direction control is enabled, 0 otherwise.
  *
  * @sa UART_SetRS485DirControlPin
  */
__INLINE static unsigned int UART_RS485AutoDirControlIsEnabled(UART_Type *uart)
{
    return (uart->RS485CTRL & UART_DCTRL) ? 1:0;
}

/** @brief Set the polarity of the RS485 direction control pin on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  polarity     The new polarity setting
  */
__INLINE static void UART_SetRS485DirControlPolarity(UART_Type *uart,
                                                     UART_RS485DirControlPolarity_Type polarity)
{
    lpclib_assert(UART_IS_RS485DIRCONTROLPOLARITY(polarity));

    uart->RS485CTRL = (uart->RS485CTRL & ~UART_OINV) | polarity;
}

/** @brief Get the polarity of the RS485 direction control pin on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  The current polarity of the RS485 direction control pin.
  */
__INLINE static UART_RS485DirControlPolarity_Type UART_GetRS485DirControlPolarity(UART_Type *uart)
{
    return uart->RS485CTRL & UART_OINV;
}

/** @brief Set the RS485 address on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  addr         The new address (0-255)
  */
__INLINE static void UART_SetRS485Address(UART_Type *uart, unsigned int addr)
{
    lpclib_assert(addr <= 255);

    uart->ADRMATCH = addr;
}

/** @brief Get the polarity of the RS485 direction control pin on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @return                  The current RS485 address.
  */
__INLINE static unsigned int UART_GetRS485Address(UART_Type *uart)
{
    return uart->ADRMATCH;
}

/** @brief Set the RS485 direction pin delay on an UART.
  * @param[in]  uart         A pointer to the UART instance
  * @param[in]  delay        The delay, in baud clocks (0-255)
  *
  * @note
  * This controls the delay between the last stop bit leaving the Tx FIFO
  * and the de-assertion of the direction control pin.
  */
__INLINE static void UART_SetRS485DirDelay(UART_Type *uart, unsigned int delay)
{
    lpclib_assert(delay <= 255);

    uart->RS485DLY = delay;
}

/** @brief Get the current RS485 direction pin delay on an UART.
  * @param[in]  uart         A pointer to the UART instance
  *
  * @sa UART_SetRS485DirDelay
  */
__INLINE static unsigned int UART_GetRS485DirDelay(UART_Type *uart)
{
    return uart->RS485DLY;
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


#endif /* #ifndef NXP_LPC_UART_H_ */
