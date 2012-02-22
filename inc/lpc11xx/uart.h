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
 * - The UART's (AHB/APB/VPB) input clock line must be enabled (& on some
 *   chips, e.g. LPC11xx, the SSP input clock divider configured).
 * - IO Pins must be configured for UART use
 * - For interrupt use, an interrupt handler must be declared and
 *   the UART's interrupt line must be enabled in the microcontroller's
 *   interrupt controller.
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

/** @defgroup UART_ITConfigBits UART Interrupt Configuration Bits
  * @{
  */

/*
 * These 16550-based UARTs have an annoying disconnect between enabling
 *  interrupts and identification of pending interrupts...
 */

#define UART_IT_Mask                   (0x0307)            /*!< Mask of valid int config bits    */

#define UART_IT_RxData                 (0x0001)            /*!< Rx'd Data Interrupt Enable       */
#define UART_IT_TxData                 (0x0002)            /*!< Tx'd Data Interrupt Enable       */
#define UART_IT_RxLineStatus           (0x0004)            /*!< Receiver Line Status Int Enable  */
#define UART_IT_AutobaudEnd            (0x0100)            /*!< End of Auto Baud Int Enable      */
#define UART_IT_AutobaudTimeout        (0x0200)            /*!< Auto Baud Timeout Int Enable     */

/*! @brief Type for passing UART interupt enable bits */
typedef uint32_t UART_IT_Type;

/*! @brief Macro to test whether parameter is a valid UART interrupt mask value */
#define UART_IS_IT(IT) (((IT) & ~UART_IT_Mask) == 0)

/** @} */

/** @defgroup UART_ITIDs UART Pending Interrupt ID Values
  * @{
  */

/*! @brief UART Interrupt ID values (read from UART to determine cause of interrupt) */
typedef enum {
    UART_ITID_None                     = 0x01,             /*!< No Pending Interrupts            */
    UART_ITID_ModemStatus              = 0x00,             /*!< Modem Status Interrupt Pending   */
    UART_ITID_TxEmpty                  = 0x02,             /*!< Transmitter Empty Int Pending    */
    UART_ITID_RxDataAvailable          = 0x04,             /*!< Rx'd Data Available Int Pending  */
    UART_ITID_RxLineStatus             = 0x06,             /*!< Receiver Line Status Pending Int */
    UART_ITID_CharacterTimeOut         = 0x0c,             /*!< Rx'd Char Timeout Int Pending    */
} UART_ITID_Type;

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
#define UART_IS_WORDSIZE(WORDSIZE) (((WORDSIZE) >= UART_WordLength_5b) \
                                    && ((WORDSIZE) <= UART_WordLength_8b))

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

#define UART_AutobaudIT_Mask           (0x0300)            /*!< Mask of all autobaud int bits    */

#define UART_AutobaudIT_Complete       (1 << 8)            /*!< Autobaud completed interrupt     */
#define UART_AutobaudIT_Timeout        (1 << 9)            /*!< Autobaud timed out interrupt     */

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

/** @brief Receive a byte via an UART.
  * @param Uart         A pointer to the UART instance
  * @return None.
  *
  * @note
  * This returns the last byte received by the UART; it does not cause
  * a transfer to occur.
  */
__INLINE static uint8_t UART_Recv(UART_Type *Uart)
{
    return Uart->RBR;
}

/** @brief Send a byte via an UART.
  * @param Uart         A pointer to the UART instance
  * @param b            The byte to send
  * @return             None.
  */
__INLINE static void UART_Send(UART_Type *Uart, uint8_t b)
{
    Uart->THR = b;
}

/** @brief Enable interrupts on an UART.
  * @param Uart         A pointer to the UART instance
  * @param IT           A bitmask of the interrupts to enable
  * @return             None.
  */
__INLINE static void UART_EnableIT(UART_Type *Uart, UART_IT_Type IT)
{
    lpclib_assert(UART_IS_IT(IT));

    Uart->IER |= IT;
}

/** @brief Disable interrupts on an UART.
  * @param   Uart       A pointer to the UART instance
  * @param   IT         A bitmask of the interrupts to disable
  * @return             None.
  */
__INLINE static void UART_DisableIT(UART_Type *Uart, UART_IT_Type IT)
{
    lpclib_assert(UART_IS_IT(IT));

    Uart->IER &= ~IT;
}

/** @brief Get a mask of interrupts that are enabled on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             A bitmask of enabled UART interrupts.
  */
__INLINE static UART_IT_Type UART_GetEnabledIT(UART_Type *Uart)
{
    return Uart->IER;
}

/** @brief Get the ID of the highest pending interrupt on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             The ID of the highest pending interrupt on the UART.
  */
__INLINE static UART_ITID_Type UART_GetPendingITID(UART_Type *Uart)
{
    return (Uart->IIR & UART_INTID_Mask);
}

/** @brief Set the word length for bytes sent/received via an UART.
  * @param Uart         A pointer to the UART instance
  * @param WordLength   A token representing the new UART word length
  * @return             None.
  */
__INLINE static void UART_SetWordLength(UART_Type *Uart, UART_WordLength_Type WordLength)
{
    lpclib_assert(UART_IS_WORDLENGTH(WordLength));

    Uart->LCR = (Uart->LCR & ~UART_WORDLEN_Mask) | (WordLength - 5);
}

/** @brief Get the current word length for bytes sent/received via an UART.
  * @param Uart         A pointer to the UART instance
  * @return             A token respresenting the current UART word length.
  */
__INLINE static UART_WordLength_Type UART_GetWordLength(UART_Type *Uart)
{
    return (Uart->LCR & UART_WORDLEN_Mask);
}

/** @brief Set the # of stop bits for bytes sent/received via an UART.
  * @param Uart         A pointer to the UART instance
  * @param StopBits     A token indicating the number of stop bits
  * @return             None.
  */
__INLINE static void UART_SetStopBits(UART_Type *Uart, UART_StopBits_Type StopBits)
{
    lpclib_assert(UART_IS_STOPBITS(StopBits));

    Uart->LCR = (Uart->LCR & ~UART_2STOPBITS) | StopBits;
}

/** @brief Get the current # of stop bits for bytes sent/received via an UART.
  * @param Uart         A pointer to the UART instance
  * @return             A token representing the current # of stop bits.
  */
__INLINE static UART_StopBits_Type UART_GetStopBits(UART_Type *Uart)
{
    return (Uart->LCR & UART_2STOPBITS);
}

/** @brief Set the parity type for bytes sent/received via an UART.
  * @param Uart         A pointer to the UART instance
  * @param Parity       A token indicating type of parity to use
  * @return             None.
  */
__INLINE static void UART_SetParity(UART_Type *Uart, UART_Parity_Type Parity)
{
    lpclib_assert(UART_IS_PARITY(Parity));

    Uart->LCR = (Uart->LCR & ~UART_PARITY_Mask) | Parity;
}

/** @brief Get the current parity type for bytes sent/received via an UART.
  * @param Uart         A pointer to the UART instance
  * @return             A token indicating the current parity type.
  */
__INLINE static UART_Parity_Type UART_GetParity(UART_Type *Uart)
{
    return (Uart->LCR & UART_PARITY_Mask);
}

/** @brief Get an UART's line status bits.
  * @param Uart         A pointer to the UART instance
  * @return             The UART's line status bits (9 bits).
  */
__INLINE static uint32_t UART_GetLineStatus(UART_Type *Uart)
{
    return Uart->LSR;
}

/** @brief Get an UART's modem status bits.
  * @param Uart         A pointer to the UART instance
  * @return             The UART's modem status bits (8 bits).
  */
__INLINE static uint32_t UART_GetModemStatus(UART_Type *Uart)
{
    return Uart->MSR;
}

/** @brief Enable an UART's transmitter.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_EnableTx(UART_Type *Uart)
{
    Uart->TER = UART_TXEN;
}

/** @brief Disable an UART's transmitter.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_DisableTx(UART_Type *Uart)
{
    Uart->TER = 0;
}

/** @brief Test whether an UART's transmitter is enabled.
  * @param Uart         A pointer to the UART instance
  * @return             1 if the transmitter is enabled, 0 otherwise.
  */
__INLINE static unsigned int UART_TxIsEnabled(UART_Type *Uart)
{
    return (Uart->TER & UART_TXEN) ? 1:0;
}

/** @brief Begin signaling BREAK on an UART's TX line.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  *
  * @note
  * BREAK is signaled by holding the TX line in a '1' state beyond
  * the length of a frame (generally at least 15 bit periods).  This
  * function begins the signaling; timing must be done in software.
  */
__INLINE static void UART_BeginBreak(UART_Type *Uart)
{
    Uart->LCR |= UART_BREAK;
}

/** @brief End signaling BREAK on an UART's TX line.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  *
  * @sa UART_BeginBreak
  */
__INLINE static void UART_EndBreak(UART_Type *Uart)
{
    Uart->LCR &= ~UART_BREAK;
}

/** @brief Test whether an UART is currently signaling BREAK.
  * @param Uart         A pointer to the UART instance
  * @return             1 if the UART is signaling BREAK, 0 otherwise.
  *
  * @sa UART_BeginBreak
  */
__INLINE static unsigned int UART_IsInBreak(UART_Type *Uart)
{
    return (Uart->LCR & UART_BREAK) ? 1:0;
}

/** @brief Enable loopback mode on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_EnableLoopback(UART_Type *Uart)
{
    Uart->MCR |= UART_LOOPBACK;
}

/** @brief Disable loopback mode on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_DisableLoopback(UART_Type *Uart)
{
    Uart->MCR &= ~UART_LOOPBACK;
}

/** @brief Test whether loopback mode is enabled on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             1 if loopback mode is enabled, 0 otherwise.
  */
__INLINE static unsigned int UART_LoopbackIsEnabled(UART_Type *Uart)
{
    return (Uart->MCR & UART_LOOPBACK) ? 1:0;
}

/** @brief Set the state of an UART's DTR line.
  * @param Uart         A pointer to the UART instance
  * @param State        The new DTR state (non-zero value to set, zero to clear)
  * @return             None.
  */
__INLINE static void UART_SetDTR(UART_Type *Uart, unsigned int State)
{
    if (State) {
        Uart->MCR |= UART_DTR;
    } else {
        Uart->MCR &= UART_DTR;
    }
}

/** @brief Get the current state of an UART's DTR Line.
  * @param Uart         A pointer to the UART instance
  * @return             1 if in DTR is set, 0 otherwise.
  */
__INLINE static unsigned int UART_DTRIsSet(UART_Type *Uart)
{
    return (Uart->MCR & UART_DTR) ? 1:0;
}

/** @brief Set the state of an UART's RTS line.
  * @param Uart         A pointer to the UART instance
  * @param State        The new RTS state (non-zero value to set, zero to clear)
  * @return             None.
  */
__INLINE static void UART_SetRTS(UART_Type *Uart, unsigned int State)
{
    if (State) {
        Uart->MCR |= UART_RTS;
    } else {
        Uart->MCR &= UART_RTS;
    }
}

/** @brief Get the current state of an UART's RTS line.
  * @param Uart         A pointer to the UART instance
  * @return             1 if in RTS is set, 0 otherwise.
  */
__INLINE static uint8_t UART_GetRTS(UART_Type *Uart)
{
    return (Uart->MCR & UART_RTS) ? 1:0;
}

/** @brief Enable automatic RTS flow control on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_AutoRTSEnable(UART_Type *Uart)
{
    Uart->MCR |= UART_RTSENA;
}

/** @brief Disable automatic RTS flow control on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_AutoRTSDisable(UART_Type *Uart)
{
    Uart->MCR &= ~UART_RTSENA;
}

/** @brief Test whether automatic RTS flow control is enabled on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             1 if automatic RTS flow control is enabled, 0 otherwise.
  */
__INLINE static uint8_t UART_AutoRTSIsEnabled(UART_Type *Uart)
{
    return (Uart->MCR & UART_RTSENA) ? 1:0;
}

/** @brief Enable automatic CTS flow control on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_AutoCTSEnable(UART_Type *Uart)
{
    Uart->MCR |= UART_CTSENA;
}

/** @brief Disable automatic CTS flow control on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_AutoCTSDisable(UART_Type *Uart)
{
    Uart->MCR &= ~UART_CTSENA;
}

/** @brief Test whether automatic CTS flow control is enabled on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             1 if automatic CTS flow control is enabled, 0 otherwise.
  */
__INLINE static uint8_t UART_AutoCTSIsEnabled(UART_Type *Uart)
{
    return (Uart->MCR & UART_CTSENA) ? 1:0;
}

/** @brief Set the number of bytes in an UART's Rx FIFO that will trigger an interrupt.
  * @param Uart         A pointer to the UART instance
  * @param Trigger      A token indicating the number of bytes in the FIFO that will trigger an interrupt
  * @return             None.
  */
__INLINE static void UART_SetRxFifoTrigger(UART_Type *Uart, UART_RxFifoTrigger_Type Trigger)
{
    lpclib_assert(UART_IS_RXFIFOTRIGGER(Trigger));

    Uart->FCR = Trigger | UART_FIFOEN;
}

/** @brief Flush an UART's Receive FIFO.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_FlushRxFifo(UART_Type *Uart)
{
    Uart->FCR = (Uart->IIR & UART_FIFO_Mask)
                 | UART_FIFOEN | UART_FIFO_RX_RESET;
}

/** @brief Flush an UART's Transmit FIFO.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_FlushTxFifo(UART_Type *Uart)
{
    Uart->FCR = (Uart->IIR & UART_FIFO_Mask)
                 | UART_FIFOEN | UART_FIFO_TX_RESET;
}

/** @brief Flush an UART's (Rx & Tx) FIFOs.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_FlushFifos(UART_Type *Uart)
{
    Uart->FCR = (Uart->IIR & UART_FIFO_Mask)
                 | UART_FIFOEN | UART_FIFO_TX_RESET | UART_FIFO_RX_RESET;
}

/** @brief Enable an UART's FIFOs.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  *
  * @note
  * FIFOs should be enabled during normal use (according to the datasheet,
  * enabling the FIFOs is necessary for proper operation).
  */
__INLINE static void UART_EnableFifos(UART_Type *Uart)
{
    Uart->FCR = UART_FIFOEN;
}

/** @brief Disable an UART's FIFOs.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  *
  * @note
  * FIFOs should not be disabled during normal use.
  *
  * Disabling FIFOS will:
  * - Clear the RX Trigger, setting it back to 1 character.
  * - Flush the FIFOs.
  */
__INLINE static void UART_DisableFifos(UART_Type *Uart)
{
    Uart->FCR = 0;
}

/** @brief Set an UART's baud rate divisor.
  * @param Uart         A pointer to the UART instance
  * @param Divisor      The number of prescaled UART clocks per bit (16 bits)
  * @return             None.
  *
  * @note
  * A divisor of 0 is treated as a divisor of 1.
  */
__INLINE static void UART_SetDivisor(UART_Type *Uart, uint16_t Divisor)
{
    Uart->LCR |= UART_DLAB;
    Uart->DLL = Divisor & 0xff;
    Uart->DLM = Divisor >> 8;
    Uart->LCR &= ~UART_DLAB;
}

/** @brief Get an UART's current baud rate divisor.
  * @param Uart         A pointer to the UART instance
  * @return             The configured # of prescaled UART clocks per bit.
  */
__INLINE static uint16_t UART_GetDivisor(UART_Type *Uart)
{
    uint16_t Divisor;


    Uart->LCR |= UART_DLAB;
    Divisor = ((Uart->DLM & 0xff) << 8) | (Uart->DLL & 0xff);
    Uart->LCR &= ~UART_DLAB;

    return Divisor;
}

/** @brief Start autobaud on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  *
  * @note
  * Autobaud running flag will automatically clear when autobaud has
  * completed.
  */
__INLINE static void UART_BeginAutobaud(UART_Type *Uart)
{
    Uart->ACR |= UART_AUTOBAUD;
}

/** @brief End autobaud on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  *
  * @sa UART_BeginAutobaud
  */
__INLINE static void UART_EndAutobaud(UART_Type *Uart)
{
    Uart->ACR &= ~UART_AUTOBAUD;
}

/** @brief Test whether an UART's autobaud is active.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  *
  * @sa UART_BeginAutobaud
  */
__INLINE static void UART_AutobaudIsRunning(UART_Type *Uart)
{
    return (Uart->ACR & UART_AUTOBAUD) ? 1:0;
}

/** @brief Set an UART's autobaud mode.
  * @param Uart         A pointer to the UART instance
  * @param Mode         The new autobaud mode.
  * @return             None.
  *
  * @sa UART_BeginAutobaud
  */
__INLINE static void UART_SetAutobaudMode(UART_Type *Uart, UART_AutobaudMode_Type Mode)
{
    lpclib_assert(UART_IS_AUTOBAUDMODE(Mode));

    Uart->ACR = (UART->ACR & ~UART_MODE1) | Mode;
}

/** @brief Get an UART's current autobaud mode.
  * @param Uart         A pointer to the UART instance
  * @return             The current autobaud mode of the UART
  *
  * @sa UART_BeginAutobaud
  */
__INLINE static UART_AutobaudMode_Type UART_GetAutobaudMode(UART_Type *Uart)
{
    return UART->ACR & UART_MODE1;
}

/** @brief Enable autobaud auto-restart on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  *
  * @sa UART_BeginAutobaud
  *
  * @note
  * Autobaud autorestart causes the autobaud function to re-start
  * in case of timeout.
  */
__INLINE static void UART_EnableAutobaudAutoRestart(UART_Type *Uart)
{
    Uart->ACR |= UART_AUTORESTART;
}

/** @brief Disable autobaud auto-restart on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  *
  * @sa UART_BeginAutobaud
  * @sa UART_EnableAutobaudAutoRestart
  */
__INLINE static void UART_DisableAutobaudAutoRestart(UART_Type *Uart)
{
    Uart->ACR &= ~UART_AUTORESTART;
}

/** @brief Get an UART's current autobaud mode.
  * @param Uart         A pointer to the UART instance
  * @return             The current autobaud mode of the UART
  *
  * @sa UART_BeginAutobaud
  * @sa UART_EnableAutobaudAutoRestart
  */
__INLINE static unsigned int UART_AutobaudAutoRestartIsEnabled(UART_Type *Uart)
{
    return (UART->ACR & UART_AUTORESTART) ? 1:0;
}

/** @brief Get a bitmask of pending autobaud interrupts on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             A bitmask of pending autobaud interrupts.
  */
__INLINE static UART_AutobaudIT_Type UART_GetPendingAutobaudITs(UART_Type *Uart)
{
    return Uart->IIR & (UART_IT_ABEO | UART_IT_ABTO);
}

/** @brief Clear pending autobaud interrupts on an UART.
  * @param Uart         A pointer to the UART instance
  * @param IT           A bitmask of autobaud interrupts to clear
  * @return             None.
  */
__INLINE static void UART_ClearPendingAutobaudITs(UART_Type *Uart, UART_AutobaudIT_Type IT)
{
    lpclib_assert((IT & ~UART_AutobaudIT_Mask) == 0);

    Uart->ACR |= IT;
}

/** @brief Set an UART's fractional divider
  * @param Uart         A pointer to the UART instance
  * @param Div          The divider portion of the fractional divider (0 <= Div; disabled if 0)
  * @param Mult         The multiplier portion of the fractional divider (0 < Div <= Mult <= 15)
  * @return             None.
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
__INLINE static void UART_SetFractionalDivider(UART_Type *Uart, unsigned int Div, unsigned int Mult)
{
    lpclib_assert(Div <= 14);
    lpclib_assert((Mult > 0) && (Mult <= 15));
    lpclib_assert(Mult > Div);

    UART->FDR = (Mul << UART_MULVAL_Shift) | Div;
}

/** @brief Get the divider portion of an UART's fractional divider value.
  * @param Uart         A pointer to the UART instance
  * @return             The divider portion of the UART's fractional divider value.
  *
  * @sa UART_SetFractionalDivider
  */
__INLINE static unsigned int UART_GetFractionalDividerDiv(UART_Type *Uart)
{
    return UART->FDR & UART_DIVADDVAL_Mask;
}

/** @brief Get the multiplier portion of an UART's fractional divider value.
  * @param Uart         A pointer to the UART instance
  * @return             The multiplier portion of the UART's fractional divider value.
  *
  * @sa UART_SetFractionalDivider
  */
__INLINE static unsigned int UART_GetFractionalDividerMult(UART_Type *Uart)
{
    return (UART->FDR & UART_MULVAL_Mask) >> UART_MULVAL_Shift;
}

/** @brief Disable an UART's fractional divider.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  *
  * @note
  * Sets the fractional divider's multiplier to 1, the divider to 0.
  */
__INLINE static void UART_DisableFractionalDivider(UART_Type *Uart)
{
    UART->FDR = (1 << UART_MULVAL_Shift);
}

/** @brief Enable RS485 normal multidrop mode on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_EnableRS485NormalMultidropMode(UART_Type *Uart)
{
    UART->RS485CTRL |= UART_NMMEN;
}

/** @brief Disable RS485 normal multidrop mode on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_DisableRS485NormalMultidropMode(UART_Type *Uart)
{
    UART->RS485CTRL &= ~UART_NMMEN;
}

/** @brief Test whether RS485 normal multidrop mode is enabled on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             1 if normal multidrop mode is enabled, 0 otherwise.
  */
__INLINE static unsigned int UART_RS485NormalMultidropModeIsEnabled(UART_Type *Uart)
{
    return (UART->RS485CTRL & UART_NMMEN) ? 1:0;
}

/** @brief Enable the receiver in RS485 mode on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  *
  * @note
  * The receiver is enabled when the bit is cleared.
  */
__INLINE static void UART_EnableRS485Rx(UART_Type *Uart)
{
    UART->RS485CTRL &= ~UART_RXDIS;
}

/** @brief Disable the receiver in RS485 mode on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  *
  * @note
  * The receiver is enabled when the bit is cleared.
  */
__INLINE static void UART_DisableRS485Rx(UART_Type *Uart)
{
    UART->RS485CTRL |= UART_RXDIS;
}

/** @brief Test whether the receiver is enabled on an UART in RS485 mode.
  * @param Uart         A pointer to the UART instance
  * @return             1 if the receiver is enabled, 0 otherwise.
  *
  * @note
  * The receiver is enabled when the bit is cleared.
  */
__INLINE static unsigned int UART_RS485RxIsEnabled(UART_Type *Uart)
{
    return (UART->RS485CTRL & UART_RXDIS) ? 0:1;
}

/** @brief Enable auto address detect in RS485 mode on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_EnableRS485AutoAddressDetect(UART_Type *Uart)
{
    UART->RS485CTRL |= UART_AADEN;
}

/** @brief Disable auto address detect in RS485 mode on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_DisableRS485AutoAddressDetect(UART_Type *Uart)
{
    UART->RS485CTRL &= ~UART_AADEN;
}

/** @brief Test whether auto address detect is enabled on an UART in RS485 mode.
  * @param Uart         A pointer to the UART instance
  * @return             1 if auto auto address detect is enabled, 0 otherwise.
  */
__INLINE static unsigned int UART_RS485AutoAddressDetectIsEnabled(UART_Type *Uart)
{
    return (UART->RS485CTRL & UART_AADEN) ? 1:0;
}

/** @brief Set the pin used for direction control in RS485 mode on an UART.
  * @param Uart         A pointer to the UART instance
  * @param ControlPin   A token indicating the pin to use for RS485 direction control.
  * @return             None.
  */
__INLINE static void UART_SetRS485DirControlPin(UART_Type *Uart,
                                                UART_RS485DirControlPin_Type ControlPin)
{
    lpclib_assert(UART_IS_RS485DIRCONTROLPIN(ControlPin);

    UART->RS485CTRL = (UART_RS485CTRL & ~UART_DIRSEL) | ControlPin;
}

/** @brief Get the pin used for direction control in RS485 mode on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             A token indicating the pin used for RS485 direction control.
  */
__INLINE static UART_RS485DirControlPin_Type UART_GetRS485DirControlPin(UART_Type *Uart)
{
    return (UART->RS485CTRL & UART_DIRSEL);
}

/** @brief Enable RS485 automatic direction control on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  *
  * @sa UART_SetRS485DirControlPin
  */
__INLINE static void UART_EnableRS485AutoDirControl(UART_Type *Uart)
{
    UART->RS485CTRL |= UART_DCTRL;
}

/** @brief Disable RS485 automatic direction control on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  *
  * @sa UART_SetRS485DirControlPin
  */
__INLINE static void UART_DisableRS485AutoDirControl(UART_Type *Uart)
{
    UART->RS485CTRL &= ~UART_DCTRL;
}

/** @brief Test whether RS485 automatic direction control is enabled on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             1 if automatic direction control is enabled, 0 otherwise.
  *
  * @sa UART_SetRS485DirControlPin
  */
__INLINE static unsigned int UART_RS485AutoDirControlIsEnabled(UART_Type *Uart)
{
    return (UART->RS485CTRL & UART_DCTRL) ? 1:0;
}

/** @brief Set the polarity of the RS485 direction control pin on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  */
__INLINE static void UART_SetRS485DirControlPolarity(UART_Type *Uart,
                                                     UART_RS485DirControlPolarity_Type Polarity)
{
    UART->RS485CTRL = (UART->RS485CTRL & ~UART_OINV) | Polarity;
}

/** @brief Get the polarity of the RS485 direction control pin on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             A token indicating the current polarity of the RS485 direction control pin.
  */
__INLINE static UART_RS485DirControlPolarity_Type UART_GetRS485DirControlPolarity(UART_Type *Uart)
{
    return (UART->RS485CTRL & UART_OINV);
}

/** @brief Set the RS485 address on an UART.
  * @param Uart         A pointer to the UART instance
  * @param Addr         The new address (0-255)
  * @return             None.
  */
__INLINE static void UART_SetRS485Address(UART_Type *Uart, unsigned int Address)
{
    lpclib_assert(Address <= 255);

    UART->RS485ADRMATCH = Address;
}

/** @brief Get the polarity of the RS485 direction control pin on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             The current RS485 address.
  */
__INLINE static unsigned int UART_GetRS485Address(UART_Type *Uart)
{
    return UART->RS485ADRMATCH;
}

/** @brief Set the RS485 direction pin delay on an UART.
  * @param Uart         A pointer to the UART instance
  * @param Delay        The delay, in baud clocks (0-255)
  * @return             None.
  *
  * @note
  * This controls the delay between the last stop bit leaving the Tx FIFO
  * and the de-assertion of the direction control pin.
  */
__INLINE static void UART_SetRS485DirDelay(UART_Type *Uart, unsigned int Delay)
{
    lpclib_assert(Delay <= 255);

    UART->RS485DLY = Delay;
}

/** @brief Get the current RS485 direction pin delay on an UART.
  * @param Uart         A pointer to the UART instance
  * @return             None.
  *
  * @sa UART_SetRS485DirDelay
  */
__INLINE static unsigned int UART_GetRS485DirDelay(UART_Type *Uart)
{
    return UART->RS485DLY;
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
