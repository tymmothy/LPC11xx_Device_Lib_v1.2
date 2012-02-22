/**************************************************************************//**
 * @file     ssp.h
 * @brief   SSP Serial Interface Header for NXP LPC Microcontrollers
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. January 2012
 ******************************************************************************
 * @section Overview
 * This file gives a basic interface to NXP LPC microcontroller
 * SSPs.  It abstracts such things as setting word sizes, frame formats,
 * clock polarities, etc., as well as simplifying use of SSP interrupts
 * and reading/writing data to/from the SSP.
 *
 * @note
 * This file does not handle the following necessary steps for SSP use:
 * - IO Pins must be configured for SSP use
 * - The SSP's (AHB/APB/VPB) bus clock line must be enabled (& on some
 *   chips, e.g. LPC11xx, the SSP input clock divider configured).
 * - For interrupt use, an interrupt handler must be declared and
 *   the SSP's interrupt line must be enabled in the microcontroller's
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
 *****************************************************************************/

#ifndef NXP_LPC_SSP_H_
#define NXP_LPC_SSP_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "lpc11xx.h"
#include "lpclib_assert.h"


/**
  * @defgroup SSP_AbstractionLayer SSP (Synchronous Serial Peripheral) Abstraction layer
  * @ingroup  LPC_Peripheral_AbstractionLayer
  * @{
  */

/* Types & Type-Related Definitions -----------------------------------------*/

/** @defgroup SSP_Types SSP Interface Types and Type-Related Definitions
  * @{
  */

/** @defgroup SSP_WordLengths SSP Word Lengths
  * @{
  */

/*! @brief SSP Word Length Settings */
typedef enum {
   SSP_WordLength_4 = 4,               /*!< Send/Receive 4 bit words         */
   SSP_WordLength_5,                   /*!< Send/Receive 5 bit words         */
   SSP_WordLength_6,                   /*!< Send/Receive 6 bit words         */
   SSP_WordLength_7,                   /*!< Send/Receive 7 bit words         */
   SSP_WordLength_8,                   /*!< Send/Receive 8 bit words         */
   SSP_WordLength_9,                   /*!< Send/Receive 9 bit words         */
   SSP_WordLength_10,                  /*!< Send/Receive 10 bit words        */
   SSP_WordLength_11,                  /*!< Send/Receive 11 bit words        */
   SSP_WordLength_12,                  /*!< Send/Receive 12 bit words        */
   SSP_WordLength_13,                  /*!< Send/Receive 13 bit words        */
   SSP_WordLength_14,                  /*!< Send/Receive 14 bit words        */
   SSP_WordLength_15,                  /*!< Send/Receive 15 bit words        */
   SSP_WordLength_16                   /*!< Send/Receive 16 bit words        */
} SSP_WordLength_Type;

/*! @brief Macro to test whether parameter is a valid SSP Word Length value */
#define SSP_IS_WORD_LENGTH(WordLength) (((WordLength) >= SSP_WordLength_4) \
                                     && ((WordLength) <= SSP_WordLength_16))

/** @} */

/** @defgroup SSP_FrameFormats SSP Frame Formats
  * @{
  */

/*! @brief SSP Frame Format Configurations */
typedef enum {
   SSP_FrameFormat_SPI = 0x00,         /*!< Data formatted as SPI Frames     */
   SSP_FrameFormat_TI = 0x10,          /*!< Data formatted as TI Frames      */
   SSP_FrameFormat_MW = 0x20           /*!< Data formatted as Microwire      */
} SSP_FrameFormat_Type;

/*! @brief Macro to test whether parameter is a valid SSP Frame Format value */
#define SSP_IS_FRAME_FORMAT(FrameFormat) (((FrameFormat) == SSP_FrameFormatSPI)\
                                       || ((FrameFormat) == SSP_FrameFormatTI) \
                                       || ((FrameFormat) == SSP_FrameFormatMW))

/** @} */

/** @defgroup SSP_ClockPolarities SSP Clock Polarities
  * @{
  */

/** @brief SSP Inter-Frame Clock Polarity Configurations */
typedef enum {
   SSP_ClockPolarity_Low  = 0x00,      /*!< Clock line low between frames    */
   SSP_ClockPolarity_High = 0x40       /*!< Clock line high between frames   */
} SSP_ClockPolarity_Type;

/*! @brief Macro to test whether parameter is a valid SSP Clock Polarity value */
#define SSP_IS_CLOCK_POLARITY(Polarity) (((Polarity) == SSP_ClockPolarityLow) \
                                      || ((Polarity) == SSP_ClockPolarityHigh))

/** @} */

/** @defgroup SSP_ClockPhases SSP Clock Phases
  * @{
  */

/** @brief SSP Clock Phase Configurations */
typedef enum {
   SSP_ClockPhase_A = 0x00,            /*!< Data latched on 1st clock change */
   SSP_ClockPhase_B = 0x80             /*!< Data latched on 2nd clock change */
} SSP_ClockPhase_Type;

/*! @brief Macro to test whether parameter is a valid SSP Clock Phase value */
#define SSP_IS_CLOCK_PHASE(Phase) (((Phase) == SSP_ClockPhaseA) \
                                || ((Phase) == SSP_ClockPhaseB))

/** @} */

/** @defgroup SSP_CommunicationModes SSP Communication Modes
  * @{
  */

/*! @brief SSP Communication Mode (Master/Slave/Etc.) Configurations */
typedef enum {
    SSP_Mode_Master         = 0x00,    /*!< SSP is wire Master               */
    SSP_Mode_Slave          = 0x04,    /*!< SSP is wire Slave                */
    SSP_Mode_SlaveInputOnly = 0x0c     /*!< SSP is wire Slave, out disabled  */
} SSP_Mode_Type;

/*! @brief Macro to test whether parameter is a valid SSP Communication Mode value */
#define SSP_IS_MODE(Mode) (((Mode) == SSP_ModeMaster) \
                        || ((Mode) == SSP_ModeSlave)  \
                        || ((Mode) == SSP_ModeSlaveInputOnly))

/** @} */

/** @defgroup SSP_Interrupts SSP Interrupts
  * @{
  */

/*! @brief SSP Interrupts */
typedef enum {
    SSP_IT_RxOverrun   = 0x01,         /*!< Receive Overrun Interrupt        */
    SSP_IT_RxTimer     = 0x02,         /*!<  Rx FIFO read timeout            */
    SSP_IT_RxHalfFull  = 0x04,         /*!< Rx FIFO at least half full       */
    SSP_IT_TxHalfEmpty = 0x08,         /*!<  Tx FIFO at least half empty     */
} SSP_IT_Type;

/*! @brief Macro to test whether parameter is a valid SSP Interrupt value */
#define SSP_IS_IT(Interrupt) (((Interrupt) == SSP_IT_RxOverrun)   \
                           || ((Interrupt) == SSP_IT_RxTimeout)   \
                           || ((Interrupt) == SSP_IT_RxHalfFull)  \
                           || ((Interrupt) == SSP_IT_TxHalfEmpty))

/** @} */

/**
  * @}
  */

/* Inline Functions ---------------------------------------------------------*/

/**
  * @defgroup SSP_InlineFunctions SSP Interface Inline Functions
  * @{
  */

/** @brief Send a word via an SSP.
  * @param SSP          A pointer to the SSP instance
  * @param Word         The word to send
  * @return             None.
  */
__INLINE static void SSP_Send(SSP_Type *SSP, uint16_t Word)
{
    SSP->DR = Word;
}

/** @brief Retrieve a word from an SSP's incoming FIFO
  * @param SSP          A pointer to the SSP instance
  * @return             The word read from the SSP's FIFO.
  */
__INLINE static uint16_t SSP_Recv(SSP_Type *SSP)
{
    return SSP->DR;
}

/** @brief Send & Receive a Word via the SSP's Outgoing FIFO
  * @param SSP          A pointer to the SSP instance
  * @param WordOut      The word to send
  * @return The word read from the SSP's FIFO.
  */
__INLINE static uint16_t SSP_Xfer(SSP_Type *SSP, uint16_t WordOut)
{
    SSP->DR = WordOut;
    while (!SSP_RxIsAvailable(SSP));
    return SSP->DR;
}

/** @brief Flush an SSP's receive FIFO.
  * @param SSP          A pointer to the SSP instance
  * @return             None.
  */
__INLINE static void SSP_FlushRxFifo(SSP_Type *SSP)
{
    while (SSP_RxIsAvailable(SSP)) {
        SSP_Recv(SSP);
    }
}

/** @brief Enable loopback mode on an SSP.
  * @param SSP          A pointer to the SSP instance
  * @return             None.
  */
__INLINE static void SSP_EnableLoopback(SSP_Type *SSP)
{
    SSP->CR1 |= SSP_LBM;
}

/** @brief Disable loopback mode on an SSP.
  * @param SSP          A pointer to the SSP instance
  * @return             None.
  */
__INLINE static void SSP_DisableLoopback(SSP_Type *SSP)
{
    SSP->CR1 &= ~SSP_LBM;
}

/** @brief Test whether loopback mode is enabled on an SSP.
  * @param SSP          A pointer to the SSP instance
  * @return             1 if loopback mode is enabled, 0 otherwise.
  */
__INLINE static unsigned int SSP_LoopbackIsEnabled(SSP_Type *SSP)
{
    return (SSP->CR1 & SSP_LBM) ? 1:0;
}

/** @brief Enable an SSP.
  * @param SSP          A pointer to the SSP instance
  * @return             None.
  */
__INLINE static void SSP_Enable(SSP_Type *SSP)
{
    SSP->CR1 |= SSP_SSE;
}

/** @brief Disable an SSP.
  * @param SSP          A pointer to the SSP instance
  * @return             None.
  */
__INLINE static void SSP_Disable(SSP_Type *SSP)
{
    SSP->CR1 &= ~SSP_SSE;
}

/** @brief Test whether an SSP is enabled
  * @param SSP          A pointer to the SSP instance
  * @return             1 if the SSP is enabled, 0 otherwise.
  */
__INLINE static unsigned int SSP_IsEnabled(SSP_Type *SSP)
{
    return (SSP->CR1 & SSP_SSE) ? 1:0;
}

/** @brief Test whether an SSP's transmit FIFO is empty.
  * @param SSP          A pointer to the SSP instance
  * @return             1 if the transmit FIFO is empty, 0 otherwise.
  */
__INLINE static unsigned int SSP_TxFIFOIsEmpty(SSP_Type *SSP)
{
    return (SSP->SR & SSP_TFE) ? 1:0;
}

/** @brief Test whether an SSP has space available in the transmit FIFO.
  * @param SSP          A pointer to the SSP instance
  * @return             1 if the transmit FIFO has space available, 0 otherwise.
  */
__INLINE static unsigned int SSP_TxIsReady(SSP_Type *SSP)
{
    return (SSP->SR & SSP_TNF) ? 1:0;
}

/** @brief Test whether an SSP has data in the receive FIFO.
  * @param SSP          A pointer to the SSP instance
  * @return             1 if the receive FIFO has data available, 0 otherwise.
  */
__INLINE static unsigned int SSP_RxIsAvailable(SSP_Type *SSP)
{
    return (SSP->SR & SSP_RNE) ? 1:0;
}

/** @brief Test whether an SSP's receive FIFO is full.
  * @param SSP          A pointer to the SSP instance
  * @return             1 if the receive FIFO is full, 0 otherwise.
  */
__INLINE static unsigned int SSP_RxFIFOIsFull(SSP_Type *SSP)
{
    return (SSP->SR & SSP_RFF) ? 1:0;
}

/** @brief Test whether an SSP is currently busy (transferring data).
  * @param SSP          A pointer to the SSP instance
  * @return             1 if the SSP is busy, 0 otherwise.
  */
__INLINE static unsigned int SSP_IsBusy(SSP_Type *SSP)
{
    return (SSP->SR & SSP_BSY) ? 1:0;
}

/** @brief Enable specific interrupts on an SSP.
  * @param SSP          A pointer to the SSP instance
  * @param IT           A bitmask of SSP interrupts to enable
  * @return             None.
  */
__INLINE static void SSP_EnableIT(SSP_Type *SSP, SSP_IT_Type IT)
{
    lpclib_assert(SSP_IS_IT(IT));

    SSP->IMSC |= IT;
}

/** @brief Disable specific interrupts on an SSP.
  * @param SSP          A pointer to the SSP instance
  * @param  IT          A bitmask of SSP interrupts to disable
  * @return             None.
  */
__INLINE static void SSP_DisableIT(SSP_Type *SSP, SSP_IT_Type IT)
{
    lpclib_assert(SSP_IS_IT(IT));

    SSP->IMSC &= ~IT;
}

/** @brief Get a bitmask of the enabled interrupts on an SSP.
  * @param SSP          A pointer to the SSP instance
  * @return             A bitmask of the SSP's enabled interrupts.
  */
__INLINE static SSP_IT_Type SSP_GetEnabledIT(SSP_Type *SSP)
{
    return SSP->IMSC;
}

/** @brief Get a bitmask of interrupts currently pending on an SSP.
  * @param SSP          A pointer to the SSP instance
  * @return             A bitmask of the SSP's pending interrupt bits
  */
__INLINE static uint32_t SSP_GetPendingIT(SSP_Type *SSP)
{
    return SSP->MIS;
}

/** @brief Clear currently pending interrupts on an SSP.
  * @param SSP          A pointer to the SSP instance
  * @param IT           A bitmask of pending SSP interrupts to clear
  * @return             None.
  *
  * @note
  * Only SSP_IT_RxOverrun and SSP_IT_RxTimer interrupts can be cleared (others
  * are wired to the FIFO status).
  */
__INLINE static void SSP_ClearPendingIT(SSP_Type *SSP, SSP_IT_Type IT)
{
    lpclib_assert((IT == SSP_IT_RxOverrun) || (IT == SSP_IT_RxTimer));

    SSP->ICR |= IT;
}

/** @brief Set the operating mode (master or slave or input-only slave) of an SSP.
  * @param SSP          A pointer to the SSP instance
  * @param Mode         Token indicating the new operating mode
  * @return             None.
  */
__INLINE static void SSP_SetMode(SSP_Type *SSP, SSP_Mode_Type Mode)
{
    lpclib_assert(SSP_IS_MODE(Mode));

    SSP->CR1 = (SSP->CR1 & ~SSP_CR1_MODE_Mask) | Mode;
}

/** @brief Get the operating mode (master or slave or input-only slave) of an SSP.
  * @param SSP          A pointer to the SSP instance
  * @return             The current operating mode of the SSP.
  */
__INLINE static SSP_Mode_Type SSP_GetMode(SSP_Type *SSP)
{
    return SSP->CR1 & SSP_CR1_MODE_Mask;
}

/** @brief Set an SSP's word length.
  * @param SSP          A pointer to the SSP instance
  * @param WordLength   A token indicating the new SSP transfer word length
  * @return             None.
  */
__INLINE static void SSP_SetWordLength(SSP_Type *SSP, SSP_WordLength_Type WordLength)
{
    lpclib_assert(SSP_IS_WORD_LENGTH(WordLength));

    SSP->CR0 = (SSP->CR0 & ~(SSP_DSS_Mask)) | (WordLength - 1);
}

/** @brief Get an SSP's transfer word length.
  * @param SSP          A pointer to the SSP instance
  * @return             A token indicating the SSP's current transfer word length.
  */
__INLINE static SSP_WordLength_Type SSP_GetWordLength(SSP_Type *SSP)
{
    return (SSP->CR0 & SSP_DSS_Mask) + 1;
}

/** @brief Set the format of frames transferred on an SSP.
  * @param SSP          A pointer to the SSP instance
  * @param FrameFormat  A token indicating the new format of SSP frames
  * @return             None.
  */
__INLINE static void SSP_SetFrameFormat(SSP_Type *SSP, SSP_FrameFormat_Type FrameFormat)
{
    lpclib_assert(SSP_IS_FRAMEFORMAT(FrameFormat));

    SSP->CR0 = (SSP->CR0 & ~(SSP_FRF_Mask)) | FrameFormat;
}

/** @brief Get the current format of frames transferred on an SSP.
  * @param SSP          A pointer to the SSP instance
  * @return             A token indicating the current format of SSP frames.
  */
__INLINE static SSP_FrameFormat_Type SSP_GetFrameFormat(SSP_Type *SSP)
{
    return (SSP->CR0 & SSP_FRF_Mask);
}

/** @brief Set the idle-state polarity of an SSP's clock line.
  * @param SSP          A pointer to the SSP instance
  * @param Polarity     A token indicating the new idle-stat clock line polarity
  * @return             None.
  */
__INLINE static void SSP_SetClockPolarity(SSP_Type *SSP, SSP_ClockPolarity_Type Polarity)
{
    SSP->CR0 = (SSP->CR0 & ~(SSP_CPOL)) | Polarity;
}

/** @brief Get the idle-state polarity of an SSP's clock line.
  * @param SSP          A pointer to the SSP instance
  * @return             A token indicating the SSP's current idle-state clock line polarity.
  */
__INLINE static SSP_ClockPolarity_Type SSP_SetClockPolarity(SSP_Type *SSP,
                                                            SSP_ClockPolarity_Type Polarity)
{
    return (SSP->CR0 & SSP_CPOL);
}

/** @brief Set the clock line phase on which an SSP latches data.
  * @param SSP          A pointer to the SSP instance
  * @param Phase        A token indicating the new clock line phase on which to latch data
  * @return             None.
  */
__INLINE static void SSP_SetClockPhase(SSP_Type *SSP, SSP_ClockPhase_Type Phase)
{
    SSP->CR0 = (SSP->CR0 & ~(SSP_CPHA)) | Phase;
}

/** @brief Get the current clock line phase on which an SSP latches data.
  * @param SSP          A pointer to the SSP instance
  * @return             A token indicating the clock line phase on which the SSP latches data.
  */
__INLINE static SSP_ClockPhase_Type SSP_SetClockPhase(SSP_Type *SSP)
{
    return (SSP->CR0 & SSP_CPHA);
}

/** @brief Set the number of prescaler ticks per bit transferred on an SSP.
  * @param SSP          A pointer to the SSP instance
  * @param Ticks        The new number of prescaler ticks per bit
  * @return             None.
  *
  * @note
  * - This function takes the actual # of prescaler counts, not the (off-by-1) SCR value
  * - Final bit frequency = PCLK / (Prescale_Divisor * Count).
  */
__INLINE static void SSP_SetPrescalerTicksPerBit(SSP_Type *SSP, unsigned int Ticks)
{
    lpclib_assert((Ticks > 0) && (Ticks <= 256));

    SSP->CR0 = (SSP->CR0 & ~(SSP_SCR_Mask)) | ((Ticks - 1) << SSP_SCR_Shift);
}

/** @brief Set the current number of prescaler counts per bit transferred on an SSP.
  * @param SSP          A pointer to the SSP instance
  * @return             The current number of SSP prescaler counts per bit.
  *
  * @note
  * - This function returns the actual # of prescaler counts, not the (off-by-1) SCR value
  */
__INLINE static unsigned int SSP_GetPrescalerTicksPerBit(SSP_Type *SSP, unsigned int Ticks)
{
    return ((SSP->CR0 & SSP_SCR_Mask) >> SSP_SCR_Shift) + 1;
}

/** @brief Set the input clock prescaler value on an SSP.
  * @param SSP          A pointer to the SSP instance
  * @param Prescaler    The new prescaler value (must be an even number; 2 <= n <= 254)
  * @return             None.
  */
__INLINE static void SSP_SetClockPrescaler(SSP_Type *SSP, unsigned int Prescaler)
{
    lpclib_assert(Prescaler <= 254);
    lpclib_assert((Prescaler & 1) == 0);

    SSP->CPSR = Prescaler;
}

/** @brief Get the current input clock prescaler value on an SSP.
  * @param SSP          A pointer to the SSP instance
  * @return             The SSP's current prescaler value.
  */
__INLINE static unsigned int SSP_GetClockPrescaler(SSP_Type *SSP)
{
    return SSP->CPSR;
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

#endif /* #ifndef NXP_LPC_SSP_H_ */
