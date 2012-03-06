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
 * - The SSP's (AHB or APB/VPB) clock line must be configured & enabled.
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
    SSP_WordLength_4 = 3,              /*!< Send/Receive 4 bit words         */
    SSP_WordLength_5,                  /*!< Send/Receive 5 bit words         */
    SSP_WordLength_6,                  /*!< Send/Receive 6 bit words         */
    SSP_WordLength_7,                  /*!< Send/Receive 7 bit words         */
    SSP_WordLength_8,                  /*!< Send/Receive 8 bit words         */
    SSP_WordLength_9,                  /*!< Send/Receive 9 bit words         */
    SSP_WordLength_10,                 /*!< Send/Receive 10 bit words        */
    SSP_WordLength_11,                 /*!< Send/Receive 11 bit words        */
    SSP_WordLength_12,                 /*!< Send/Receive 12 bit words        */
    SSP_WordLength_13,                 /*!< Send/Receive 13 bit words        */
    SSP_WordLength_14,                 /*!< Send/Receive 14 bit words        */
    SSP_WordLength_15,                 /*!< Send/Receive 15 bit words        */
    SSP_WordLength_16                  /*!< Send/Receive 16 bit words        */
} SSP_WordLength_Type;

/*! @brief Macro to test whether parameter is a valid SSP Word Length value */
#define SSP_IS_WORDLENGTH(WordLength) (((WordLength) >= SSP_WordLength_4) \
                                    && ((WordLength) <= SSP_WordLength_16))

/** @} */

/** @defgroup SSP_FrameFormats SSP Frame Formats
  * @{
  */

/*! @brief SSP Frame Format Configurations */
typedef enum {
    SSP_FrameFormat_SPI = 0x00,        /*!< Data formatted as SPI Frames     */
    SSP_FrameFormat_TI  = 0x10,        /*!< Data formatted as TI Frames      */
    SSP_FrameFormat_MW  = 0x20         /*!< Data formatted as Microwire      */
} SSP_FrameFormat_Type;

/*! @brief Macro to test whether parameter is a valid SSP Frame Format value */
#define SSP_IS_FRAMEFORMAT(FrameFormat) (((FrameFormat) == SSP_FrameFormatSPI)\
                                      || ((FrameFormat) == SSP_FrameFormatTI) \
                                      || ((FrameFormat) == SSP_FrameFormatMW))

/** @} */

/** @defgroup SSP_ClockPolarities SSP Clock Polarities
  * @{
  */

/** @brief SSP Inter-Frame Clock Polarity Configurations */
typedef enum {
    SSP_ClockPolarity_Low  = 0x00,     /*!< Clock line low between frames    */
    SSP_ClockPolarity_High = 0x40      /*!< Clock line high between frames   */
} SSP_ClockPolarity_Type;

/*! @brief Macro to test whether parameter is a valid SSP Clock Polarity value */
#define SSP_IS_CLOCKPOLARITY(Polarity) (((Polarity) == SSP_ClockPolarityLow) \
                                     || ((Polarity) == SSP_ClockPolarityHigh))

/** @} */

/** @defgroup SSP_ClockPhases SSP Clock Phases
  * @{
  */

/** @brief SSP Clock Phase Configurations */
typedef enum {
    SSP_ClockPhase_A = 0x00,           /*!< Data latched on 1st clock change */
    SSP_ClockPhase_B = 0x80            /*!< Data latched on 2nd clock change */
} SSP_ClockPhase_Type;

/*! @brief Macro to test whether parameter is a valid SSP Clock Phase value */
#define SSP_IS_CLOCKPHASE(Phase) (((Phase) == SSP_ClockPhaseA) \
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

/** @defgroup SSP_Interrupt_Bits SSP Interrupt Bits
  * @{
  */

#define SSP_ITMask_Mask                (0x000f)            /*!< Mask of all interrupt bits       */

#define SSP_ITMask_RxOverrun           (1 << 0)            /*!< Receive overrun interrupt        */
#define SSP_ITMask_RxTimer             (1 << 1)            /*!< Rx FIFO read timeout interrupt   */
#define SSP_ITMask_RxHalfFull          (1 << 2)            /*!< Receive FIFO at least half full  */
#define SSP_ITMask_TxHalfEmpty         (1 << 3)            /*!< Tx FIFO at least half empty      */

/*! @brief Type for passing SSP interrupt bits */
typedef uint32_t SSP_ITMask_Type;

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
  * @param[in]  ssp          A pointer to the SSP instance
  * @param[in]  word         A word to send
  */
__INLINE static void SSP_Send(SSP_Type *ssp, uint16_t word)
{
    ssp->DR = word;
}

/** @brief Retrieve a word from an SSP's receive FIFO.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                  The word read from the SSP's receive FIFO.
  */
__INLINE static uint16_t SSP_Recv(SSP_Type *ssp)
{
    return ssp->DR;
}

/** @brief Send & receive a word via the SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  * @param[in]  word_out     A word to send
  * @return                  The word read from the SSP's incoming FIFO.
  */
__INLINE static uint16_t SSP_Xfer(SSP_Type *ssp, uint16_t word_out)
{
    ssp->DR = word_out;
    while (!SSP_RxIsAvailable(ssp));
    return ssp->DR;
}

/** @brief Flush an SSP's receive FIFO.
  * @param[in]  ssp          A pointer to the SSP instance
  */
__INLINE static void SSP_FlushRxFifo(SSP_Type *ssp)
{
    while (SSP_RxIsAvailable(ssp)) {
        SSP_Recv(ssp);
    }
}

/** @brief Enable loopback mode on an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  */
__INLINE static void SSP_EnableLoopback(SSP_Type *ssp)
{
    ssp->CR1 |= SSP_LBM;
}

/** @brief Disable loopback mode on an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  */
__INLINE static void SSP_DisableLoopback(SSP_Type *ssp)
{
    ssp->CR1 &= ~SSP_LBM;
}

/** @brief Test whether loopback mode is enabled on an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                  1 if loopback mode is enabled, 0 otherwise.
  */
__INLINE static unsigned int SSP_LoopbackIsEnabled(SSP_Type *ssp)
{
    return (ssp->CR1 & SSP_LBM) ? 1:0;
}

/** @brief Enable an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  */
__INLINE static void SSP_Enable(SSP_Type *ssp)
{
    ssp->CR1 |= SSP_SSE;
}

/** @brief Disable an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  */
__INLINE static void SSP_Disable(SSP_Type *ssp)
{
    ssp->CR1 &= ~SSP_SSE;
}

/** @brief Test whether an SSP is enabled.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                 1 if the SSP is enabled, 0 otherwise.
  */
__INLINE static unsigned int SSP_IsEnabled(SSP_Type *ssp)
{
    return (ssp->CR1 & SSP_SSE) ? 1:0;
}

/** @brief Test whether an SSP's transmit FIFO is empty.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                 1 if the transmit FIFO is empty, 0 otherwise.
  */
__INLINE static unsigned int SSP_TxFIFOIsEmpty(SSP_Type *ssp)
{
    return (ssp->SR & SSP_TFE) ? 1:0;
}

/** @brief Test whether an SSP has space available in the transmit FIFO.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                  1 if the transmit FIFO has space available, 0 otherwise.
  */
__INLINE static unsigned int SSP_TxIsReady(SSP_Type *ssp)
{
    return (ssp->SR & SSP_TNF) ? 1:0;
}

/** @brief Test whether an SSP has data available in the receive FIFO.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                  1 if the receive FIFO has data available, 0 otherwise.
  */
__INLINE static unsigned int SSP_RxIsAvailable(SSP_Type *ssp)
{
    return (ssp->SR & SSP_RNE) ? 1:0;
}

/** @brief Test whether an SSP's receive FIFO is full.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                  1 if the receive FIFO is full, 0 otherwise.
  */
__INLINE static unsigned int SSP_RxFIFOIsFull(SSP_Type *ssp)
{
    return (ssp->SR & SSP_RFF) ? 1:0;
}

/** @brief Test whether an SSP is currently busy transferring data.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                  1 if the SSP is busy, 0 otherwise.
  */
__INLINE static unsigned int SSP_IsBusy(SSP_Type *ssp)
{
    return (ssp->SR & SSP_BSY) ? 1:0;
}

/** @brief Enable specific interrupts on an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  * @param[in]  it_mask      A bitmask of SSP interrupts to enable
  */
__INLINE static void SSP_EnableIT(SSP_Type *ssp, uint32_t it_mask)
{
    lpclib_assert((it_mask & ~SSP_ITMask_Mask) == 0);

    ssp->IMSC |= it_mask;
}

/** @brief Disable specific interrupts on an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  * @param[in]  it_mask      A bitmask of SSP interrupts to disable
  */
__INLINE static void SSP_DisableIT(SSP_Type *ssp, uint32_t it_mask)
{
    lpclib_assert((it_mask & ~SSP_ITMask_Mask) == 0);

    ssp->IMSC &= ~it_mask;
}

/** @brief Get a bitmask of the enabled interrupts on an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                  A bitmask of the SSP's enabled interrupts.
  */
__INLINE static SSP_ITMask_Type SSP_GetEnabledIT(SSP_Type *ssp)
{
    return ssp->IMSC;
}

/** @brief Get a bitmask of interrupts currently pending on an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                  A bitmask of the SSP's currently pending interrupts.
  */
__INLINE static SSP_ITMask_Type SSP_GetPendingIT(SSP_Type *ssp)
{
    return ssp->MIS;
}

/** @brief Clear currently pending interrupts on an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  * @param[in]  it_mask      A bitmask of pending SSP interrupts to clear
  *
  * @note
  * Only SSP_IT_RxOverrun and SSP_IT_RxTimer interrupts can be cleared (others
  * are wired to the FIFO status).
  */
__INLINE static void SSP_ClearPendingIT(SSP_Type *ssp, uint32_t it_mask)
{
    lpclib_assert((it_mask & ~(SSP_ITMask_RxOverrun | SSP_ITMask_RxTimer)) == 0);

    ssp->ICR |= it_mask;
}

/** @brief Set the operating mode (master or slave or input-only slave) of an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  * @param[in]  mode         The new operating mode
  */
__INLINE static void SSP_SetMode(SSP_Type *ssp, SSP_Mode_Type mode)
{
    lpclib_assert(SSP_IS_MODE(mode));

    ssp->CR1 = (ssp->CR1 & ~SSP_CR1_MODE_Mask) | mode;
}

/** @brief Get the operating mode (master or slave or input-only slave) of an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                  The current operating mode of the SSP.
  */
__INLINE static SSP_Mode_Type SSP_GetMode(SSP_Type *ssp)
{
    return ssp->CR1 & SSP_CR1_MODE_Mask;
}

/** @brief Set an SSP's word length.
  * @param[in]  ssp          A pointer to the SSP instance
  * @param[in]  length       The new SSP transfer word length
  */
__INLINE static void SSP_SetWordLength(SSP_Type *ssp, SSP_WordLength_Type length)
{
    lpclib_assert(SSP_IS_WORDLENGTH(length));

    ssp->CR0 = (ssp->CR0 & ~SSP_DSS_Mask) | length;
}

/** @brief Get an SSP's current word length.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                  The SSP's current word length.
  */
__INLINE static SSP_WordLength_Type SSP_GetWordLength(SSP_Type *ssp)
{
    return ssp->CR0 & SSP_DSS_Mask;
}

/** @brief Set the format of frames transferred on an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  * @param[in]  format       The new format for SSP frames
  */
__INLINE static void SSP_SetFrameFormat(SSP_Type *ssp, SSP_FrameFormat_Type format)
{
    lpclib_assert(SSP_IS_FRAMEFORMAT(format));

    ssp->CR0 = (ssp->CR0 & ~SSP_FRF_Mask) | format;
}

/** @brief Get the current format of frames transferred on an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                  The current format of SSP frames.
  */
__INLINE static SSP_FrameFormat_Type SSP_GetFrameFormat(SSP_Type *ssp)
{
    return ssp->CR0 & SSP_FRF_Mask;
}

/** @brief Set the idle-state polarity of an SSP's clock line.
  * @param[in]  ssp          A pointer to the SSP instance
  * @param[in]  polarity     The new idle-stat clock line polarity
  */
__INLINE static void SSP_SetClockPolarity(SSP_Type *ssp, SSP_ClockPolarity_Type polarity)
{
    lpclib_assert(SSP_IS_CLOCKPOLARITY(polarity));

    ssp->CR0 = (ssp->CR0 & ~SSP_CPOL) | polarity;
}

/** @brief Get the idle-state polarity of an SSP's clock line.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                  The SSP's current idle-state clock line polarity.
  */
__INLINE static SSP_ClockPolarity_Type SSP_GetClockPolarity(SSP_Type *ssp)
{
    return ssp->CR0 & SSP_CPOL;
}

/** @brief Set the clock line phase on which an SSP latches data.
  * @param[in]  ssp          A pointer to the SSP instance
  * @param[in]  phase        The new clock line phase on which to latch data
  */
__INLINE static void SSP_SetClockPhase(SSP_Type *ssp, SSP_ClockPhase_Type phase)
{
    lpclib_assert(SSP_IS_CLOCKPHASE(polarity));

    ssp->CR0 = (ssp->CR0 & ~SSP_CPHA) | phase;
}

/** @brief Get the current clock line phase on which an SSP latches data.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                  The clock line phase on which the SSP latches data.
  */
__INLINE static SSP_ClockPhase_Type SSP_SetClockPhase(SSP_Type *ssp)
{
    return ssp->CR0 & SSP_CPHA;
}

/** @brief Set the number of prescaler ticks per bit transferred on an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  * @param[in]  ticks        The new number of prescaler ticks per bit
  *
  * @note
  * - This function takes the actual # of prescaler counts, not the (off-by-1) SCR value
  * - Final bit frequency = PCLK / (Prescale_Divisor * Count).
  */
__INLINE static void SSP_SetPrescalerTicksPerBit(SSP_Type *ssp, unsigned int ticks)
{
    lpclib_assert((ticks > 0) && (ticks <= 256));

    ssp->CR0 = (ssp->CR0 & ~SSP_SCR_Mask) | ((ticks - 1) << SSP_SCR_Shift);
}

/** @brief Set the current number of prescaler ticks per bit transferred on an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                  The current number of SSP prescaler counts per bit.
  *
  * @note
  * - This function returns the actual # of prescaler counts, not the (off-by-1) SCR value
  */
__INLINE static unsigned int SSP_GetPrescalerTicksPerBit(SSP_Type *ssp)
{
    return ((ssp->CR0 & SSP_SCR_Mask) >> SSP_SCR_Shift) + 1;
}

/** @brief Set the input clock prescaler value on an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  * @param[in]  prescaler    The new prescaler value (An even number; 2 <= n <= 254)
  */
__INLINE static void SSP_SetClockPrescaler(SSP_Type *ssp, unsigned int prescaler)
{
    lpclib_assert(prescaler <= 254);
    lpclib_assert((prescaler & 1) == 0);

    ssp->CPSR = prescaler;
}

/** @brief Get the current input clock prescaler value on an SSP.
  * @param[in]  ssp          A pointer to the SSP instance
  * @return                  The SSP's current prescaler value.
  */
__INLINE static unsigned int SSP_GetClockPrescaler(SSP_Type *ssp)
{
    return ssp->CPSR;
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
