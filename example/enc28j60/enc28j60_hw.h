/******************************************************************************
 * @file:    enc28j60_hw.h
 * @purpose: Hardware interface header file for ENC28J60 test application.
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    15. January 2012
 * @license: Simplified BSD License
 ******************************************************************************
 *
 * This version is for interfacing the ENC28J60 with an NXP LPC11xx series
 * Microcontroller.
 *
 ******************************************************************************
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
 * THIS SOFTWARE IS PROVIDED BY TIMOTHY TWILLMAN ''AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL <COPYRIGHT HOLDER> ORCONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, ORCONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of Timothy Twilllman.
 *****************************************************************************/

#ifndef ENC28J60_HW_H_
#define ENC28J60_HW_H_

/* Includes -----------------------------------------------------------------*/

#include "lpc11xx.h"
#include "lpc11xx_iocon.h"
#include "lpc11xx_gpio.h"
#include "lpc11xx_ssp.h"


/* Defines ------------------------------------------------------------------*/

/* GPIO definitions for control of chip select line on the ENC28J60 */
#define ENC28J60_CS_PORT       GPIO0
#define ENC28J60_CS_PIN        GPIO_Pin_7
#define ENC28J60_CS_PINCONFIG  IOCON_PinConfig_0_7_PIO

/* SPI (SSP) interface used for communication with the ENC28J60 */
#define ENC28J60_SSP           SSP0


/* Inline Functions ---------------------------------------------------------*/

/** @brief Send/Receive 1 byte to & from the ENC28J60 Ethernet Controller
  * @param  b     Byte to send via SSP
  * @return Byte received via SSP when bval was sent.
  */
static __INLINE uint8_t enc28j60_xfer(uint8_t b)
{
    return SSP_Xfer(ENC28J60_SSP, b);
}


/** @brief Write a block of data to the ENC28J60 Ethernet Controller
  * @param  buf   Data to write to the device
  * @param  len   Number of bytes in data to write
  * @return None.
  */
static __INLINE void enc28j60_writeblock(const uint8_t *buf, uint16_t len)
{
    while (len--) {
        SSP_Xfer(*buf++);
    }
}


/** @brief Read a block of data from the ENC28J60 Ethernet Controller
  * @param  buf   Data to read from the device
  * @param  len   Number of bytes of data to read
  * @return None.
  */
static __INLINE void enc28j60_readblock(uint8_t *buf, uint16_t len)
{
    while (len--) {
        *buf++ = SSP_Xfer(0);
    }
}


/** @brief Select the ENC28J60 Ethernet Controller (set CS line low)
  * @return None.
  */
static __INLINE void enc28j60_select(void)
{
    GPIO_ClearPins(ENC28J60_CS_PORT, ENC28J60_CS_PIN);
}


/** @brief De-select the ENC28J60 Ethernet Controller (set CS line high)
  * @return None.
  */
static __INLINE void enc28j60_deselect(void)
{
    GPIO_SetPins(ENC28J60_CS_PORT, ENC28J60_CS_PIN);
}


/** @brief Perform necessary hardware IO initialization for the ENC28J60 device
  * @return 0 on success, a negative value on failure
  *
  * NOTE: This version always succeeds & returns 0.
  */
static __INLINE int enc28j60_hw_init(void)
{
    /* Set up Chip Select pin; the SPI interface is not set up here, and
     * needs to be initialized by the core application.
     */
    IOCON_SetPinConfig(ENC28J60_CS_PINCONFIG, IOCON_Mode_Normal);
    GPIO_SetPinDirections(ENC28J60_CS_PORT, ENC28J60_CS_PIN, GPIO_Direction_Out);

    /* Specifically deselect the ethernet chip. */
    enc27j60_deselect();

    return 0;
}

#endif /* #ifndef ENC28J60_HW_H_ */
