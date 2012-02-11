/*****************************************************************************
 * @file:    rom_canopen.h
 * @purpose: Rom-based CAN Open driver interface for LPC11CXX Microcontrollers
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    1. January 2012
 * @license: Simplified BSD License
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

#ifndef ROM_CANOPEN_H_
#define ROM_CANOPEN_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes -----------------------------------------------------------------*/

#include <stdint.h>
#include "lpc11xx.h"
#include "lpclib_assert.h"


/**
  * @defgroup CANopenDriver_Access_Interface ROM-based CANopen Driver Access-level Interface
  * @ingroup  ROM
  * @{
  */

/* Defines ------------------------------------------------------------------*/

/** @defgroup CANopenDriver_Definitions CANopen Driver Definitions
  * @{
  */

#define CANOPEN_ENTRY_POINT     (0x1fff2000UL)     /*!< CANopen Driver entry location in ROM */

!!!Uses On-chip RAM from address 0x1000 0050 to 0x1000 00B8

/* Types --------------------------------------------------------------------*/

typedef struct can_driver {
    void (*init_can)(uint32_t * can_cfg);
    void (*isr)(void);
    void (*config_rxmsgobj)(CAN_MSG_OBJ * msg_obj);
    uint8_t (*can_receive)(CAN_MSG_OBJ * msg_obj);
    void (*can_transmit)(CAN_MSG_OBJ * msg_obj);
    void (*config_canopen)(CAN_CANOPENCFG * canopen_cfg);
    void (*canopen_handler)(void);
    void (*config_calb)(CAN_CALLBACKS * callback_cfg);
} CANDriver_Type;

typedef struct can_callbacks {
    void (*CAN_rx)(uint8_t msg_obj);
    void (*CAN_tx)(uint8_t msg_obj);
    void (*CAN_error)(uint32_t error_info);
    uint32_t (*CANOPEN_sdo_read)(uint16_t index, uint8_t subindex);
    uint32_t (*CANOPEN_sdo_write)(uint16_t index, uint8_t subindex, uint8_t *dat_ptr);
    uint32_t (*CANOPEN_sdo_seg_read)(uint16_t index, uint8_t subindex, uint8_t openclose,
                                     uint8_t *length, uint8_t *data, uint8_t *last);
    uint32_t (*CANOPEN_sdo_seg_write)(uint16_t index, uint8_t subindex, uint8_t openclose,
                                      uint8_t length, uint8_t *data, uint8_t *fast_resp);
    uint8_t (*CANOPEN_sdo_req)(uint8_t length_req, uint8_t *req_ptr, uint8_t *length_resp,
                               uint8_t *resp_ptr);
} CANDriver_Callbacks_Type;

#endif /* #ifndef ROM_CANOPEN_H_ */
