/**************************************************************************//**
 * @file     iap.h
 * @brief    Self-Programming Interface Header for NXP LPC Microcontrollers
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. January 2012
 ******************************************************************************
 * @section Overview
 * This file gives an abstract interface to NXP LPC Flash Programming via
 * the IAP (In-Application Programming) interface.
 *
 * @note
 * - During Flash programming, interrupts are not disabled by the IAP
 *   interface.
 * - IAP write/erase calls use 32 bytes @ the top of RAM
 * - Flash is not accessible during a write/erase
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

#ifndef NXP_LPC_IAP_H_
#define NXP_LPC_IAP_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "lpc11xx.h"
#include "system_lpc11xx.h"
#include "lpclib_assert.h"


/**
  * @defgroup IAP_AbstractionLayer IAP (Flash Programming) Abstraction Layer
  * @ingroup  LPC_Peripheral_AbstractionLayer
  * @{
  */

/* Defines ------------------------------------------------------------------*/

/**
  * @defgroup IAP_Definitions IAP Interface Definitions
  * @{
  */

/* NOTE: Clock speed needed for FLASH Erase / Write Operations */
#define IAP_KHz()            (SystemCoreClock / 1024) /*!< System Frequency in KHz               */
#define IAP_ENTRY_POINT      (0x1fff1ff1UL)           /*!< Location of IAP entry function in ROM */
#define IAP_SECTOR_SIZE      (4096)                   /*!< Flash Sector Size                     */
#define IAP_SECTOR_Shift     (12)                     /*!< Number of bits in sector page addrs   */

/**
  * @}
  */


/* Types & Type-Related Definitions -----------------------------------------*/

/**
  * @defgroup IAP_Types IAP Interface Types and Type-Related Definitions
  * @{
  */

/** @defgroup IAP_Commands Command bytes for calling IAP functions
  * @{
  */

/*! @brief IAP Command Byte Values */
typedef enum {
    IAP_Command_PrepareSectors = 50,                       /*!< Prepare Sectors for Erase/Write  */
    IAP_Command_CopyRamToFlash,                            /*!< Copy Data from RAM to FLASH      */
    IAP_Command_EraseSectors,                              /*!< Erase FLASH Sectors              */
    IAP_Command_BlankCheckSectors,                         /*!< Blank Check FLASH Sectors        */
    IAP_Command_ReadPartID,                                /*!< Read the CPU's Part ID           */
    IAP_Command_ReadBootCodeVersion,                       /*!< Read the CPU's Boot Code Version */
    IAP_Command_Compare,                                   /*!< Compare Data in RAM/FLASH        */
    IAP_Command_ReinvokeISP,                               /*!< Jump to InSystem Programming ROM */
} IAP_CommandType;

/*! @brief Macro to test whether parameter is a valid IAP Command Byte */
#define IAP_IS_COMMAND_TYPE(Command) (((Command) >= IAP_Command_PrepareSectors) \
                                   && ((Command) <= IAP_Command_ReinvokeISP))
/** @} */

/** @defgroup IAP_Status_Codes Return values from IAP ROM routines
  * @{
  */

/*! @brief IAP Status Return Codes */
typedef enum {
    IAP_Status_Success = 0,                                /*!< IAP command succeeded            */
    IAP_Status_InvalidCommand,                             /*!< Invalid IAP command byte         */
    IAP_Status_SourceAddressError,                         /*!< Invalid source address           */
    IAP_Status_DestinationAddressError,                    /*!< Invalid destination address      */
    IAP_Status_SourceAddressNotMappedError,                /*!< Source address not accessible    */
    IAP_Status_DestinationAddressNotMappedError,           /*!< Dest address not accessible      */
    IAP_Status_CountError,                                 /*!< Bad byte count                   */
    IAP_Status_InvalidSectorError,                         /*!< Invalid flash sector             */
    IAP_Status_SectorNotBlankError,                        /*!< Flash sector not blank           */
    IAP_Status_SectorNotPreparedError,                     /*!< Flash sector not prepared        */
    IAP_Status_CompareError,                               /*!< Comparison failed                */
    IAP_Status_Busy,                                       /*!< Flash hardware busy              */
    IAP_Status_ParameterError,                             /*!< Problem with IAP parameters      */
    IAP_Status_AddressError,                               /*!< Bad address                      */
    IAP_Status_AddressNotMappedError,                      /*!< Address not accessible           */
    IAP_Status_CommandLockedError,                         /*!< Command locked                   */
    IAP_Status_InvalidCodeError,                           /*!< Problem with code                */
    IAP_Status_InvalidBaudRateError,                       /*!< Bad requested baud rate          */
    IAP_Status_InvalidStopBitError,                        /*!< Bad requested stop bit setting   */
    IAP_Status_CodeReadProtectedError,                     /*!< CRP prevents requested access    */
} IAP_StatusType;

/** @} */

/** @defgroup IAP_Write_Erase_Valid_Byte_Counts Valid Byte Counts for IAP Write / Erase Commands
  * @{
  */

/*! @brief IAP Write / Erase Valid Byte Counts.
 *  These are broken out specifically to make it clear when sending IAP
 *  commands that only a narrow range of values are allowed.
 */
typedef enum {
    IAP_ByteCount_256                  =  256,             /*!<  256 Bytes                       */
    IAP_ByteCount_512                  =  512,             /*!<  512 Bytes                       */
    IAP_ByteCount_1024                 = 1024,             /*!< 1024 Bytes                       */
    IAP_ByteCount_4096                 = 4096              /*!< 4096 Bytes                       */
} IAP_ByteCountType;

/*! @brief Macro to test whether parameter is a valid byte count for IAP write / erase commands. */
#define IAP_IS_BYTE_COUNT_TYPE(ByteCount) (((ByteCount) == IAP_ByteCount_256) \
                                       || ((ByteCount) == IAP_ByteCount_512)  \
                                       || ((ByteCount) == IAP_ByteCount_1024) \
                                       || ((ByteCount) == IAP_ByteCount_4096))

/** @} */

/**
  * @}
  */


/* Exported Functions -------------------------------------------------------*/

/** @defgroup IAP_ExportedFunctions IAP Interface Exported Functions
  * @{
  */

/** @brief Call into the IAP system
  * @param[in]  command      The IAP Command to Execute
  * @param[in]  p0           First argument to pass to IAP  (call-dependent)
  * @param[in]  p1           Second argument to pass to IAP (call-dependent)
  * @param[in]  p2           Third argument to pass to IAP  (call-dependent)
  * @param[in]  p3           Fourth argument to pass to IAP (call-dependent)
  * @param[out] result       Pointer to location to write any results passed back from IAP
  * @return                  An IAP Status Code giving status of operation.
  *
  * Result can be set to NULL if no result is necessary.
  */
IAP_StatusType IAP_Call(IAP_CommandType command, uint32_t p0, uint32_t p1,
                        uint32_t p2, uint32_t p3, uint32_t *result);

/** @} */


/* Inline Functions ---------------------------------------------------------*/

/** @defgroup IAP_InlineFunctions IAP Interface Inline Functions
  * @{
  */

/** @brief Prepare Flash sectors for erase or write operations.
  * @param[in]  start_addr   The (sector-aligned) location in memory to start preparing
  * @param[in]  nsectors     The number of sectors to prepare
  * @return                  0 on success, negative IAP status value on error.
  */
static __INLINE int IAP_PrepareSectors(uint32_t start_addr, uint32_t nsectors)
{
    int ret;

    lpclib_assert((start_addr & (IAP_SECTOR_SIZE - 1)) == 0);

    ret = IAP_Call(IAP_Command_PrepareSectors, (start_addr >> IAP_SECTOR_Shift),
                  (start_addr >> IAP_SECTOR_Shift) + (nsectors - 1), 0, 0, 0);
    return -ret;
}

/**
  * @brief Copy data from RAM to Flash pages.
  * @param[in]  dest_addr    The (256 byte-aligned) location in Flash to copy to
  * @param[in]  src_addr     The (word-aligned) location in memory to end copy
  * @param[in]  nbytes       The number of bytes to copy (should be 256/512/1024/4096)
  * @return                  0 on success, negative IAP status value on error.
  */
static __INLINE int IAP_CopyRamToFlash(uint32_t dest_addr, uint32_t src_addr,
                                       IAP_ByteCountType nbytes)
{
    int ret;

    lpclib_assert((dest_addr & 0xff) == 0);
    lpclib_assert((src_addr & 0xff) == 0);
    lpclib_assert(IAP_IS_BYTE_COUNT_TYPE(nbytes));

    ret = IAP_Call(IAP_Command_CopyRamToFlash, dest_addr, src_addr,
                   nbytes, IAP_KHz(), 0);
    return -ret;
}

/** @brief Erase Flash pages.
  * @param[in]  start_addr   The (sector-aligned) location in memory to start erase
  * @param[in]  nsectors     The number of sectors to erase
  * @return                  0 on success, negative IAP status value on error.
  *
  * @note
  * IAP_PrepareSectors must be called on the sectors that are going to be
  * erased before this function is called.
  */
static __INLINE int IAP_EraseSectors(uint32_t start_addr, uint32_t nsectors)
{
    int ret;


    lpclib_assert((start_addr & (IAP_SECTOR_SIZE - 1)) == 0);

    ret = IAP_Call(IAP_Command_EraseSectors, (start_addr >> IAP_SECTOR_Shift),
                   (start_addr >> IAP_SECTOR_Shift) + (nsectors - 1), IAP_KHz(), 0, 0);

    return -ret;
}

/** @brief Blank-check Flash memory.
  * @param[in]   start_addr  The (sector-aligned) location in memory to start check
  * @param[in]   nsectors    The number of sectors to blank-check
  * @param[out]  nb_offset   The offset of the first non-blank location
  * @return                  0 on success, negative IAP status value on error.
  */
static __INLINE int IAP_BlankCheckSectors(uint32_t start_addr,
                                 uint32_t nsectors, uint32_t *nb_offset)
{
    int ret;


    lpclib_assert((start_addr & (IAP_SECTOR_SIZE -1)) == 0);

    ret = IAP_Call(IAP_Command_BlankCheckSectors, (start_addr >> IAP_SECTOR_Shift),
                   (start_addr >> IAP_SECTOR_Shift) + (nsectors - 1), 0, 0, nb_offset);
    return -ret;
}

/** @brief Read the MCU's Part ID.
  * @param[out]  id          The returned MCU's part ID
  * @return                  0 on success, negative IAP status value on error.
  */
static __INLINE int IAP_ReadPartID(uint32_t *id)
{
    int ret;


    ret = IAP_Call(IAP_Command_ReadPartID, 0, 0, 0, 0, id);

    return -ret;
}

/** @brief Read the MCU's boot code version.
  * @param[out] version      The returned boot code version
  * @return                  0 on success, negative IAP status value on error.
  */
static __INLINE int IAP_ReadBootCodeVersion(uint32_t *version)
{
    int ret;


    ret = IAP_Call(IAP_Command_ReadBootCodeVersion, 0, 0, 0, 0, version);

    return -ret;
}

/** @brief Compare RAM or Flash contents, returning offset of first difference.
  * @param[in]  addr1        The (word-aligned) location of first segment to compare
  * @param[in]  addr2        The (word-aligned) location of second segment to compare
  * @param[in]  nbytes       The number of bytes to compare (a multiple of 4 bytes)
  * @param[out] diff_offset  The offset of the first difference between segments
  * @return                  0 on success, negative IAP status value on error.
  */
static __INLINE int IAP_Compare(uint32_t addr1, uint32_t addr2,
                              uint32_t nbytes, uint32_t *diff_offset)
{
    int ret;


    lpclib_assert((nbytes & 0x03) == 0);
    lpclib_assert((addr1 & 0x03) == 0);
    lpclib_assert((addr2 & 0x03) == 0);

    ret = IAP_Call(IAP_Command_Compare, addr1, addr2, nbytes, 0, diff_offset);

    return -ret;
}

/** @brief Execute the microcontroller's in-system programming function.
  */
static __INLINE void IAP_ReinvokeISP(void) __attribute__((noreturn));
static __INLINE void IAP_ReinvokeISP(void)
{
    IAP_Call(IAP_Command_ReinvokeISP, 0, 0, 0, 0, 0);
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

#endif /* #ifndef NXP_LPC_IAP_H_ */
