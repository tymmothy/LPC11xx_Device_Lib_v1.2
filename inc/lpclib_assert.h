/**************************************************************************//**
 * @file     lpclib_assert.h
 * @brief    Header file for library assert debugging
 * @version  V1.0
 * @author   Tymm Twillman
 * @date     1. January 2012
 ******************************************************************************
 * @section Overview
 * This file gives an interface to an assert function that can be used during
 * library debugging.  When enabled, any use of lpclib_assert() where the
 * passed in parameter evaluates to 0 will cause the program to jump into
 * an infinite loop.  This can be helpful in finding bugs in the library
 * code.  When disabled, lpclib_assert() does nothing.
 *
 * To enable the assert checking, define the symbol LPCLIB_DEBUG when
 * compiling the library, and when compiling any applications that use
 * the library.
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

#ifndef LPCLIB_ASSERT_H_
#define LPCLIB_ASSERT_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup LIB_DEBUG Library Debugging Interface
  * @{
  */

/* Defines ------------------------------------------------------------------*/

/*! assert()-like function wrapper used when LPC_LIB_DEBUG is set */
#ifdef LPCLIB_DEBUG
# define lpclib_assert(x) if ((x) == 0) { lpclib_assert_failed(); }
#else
# define lpclib_assert(x) do {} while(0)
#endif


/* Exported Functions -------------------------------------------------------*/

/** @brief  Infinite loop function to trap assertion failures.
  * @return Does not return (infinite loop).
  */
extern void lpclib_assert_failed(void);

/**
  * @}
  */


#ifdef __cplusplus
};
#endif

#endif /* #ifndef LPCLIB_ASSERT_H_ */
