/****************************************************************************
 * @file:    lpclib_assert.h
 * @purpose: Header file for microcontrololer assert debugging
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    1. June 2010
 *--------------------------------------------------------------------------
 *
 * lpclib_assert() is a wrapper that calls lpclib_assert_failed()
 * on an assertion failure (when the passed in value is 0), if library
 * debugging is enabled (LPCLIB_DEBUG is defined); otherwise it does
 * nothing.
 *
 * lpc_lib_assert_failed() disables interrupts and goes into an infinite
 * loop to make it relatively easy to track down problems with a debugger.
 *
 * To use: Define LPCLIB_DEBUG when compiling & using the library.
 *
 ****************************************************************************/

#ifndef LPCLIB_ASSERT_H_
#define LPCLIB_ASSERT_H_


/* Defines -----------------------------------------------------------------*/

/** @defgroup LIB_DEBUG Library Debugging Interface
  * @{
  */

/*! Function wrapper that is only used when LPC_LIB_DEBUG is set */
#ifdef LPCLIB_DEBUG
# define lpclib_assert(x) (if ((x) == 0) { lpclib_assert_failed(); } )
#else
# define lpclib_assert(x) do {} while(0);
#endif


/* Exported Functions ------------------------------------------------------*/

/** @brief  Infinite loop function to trap assertion failures.
  * @return Does not return (infinite loop)
  */
void lpclib_assert_failed(void);

/**
  * @}
  */


#endif /* #ifndef LPCLIB_ASSERT_H_ */
