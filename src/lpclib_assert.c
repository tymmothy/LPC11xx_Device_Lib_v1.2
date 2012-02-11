/******************************************************************************
 * @file:    lpclib_assert.c
 * @purpose: "assertion failed" function for library debugging
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    31. December 2011
 ******************************************************************************/

/* Includes -----------------------------------------------------------------*/

#include "lpc11xx.h"
#include "lpclib_assert.h"

#include "lpc11xx/wdt.h"


/* Functions ----------------------------------------------------------------*/

/** @brief  Infinite loop function for library debugging
  * @return Does not return (infinite loop)
  *
  * This is declared weak for easy overriding by user-defined functions.
  * It can be set to tickle the watchdog to make failures easier to track down.
  */
void lpclib_assert_failed(void) __attribute__((weak,noreturn));
void lpclib_assert_failed(void)
{
    __disable_irq();
    
    while(1) {

#ifdef LPCLIB_ASSERT_TICKLE_WATCHDOG
        WDT_Reset();
#endif
    }
}
