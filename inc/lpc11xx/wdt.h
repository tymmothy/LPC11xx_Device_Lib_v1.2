/****************************************************************************
 * @file:    wdt.h
 * @purpose: Watchdog Timer Interface Header for NXP LPC Microcontrollers
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    1. June 2010
 ****************************************************************************/

#ifndef LPC_WDT_H_
#define LPC_WDT_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ----------------------------------------------------------------*/

#include <stdint.h>
#include "lpc11xx.h"


/**
  * @defgroup WDT_Access_Interface Watchdog Timer Access-level Interface
  * @ingroup  LPC_Peripheral_Access_Layer
  * @{
  */

/* Types & Type-Related Definitions -----------------------------------------*/

/** @defgroup WDT_Access_Types WDT Access-level Interface Types & Definitions
  * @{
  */

/** @defgroup WDT_Feed_Values Watchdog Timer Feed Register Values
  * @{
  */

#define WDT_Feed_A                     (0xAA)              /*!< First "feed" value to update WDT */
#define WDT_Feed_B                     (0x55)              /*!< 2nd   "feed" value to update WDT */

/** @} */

/**
  * @}
  */

/* Inline Functions ---------------------------------------------------------*/

/** @defgroup WDT_Inline_Functions WDT Access-level Inline Functions
  * @{
  */

/** @brief  Enable the Watchdog Timer (cannot be cleared w/o reset)
  * @return None.
  */
__INLINE static void WDT_Enable(void)
{
    WDT->MOD |= WDT_WDEN;
}

/** @brief  Determine Whether the Watchdog Timer is Enabled
  * @return 1 if the WDT is enabled, 0 otherwise
  */
__INLINE static uint32_t WDT_IsEnabled(void)
{
    return (WDT->MOD & WDT_WDEN) ? 1:0;
}

/** @brief  Enable WDT Resetting the Chip (cannot be cleared w/o reset)
  * @return None.
  */
__INLINE static void WDT_EnableChipReset(void)
{
   WDT->MOD |= WDT_WDRESET;
}

/** @brief  Determine Whether the Watchdog Chip Reset is Enabled
  * @return 1 if the WDT Chip Reset is enabled, 0 otherwise
  */
__INLINE static uint32_t WDT_ChipResetIsEnabled(void)
{
    return (WDT->MOD & WDT_WDRESET) ? 1:0;
}

/** @brief Check Whether the Watchdog Timer has Timed Out
  * @return 1 if the timer has timed out, 0 otherwise.
  */
__INLINE static uint8_t WDT_TimedOut(void)
{
    return (WDT->MOD & WDT_WDTOF) ? 1:0;
}

/** @brief Check Whether the Watchdog Timer Interrupt is Pending
  * @return 1 if the timer interrupt is pending, 0 otherwise
  */
__INLINE static uint8_t WDT_ITIsPending(void)
{
    return (WDT->MOD & WDT_WDINT) ? 1:0;
}

/** @brief Get the Current Value of the Watchdog Timer
  * @return Current value of the watchdog timer
  */
__INLINE static uint32_t WDT_GetCurrentValue(void)
{
    return WDT->TV;
}

/** @brief Feed the Watchdog Timer (load the timer constant)
  * @return None.
  *
  * This is necessary to load the current value in the Timer Constant
  *  register as the watchdog's Timer Reload value.  This is also necessary
  *  to start the watchdog, after it's been enabled, and enable changes to
  *  WDT settings.
  */
__INLINE static void WDT_Feed(void)
{
    uint32_t primask;

    __asm__ __volatile__ ("    mrs    %0, primask":"=r"(primask));

    __disable_irq();
    WDT->FEED = WDT_Feed_A;
    WDT->FEED = WDT_Feed_B;
    __asm__ __volatile__ ("    msr    primask, %0"::"r"(primask));
}

/** @brief Set the Watchdog's Timeout Constant (value that gets reloaded)
  * @param  Timeout  Number of watchdog ticks before the watchdog times out
  * @return None.
  */
__INLINE static void WDT_SetTimeout(uint32_t Timeout)
{
    WDT->TC = Timeout;
}

/** @brief Get the Reload Value of the Watchdog Timer
  * @return Reload value of the timer
  */
__INLINE static uint32_t WDT_GetTimeout(void)
{
    return WDT->TC;
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

#endif /* #ifndef LPC_WDT_H_ */
