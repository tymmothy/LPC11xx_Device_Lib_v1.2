/******************************************************************************
 * @file:    system_lpc11xx.h
 * @purpose: Header File for low-level fuctions for LPC11xx CPUs
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    1. June 2010
 *****************************************************************************/
 
#ifndef SYSTEM_LPC11XX_H_
#define SYSTEM_LPC11XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Exported Variables ---------------------------------------------------------------------------*/

/** @defgroup LPC11xx_System_Variables Core Variables for LPC11xx MCUs
  * @{
  */
  
extern uint32_t SystemCoreClock;                           /*!< Speed of MCU Core Clock          */
extern uint32_t SystemAHBClock;                            /*!< Speed of AHB Bus                 */

/**
  * @}
  */


/* Exported Functions ---------------------------------------------------------------------------*/

/** @defgroup LPC11xx_System_Functions Low-level System Functions for LPC11xx MCUs
  * @{
  */

/** @brief  Set up system clocks & low-level hardware.
  *
  * @return None.
  *
  * Called on bring-up of CPU for hardware configuration.
  */
extern void SystemInit(void);

/** @brief  Update SystemCoreClock and SystemAHBClock variables to match current clock config
  *
  * @return None.
  *
  * Checks system clocking registers to determine current CPU core clock speed.
  */
extern void SystemCoreClockUpdate(void);

/**
  * @}
  */

#ifdef __cplusplus
};
#endif

#endif /* #ifndef SYSTEM_LPC11XX_H_ */
