/******************************************************************************
 * @file:    lpc11xx_pll.c
 * @purpose: Utility functions for using PLL's on NXP LPC microcontrollers.
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    3. January 2012
 *****************************************************************************/

/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "lpc11xx.h"
#include "lpc11xx/syscon.h"


/* Functions ----------------------------------------------------------------*/

/** @brief  Calculate the scaler value to load into a PLL's config register
  * @param  [in]  clk_in  PLL input frequency, in Hz
  * @param  [in]  clk_out Desired output PLL frequency, in Hz
  * @param  [out] err     The actual output PLL freq. using the returned value
  *
  * @return Value to load into PLL's config register for given output frequency
  *
  * err can be (null) in which case it will not be filled in.
  */
int16_t PLL_ScalerCalc(uint32_t clk_in, uint32_t clk_out, int32_t *err)
{
    uint32_t actual;
    uint32_t m;
    uint32_t p;


    /* 9,750,000 Hz is lowest valid setting for this PLL.
     * 160,000,000 Hz is highest (though obviously beyond CPU capabilities)
     */

	/* Calculate m (multiplier) value -- round to closest possible
	 *  output value to that requested
	 */
    m = (clk_out + (clk_in / 2)) / clk_in;
    
    /* If requested value is out of bounds, just get as close as possible... */
    if (m < 1) {
    	m = 1;
    } else if (m > 0x20) {
        m = 0x20;
    }
    
    /* Determine what the actual output frequency will be with the
     *  calculated m value
     */
    actual = m * clk_in;

	/* Decide which PLL P divisor to select based on output clock frequency */
    if (actual < 19500000UL) {
        p = SYSCON_SYSPLLCTRL_PSEL_DIV8;
    } else if (actual < 39000000UL) {
        p = SYSCON_SYSPLLCTRL_PSEL_DIV4;
    } else if (actual < 78000000UL) {
        p = SYSCON_SYSPLLCTRL_PSEL_DIV2;
    } else {
        return -1;
    }

	/* If a place was passed in to give the output error, fill it in */
    if (err) {
        *err = (actual - clk_out);
    }

	/* Munge the p/m values into the register setting value */
    return ((p << 5) | (m - 1));
}
