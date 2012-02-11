/******************************************************************************
 * @file:    lpc11xx_crp.c
 * @purpose: Code Read Protection interface for NXP LPC Microcontrollers
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    3. January 2012
 *****************************************************************************/

/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "lpc11xx/crp.h"


/* Defines ------------------------------------------------------------------*/

/* Default to disabling code protection. */

#ifndef CRP_Val
# define CRP_Val (CRP_DISABLE)
#endif


/* Globals ------------------------------------------------------------------*/

/*! @brief Code Protection Setting */
__attribute__ ((section(".crp"))) const uint32_t CodeProtection = (uint32_t)CRP_Val;
