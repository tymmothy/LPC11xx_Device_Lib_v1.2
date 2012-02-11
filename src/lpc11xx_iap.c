/*****************************************************************************
 * @file:    iap.c
 * @purpose: Self-Programming Interface for NXP LPC Microcontrollers
 * @version: V1.0
 * @author:  Tymm Twillman
 * @date:    1. January 2012
 *****************************************************************************/

/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "lpc11xx/iap.h"


/* File-local Types ---------------------------------------------------------*/

/*! Type used for calling IAP function */
typedef void (*IAP_Type)(uint32_t [], uint32_t []);


/* Static Variables ---------------------------------------------------------*/

/*! (ROM) Function For Calling IAP Routines */
static const IAP_Type iap_entry = (IAP_Type)IAP_ENTRY_POINT;


/* Functions ----------------------------------------------------------------*/

/** @brief  Call into the IAP system
  * @param  Command    The IAP Command to Execute
  * @param  p0         First argument to pass to IAP (call-dependent)
  * @param  p1         Second argument to pass to IAP (call-dependent)
  * @param  p3         Third argument to pass to IAP (call-dependent)
  * @param  p4         Fourth argument to pass to IAP (call-dependent)
  * @param  Result     Location to write any results passed back from IAP
  * @return            An IAP Status Code giving status of operation.
  *
  * Result can be set to NULL if no result is necessary.
  */
IAP_StatusType IAP_Call(IAP_CommandType Command, uint32_t p0, uint32_t p1, 
                      uint32_t p2, uint32_t p3, uint32_t *Result)
{
    uint32_t command[5];
    uint32_t result[4];


    command[0] = Command;
    command[1] = p0;
    command[2] = p1;
    command[3] = p2;
    command[4] = p3;

    iap_entry(command, result);

    if (Result) {
        *Result = result[1];
    }

    return result[0];
}
