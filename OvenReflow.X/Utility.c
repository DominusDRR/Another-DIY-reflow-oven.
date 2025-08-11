/*******************************************************************************
 CLOCK PLIB

  Company:
    Sicoy Ltda.

  File Name:
    Utility.c

  Summary:
    File of useful functions in the project

  Description:
    None
 * 
 * Author: Fabian Romo

*******************************************************************************/
#include "definitions.h"

//Gets the unsigned 32-bit absolute value
uint32_t abs_diff_uint32(uint32_t a, uint32_t b)
{
    return (a > b)? (a - b):(b - a);
} 
