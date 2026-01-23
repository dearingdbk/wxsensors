/*
 * File:     atmosvue30_utils.c
 * Author:   Bruce Dearing
 * Date:     26/11/2025
 * Version:  1.0
 * Purpose:  Program to declare helper functions for sensor initialization for barometric sensor emulation.
 *
 * Mods:
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <regex.h>
#include <time.h>
#include <ctype.h>
#include "atmosvue30_utils.h"



#define MAX_TOKENS 32
#define DT_STRING 7



/*
 * Name:         get_pressure_units_text
 * Purpose:      gets matching text name of pressure units used.
 * Arguments:    ptr - a pointer to a bp_sensor struct.
 *
 * Output:       None.
 * Modifies:     None.
 * Returns:      mbar, or the associated text value of the unit value provided.
 * Assumptions:
 *
 * Bugs:         None known.
 * Notes:
 */


