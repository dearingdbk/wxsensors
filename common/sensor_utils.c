/*
 * File:     sensor_utils.c
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
#include <string.h>
#include <regex.h>
#include <time.h>
#include <ctype.h>
#include "sensor_utils.h"

#define MAX_TOKENS 32
#define DT_STRING 7


/*
 * Name:         init_wind
 * Purpose:      Initializes a wind sensor.
 * Arguments:    ptr - a pointer to a wind_sensor struct.
 *
 * Output:       Error message if unable to allocate memory for the struct.
 * Modifies:     Updates the default values of provided sensor.
 * Returns:      -1 on failure, 1 on success.
 * Assumptions:
 *
 * Bugs:         None known.
 * Notes:        int snprintf(char *str, size_t size, const char *format, ...);
 */
int init_wind(wind_sensor **ptr) {
    *ptr = malloc(sizeof(wind_sensor));
    if (*ptr == NULL) {
        perror("Failed to allocate cur_sensor");
        return -1;
    }
    (*ptr)->a_val = 0; // sets the A value of configuration to 0 N/A
    (*ptr)->b_val = 3; // BAUD sets the B value of configuration to 3 - 9600
    (*ptr)->c_val = 1; // Wraparound sets the C value of configuration to 1 - 0 to 539 degrees
    (*ptr)->e_val = 1; // duplex sets the E value of configuration to 1 - Full Duplex
    (*ptr)->f_val = 1; // 8N1 sets the F value of configuration to 1 - 8 bits, no parity, 1 stop bit
    (*ptr)->g_val = 0; // Averaging sets the G value of configuration to 0000  (0000 to 3600)
    (*ptr)->h_val = 2; // Heating sets the H value of configuration to 2 - Activated
    (*ptr)->j_val = 0; // sets the J value of configuration to 0 N/A
    (*ptr)->k_val = 1; // NMEA sets the K value of configuration to 1 - NMEA String "IIMWV"
    (*ptr)->l_val = 1; // ASCII terminator - sets the L value of configuration to 1 - CR LF
    (*ptr)->m_val = 2; // Message Format sets the M value of configuration to 2 - ASCII Polar Continuous
    (*ptr)->n_val = 'A'; // Unit Identifier Address sets the N value of configuration to A
    (*ptr)->o_val = 1; // ASCII Output format sets the O value of configuration to 1 - CSV
    (*ptr)->p_val = 1; // Output rate sets the P value of configuration to 1 - 1 output per second
    (*ptr)->t_val = 0; // sets the T value of configuration to 0 N/A
    (*ptr)->u_val = 1; // Digital Output Units sets the U value of configuration to 1 - metres per second
    (*ptr)->v_val = 1; // Vertical Padding sets the V value of configuration to 1 - disable vertical padding
    (*ptr)->x_val = 1; // 45 degree alignment sets the X value of configuration to 1 - aligns U axis with north south
    (*ptr)->y_val = 1; // sets the Y value of configuration to 1
    (*ptr)->z_val = 1; // sets the Z value of configuration to 1

    return 1;
}


/*
 * Name:         get_wind_units
 * Purpose:      takes the integer stored in the struct value u_val and returns a char representing that value.
 * Arguments:    chk_val the int value stored within u_val.
 *
 * Output:
 * Modifies:     Updates the default values of provided sensor.
 * Returns:      the char related to the units of measure.
 * Assumptions:
 *
 * Bugs:         None known.
 * Notes:
 */

char get_wind_units(int chk_val) {
	switch (chk_val) {
		case 1: return 'M'; // Return Metres per Second
		case 2: return 'N'; // Return Knots
		case 3: return 'P'; // Return Miles per hour
		case 4: return 'K'; // Return Kilometres per hour
		case 5: return 'F'; // Return feet per minute
		default: return 'M'; // Return default M
	}
}
