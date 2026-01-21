/*
 * File:     barometric_utils.c
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
#include "barometric_utils.h"



#define MAX_TOKENS 32
#define DT_STRING 7
char units_of_measure[25][50]; // Global array to hold the units of measurement available.
double coefficients[57]; // Global array to hold K coefficients of float values.


/*
 * Name:         init_coefficients
 * Purpose:      Initializes a global array of coefficients.
 * Arguments:    None
 *
 * Output:       None.
 * Modifies:     coefficients array of floats.
 * Returns:      None.
 * Assumptions:  coefficients has been allocated.
 *
 * Bugs:         None known.
 * Notes:
 */
void init_coefficients() {
    coefficients[0] = 9.173625E+02; coefficients[1] = -8.654275E-02; coefficients[2] = 3.705644E-05;
    coefficients[3] = 9.173625E+02; coefficients[4] = 9.173625E+02; coefficients[5] = 9.173625E+02;
    coefficients[6] = 9.173625E+02; coefficients[7] = 9.173625E+02; coefficients[8] = 9.173625E+02;
    coefficients[9] = 9.173625E+02; coefficients[10] = 9.173625E+02; coefficients[11] = 9.173625E+02;
    coefficients[12] = 9.173625E+02; coefficients[13] = 9.173625E+02; coefficients[14] = 9.173625E+02;
    coefficients[15] = 9.173625E+02; coefficients[16] = 9.173625E+02; coefficients[17] = 9.173625E+02;
    coefficients[18] = 9.173625E+02; coefficients[19] = 9.173625E+02; coefficients[20] = 9.173625E+02;
    coefficients[21] = 9.173625E+02; coefficients[22] = 9.173625E+02; coefficients[23] = 9.173625E+02;
    coefficients[24] = 9.173625E+02; coefficients[25] = 9.173625E+02; coefficients[26] = 9.173625E+02;
    coefficients[27] = 9.173625E+02; coefficients[28] = 9.173625E+02; coefficients[29] = 9.173625E+02;
    coefficients[30] = 9.173625E+02; coefficients[31] = 9.173625E+02; coefficients[32] = 9.173625E+02;
    coefficients[33] = 9.173625E+02; coefficients[34] = 9.173625E+02; coefficients[35] = 9.173625E+02;
    coefficients[36] = 9.173625E+02; coefficients[37] = 9.173625E+02; coefficients[38] = 9.173625E+02;
    coefficients[39] = 9.173625E+02; coefficients[40] = 9.173625E+02; coefficients[41] = 9.173625E+02;
    coefficients[42] = 9.173625E+02; coefficients[43] = 9.173625E+02; coefficients[44] = 9.173625E+02;
    coefficients[45] = 9.173625E+02; coefficients[46] = 9.173625E+02; coefficients[47] = 9.173625E+02;
    coefficients[48] = 9.173625E+02; coefficients[49] = 9.173625E+02; coefficients[50] = 9.173625E+02;
    coefficients[51] = 9.173625E+02; coefficients[52] = 9.173625E+02; coefficients[53] = 9.173625E+02;
    coefficients[54] = 9.173625E+02; coefficients[55] = 9.173625E+02; coefficients[56] = 9.173625E+02;
    coefficients[57] = 9.173625E+02;
}



/*
 * Name:         init_sensor
 * Purpose:      Initializes a barometric sensor.
 * Arguments:    ptr - a pointer to a bp_sensor struct.
 *
 * Output:       Error message if unable to allocate memory for the struct.
 * Modifies:     Updates the default values of provided sensor.
 * Returns:      -1 on failure, 1 on success.
 * Assumptions:  current_u_of_m, is an extern defined global declared in barometric.c, and is set to 6 'hPa'.
 *
 * Bugs:         None known.
 * Notes:
 */
int init_sensor(bp_sensor **ptr) {
    *ptr = malloc(sizeof(bp_sensor));
    if (*ptr == NULL) {
        perror("Failed to allocate cur_sensor");
        return -1;
    }
    //snprintf((*ptr)->units, 50, units_of_measure[current_u_of_m]); // sets default unit of measurement to hPa.
    //snprintf((*ptr)->pin_set, 50, "000"); // sets the default pin to 000.
    snprintf((*ptr)->model_number, MAX_MODEL_NUM - 1, "%s", "DPS8100"); // sets the default model number.
    snprintf((*ptr)->user_message, MAX_MSG_STR - 1, "%s", ""); // sets the default user message to empty string '\0'.
    (*ptr)->min_pressure = 0.0f;
    (*ptr)->max_pressure = 1100.0f;
	(*ptr)->pressure_units = 4;  // 0-24, see unit codes, default is hPa.
	(*ptr)->sensor_type = 0;  // 0 gauge 1 absolute.
	(*ptr)->device_address = 0; // This is direct mode, our main, will re-address these sensors.
    (*ptr)->filter_number = 0; // sets the defualt filter factor to 0.
	(*ptr)->filter_prescaler = 0; // sets the defualt filter pre-scaler to 0.
	(*ptr)->transmission_interval = 1.0f; // sets the update rate to 1 reading/second
    (*ptr)->output_format = 1; // sets the defualt format to 1 R.
    (*ptr)->baud_rate = 9600; // Default to 9600
    (*ptr)->parity = 'N'; // I, N, O, E
    (*ptr)->data_bits = 8; // Default to 8
    (*ptr)->stop_bits = 1; // 1 or 2.
	(*ptr)->wait_interval = 22; // Number of chars trasmitted at 9600 baud for each sensor to wait.
	(*ptr)->term_chars = 1; // 1 or 2.
    (*ptr)->user_gain = 1.0f; // default 1.0f.
    (*ptr)->user_offset = 0.0f; // default 0.0f
    (*ptr)->slope = 0.0f; // ??
    (*ptr)->set_point = 0.0f; //
    (*ptr)->pin = 0; //
    (*ptr)->pin_set = false; //


    // Initialize the timer
    clock_gettime(CLOCK_MONOTONIC, &((*ptr)->last_send_time));

    return 1;
}

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
const char* get_pressure_units_text(uint8_t code) {
    static const char *units[] = {
        "mbar", "Pa", "kPa", "MPa", "hPa", "bar",
        "kg/cm2", "kg/m2", "mmHg", "cmHg", "mHg",
        "mmH2O", "cmH2O", "mH2O", "torr", "atm",
        "psi", "lb/ft2", "inHg", "inH2O4C", "ftH2O4C",
        "mbar", "inH2O20C", "ftH2O20C", "mbar"
    };
    if (code < 25) return units[code];
    return "mbar";
}

/*
 * Name:         update_message
 * Purpose:      Updates the user message of the provided bp_sensor, with msg.
 * Arguments:    None
 *
 * Output:       None.
 * Modifies:     None.
 * Returns:      1
 * Assumptions:  None.
 *
 * Bugs:         None known.
 * Notes:        None.
 */
int update_message(bp_sensor **ptr, char *msg) {
    snprintf((*ptr)->user_message, 16, "%s", msg); // sets the user message to msg.

    return 1;

}

/*
 * Name:         update_units
 * Purpose:      Updates the units varible of the provided bp_sensor struct with the corresponding value from unit_id.
 * Arguments:    None
 *
 * Output:       None.
 * Modifies:     The units value of the provided bp_sensor.
 * Returns:      -1 on failure, 1 on success.
 * Assumptions:
 *
 * Bugs:         None known.
 * Notes:
 */
int update_units(bp_sensor **ptr, uint8_t unit_id) {
    if (unit_id < 25) {
        (*ptr)->pressure_units = unit_id; // sets default unit of measurement to unit ID.
        return 1;
    }
    return -1;
}


/*
 * Name:         is_ready_to_send
 * Purpose:      checks if a sensor is within its window to send data.
 * Arguments:    None
 *
 * Output:       None.
 * Modifies:     The units value of the provided bp_sensor.
 * Returns:      true if sensor is in the window, false if not enough time has gone by.
 * Assumptions:
 *
 * Bugs:         None known.
 * Notes:
 */
bool is_ready_to_send(bp_sensor *s) {
    if (s == NULL || s->transmission_interval <= 0.0f) return false;

    struct timespec now;
    // CLOCK_MONOTONIC is immune to system time changes (DST, NTP updates)
    clock_gettime(CLOCK_MONOTONIC, &now);

    // Calculate the difference in seconds
    double elapsed = (double)(now.tv_sec - s->last_send_time.tv_sec) +
                     (double)(now.tv_nsec - s->last_send_time.tv_nsec) / 1.0e9;

    return (elapsed >= (double)s->transmission_interval);
}
