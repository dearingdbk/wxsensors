/*
 * File:     sensor_utils.c
 * Author:   Bruce Dearing
 * Date:     26/11/2025
 * Version:  1.0
 * Purpose:  Program to handle setting up a serial connection and two threads
 *           one to listen, and one to respond over RS-485 / RS-422
 *           Two-thread serial handler:
 *            - Receiver thread parses and responds to commands
 *            - Sender thread periodically transmits data when active
 * Mods:
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
//#include <errno.h>
//#include <termios.h>
//#include <pthread.h>
//#include <sys/ioctl.h>
//#include <linux/serial.h>
#include <regex.h>
#include <signal.h>
//#include <stdatomic.h>
//#include <stdarg.h>
//#include <time.h>
#include "sensor_utils.h"

char units_of_measure[25][50]; // Global array to hold the units of measurement available.
double coefficients[57]; // Global array to hold K coefficients of float values.


/*typedef struct {
    char *unit_type[MAX_INPUT_STR];
    char *serial_number[MAX_INPUT_STR];
    char *style[MAX_INPUT_STR];
    char *min_pressure[MAX_INPUT_STR];
    char *max_pressure[MAX_INPUT_STR];
    char *manufacture_date[MAX_INPUT_STR];
    char *software_version[MAX_INPUT_STR];
    int trans_interval[MAX_INPUT_STR];
    char *units_sent[MAX_INPUT_STR];
    int measurement_speed[MAX_INPUT_STR];
    char *filter_factor[MAX_INPUT_STR];
    char *filter_step[MAX_INPUT_STR];
    char *user_message[MAX_INPUT_STR];
    char *units[MAX_INPUT_STR];
    char *pin_set[MAX_INPUT_STR];
    char *user_zero[MAX_INPUT_STR];
    char *user_fs[MAX_INPUT_STR];
    char *sensor_sn[MAX_INPUT_STR];
    char *internal_chksum[MAX_INPUT_STR];
} bp_sensor; */

/*
 * Name:         init_units
 * Purpose:      Initializes a global array of measurement units.
 * Arguments:    None
 *
 * Output:       None.
 * Modifies:     units_of_measure array.
 * Returns:      None.
 * Assumptions:  units_of_measure has been allocated.
 *
 * Bugs:         None known.
 * Notes:
 */
void init_units() {
    snprintf(units_of_measure[0], 50, "mbar");
    snprintf(units_of_measure[1], 50, "Pa");
    snprintf(units_of_measure[2], 50, "kPa");
    snprintf(units_of_measure[3], 50, "MPa");
    snprintf(units_of_measure[4], 50, "hPa");
    snprintf(units_of_measure[5], 50, "bar");
    snprintf(units_of_measure[6], 50, "kg/cm\u00B2");
    snprintf(units_of_measure[7], 50, "kg/m\u00B2");
    snprintf(units_of_measure[8], 50, "mmHg");
    snprintf(units_of_measure[9], 50, "cmHg");
    snprintf(units_of_measure[10], 50, "mHg");
    snprintf(units_of_measure[11], 50, "mmH\u2082O");
    snprintf(units_of_measure[12], 50, "cmH\u2082O");
    snprintf(units_of_measure[13], 50, "mH\u2082O");
    snprintf(units_of_measure[14], 50, "torr");
    snprintf(units_of_measure[15], 50, "atm");
    snprintf(units_of_measure[16], 50, "psi");
    snprintf(units_of_measure[17], 50, "lb/ft\u2082");
    snprintf(units_of_measure[18], 50, "inHg");
    snprintf(units_of_measure[19], 50, "inH\u2082O4\u00B0C");
    snprintf(units_of_measure[20], 50, "ftH\u2082O4\u00B0C");
    snprintf(units_of_measure[21], 50, "mbar");
    snprintf(units_of_measure[22], 50, "inH\u2082O20\u00B0C");
    snprintf(units_of_measure[23], 50, "ftH\u2082O20\u00B0C");
    snprintf(units_of_measure[24], 50, "mbar");
}
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
    coefficients[0] = 9.173625E+02;
    coefficients[1] = -8.654275E-02;
    coefficients[2] = 3.705644E-05;
    coefficients[3] = 9.173625E+02;
    coefficients[4] = 9.173625E+02;
    coefficients[5] = 9.173625E+02;
    coefficients[6] = 9.173625E+02;
    coefficients[7] = 9.173625E+02;
    coefficients[8] = 9.173625E+02;
    coefficients[9] = 9.173625E+02;
    coefficients[10] = 9.173625E+02;
    coefficients[11] = 9.173625E+02;
    coefficients[12] = 9.173625E+02;
    coefficients[13] = 9.173625E+02;
    coefficients[14] = 9.173625E+02;
    coefficients[15] = 9.173625E+02;
    coefficients[16] = 9.173625E+02;
    coefficients[17] = 9.173625E+02;
    coefficients[18] = 9.173625E+02;
    coefficients[19] = 9.173625E+02;
    coefficients[20] = 9.173625E+02;
    coefficients[21] = 9.173625E+02;
    coefficients[22] = 9.173625E+02;
    coefficients[23] = 9.173625E+02;
    coefficients[24] = 9.173625E+02;
    coefficients[25] = 9.173625E+02;
    coefficients[26] = 9.173625E+02;
    coefficients[27] = 9.173625E+02;
    coefficients[28] = 9.173625E+02;
    coefficients[29] = 9.173625E+02;
    coefficients[30] = 9.173625E+02;
    coefficients[31] = 9.173625E+02;
    coefficients[32] = 9.173625E+02;
    coefficients[33] = 9.173625E+02;
    coefficients[34] = 9.173625E+02;
    coefficients[35] = 9.173625E+02;
    coefficients[36] = 9.173625E+02;
    coefficients[37] = 9.173625E+02;
    coefficients[38] = 9.173625E+02;
    coefficients[39] = 9.173625E+02;
    coefficients[40] = 9.173625E+02;
    coefficients[41] = 9.173625E+02;
    coefficients[42] = 9.173625E+02;
    coefficients[43] = 9.173625E+02;
    coefficients[44] = 9.173625E+02;
    coefficients[45] = 9.173625E+02;
    coefficients[46] = 9.173625E+02;
    coefficients[47] = 9.173625E+02;
    coefficients[48] = 9.173625E+02;
    coefficients[49] = 9.173625E+02;
    coefficients[50] = 9.173625E+02;
    coefficients[51] = 9.173625E+02;
    coefficients[52] = 9.173625E+02;
    coefficients[53] = 9.173625E+02;
    coefficients[54] = 9.173625E+02;
    coefficients[55] = 9.173625E+02;
    coefficients[56] = 9.173625E+02;
    coefficients[57] = 9.173625E+02;
}


/*
 * Name:         init_sensor
 * Purpose:      Initializes a sensor.
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
    snprintf((*ptr)->units, 50, units_of_measure[current_u_of_m]); // sets default unit of measurement to mbar.
    snprintf((*ptr)->pin_set, 50, "000"); // sets the default pin to 000.
    return 1;
}


/*
 * Name:         update_message
 * Purpose:      Captures any kill signals, and sets volitile bool 'terminate' and 'kill_flag' to true, allowing thw while loop to break, and threads to join.
 * Arguments:    None
 *
 * Output:       None.
 * Modifies:     Changes terminate to true.
 * Returns:      None.
 * Assumptions:  Terminate is set to false.
 *
 * Bugs:         None known.
 * Notes:        Signal handler: must do async-safe ops only (set sig_atomic_t flags)
 */
int update_message(bp_sensor **ptr, char *msg) {
    snprintf((*ptr)->user_message, 16, "%s", msg); // sets the user message to msg.

    return 1;

}

int update_units(bp_sensor **ptr, uint8_t unit_id) {
    if (unit_id < 25) {
        snprintf((*ptr)->units, 50, "%s", units_of_measure[unit_id]); // sets default unit of measurement to unit ID.
        return 1;
    }
    return -1;
}
