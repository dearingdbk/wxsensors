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
char units_of_measure[25][50]; // Global array to hold the units of measurement available.
double coefficients[57]; // Global array to hold K coefficients of float values.


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
    snprintf((*ptr)->units, 50, units_of_measure[current_u_of_m]); // sets default unit of measurement to hPa.
    snprintf((*ptr)->pin_set, 50, "000"); // sets the default pin to 000.
    snprintf((*ptr)->user_message, 16, "%s", ""); // sets the default user message to empty string '\0'.
    (*ptr)->trans_interval = 1; // sets the update rate to 1 reading/second
    (*ptr)->filter_factor = 0; // sets the defualt filter factor to 0.
    return 1;
}


/*
 * Name:         init_flash
 * Purpose:      Initializes a BTD-300 lightning sensor.
 * Arguments:    ptr - a pointer to a flash_sensor struct.
 *
 * Output:       Error message if unable to allocate memory for the struct.
 * Modifies:     Updates the default values of provided sensor.
 * Returns:      -1 on failure, 1 on success.
 * Assumptions:  current_u_of_m, is an extern defined global declared in barometric.c, and is set to 6 'hPa'.
 *
 * Bugs:         None known.
 * Notes:        int snprintf(char *str, size_t size, const char *format, ...);
 */
int init_flash(flash_sensor **ptr) {
    *ptr = malloc(sizeof(flash_sensor));
    if (*ptr == NULL) {
        perror("Failed to allocate cur_sensor");
        return -1;
    }
    snprintf((*ptr)->serial_num, 13, "000008675309"); // sets the serial number fo the sensor.
    (*ptr)->overhead = 926; // sets the overhead lightning limit to 5 NM or 926 decametres
    (*ptr)->vicinity = 1852; // sets the overhead lightning limit to 10 NM or 1852 decametres
    (*ptr)->near_distant = 3704; // sets the overhead lightning limit to 20 NM or 3704 decametres
    (*ptr)->far_distant = 5556; // sets the overhead lightning limit to 30 NM or 5556 decametres

    return 1;
}


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

/*
 * Name:         reset_flash
 * Purpose:      resets a BTD-300 lightning sensor.
 * Arguments:    ptr - a pointer to a flash_sensor struct.
 *
 * Output:       Error message if unable to allocate memory for the struct.
 * Modifies:     Updates the default values of provided sensor.
 * Returns:      -1 on failure, 1 on success.
 * Assumptions:
 *
 * Bugs:         None known.
 * Notes:        int snprintf(char *str, size_t size, const char *format, ...);
 */
int reset_flash(flash_sensor **ptr) {
    // *ptr = malloc(sizeof(flash_sensor));
    if (*ptr == NULL) {
        perror("Failed to allocate cur_sensor");
        return -1;
    }
    (*ptr)->overhead = 926; // sets the overhead lightning limit to 5 NM or 926 decametres
    (*ptr)->vicinity = 1852; // sets the overhead lightning limit to 10 NM or 1852 decametres
    (*ptr)->near_distant = 3704; // sets the overhead lightning limit to 20 NM or 3704 decametres
    (*ptr)->far_distant = 5556; // sets the overhead lightning limit to 30 NM or 5556 decametres

    return 1;
}
/*
 * Name:         set_dist
 * Purpose:      sets a BTD-300 lightning sensor.
 * Arguments:    ptr - a pointer to a flash_sensor struct.
 *
 * Output:       Error message if unable to allocate memory for the struct.
 * Modifies:     Updates the default values of provided sensor.
 * Returns:      -1 on failure, 1 on success.
 * Assumptions:  "DISTx,yyyy" Set Distance Limits x == 0-OH, 1-V, 2-ND, 3-FD. yyyy == decametres
 *
 * Bugs:         None known.
 * Notes:        int snprintf(char *str, size_t size, const char *format, ...);
 */
int set_dist(flash_sensor **ptr, const char *buf) {
    if (*ptr == NULL) {
        perror("Failed to allocate cur_sensor");
        return -1;
    }

	if (buf == NULL) return -1;
    int key = (isdigit(buf[4]) == 0) ? buf[4] - '0' : 99;
	char distance[5]; // Space for 4 digits + null terminator

	strncpy(distance, &buf[6], 4);
	distance[5] = '\0';

	uint16_t value = (uint16_t)strtoul(distance, NULL, 10);

	switch(key) {
		case 0:
    		(*ptr)->overhead = value; // sets the overhead lightning limit to 'value'
			break;
		case 1:
		    (*ptr)->vicinity = value; // sets the overhead lightning limit to 'value'
			break;
		case 2:
		    (*ptr)->near_distant = value; // sets the overhead lightning limit to 'value'
			break;
		case 3:
		    (*ptr)->far_distant = value; // sets the overhead lightning limit to 'value'
			break;
		default:
			// 99 or another bad number, do nothing.
			break;
	}

    return 1;
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
        snprintf((*ptr)->units, 50, "%s", units_of_measure[unit_id]); // sets default unit of measurement to unit ID.
        return 1;
    }
    return -1;
}


/*
 * Name:         parse_btd_datetime
 * Purpose:      Converts BTD date/time strings (DDMMYY, HHMMSS) to time_t
 * Arguments:    date_str - "DDMMYY" format string
 *               time_str - "HHMMSS" format string
 *
 * Returns:      time_t value, or -1 on error
 */
time_t parse_btd_datetime(const char *date_str, const char *time_str) {
    struct tm tm_info = {0};
    uint8_t day, month, year, hour, min, sec;

    if (!date_str || !time_str) {
        return -1;
    }

    if (strlen(date_str) != 6 || strlen(time_str) != 6) {
        return -1;
    }
    // Use %hhu for unsigned 8-bit integers to prevent sign-flip errors
    if (sscanf(date_str, "%2hhu%2hhu%2hhu", &day, &month, &year) != 3) { // hhu for half-half-word i.e. a char or uint8_t unsigned.
        return -1;
    }
    if (sscanf(time_str, "%2hhu%2hhu%2hhu", &hour, &min, &sec) != 3) {
        return -1;
    }

    /* Validate ranges */
    if (day < 1 || day > 31 || month < 1 || month > 12) {
        return -1;
    }
    if (hour > 23 || min > 59 || sec > 59) {
        return -1;
    }

    tm_info.tm_mday = day;
    tm_info.tm_mon = month - 1;
    tm_info.tm_year = year + 100; // tm_year uses 1900 as the baseline, anything over year 2000 must be 100 + the year if using 2 digits.
    tm_info.tm_hour = hour;
    tm_info.tm_min = min;
    tm_info.tm_sec = sec;
    tm_info.tm_isdst = -1;

    return mktime(&tm_info);
}

/*
 * Name:         format_btd_datetime
 * Purpose:      Formats a time_t value into BTD date/time strings
 * Arguments:    t - time_t value to format
 *               date_str - output buffer for "DDMMYY" (min 7 bytes)
 *               time_str - output buffer for "HHMMSS" (min 7 bytes)
 *
 * Returns:      0 on success, -1 on error
 */
int format_btd_datetime(time_t t, char *date_str, char *time_str) {
    struct tm *tm_info;

    tm_info = localtime(&t);
    if (!tm_info) {
        return -1;
    }
    // Use %hhu for unsigned 8-bit integers to prevent sign-flip errors
    snprintf(date_str, DT_STRING, "%02u%02u%02u",
             tm_info->tm_mday, tm_info->tm_mon + 1, tm_info->tm_year % 100);
    snprintf(time_str, DT_STRING, "%02u%02u%02u",
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);

    return 0;
}



/*
 * Name:         update_btd_timestamps
 * Purpose:      Parses a BTD-300 DATA message and updates all date/time fields
 *               to current time while preserving the relative time deltas between
 *               the system timestamp and each flash timestamp.
 * Arguments:    input  - the original DATA message line from file
 *               output - buffer to store the updated message
 *               output_size - size of output buffer
 *
 * Output:       None.
 * Modifies:     Writes updated message to output buffer.
 * Returns:      0 on success, -1 on error.
 * Assumptions:  Input is a valid BTD-300 DATA message format.
 *
 * Bugs:         None known.
 * Notes:        Token positions (0-indexed):
 *               - 2,3: system date/time (reference)
 *               - 4: flash count (0-4)
 *               - 8,9: flash 1 date/time
 *               - 13,14: flash 2 date/time
 *               - 18,19: flash 3 date/time
 *               - 23,24: flash 4 date/time
 *
 *               Flash timestamps are only updated if flash_count > 0.
 *               Only updates the number of flashes indicated by field 4.
 */
int update_btd_timestamps(const char *input, char *output, size_t output_size) {
    char *tokens[MAX_TOKENS];
    char *input_copy = NULL;
    char *token = NULL;
    char *saveptr = NULL;
    int token_count = 0;
    int flash_count = 0;
    int i;

    // Flash timestamp positions (date_pos, time_pos) - each flash block is 5 tokens
    const int flash_date_pos[] = {8, 13, 18, 23};
    const int flash_time_pos[] = {9, 14, 19, 24};

    time_t now;
    time_t original_system_time;
    time_t original_flash_time;
    time_t new_flash_time;
    double delta;

    char new_date[8];
    char new_time[8];

    if (!input || !output || output_size == 0) {
        return -1;
    }

    // Make a copy of input for tokenization
    input_copy = strdup(input);
    if (!input_copy) {
        return -1;
    }

    // Tokenize by comma
    token = strtok_r(input_copy, ",", &saveptr);
    while (token != NULL && token_count < MAX_TOKENS) {
        tokens[token_count] = strdup(token);
        if (!tokens[token_count]) {
            for (i = 0; i < token_count; i++) {
                free(tokens[i]);
            }
            free(input_copy);
            return -1;
        }
        token_count++;
        token = strtok_r(NULL, ",", &saveptr);
    }
    free(input_copy);

    /* Need at least: DATA:, ID, date, time, flash_count (positions 0-4) */
    if (token_count < 5) {
        for (i = 0; i < token_count; i++) {
            free(tokens[i]);
        }
        return -1;
    }

    /* Get flash count from position 4 */
    flash_count = atoi(tokens[4]);
    if (flash_count < 0 || flash_count > 4) {
        flash_count = 0; /* Invalid count, assume no flashes */
    }

    /* Parse original system timestamp (positions 2,3) */
    original_system_time = parse_btd_datetime(tokens[2], tokens[3]);
    if (original_system_time == -1) {
        for (i = 0; i < token_count; i++) {
            free(tokens[i]);
        }
        return -1;
    }

    /* Get current time */
    now = time(NULL);

    /* Update system timestamp to current time */
    if (format_btd_datetime(now, new_date, new_time) == 0) {
        free(tokens[2]);
        free(tokens[3]);
        tokens[2] = strdup(new_date);
        tokens[3] = strdup(new_time);
    }

    /* Only process flash timestamps if there are flashes */
    for (i = 0; i < flash_count; i++) {
        int date_pos = flash_date_pos[i];
        int time_pos = flash_time_pos[i];

        /* Check if this flash exists in the message */
        if (date_pos >= token_count || time_pos >= token_count) {
            break;
        }

        /* Parse original flash time */
        original_flash_time = parse_btd_datetime(tokens[date_pos], tokens[time_pos]);
        if (original_flash_time == -1) {
            continue; /* Skip if parse fails */
        }

        /* Calculate delta (flash time - system time) in seconds */
        delta = difftime(original_flash_time, original_system_time);

        /* Apply delta to current time */
        new_flash_time = now + (time_t)delta;

        /* Format and replace tokens */
        if (format_btd_datetime(new_flash_time, new_date, new_time) == 0) {
            free(tokens[date_pos]);
            free(tokens[time_pos]);
            tokens[date_pos] = strdup(new_date);
            tokens[time_pos] = strdup(new_time);
        }
    }

    /* Reconstruct the message */
    output[0] = '\0';
    for (i = 0; i < token_count; i++) {
        if (i > 0) {
            strncat(output, ",", output_size - strlen(output) - 1);
        }
        strncat(output, tokens[i], output_size - strlen(output) - 1);
    }

    /* Cleanup */
    for (i = 0; i < token_count; i++) {
        free(tokens[i]);
    }

    return 0;
}

