/*
 * File:     WO75_utils.c
 * Author:   Bruce Dearing
 * Date:     16/01/2026
 * Purpose:  Implementation of wind observer 75 specific logic.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "crc_utils.h"
#include "windobserver75_utils.h"


/*
 * Name:         init_WO75_sensor
 * Purpose:      Allocates memory for a WO75 sensor structure and initializes
 * 				 all members (serial, baud, modules, etc.) to default factory values.
 * Arguments:    ptr - A pointer to a pointer of type WO75_sensor, used to
 * 				 return the address of the allocated memory.
 *
 * Output:       An allocated and populated WO75_sensor structure.
 * Modifies:     Allocates memory on the heap and updates the provided pointer.
 * Returns:      0 on success, -1 if memory allocation fails.
 * Assumptions:  The provided ptr is a valid address of a pointer.
 *
 * Bugs:         None known.
 * Notes:        Uses CLOCK_MONOTONIC for thread timing and UTC (gmtime) for
 * 				 the initial date string.
 *				 Must be freed by the caller.
 */
int init_WO75_sensor(WO75_sensor **ptr) {
    *ptr = malloc(sizeof(WO75_sensor));
    if (!*ptr) return -1;
    WO75_sensor *s = *ptr;
	// Identity
    s->address = 'A';
	// Configuration
    s->mode = SMODE_M2;
	s->units = NUM_TO_UNITS[1]; // U1-U5 default is U1 metres per second.
    s->output_rate = HZ_TO_NANOSECONDS[4]; // 1-10 Hz 4 is the default 0.25 seconds.

	// Timing
	time_t now;
	time(&now); // Get our current epoch time.
	gmtime_r(&now, &s->sensor_time); // Store current epoch time in our tm struct.
	clock_gettime(CLOCK_MONOTONIC, &s->last_send_time); // timespec time, for when the sensor sent last message.
	clock_gettime(CLOCK_MONOTONIC, &s->sensor_start_time); // timespec time for when the sensor initialized.
	s->initialized = true;
    return 0;
}

/*
 * Name:         WO75_is_ready_to_send
 * Purpose:      Determines if the required time interval has elapsed since the
 * 				 last data transmission based on the sensor's configuration.
 * Arguments:    sensor - Pointer to the skyvue8_sensor structure containing
 * 				 mode and timing data.
 *
 * Output:       None.
 * Modifies:     None.
 * Returns:      true if the sensor is in RUN mode and the interval has passed;
 * 				 false otherwise.
 * Assumptions:  sensor->last_send_time was initialized with CLOCK_MONOTONIC.
 *
 * Bugs:         None known.
 * Notes:        Uses CLOCK_MONOTONIC to ensure timing remains consistent even
 * 				 if the system real-time clock is adjusted.
 */
bool WO75_is_ready_to_send(WO75_sensor *sensor) {
    if (!sensor) return false;
	// Because this specific Gill Windobserver 75 sensor operates on sub-second timing, we have removed the checks for last-send time on all continuous modes.
    if (sensor->mode == SMODE_M1) return true;
    if (sensor->mode == SMODE_M2) return true;
    if (sensor->mode == SMODE_M5) return true;
    return false;
}


/*
 * Name:         check_sum
 * Purpose:      Takes a '\0' delimited string, and returns a checksum of the characters XOR.
 * Arguments:    str_to_chk the string that checksum will be calculated for
 *
 * Output:       None.
 * Modifies:     None.
 * Returns:      returns an unsigned 8 bit integer of the checksum of str_to_chk.
 * Assumptions:  Terminate is set to false.
 *
 * Bugs:         None known.
 * Notes:        To print in HEX utilize dprintf(serial_fd, "%c%s%c%02X\r\n",2, str_to_chk, check_sum(str_to_chk));
 */
uint8_t check_sum(const char *str_to_chk) {

    uint8_t checksum = 0;
    if (str_to_chk == NULL) {
        return 0;
    }
    while (*str_to_chk != '\0') {
        checksum ^= (uint8_t)(*str_to_chk);
        str_to_chk++;
    }
    return checksum;
}

