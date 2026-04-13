/*
 * File:     ptb330_utils.c
 * Author:   Bruce Dearing
 * Date:     16/01/2026
 * Purpose:  Implementation of PTB330-specific logic.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "crc_utils.h"
#include "skyvue8_utils.h"


/*
 * Name:         init_skyvue8_sensor
 * Purpose:      Allocates memory for a PTB330 sensor structure and initializes
 * 				 all members (serial, baud, modules, etc.) to default factory values.
 * Arguments:    ptr - A pointer to a pointer of type ptb330_sensor, used to
 * 				 return the address of the allocated memory.
 *
 * Output:       An allocated and populated ptb330_sensor structure.
 * Modifies:     Allocates memory on the heap and updates the provided pointer.
 * Returns:      0 on success, -1 if memory allocation fails.
 * Assumptions:  The provided ptr is a valid address of a pointer.
 *
 * Bugs:         None known.
 * Notes:        Uses CLOCK_MONOTONIC for thread timing and UTC (gmtime) for
 * 				 the initial date string.
 *				 Must be freed by the caller.
 */
int init_skyvue8_sensor(skyvue8_sensor **ptr) {
    *ptr = malloc(sizeof(skyvue8_sensor));
    if (!*ptr) return -1;

    skyvue8_sensor *s = *ptr;
	// Identity
	strncpy(s->serial_number, "SN1000", MAX_SN_LEN);
    strncpy(s->software_version, "004", 4);
    s->address = '0';
	// Configuration
    s->mode = SMODE_POLL;
	s->baud = 10; // 115200 default.
	s->data_f = 8;
	s->parity = 'N';
	s->stop_b = 1;
	s->initialized = true;
	s->measurement_period = 0; 		// 2-600, or 0 equals polled.
	s->message_interval = 0; 		// 2-600, or 0 equals polled.
	clock_gettime(CLOCK_MONOTONIC, &s->last_send_time);
	time_t now = time(NULL);
	strftime(s->date_string, sizeof(s->date_string), "%Y/%m/%d", gmtime(&now));
	strftime(s->time_string, sizeof(s->time_string), "%H:%M:%S", gmtime(&now));
    return 0;
}

/*
 * Name:         skyvue8_is_ready_to_send
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
bool skyvue8_is_ready_to_send(skyvue8_sensor *sensor) {
    if (!sensor || sensor->mode != SMODE_RUN) return false;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    long seconds = now.tv_sec - sensor->last_send_time.tv_sec;
    if (seconds >= (long)sensor->message_interval) return true;

    return false;
}
