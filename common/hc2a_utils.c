/*
 * File:     hc2a_utils.c
 * Author:   Bruce Dearing
 * Date:     16/01/2026
 * Purpose:  Implementation of HC2A HygroClip Relative Humidity and Temperature Sensor specific logic.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "crc_utils.h"
#include "hc2a_utils.h"


/*
 * Name:         init_HC2A_sensor
 * Purpose:      Allocates memory for an HC2A sensor structure and initializes
 * 				 all members (serial, baud, modules, etc.) to default factory values.
 * Arguments:    ptr - A pointer to a pointer of type HC2A_sensor, used to
 * 				 return the address of the allocated memory.
 *
 * Output:       An allocated and populated HC2A_sensor structure.
 * Modifies:     Allocates memory on the heap and updates the provided pointer.
 * Returns:      0 on success, -1 if memory allocation fails.
 * Assumptions:  The provided ptr is a valid address of a pointer.
 *
 * Bugs:         None known.
 * Notes:        Uses CLOCK_MONOTONIC for thread timing and UTC (gmtime) for
 * 				 the initial date string.
 *				 Must be freed by the caller.
 */
int init_HC2A_sensor(HC2A_sensor **ptr) {
    *ptr = malloc(sizeof(HC2A_sensor));
    if (!*ptr) return -1;
    HC2A_sensor *s = *ptr;
	// Identity
    s->address = 00;
    s->unit_ident = 'F';
	// Configuration
    s->mode = SMODE_M2;
    s->probe_type = 1; // 1 = digital probe, 2 = analog probe, 3 = pressure probe.
    s->device_type = 20;                   // 
    strncpy(s->serial_number, "0025036130", MAX_SN_LEN - 1);
    s->serial_number[MAX_SN_LEN - 1] = '\0';
    strncpy(s->device_name, "HC2A", MAX_NAME_STR - 1);
    s->device_name[MAX_NAME_STR - 1] = '\0';
    strncpy(s->firmware_version, "V1.2-1", MAX_FIRM_VER - 1);
    s->firmware_version[MAX_FIRM_VER - 1] = '\0';
	memset(&s->temp_trend, 0, sizeof(s->temp_trend));
	memset(&s->rh_trend, 0, sizeof(s->rh_trend));
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
 * Name:         HC2A_is_ready_to_send
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
bool HC2A_is_ready_to_send(HC2A_sensor *sensor) {
    if (!sensor) return false;
	// Because this specific Gill Windobserver 75 sensor operates on sub-second timing, we have removed the checks for last-send time on all continuous modes.
    if (sensor->mode == SMODE_M1) return true;
    if (sensor->mode == SMODE_M2) return true;
    return false;
}


char trend_tracker_update(trend_tracker_t *t, float value)
{
    char trend = '=';

    if (t->count == TREND_HISTORY_LEN) {
        // Buffer is full: the slot about to be overwritten holds the
        // reading from exactly TREND_WINDOW_S seconds ago.
        float baseline = t->samples[t->head];
        float delta = value - baseline;

        if (delta > TREND_THRESHOLD) {
            trend = '+';
        } else if (delta < -TREND_THRESHOLD) {
            trend = '-';
        }
    }
    // else: fewer than 60s of history so far, stay at '='

    t->samples[t->head] = value;
    t->head = (t->head + 1) % TREND_HISTORY_LEN;
    if (t->count < TREND_HISTORY_LEN) {
        t->count++;
    }

    return trend;
}
