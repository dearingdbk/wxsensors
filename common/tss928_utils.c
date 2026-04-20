/*
 * File:     tss928_utils.c
 * Author:   Bruce Dearing
 * Date:     16/01/2026
 * Purpose:  Implementation of BTD-300-specific logic.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "crc_utils.h"
#include "tss928_utils.h"


/*
 * Name:         init_tss928_sensor
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
int init_TSS928_sensor(TSS928_sensor **ptr) {
    *ptr = malloc(sizeof(TSS928_sensor));
    if (!*ptr) return -1;
    TSS928_sensor *s = *ptr;
	memset(&s->strikes, 0, sizeof(s->strikes));
	// Identity
	strncpy(s->serial_number, "000008675309", MAX_SN_LEN);
	strncpy(s->loader_version, "TSS928 Loader Version 1.5", MAX_UNIT_STR);
	strncpy(s->software_version, "TSS928 2.0 September 6, 2001", MAX_UNIT_STR);
	strncpy(s->copyright_information, "Copyright (c) 2001, Global Atmospherics, Inc", MAX_UNIT_STR);

    s->address = 1;
	// Configuration
    s->mode = SMODE_POLL;
    s->message_interval = 0; // 0 means the TSS928 is polled.
    s->overhead = 5; // sets the overhead lightning limit to 5 NM or 926 decametres
    s->near = 10; // sets the overhead lightning limit to 20 NM or 3704 decametres
    s->distant = 30; // sets the overhead lightning limit to 30 NM or 5556 decametres
	s->rotation_angle = 120;
	s->strikes.aging_interval = 15;
	s->strikes.current_minute_index = 0;
    s->strikes.total_strikes_since_reset = 0; // Stores the total strikes since the system has been running.

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
 * Name:         TSS928_is_ready_to_send
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
bool TSS928_is_ready_to_send(TSS928_sensor *sensor) {
    if (!sensor || sensor->mode != SMODE_RUN) return false;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    long seconds = now.tv_sec - sensor->last_send_time.tv_sec;
    if (seconds >= (long)sensor->message_interval) return true;

    return false;
}

/*
 * Name:         record_ground_strike()
 * Purpose:      Record the number of strikes/flashes of Cloud-Ground lightning.
 * Arguments:    bin - A StrikeBin struct containing a cicular bin of the strikes up to the aging interval.
 *               ring_index - the uint8_t representing the ring range of the strike 0/NEAR or 1/DIST.
 * 		         quadrant_index - the uint8_t representing the quadrant of the strike 0/N,1/NE,2/E,3/SE,4/S,5/SW,6/W,7/NW.
 *				 strike_count - the number of strikes/flashes to record during this interval.
 * Output:       NIL.
 * Modifies:     ground_history[][][].
 *				 ground_total[][].
 * Returns:      NIL.
 * Assumptions:  The provided bin is a valid address of a StrikeBin * pointer, and the ring_index, and quadrant_index are within range.
 *
 * Bugs:         None known.
 * Notes:
 *
 */
void record_ground_strike(StrikeBin *bin, uint8_t ring_index, uint8_t quadrant_index, uint8_t strike_count){
	bin->ground_history[ring_index][quadrant_index][bin->current_minute_index]+= strike_count;
	bin->ground_totals[ring_index][quadrant_index]+= strike_count;
}

/*
 * Name:         record_overhead_strike()
 * Purpose:      Record the number of strikes/flashes of Cloud-Ground lightning.
 * Arguments:    bin - A StrikeBin struct containing a cicular bin of the strikes up to the aging interval.
 *				 strike_count - the number of strikes/flashes to record during this interval.
 * Output:       NIL.
 * Returns:      NIL.
 * Modifies:     overhead_history[]
 *				 overhead_total
 * Assumptions:  The provided bin is a valid address of a StrikeBin * pointer.
 *
 * Bugs:         None known.
 * Notes:
 *
 */
void record_overhead_strike(StrikeBin *bin, uint8_t strike_count){
	bin->overhead_history[bin->current_minute_index]+= strike_count;
	bin->overhead_total+= strike_count;
}


/*
 * Name:         record_cloud_strike()
 * Purpose:      Record the number of strikes/flashes of Intracloud lightning.
 * Arguments:    bin - A StrikeBin struct containing a cicular bin of the strikes up to the aging interval.
 *				 strike_count - the number of strikes/flashes to record during this interval.
 * Output:       NIL.
 * Returns:      NIL.
 * Modifies:     cloud_history[]
 *				 cloud_total
 * Assumptions:  The provided bin is a valid address of a StrikeBin * pointer.
 *
 * Bugs:         None known.
 * Notes:
 *
 */
void record_cloud_strike(StrikeBin *bin, uint8_t strike_count){
	bin->cloud_history[bin->current_minute_index]+= strike_count;
	bin->cloud_total+= strike_count;
}


/*
 * Name:         advance_one_minute()
 * Purpose:      Advance the circular bin array storing strike/flash counts.
 * Arguments:    bin - A StrikeBin struct containing a cicular bin of the strikes up to the aging interval.
 * Output:       NIL.
 * Returns:      NIL.
 * Modifies:     cloud_history[]
 *				 cloud_total
 * Assumptions:  The provided bin is a valid address of a StrikeBin * pointer.
 *
 * Bugs:         None known.
 * Notes:
 *
 */
void advance_one_minute(StrikeBin *bin){
	uint8_t stale_index = bin->current_minute_index; // Set the stale index to the current minute index.

	for (int i = 0; i < RANGE_RINGS; i++) {
		for (int j = 0; j < QUADRANTS; j++) {
			bin->ground_totals[i][j] -= bin->ground_history[i][j][stale_index]; // loop through and subtract the stale bin from the totals
			bin->ground_history[i][j][stale_index] = 0; // zero out the stale bin.
		}
	}

	bin->overhead_total -= bin->overhead_history[stale_index];
	bin->overhead_history[stale_index] = 0;

	bin->cloud_total -= bin->cloud_history[stale_index];
	bin->cloud_history[stale_index] = 0;

	bin->current_minute_index = (bin->current_minute_index + 1) % bin->aging_interval;
}


/*
 * Name:         conduct_self_test()
 * Purpose:      Conducts a self test of the sensor, for this implementation it just means resetiing the total_strikes.
 * Arguments:    sensor - A TSS-928 sensor struct.
 * Output:       NIL.
 * Returns:      NIL.
 * Modifies:     sensor->strikes.total_strikes_since_reset.
 * Assumptions:  The provided sensor is a valid address of a TSS928_sensor * pointer.
 *
 * Bugs:         None known.
 * Notes:
 *
 */
void conduct_self_test(TSS928_sensor *sensor) {
	sensor->strikes.total_strikes_since_reset = 0; // reset the total strikes.
}


/*
 * Name:         reset_sensor()
 * Purpose:      Conducts a self test of the sensor, for this implementation it just means resetiing the total_strikes.
 * Arguments:    sensor - A TSS-928 sensor struct.
 * Output:       NIL.
 * Returns:      NIL.
 * Modifies:     sensor->strikes.total_strikes_since_reset.
 * Assumptions:  The provided sensor is a valid address of a TSS928_sensor * pointer.
 *
 * Bugs:         None known.
 * Notes:
 *
 */
void reset_sensor(TSS928_sensor *sensor) {
	memset(&sensor->strikes, 0, sizeof(sensor->strikes));
	clock_gettime(CLOCK_MONOTONIC, &sensor->sensor_start_time); // Reset the runtime of the sensor.
}


/*
 * Name:         update_sensor_time()
 * Purpose:      Updates the human readable UTP time of the sensor.
 * Arguments:    time_str - The char * string representing the new time for the sensor.
 *				 sensor_time a tm struct within a TSS-928 sensor.
 * Output:       NIL.
 * Returns:      -1 on invalid format fo the time_str string.
 *               -2 if the times are out of range.
 *				 0 if the update of the new time is successful.
 * Modifies:     TSS928 sensor->sensor_time->tm_hour.
 * 			     TSS928 sensor->sensor_time->tm_min.
 * 			     TSS928 sensor->sensor_time->tm_sec.
 * Assumptions:  The provided sensor is a valid address of a TSS928_sensor tm struct * pointer.
 *
 * Bugs:         None known.
 * Notes:
 *
 */
int update_sensor_time(const char *time_str, struct tm *sensor_time) {
    int h, m, s;

    if (sscanf(time_str, "%d:%d:%d", &h, &m, &s) != 3) {
        return -1; // Invalid format
    }

    if (h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) {
        return -2; // Out of range
    }

    sensor_time->tm_hour = h;
    sensor_time->tm_min = m;
    sensor_time->tm_sec = s;

    return 0;
}

/*
 * Name:         restore_sensor()
 * Purpose:      Restores the sensor to default settings.
 * Arguments:    sensor a TSS-928 sensor pointer.
 * Output:       NIL.
 * Returns:      NIL.
 * Modifies:     TSS928 sensor->rotation_angle.
 * 			     TSS928 sensor->strikes.aging_interval.
 * Assumptions:  The provided sensor is a valid address of a TSS928_sensor tm struct * pointer.
 *
 * Bugs:         None known.
 * Notes:
 *
*/
void restore_sensor(TSS928_sensor *sensor) {
	sensor->rotation_angle = 0;
	sensor->strikes.aging_interval = 15;
}
