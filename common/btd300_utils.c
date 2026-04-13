/*
 * File:     btd300_utils.c
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
#include "btd300_utils.h"


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
int init_BTD300_sensor(BTD300_sensor **ptr) {
    *ptr = malloc(sizeof(BTD300_sensor));
    if (!*ptr) return -1;

    BTD300_sensor *s = *ptr;
	// Identity
	strncpy(s->serial_number, "000008675309", MAX_SN_LEN);

    s->address = 1;
	// Configuration
    s->mode = SMODE_RUN;
    s->message_interval = 2; // 0 means the skyvue8 is polled.
    s->overhead = 926; // sets the overhead lightning limit to 5 NM or 926 decametres
    s->vicinity = 1852; // sets the overhead lightning limit to 10 NM or 1852 decametres
    s->near_distant = 3704; // sets the overhead lightning limit to 20 NM or 3704 decametres
    s->far_distant = 5556; // sets the overhead lightning limit to 30 NM or 5556 decametres
	clock_gettime(CLOCK_MONOTONIC, &s->last_send_time);
	s->initialized = true;
    return 0;
}

/*
 * Name:         BTD300_is_ready_to_send
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
bool BTD300_is_ready_to_send(BTD300_sensor *sensor) {
    if (!sensor || sensor->mode != SMODE_RUN) return false;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    long seconds = now.tv_sec - sensor->last_send_time.tv_sec;
    if (seconds >= (long)sensor->message_interval) return true;

    return false;
}


/*
 * Name:         reset_flash
 * Purpose:      resets a BTD-300 lightning sensor.
 * Arguments:    ptr - a pointer to a flash_sensor struct.
 *
 * Output:       Error message if unable to allocate memory for the struct.
 * Modifies:     Updates the default values of provided sensor.
 * Returns:      1 on failure, 0 on success.
 * Assumptions:
 *
 * Bugs:         None known.
 * Notes:        int snprintf(char *str, size_t size, const char *format, ...);
 */
int reset_flash(BTD300_sensor **ptr) {
    if (*ptr == NULL) {
        perror("Unable to communicate with sensor\n");
        return 1;
    }
	BTD300_sensor *s = *ptr;
    s->overhead = 926; // sets the overhead lightning limit to 5 NM or 926 decametres
    s->vicinity = 1852; // sets the overhead lightning limit to 10 NM or 1852 decametres
    s->near_distant = 3704; // sets the overhead lightning limit to 20 NM or 3704 decametres
    s->far_distant = 5556; // sets the overhead lightning limit to 30 NM or 5556 decametres

    return 0;
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
int set_dist(BTD300_sensor **ptr, int distance_id, int distance) {
    if (*ptr == NULL) {
        perror("Unable to communicate with sensor\n");
        return -1;
    }

	switch(distance_id) {
		case 0:
    		(*ptr)->overhead = distance; // sets the overhead lightning limit to 'distance'
			break;
		case 1:
		    (*ptr)->vicinity = distance; // sets the overhead lightning limit to 'distance'
			break;
		case 2:
		    (*ptr)->near_distant = distance; // sets the overhead lightning limit to 'distance'
			break;
		case 3:
		    (*ptr)->far_distant = distance; // sets the overhead lightning limit to 'distance'
			break;
		default:
			// 99 or another bad number, do nothing.
			break;
	}

    return 1;
}


/*
 * Name:         parse_to_epoch
 * Purpose:      Parse a DDMMYY string and HHMMSS string into a UTC epoch time_t.
 * 				 e.g. date="010124", time="235959" -> epoch for 2024-01-01 23:59:59 UTC.
 * Arguments:    date_token a date string to convert to epoch time.
 *				 time_token a time string to convert to epoch time.
 * Output:       NIL.
 * Modifies:     Local copy of a tm struct.
 * Returns:      time_t - integer representing epoch time of the converted data and time strings.
 * Assumptions:  The provided date and time token strings are valid dates, and in the correct format.
 *
 * Bugs:         None known.
 * Notes:        Uses UTC time.
 *
 */
time_t parse_to_epoch(const char *date_token, const char *time_token)
{
    struct tm t = {0}; // Use a local tm struct, zeroized.

    // Parse DDMMYY
    t.tm_mday  =  (date_token[0] - '0') * 10 + (date_token[1] - '0');
    t.tm_mon   = ((date_token[2] - '0') * 10 + (date_token[3] - '0')) - 1; // tm_mon is 0 indexed, subtract 1, to get the right month (03 == April).
    t.tm_year  =  (date_token[4] - '0') * 10 + (date_token[5] - '0') + 100; // years since 1900, add 100 to account for the century shift.

    // Parse HHMMSS
    t.tm_hour  = (time_token[0] - '0') * 10 + (time_token[1] - '0');
    t.tm_min   = (time_token[2] - '0') * 10 + (time_token[3] - '0');
    t.tm_sec   = (time_token[4] - '0') * 10 + (time_token[5] - '0');

    t.tm_isdst = 0; // We are using UTC, Daylight Savings Time is not in effect (Standard Time).

    return timegm(&t);
}


/*
 * Name:         epoch_to_date
 * Purpose:      Format an epoch time_t back to a DDMMYY string.
 * Arguments:    epoch - A time_t pointer used to get the current date.
 *               buf - A char* pointer used to hold the current date string DDMMYY, buf must be at least 7 bytes.
 * Output:       NIL.
 * Modifies:     buf - snprintf to the char* pointer the date string.
 * Returns:      NIL.
 * Assumptions:  The provided buf is a valid address of a char * pointer, of size 7 bytes, and time_t epoch is not null.
 *
 * Bugs:         None known.
 * Notes:        Uses UTC time.
 *
 */
void epoch_to_date(time_t epoch, char *buf)
{
    struct tm t;
	gmtime_r(&epoch, &t);
	strftime(buf, sizeof(buf), "%d%m%y", &t);
}

/*
 * Name:         epoch_to_time
 * Purpose:      Format an epoch time_t back to a HHMMSS string.
 * Arguments:    epoch - A time_t pointer used to get the current epoch time.
 *               buf - A char* pointer used to hold the current time string HHMMSS, buf must be at least 7 bytes.
 * Output:       NIL.
 * Modifies:     buf - snprintf to the char* pointer the time string.
 * Returns:      NIL.
 * Assumptions:  The provided buf is a valid address of a char * pointer, of size 7 bytes, and time_t epoch is not null.
 *
 * Bugs:         None known.
 * Notes:        Uses UTC time.
 *
 */
void epoch_to_time(time_t epoch, char *buf)
{
    struct tm t;
	gmtime_r(&epoch, &t);
	strftime(buf, sizeof(buf), "%H%M%S", &t);
}
