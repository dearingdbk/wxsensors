/*
 * File:     g0872f1_utils.c
 * Author:   Bruce Dearing
 * Date:     16/01/2026
 * Purpose:  Implementation of Goodrich 0872F1 Ice Detector specific logic
 *           (sensor init, Stull wet-bulb temperature).
 *
 * Build:    needs -lm (atan, sqrt, pow), e.g.  gcc ... ice.c g0872f1_utils.c ... -lm -lpthread
 */

#define _POSIX_C_SOURCE 200809L   // gmtime_r / clock_gettime under -std=c11

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "crc_utils.h"
#include "g0872f1_utils.h"

// Stull (2011) stated validity of the wet-bulb approximation.
#define STULL_RH_MIN_PCT   5.0
#define STULL_RH_MAX_PCT  99.0

/*
 * Name:         init_G0872F1_sensor
 * Purpose:      Allocates a G0872F1_sensor structure and initializes its members.
 * Arguments:    ptr - address of a pointer that receives the allocated structure.
 *
 * Output:       An allocated and populated G0872F1_sensor structure.
 * Modifies:     Allocates memory on the heap and updates the provided pointer.
 * Returns:      0 on success, -1 if memory allocation fails.
 * Assumptions:  The provided ptr is a valid address of a pointer.
 *
 * Bugs:         None known.
 * Notes:        Zero-initialised (calloc), so fields not set below (e.g. output_rate) are 0
 *               rather than heap garbage. Uses CLOCK_MONOTONIC for timing and UTC (gmtime_r)
 *               for the initial date. Must be freed by the caller.
 */
int init_G0872F1_sensor(G0872F1_sensor **ptr) {
    *ptr = calloc(1, sizeof(G0872F1_sensor));
    if (!*ptr) return -1;
    G0872F1_sensor *s = *ptr;
    // Identity
    s->address = 0;
    s->unit_ident = 'F';
    // Configuration
    s->mode = SMODE_M2;          // ASCII polled
    // TODO: probe_type / device_type / serial / firmware below were carried over from the HC2A
    //       and don't mean anything for the 0872F1. Replace or drop.
    s->probe_type = 1;
    s->device_type = 20;
    strncpy(s->serial_number, "0025036130", MAX_SN_LEN - 1);
    s->serial_number[MAX_SN_LEN - 1] = '\0';
    strncpy(s->device_name, "G0872F1", MAX_NAME_STR - 1);
    s->device_name[MAX_NAME_STR - 1] = '\0';
    strncpy(s->firmware_version, "V1.2-1", MAX_FIRM_VER - 1);
    s->firmware_version[MAX_FIRM_VER - 1] = '\0';
    // Timing
    time_t now;
    time(&now);
    gmtime_r(&now, &s->sensor_time);
    clock_gettime(CLOCK_MONOTONIC, &s->last_send_time);
    clock_gettime(CLOCK_MONOTONIC, &s->sensor_start_time);
    s->initialized = true;
    return 0;
}

/*
 * Name:         G0872F1_is_ready_to_send
 * Purpose:      Legacy from the continuous-output sensors. The 0872F1 emulator is polled and
 *               ice.c no longer calls this; kept only so the prototype in the header still links.
 * Returns:      true for M1/M2, false otherwise (or NULL sensor).
 */
bool G0872F1_is_ready_to_send(G0872F1_sensor *sensor) {
    if (!sensor) return false;
    if (sensor->mode == SMODE_M1) return true;
    if (sensor->mode == SMODE_M2) return true;
    return false;
}

/*
 * Name:         wet_bulb_temp_c
 * Purpose:      Wet-bulb temperature from dry-bulb temperature and RH (Stull 2011).
 * Arguments:    temp_c: dry-bulb temperature, deg C. rh_pct: relative humidity, percent.
 * Returns:      Wet-bulb temperature, deg C.
 * Notes:        Stull is stated valid for RH 5-99 % and -20 to 50 deg C. RH is clamped to
 *               that range, and the result is capped at the dry-bulb temperature (wet-bulb
 *               can't exceed it). Temperature is NOT clamped; below -20 deg C the result is
 *               an extrapolation, which is harmless for icing (wet-bulb is far below 0 anyway).
 */
double wet_bulb_temp_c(double temp_c, double rh_pct) {
    if (rh_pct < STULL_RH_MIN_PCT) rh_pct = STULL_RH_MIN_PCT;
    if (rh_pct > STULL_RH_MAX_PCT) rh_pct = STULL_RH_MAX_PCT;

    double wb_temp = temp_c * atan(STULL_C1 * sqrt(rh_pct + STULL_C2))
                   + atan(temp_c + rh_pct)
                   - atan(rh_pct - STULL_C3)
                   + STULL_C4 * pow(rh_pct, 1.5) * atan(STULL_C5 * rh_pct)
                   - STULL_C6;

    return (wb_temp > temp_c) ? temp_c : wb_temp;
}

/*
 * Name:         generate_jitter
 * Purpose:      jitters the Hz of the ice accretion sensor, to real-life scenarios.
 * Arguments:    NIL
 * Returns:      [-30.0, +30.0] 
 * Notes:        Uniform jitter. Not used by ice.c; call srand() once at startup if this is used.
 */
float generate_jitter(void) {
    return ((float)rand() / (float)RAND_MAX) * 60.0f - 30.0f;
}
