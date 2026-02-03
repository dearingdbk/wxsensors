/*
 * File:     atmosvue30_utils.c
 * Author:   Bruce Dearing
 * Date:     23/01/2026
 * Version:  1.0
 * Purpose:  Implementation of AtmosVUE 30 Aviation Weather System emulation utilities.
 *           Provides functions for sensor initialization, command parsing, message
 *           formatting, CRC calculation, and measurement simulation.
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
#include "atmosvue30_utils.h"
#include <math.h>

#define MAX_TOKENS 32
#define DT_STRING 7


// Message format names
const char* message_format_names[15] = {
    "Basic", "Partial", "Full",
    "Basic SYNOP", "Partial SYNOP", "Full SYNOP",
    "Basic METAR", "Partial METAR", "Full METAR",
    "Generic Basic SYNOP", "Generic Partial SYNOP", "Generic Full SYNOP",
    "Custom", "Vaisala FD12", "RVR Output"
};

// SYNOP code descriptions (subset)
const char* synop_codes[90] = {
    [0] = "No significant weather",
    [4] = "Haze/smoke/dust (vis >= 1km)",
    [5] = "Haze/smoke/dust (vis < 1km)",
    [10] = "Mist",
    [20] = "Fog (past hour)",
    [30] = "Fog",
    [40] = "Precipitation",
    [50] = "Drizzle",
    [51] = "Drizzle: slight",
    [60] = "Rain",
    [61] = "Rain: slight",
    [70] = "Snow",
    [71] = "Snow: slight",
    [80] = "Shower(s)",
    [89] = "Hail"
};

// METAR codes
const char* metar_codes[20] = {
    "NSW", "UP", "HZ", "BR", "FG", "DZ", "RA", "SN", "SG", "PL",
    "FZBR", "FZFG", "FZDZ", "FZRA", "-DZ", "+DZ", "-RA", "+RA", "-SN", "+SN"
};

/*
 * Name:         init_av30_sensor
 * Purpose:      Initializes an AtmosVue30 sensor.
 * Arguments:    ptr - a pointer to a av30_sensor struct.
 *
 * Output:       Error message if unable to allocate memory for the struct.
 * Modifies:     Updates the default values of provided sensor.
 * Returns:      -1 on failure, 1 on success.
 * Assumptions:  None.
 *
 * Bugs:         None known.
 * Notes:
 */
int init_av30_sensor(av30_sensor **ptr) {
    av30_sensor *sensor = (av30_sensor *)malloc(sizeof(av30_sensor));
    if (!sensor) return -1;

    strcpy(sensor->serial_number, "32000");
    strcpy(sensor->model_number, "AtmosVUE 30");
    sensor->sensor_id = 0;

    sensor->visibility = 10000;
    sensor->visibility_units = UNITS_METRES;
    sensor->mor_format = MOR_FORMAT_TMOR;
    sensor->extinction_coeff = 0.3;
    sensor->present_weather.synop_code = 0;
    strcpy(sensor->present_weather.metar_code, "NSW");
    strcpy(sensor->present_weather.nws_code, "");
    sensor->present_weather.particle_count = 0.0;
    sensor->present_weather.intensity = 0.0;
    sensor->present_weather.accumulation = 0.0;

    sensor->blm.luminance = 25.7;
    sensor->blm.status = STATUS_NO_FAULT;
    sensor->blm.is_night = false;
    sensor->blm.units = 1;
    sensor->blm.heater_on = false;
    sensor->blm.window_contamination = 0.0;

    sensor->hv.temperature = 24.5;
    sensor->hv.relative_humidity = 33.0;
    sensor->hv.wet_bulb_temp = 0.0;
    sensor->hv.sensor_connected = true;
    sensor->hv.status = STATUS_NO_FAULT;

    sensor->internal_temperature = 25.0;

    sensor->message_format = MSG_RVR_OUTPUT;
    sensor->mode = MODE_POLLING;
    sensor->continuous_interval = 60;
    sensor->averaging_period = AVG_1_MINUTE;
    sensor->sample_timing = 1;

    sensor->comm_type = COMM_RS232;
    sensor->baud_rate = BAUD_38400;
    sensor->data_format = DATA_8N1;
    sensor->crc_checking_enabled = true;

    sensor->dew_heater_override = false;
    sensor->hood_heater_override = false;
    sensor->dew_heater_on = false;
    sensor->hood_heater_on = false;

    sensor->dirty_window_compensation = false;
    sensor->power_down_voltage = 7.0;
    sensor->rh_threshold = 80;

    sensor->system_status = STATUS_NO_FAULT;
    memset(&sensor->system_alarms, 0, sizeof(SystemAlarms));

    sensor->user_alarms.alarm1_set = true;
    sensor->user_alarms.alarm1_active = true;
    sensor->user_alarms.alarm1_distance = 1000;
    sensor->user_alarms.alarm2_set = true;
    sensor->user_alarms.alarm2_active = false;
    sensor->user_alarms.alarm2_distance = 15000;

    sensor->calibration.user_gain = 1.0;
    sensor->calibration.user_offset = 0.0;
    sensor->calibration.factory_gain = 1.0;
    sensor->calibration.factory_offset = 0.0;
    sensor->calibration.dirty_window_emitter = 0.0;
    sensor->calibration.dirty_window_detector = 0.0;
    strcpy(sensor->calibration.calibration_disk_sn, "2000");
    sensor->calibration.calibration_disk_exco = 23.7;

    sensor->custom_msg_bits = 0x121C;

    clock_gettime(CLOCK_MONOTONIC, &sensor->last_send_time);
    sensor->initialized = true;
    sensor->first_minute_elapsed = true;

    *ptr = sensor;
    return 0;
}

/*
 * Name:         av30_is_ready_to_send
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
bool av30_is_ready_to_send(av30_sensor *sensor) {
    if (!sensor || sensor->mode != MODE_CONTINUOUS) return false;
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double elapsed = ((double)now.tv_sec - sensor->last_send_time.tv_sec) +
                     ((double)now.tv_nsec - sensor->last_send_time.tv_nsec) / 1e9;
    return (elapsed >= (double)sensor->continuous_interval);
}
