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

// Baud rate mappings
static const uint32_t baud_rate_values[] = {
    1200, 2400, 38400, 19200, 57600, 115200
};

int init_av30_sensor(av30_sensor **ptr) {
    av30_sensor *sensor = (av30_sensor *)malloc(sizeof(av30_sensor));
    if (!sensor) return -1;

    strcpy(sensor->serial_number, "32000");
    strcpy(sensor->model_number, "AtmosVUE 30");
    sensor->sensor_id = 0;

    sensor->visibility = 10000;
    sensor->visibility_units = UNITS_METERS;
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

void free_sensor(av30_sensor *sensor) {
    if (sensor) free(sensor);
}


/*bool verify_crc(const char *command) {
    const char *crc_start = strrchr(command, ':');
    if (!crc_start || crc_start == command) return false;

    const char *data_end = crc_start - 1;
    while (data_end > command && *data_end != ':') data_end--;
    if (data_end == command) return false;

    size_t data_len = data_end - command;
    uint16_t calculated_crc = calculate_crc16_ccitt(command, data_len);

    uint16_t received_crc;
    if (sscanf(data_end + 1, "%hx", &received_crc) != 1) return false;

    return calculated_crc == received_crc;
}*/

uint32_t get_baud_rate_value(BaudRateCode code) {
    if (code < 0 || code > 5) return 38400;
    return baud_rate_values[code];
}

BaudRateCode get_baud_rate_code(uint32_t baud) {
    for (int i = 0; i < 6; i++) {
        if (baud_rate_values[i] == baud) return (BaudRateCode)i;
    }
    return BAUD_38400;
}

const char* get_baud_rate_string(BaudRateCode code) {
    static char buf[16];
    snprintf(buf, sizeof(buf), "%u", get_baud_rate_value(code));
    return buf;
}

const char* get_message_format_name(MessageFormat format) {
    if (format < 0 || format > 14) return "Unknown";
    return message_format_names[format];
}

const char* get_synop_description(uint8_t code) {
    if (code >= 90 || !synop_codes[code]) return "Unknown";
    return synop_codes[code];
}

const char* get_system_status_string(SystemStatus status) {
    switch (status) {
        case STATUS_NO_FAULT: return "No fault";
        case STATUS_POSSIBLE_DEGRADED: return "Possible degraded performance";
        case STATUS_DEGRADED: return "Degraded performance";
        case STATUS_MAINTENANCE_REQUIRED: return "Maintenance required";
        default: return "Unknown";
    }
}

float simulate_visibility(void) {
    static float base_visibility = 10000.0;
    float variation = ((rand() % 40) - 20) / 100.0;
    float visibility = base_visibility * (1.0 + variation);
    if (visibility < MIN_VISIBILITY_M) visibility = MIN_VISIBILITY_M;
    if (visibility > MAX_VISIBILITY_M) visibility = MAX_VISIBILITY_M;
    return visibility;
}

uint8_t simulate_synop_code(void) {
    int r = rand() % 100;
    if (r < 70) return 0;
    if (r < 80) return 10;
    if (r < 85) return 30;
    if (r < 90) return 51;
    if (r < 95) return 61;
    return 71;
}

void simulate_metar_code(char *metar, uint8_t synop) {
    switch (synop) {
        case 0:  strcpy(metar, "NSW"); break;
        case 4:
        case 5:  strcpy(metar, "HZ"); break;
        case 10: strcpy(metar, "BR"); break;
        case 30: strcpy(metar, "FG"); break;
        case 50:
        case 51: strcpy(metar, "-DZ"); break;
        case 52: strcpy(metar, "DZ"); break;
        case 53: strcpy(metar, "+DZ"); break;
        case 60:
        case 61: strcpy(metar, "-RA"); break;
        case 62: strcpy(metar, "RA"); break;
        case 63: strcpy(metar, "+RA"); break;
        case 70:
        case 71: strcpy(metar, "-SN"); break;
        case 72: strcpy(metar, "SN"); break;
        case 73: strcpy(metar, "+SN"); break;
        default: strcpy(metar, "NSW"); break;
    }
}

float simulate_luminance(void) {
    time_t now = time(NULL);
    struct tm *local = localtime(&now);
    int hour = local->tm_hour;
    if (hour >= 6 && hour < 18) {
        return 1000.0 + (rand() % 4000);
    } else {
        return 0.1 + ((rand() % 100) / 10.0);
    }
}

float simulate_temperature(void) {
    return -10.0 + ((rand() % 450) / 10.0);
}

float simulate_humidity(void) {
    return 20.0 + (rand() % 70);
}

float calculate_wet_bulb(float temp, float rh) {
    float t = temp;
    float r = rh;
    float tw = t * atan(0.151977 * sqrt(r + 8.313659)) +
               atan(t + r) - atan(r - 1.676331) +
               0.00391838 * pow(r, 1.5) * atan(0.023101 * r) - 4.686035;
    return tw;
}

void update_measurements(av30_sensor *sensor) {
    if (!sensor) return;
    float vis_m = simulate_visibility();
    sensor->visibility = (sensor->visibility_units == UNITS_METERS) ?
                         (uint16_t)vis_m : (uint16_t)(vis_m * 3.28084);
    sensor->extinction_coeff = 3000.0 / vis_m;
    sensor->present_weather.synop_code = simulate_synop_code();
    simulate_metar_code(sensor->present_weather.metar_code,
                       sensor->present_weather.synop_code);
    if (sensor->present_weather.synop_code >= 50) {
        sensor->present_weather.particle_count = 10.0 + (rand() % 100);
        sensor->present_weather.intensity = 0.1 + ((rand() % 50) / 10.0);
        sensor->present_weather.accumulation += sensor->present_weather.intensity / 60.0;
        if (sensor->present_weather.accumulation > MAX_PRECIP_ACCUM) {
            sensor->present_weather.accumulation = MAX_PRECIP_ACCUM;
        }
    } else {
        sensor->present_weather.particle_count = 0.0;
        sensor->present_weather.intensity = 0.0;
    }
    sensor->blm.luminance = simulate_luminance();
    sensor->blm.is_night = (sensor->blm.luminance < 50.0);
    if (sensor->hv.sensor_connected) {
        sensor->hv.temperature = simulate_temperature();
        sensor->hv.relative_humidity = simulate_humidity();
        sensor->hv.wet_bulb_temp = calculate_wet_bulb(sensor->hv.temperature,
                                                      sensor->hv.relative_humidity);
    }
    sensor->internal_temperature = simulate_temperature();
    update_heater_control(sensor);
    update_system_alarms(sensor);
    check_user_alarms(sensor);
}

void update_heater_control(av30_sensor *sensor) {
    if (!sensor) return;
    float temp = sensor->internal_temperature;
    if (!sensor->dew_heater_override) {
        if (temp < 35.0 && !sensor->dew_heater_on) {
            sensor->dew_heater_on = true;
        } else if (temp > 40.0 && sensor->dew_heater_on) {
            sensor->dew_heater_on = false;
        }
    }
    if (!sensor->hood_heater_override) {
        if (temp < 15.0 && !sensor->hood_heater_on) {
            sensor->hood_heater_on = true;
        } else if (temp > 25.0 && sensor->hood_heater_on) {
            sensor->hood_heater_on = false;
        }
    }
}

bool should_dew_heater_be_on(av30_sensor *sensor) {
    return sensor->dew_heater_on;
}

bool should_hood_heater_be_on(av30_sensor *sensor) {
    return sensor->hood_heater_on;
}

void update_system_alarms(av30_sensor *sensor) {
    if (!sensor) return;
    memset(&sensor->system_alarms, 0, sizeof(SystemAlarms));
    float temp = sensor->internal_temperature;
    if (temp < -40.0) {
        sensor->system_alarms.emitter_temperature = 1;
        sensor->system_status = STATUS_POSSIBLE_DEGRADED;
    } else if (temp > 70.0) {
        sensor->system_alarms.emitter_temperature = 2;
        sensor->system_status = STATUS_DEGRADED;
    }
}

void check_user_alarms(av30_sensor *sensor) {
    if (!sensor) return;
    uint16_t vis = sensor->visibility;
    if (sensor->visibility_units == UNITS_FEET) {
        vis = (uint16_t)(vis / 3.28084);
    }
    if (sensor->user_alarms.alarm1_set) {
        sensor->user_alarms.alarm1_active = (vis <= sensor->user_alarms.alarm1_distance);
    }
    if (sensor->user_alarms.alarm2_set) {
        sensor->user_alarms.alarm2_active = (vis <= sensor->user_alarms.alarm2_distance);
    }
}

void clear_alarms(av30_sensor *sensor) {
    if (!sensor) return;
    memset(&sensor->system_alarms, 0, sizeof(SystemAlarms));
    sensor->system_status = STATUS_NO_FAULT;
    sensor->user_alarms.alarm1_active = false;
    sensor->user_alarms.alarm2_active = false;
}

bool av30_is_ready_to_send(av30_sensor *sensor) {
    if (!sensor || sensor->mode != MODE_CONTINUOUS) return false;
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double elapsed = (now.tv_sec - sensor->last_send_time.tv_sec) +
                     (now.tv_nsec - sensor->last_send_time.tv_nsec) / 1e9;
    return elapsed >= sensor->continuous_interval;
}

/*int format_message(av30_sensor *sensor, char *output, size_t max_len) {
    if (!sensor || !output) return -1;
    char msg[1024];
    int len = snprintf(msg, sizeof(msg), 
        "%d %d %d %d %d %c %d %.2f %d "
        "%d %d %d %d %d %d %d %d %d %d %d %d "
        "%.2f %.2f %d %s %.1f %d %d %d %d ",
        (int)sensor->message_format, sensor->sensor_id, sensor->system_status,
        sensor->continuous_interval, sensor->visibility,
        (sensor->visibility_units == UNITS_METERS) ? 'M' : 'F',
        sensor->mor_format, sensor->extinction_coeff,
        (sensor->averaging_period == AVG_1_MINUTE) ? 1 : 10,
        sensor->system_alarms.emitter_failure, sensor->system_alarms.emitter_lens_dirty,
        sensor->system_alarms.emitter_temperature, sensor->system_alarms.detector_lens_dirty,
        sensor->system_alarms.detector_temperature, sensor->system_alarms.detector_dc_saturation,
        sensor->system_alarms.hood_temperature, sensor->system_alarms.external_temperature,
        sensor->system_alarms.signature_error, sensor->system_alarms.flash_read_error,
        sensor->system_alarms.flash_write_error, sensor->system_alarms.particle_limit,
        sensor->present_weather.particle_count, sensor->present_weather.intensity,
        sensor->present_weather.synop_code, sensor->present_weather.metar_code,
        (sensor->hv.sensor_connected) ? sensor->hv.temperature : sensor->internal_temperature,
        (sensor->hv.sensor_connected) ? (int)sensor->hv.relative_humidity : -99,
        "BLM", sensor->blm.luminance, sensor->blm.status,
        sensor->blm.is_night ? 1 : 0, sensor->blm.units
    );
    if (len < 0 || len >= sizeof(msg)) return -1;
    uint16_t crc = calculate_crc16_ccitt(msg, strlen(msg));
    len += snprintf(msg + len, sizeof(msg) - len, "%04X", crc);
    int total_len = snprintf(output, max_len, "\x02%s\x03\r\n", msg);
    return (total_len < max_len) ? total_len : -1;
}*/

/*int format_get_response(av30_sensor *sensor, char *output, size_t max_len) {
    if (!sensor || !output) return -1;
    char msg[512];
    int len = snprintf(msg, sizeof(msg),
        "%d %d %d %d %d %d %d %d %s %c %d %d %d %d %d %d %d %d %d %d %.1f %d %d",
        sensor->sensor_id,
        sensor->user_alarms.alarm1_set ? 1 : 0, sensor->user_alarms.alarm1_active ? 1 : 0,
        sensor->user_alarms.alarm1_distance,
        sensor->user_alarms.alarm2_set ? 1 : 0, sensor->user_alarms.alarm2_active ? 1 : 0,
        sensor->user_alarms.alarm2_distance, sensor->baud_rate, sensor->serial_number,
        (sensor->visibility_units == UNITS_METERS) ? 'M' : 'F',
        sensor->continuous_interval, sensor->mode, sensor->message_format, sensor->comm_type,
        (sensor->averaging_period == AVG_1_MINUTE) ? 1 : 10, sensor->sample_timing,
        sensor->dew_heater_override ? 1 : 0, sensor->hood_heater_override ? 1 : 0,
        sensor->dirty_window_compensation ? 1 : 0, sensor->crc_checking_enabled ? 1 : 0,
        sensor->power_down_voltage, sensor->rh_threshold, sensor->data_format
    );
    if (len < 0 || len >= sizeof(msg)) return -1;
    uint16_t crc = calculate_crc16_ccitt(msg, strlen(msg));
    int total_len = snprintf(output, max_len, "\x02%s %04X\x04\r\n", msg, crc);
    return (total_len < max_len) ? total_len : -1;
}*/

int format_poll_response(av30_sensor *sensor, char *output, size_t max_len) {
    return format_message(sensor, output, max_len);
}

