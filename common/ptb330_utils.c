/*
 * File:     ptb330_utils.c
 * Author:   Bruce Dearing
 * Date:     16/01/2026
 * Purpose:  Implementation of PTB330-specific logic.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "ptb330_utils.h"

int init_ptb330_sensor(ptb330_sensor **ptr) {
    *ptr = malloc(sizeof(ptb330_sensor));
    if (!*ptr) return -1;

    ptb330_sensor *s = *ptr;
    strcpy(s->serial_number, "G1234567");
    strcpy(s->software_version, "1.12");
    s->address = 0;
    s->mode = SMODE_STOP;
    s->units = UNIT_HPA;
    s->interval = 1;
    strcpy(s->format_string, "P = #P #U\\r\\n"); // Default Vaisala format
    s->pressure = 1013.25;
    s->offset = 0.0;
	clock_gettime(CLOCK_MONOTONIC, &s->last_send_time);
    //s->last_send_time.tv_sec = 0; // Immediate first send
    //s->last_send_time.tv_nsec = 0;
    s->initialized = true;
    return 0;
}

void ptb330_parse_command(const char *input, ptb330_command *cmd) {
    char buf[128];
    strncpy(buf, input, sizeof(buf));
    // Simple tokenizer for PTB330 commands
    char *token = strtok(buf, " \r\n");
    if (!token) {
        cmd->type = CMD_UNKNOWN;
        return;
    }

    // Convert to uppercase for comparison
    for(int i = 0; token[i]; i++) token[i] = toupper(token[i]);

    if (strcmp(token, "SEND") == 0)   cmd->type = CMD_SEND;
    else if (strcmp(token, "R") == 0) cmd->type = CMD_R;
    else if (strcmp(token, "S") == 0) cmd->type = CMD_S;
    else if (strcmp(token, "INTV") == 0) cmd->type = CMD_INTV;
    else if (strcmp(token, "SMODE") == 0) cmd->type = CMD_SMODE;
    else if (strcmp(token, "FORM") == 0) cmd->type = CMD_FORM;
    else if (strcmp(token, "UNIT") == 0) cmd->type = CMD_UNIT;
    else if (strcmp(token, "VERS") == 0) cmd->type = CMD_VERS;
    else cmd->type = CMD_UNKNOWN;

    // Capture remaining params
    char *params = strtok(NULL, "\r\n");
    if (params) strncpy(cmd->raw_params, params, sizeof(cmd->raw_params));
    else cmd->raw_params[0] = '\0';
}

bool ptb330_is_ready_to_send(ptb330_sensor *sensor) {
    if (!sensor || sensor->mode != SMODE_RUN) return false;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    long seconds = now.tv_sec - sensor->last_send_time.tv_sec;
    if (seconds >= (long)sensor->interval) return true;

    return false;
}


/*typedef struct {
    PTB330_Unit unit;
    const char *label;
    double multiplier; // Multiplier to convert from hPa to this unit
} UnitConversion;

static const UnitConversion unit_table[] = {
    {UNIT_HPA,   "hPa",  1.0},
    {UNIT_MBAR,  "mbar", 1.0},
    {UNIT_KPA,   "kPa",  0.1},
    {UNIT_PA,    "Pa",   100.0},
    {UNIT_INHG,  "inHg", 0.0295299},
    {UNIT_MMHG,  "mmHg", 0.750062},
    {UNIT_TORR,  "torr", 0.750062},
    {UNIT_PSI,   "psi",  0.0145038}
};*/

const char* get_unit_str(PTB330_Unit unit) {
    for (int i = 0; i < sizeof(unit_table)/sizeof(UnitConversion); i++) {
        if (unit_table[i].unit == unit) return unit_table[i].label;
    }
    return "hPa";
}

double get_scaled_pressure(float hpa_val, PTB330_Unit unit) {
    for (int i = 0; i < sizeof(unit_table)/sizeof(UnitConversion); i++) {
        if (unit_table[i].unit == unit) return (double)hpa_val * unit_table[i].multiplier;
    }
    return (double)hpa_val;
}

/*void ptb330_format_output(ptb330_sensor *sensor, char *dest, size_t max_len) {
    // In a full implementation, you would parse the format_string (e.g., #P, #U)
    // For now, we provide the standard PTB330 default output.
    const char* unit_str = "hPa";
    switch(sensor->units) {
        case UNIT_INHG: unit_str = "inHg"; break;
        case UNIT_PSI:  unit_str = "psi"; break;
        default:        unit_str = "hPa"; break;
    }

    snprintf(dest, max_len, "P = %.2f %s\r\n", sensor->pressure + sensor->offset, unit_str);
}*/


void ptb330_format_output(ptb330_sensor *sensor, char *dest, size_t max_len) {
    char buffer[256] = {0};
    char *src = sensor->format_string;
    size_t out_idx = 0;

    while (*src && out_idx < (max_len - 1)) {
        if (*src == '#') {
            src++; // Move to the token character
            switch (*src) {
                case 'P': { // Pressure (scaled)
                    double val = get_scaled_pressure(sensor->pressure + sensor->offset, sensor->units);
                    out_idx += snprintf(&buffer[out_idx], max_len - out_idx, "%.2f", val);
                    break;
                }
                case 'U': { // Unit Label
                    out_idx += snprintf(&buffer[out_idx], max_len - out_idx, "%s", get_unit_str(sensor->units));
                    break;
                }
                case 'S': { // Serial Number
                    out_idx += snprintf(&buffer[out_idx], max_len - out_idx, "%s", sensor->serial_number);
                    break;
                }
                case 'n': { // Address/Node
                    out_idx += snprintf(&buffer[out_idx], max_len - out_idx, "%02d", sensor->address);
                    break;
                }
                default:
                    if (out_idx < max_len - 1) buffer[out_idx++] = *src;
            }
        }
        // Handle standard escape sequences used in Vaisala strings
        else if (*src == '\\') {
            src++;
            if (*src == 'r') buffer[out_idx++] = '\r';
            else if (*src == 'n') buffer[out_idx++] = '\n';
            else if (*src == 't') buffer[out_idx++] = '\t';
        }
        else {
            buffer[out_idx++] = *src;
        }
        src++;
    }
    buffer[out_idx] = '\0';
    strncpy(dest, buffer, max_len);
}
/*
void ptb330_format_output(ptb330_sensor *sensor, char *dest, size_t max_len) {
    char temp[MAX_FORM_STR] = {0};
    char *src = sensor->format_string;
    int i = 0;

    while (*src && i < (max_len - 1)) {
        if (*src == '#' && *(src + 1)) {
            src++; // Skip the '#'
            switch (*src) {
                case 'P': // Pressure
                    i += snprintf(&temp[i], max_len - i, "%.2f", sensor->pressure);
                    break;
                case 'U': // Units
                    i += snprintf(&temp[i], max_len - i, "%s", get_unit_str(sensor->units));
                    break;
                case 'S': // Serial Number
                    i += snprintf(&temp[i], max_len - i, "%s", sensor->serial_number);
                    break;
                case 't': // Current Time
                    {
                        time_t now = time(NULL);
                        struct tm *tm_info = localtime(&now);
                        i += strftime(&temp[i], max_len - i, "%H:%M:%S", tm_info);
                    }
                    break;
                default: // Handle unknown as literal
                    temp[i++] = *src;
            }
        } else if (*src == '\\' && *(src + 1)) {
            src++; // Handle escape characters
            if (*src == 'r') temp[i++] = '\r';
            else if (*src == 'n') temp[i++] = '\n';
            else if (*src == 't') temp[i++] = '\t';
        } else {
            temp[i++] = *src;
        }
        src++;
    }
    temp[i] = '\0';
    strncpy(dest, temp, max_len);
}*/
