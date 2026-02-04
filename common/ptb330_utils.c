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
    s->last_send_time.tv_sec = 0; // Immediate first send
    s->last_send_time.tv_nsec = 0;
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

void ptb330_format_output(ptb330_sensor *sensor, char *dest, size_t max_len) {
    // In a full implementation, you would parse the format_string (e.g., #P, #U)
    // For now, we provide the standard PTB330 default output.
    const char* unit_str = "hPa";
    switch(sensor->units) {
        case UNIT_INHG: unit_str = "inHg"; break;
        case UNIT_PSI:  unit_str = "psi"; break;
        default:        unit_str = "hPa"; break;
    }

    snprintf(dest, max_len, "P = %.2f %s\r\n", sensor->pressure + sensor->offset, unit_str);
}
