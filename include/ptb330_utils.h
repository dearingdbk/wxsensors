/*
 * File:     ptb330_utils.h
 * Author:   Bruce Dearing
 * Date:     16/01/2026
 * Version:  1.0
 * Purpose:  Structures and prototypes for Vaisala PTB330 emulation.
 */

#ifndef PTB330_UTILS_H
#define PTB330_UTILS_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#define MAX_FORM_STR 128
#define MAX_ADDR_LEN 4
#define MAX_SN_LEN 16

typedef enum {
    SMODE_STOP,  // No output
    SMODE_POLL,  // Output only on SEND
    SMODE_RUN,   // Continuous output on startup
    SMODE_SEND   // Output once on startup, then STOP
} PTB330_SMode;

typedef enum {
    UNIT_HPA, UNIT_MBAR, UNIT_KPA, UNIT_PA,
    UNIT_INHG, UNIT_MMH2O, UNIT_MMHG, UNIT_TORR, UNIT_PSI
} PTB330_Unit;

typedef struct {
    // Identity
    char serial_number[MAX_SN_LEN];
    char software_version[12];
    uint8_t address;

    // Configuration
    PTB330_SMode mode;
    PTB330_Unit units;
    uint32_t interval;       // In seconds
    char format_string[MAX_FORM_STR];
    uint16_t send_delay;     // ms
    bool echo_enabled;

    // Pressure State
    float pressure;          // Current reading from file
    float offset;            // Linear adjustment
    float hcp_altitude;      // Height Corrected Pressure alt
    // Timing
    struct timespec last_send_time;
    bool initialized;
} ptb330_sensor;

// Command Parsing
typedef enum {
} PTB330_CmdType;

// Command type enumeration
typedef enum {
    CMD_UNKNOWN,
    CMD_SEND,
	CMD_R,
	CMD_S,
	CMD_INTV,
	CMD_ADDR,
    CMD_SMODE,
	CMD_FORM,
	CMD_UNIT,
	CMD_VERS,
    CMD_HELP,
	CMD_ERRS,
	CMD_UNKNOWN

    // Measurement commands
    CMD_POLL,           // Poll for current data
    CMD_GET,            // Get configuration settings
    // Configuration commands
    CMD_SET,            // Set configuration (saves to flash)
    CMD_SETNC,          // Set configuration (no flash commit)
    CMD_MSGSET,         // Set custom message format
    CMD_ACCRES,         // Reset accumulation

    // Error responses
    CMD_ERROR,          // General error
    CMD_INVALID_CRC,    // CRC check failed
    CMD_INVALID_ID,     // Sensor ID mismatch
    CMD_INVALID_FORMAT  // Command format error
} CommandType;

typedef struct {
    PTB330_CmdType type;
    char raw_params[128];
    int addr_target;         // For RS-485 addressing
} ptb330_command;

// Function Prototypes
int init_ptb330_sensor(ptb330_sensor **ptr);
bool ptb330_is_ready_to_send(ptb330_sensor *sensor);
void ptb330_parse_command(const char *input, ptb330_command *cmd);
void ptb330_format_output(ptb330_sensor *sensor, char *dest, size_t max_len);

#endif
