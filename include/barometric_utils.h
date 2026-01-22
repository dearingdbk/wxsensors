
/*
 * File:     barometric_utils.h
 * Author:   Bruce Dearing
 * Date:     26/11/2025
 * Version:  1.0
 * Purpose:  Program to handle setting up a serial connection and two threads
 * Mods:
 *
 *
 */

#ifndef BAROMETRIC_UTILS_H
#define BAROMETRIC_UTILS_H

#include <stdbool.h>

#define MAX_INPUT_STR 256
#define MAX_SERIAL_STR 16
#define MAX_MODEL_NUM 32
#define MAX_MSG_STR 17

extern char units_of_measure[25][50];
extern double coefficients[58];
extern int current_u_of_m;

/// BAROMETRIC PRESSURE SENSOR ///

typedef struct {
    // Sensor identification
    char serial_number[MAX_SERIAL_STR];
    char model_number[MAX_MODEL_NUM];
    char user_message[MAX_MSG_STR];  // 16 chars + null terminator

    // Pressure range
    float min_pressure;
    float max_pressure;
    uint8_t pressure_units;  // 0-24, see unit codes
    uint8_t sensor_type;     // 0=Gauge, 1=Absolute

    // Current settings
    uint8_t device_address;  // 0-98 (0 = direct mode)
    uint8_t filter_number;   // 0-5
    uint16_t filter_prescaler;  // 1-1000 ms
    float transmission_interval; // 0.01-9999 sec, 0=off
    uint8_t output_format;   // 0-12

    // Communication settings
    uint32_t baud_rate;      // 9600, 19200, 38400, 57600, 115200, 230400
    char parity;             // 'I', 'N', 'O', 'E'
    uint8_t data_bits;       // Always 8
    uint8_t stop_bits;       // 1 or 2
    uint8_t term_chars;      // 1 or 2
	uint16_t wait_interval;   // Number of chars at 9600 baud to wait

    // Calibration data
    float user_gain;         // Default 1.0
    float user_offset;       // Default 0.0
    float slope;             // Span adjustment
    float set_point;         // Zero adjustment

    // PIN protection
    uint16_t pin;            // 000-999
    bool pin_set;

    // Status flags
    bool units_sent;         // Include units in output?
    bool long_errors;        // Long vs short error messages

    // Simulated data tracking
    float current_pressure;
    float current_temperature;

	// Time stamping for interleaved sensor data sending.
	struct timespec last_send_time;
} bp_sensor;


// Command type enumeration
typedef enum {
    CMD_UNKNOWN,

    // Measurement commands
    CMD_R,              // Basic reading
    CMD_R_UNITS,        // Reading with units (*R)
    CMD_R1,             // Pressure + temperature (RPS only)
    CMD_R1_UNITS,       // Pressure + temperature with units
    CMD_R2,             // Temperature only (RPS only)
    CMD_R2_UNITS,       // Temperature with units
    CMD_R3,             // IEEE binary pressure
    CMD_R4,             // IEEE binary both
    CMD_R5,             // IEEE binary temperature

    // Information
    CMD_I,              // Identity
    CMD_I_FORMATTED,    // Formatted identity (*I)

    // General setup (no PIN required)
    CMD_A_SET,          // Set auto-send
    CMD_A_QUERY,        // Query auto-send
	CMD_A_FORMATTED,	// Query formatted
    CMD_N_SET,          // Set address
    CMD_N_QUERY,        // Query address
	CMD_N_LONG,			// Change to Long Error Messages
	CMD_N_FORMATTED,	// Query address formatted
    CMD_F_SET,          // Set filter
    CMD_F_QUERY,        // Query filter
    CMD_U_SET,          // Set units
    CMD_U_QUERY,        // Query units
    CMD_U_FORMATTED,	// Query units formatted
	CMD_U_INTERACTIVE,	// Set units interactive
	CMD_B_SET,          // Set bus wait
    CMD_B_QUERY,        // Query bus wait
    CMD_X_QUERY,        // Status check

	// Error commands
	CMD_BAD_CMD,		// !004 Bad Command
	CMD_NULL_PARAM,		// !009 Missing Param
	CMD_BAD_VALUE,		// !011 Bad Value
	CMD_BAD_FMT,

    // PIN-protected setup
    CMD_C_CAL,          // Calibration
    CMD_C_QUERY,        // Query calibration
    CMD_H_SET,          // Set slope
    CMD_H_QUERY,        // Query slope
    CMD_M_SET,          // Set message
    CMD_M_QUERY,        // Query message
    CMD_O_SET,          // Set comm settings
    CMD_O_QUERY,        // Query comm settings
    CMD_P_SET,          // Change PIN
    CMD_P_QUERY,        // Query PIN status
    CMD_S_SET,          // Set offset
    CMD_S_CLEAR,        // Clear offset (S,PIN,X)
    CMD_S_QUERY,        // Query offset
    CMD_W_SAVE          // Write to flash
} CommandType;

// Parsed command structure - holds the parsed command and all its parameters
typedef struct {
    CommandType type;           // What command is it?
    uint8_t address;            // 0-98, where 0 = direct mode
    bool is_addressed;          // true if "1:R" format
    bool is_formatted;          // true if starts with "*"
    bool is_wildcard;           // true if starts with "*" for global commands
    // Union to hold parameters for different command types
    // Only one set of parameters is valid at a time in union
    union {
        // For A command (auto-send)
        struct {
            float interval;     // 0.01-9999 seconds, 0=off
            uint8_t format;     // 0-12
        } auto_send;
        // For N command (address)
        struct {
            uint8_t address;    // 0-98
        } set_address;
        // For F command (filter)
        struct {
            uint8_t filter;     // 0-5
            uint16_t prescaler; // 1-1000 ms
        } filter_params;
        // For U command (units)
        struct {
            uint8_t unit_code;  // 0-24
        } units;
        // For C command (calibration)
        struct {
            uint16_t pin;       // 000-999
            uint8_t mode;       // 1 or 2 (first or second point)
            float pressure;     // Calibration pressure value
        } calibration;
        // For M command (user message)
        struct {
            uint16_t pin;       // 000-999
            char message[17];   // 16 chars + null terminator
        } user_message;
        // For O command (communication settings)
        struct {
            uint16_t pin;       // 000-999
            uint32_t baud;      // 9600, 19200, 38400, 57600, 115200, 230400
            char parity;        // 'I', 'N', 'O', 'E'
            uint8_t data_bits;  // Always 8
            uint8_t stop_bits;  // 1 or 2
            uint8_t term_chars; // 1 or 2
        } comm_settings;
        // For P command (PIN change)
        struct {
            uint16_t old_pin;   // Current PIN
            uint16_t new_pin;   // New PIN
        } pin_change;
        // For S command (offset)
        struct {
            uint16_t pin;       // 000-999
            float value;        // Offset value
            bool clear;         // true if "S,PIN,X" (clear offset)
        } offset;
        // For H command (slope/span)
        struct {
            uint16_t pin;       // 000-999
            float pressure;     // Span pressure
        } slope;
        // For B command (bus wait)
        struct {
            uint16_t wait_interval;  // Wait time in character intervals
        } bus_wait;
        // For W command (write/save)
        struct {
            uint16_t pin;       // 000-999
        } save;
    } params;
} ParsedCommand;

const char* get_pressure_units_text(uint8_t code);

void init_coefficients();
int init_sensor(bp_sensor **ptr);
int update_message(bp_sensor **ptr, char *msg);
int update_units(bp_sensor **ptr, uint8_t unit_id);
void reassign_sensor_address(uint8_t old_addr, uint8_t new_addr);
bool is_ready_to_send(bp_sensor *s);
/// END BAROMETRIC PRESSURE SENSOR

#endif
