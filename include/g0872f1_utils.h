/*
 * File:     g0872f1_utils.h
 * Author:   Bruce Dearing
 * Date:     17/08/2026
 * Version:  1.0
 * Purpose:  Structures and prototypes for Goodrich 0872F1 ice Sensor emulation.
 */

#ifndef G0872F1_UTILS_H
#define G0872F1_UTILS_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#define MAX_FORM_STR 128
#define MAX_SN_LEN 16
#define MAX_NAME_STR 20
#define MAX_FIRM_VER 10
#define MAX_UNIT_STR 50
#define DATE_STRING 7
#define TIME_STRING 7
#define MAX_HEADER_STR 7
#define MAX_SELF_TEST_FLAG 6
#define NS_PER_SEC 1000000000LL
#define TREND_SAMPLE_INTERVAL_S 10
#define TREND_WINDOW_S          60
#define TREND_HISTORY_LEN       (TREND_WINDOW_S / TREND_SAMPLE_INTERVAL_S + 1) // 7
#define TREND_THRESHOLD         0.02f


typedef enum {
    SMODE_M1,  // Analog mode
    SMODE_M2   // ASCII Polled
} G0872F1_SMode;

typedef struct {
    // Identity
    char unit_ident;     // A-F, default is F.
	uint8_t address;     // 0-99
	uint8_t probe_type;  // 0-3
    uint8_t device_type; // 0-255
    char serial_number[MAX_SN_LEN];
    char device_name[MAX_NAME_STR];
    char firmware_version[MAX_FIRM_VER];
    // Configuration
    G0872F1_SMode mode;
    long output_rate; // 1-10 outputs per second, default is 4 (once every 0.25 seconds) stored as nanoseconds.
    // Timing
    struct timespec last_send_time;
    struct timespec sensor_start_time;
	struct tm sensor_time;
    bool initialized;
} G0872F1_sensor;

// Command type enumeration
typedef enum {
    CMD_UNKNOWN,	// Unrecognized Command
    CMD_Z1, // "Z1" received from the terminal - Send Frequency Data.
    CMD_Z3, // "Z3##" received from the terminal - De-ice strut and probe.
    CMD_Z4, // "Z4" recieved from the terminal - Perform extended diagnostics.
    CMD_F4  // "F4" received from the terminal - Perform Field Calibration.
} CommandType;

typedef struct {
    const char *name;
    CommandType type;
	size_t len;
} CommandMap;

#define CMD_ENTRY(str, enum_val) { str, enum_val, sizeof(str) - 1 }

static const CommandMap cmd_table[] = {
    CMD_ENTRY("Z1",		CMD_Z1),
    CMD_ENTRY("Z3",    	CMD_Z3),
    CMD_ENTRY("Z4",    	CMD_Z4),
    CMD_ENTRY("F4",    	CMD_F4)
};

#define CMD_TABLE_SIZE (sizeof(cmd_table) / sizeof(CommandMap))

// Parsed command structure
typedef struct {
    CommandType type;
    char cmd_unit_ident;
    uint8_t sensor_id;  // Target sensor ID (A-Z)
    char raw_params[MAX_FORM_STR];
} ParsedCommand;


// Parsed message structure
typedef struct {
	uint8_t msg_address;
	float rel_humidity;
    float temperature;
    uint8_t rh_alarm;
    uint8_t temp_alarm;
    uint8_t alarm_byte;
} ParsedMessage;

// Function Prototypes
int init_G0872F1_sensor(G0872F1_sensor **ptr);
bool G0872F1_is_ready_to_send(G0872F1_sensor *sensor);
#endif
