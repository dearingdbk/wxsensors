/*
 * File:     hc2a_utils.h
 * Author:   Bruce Dearing
 * Date:     04/08/2026
 * Version:  1.0
 * Purpose:  Structures and prototypes for HC2A-S3 RH/Temp Sensor emulation.
 */

#ifndef HC2A_UTILS_H
#define HC2A_UTILS_H

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

typedef struct {
    float  samples[TREND_HISTORY_LEN];
    size_t head;   // index that will be written next
    size_t count;  // number of valid samples so far, caps at TREND_HISTORY_LEN
} trend_tracker_t;

typedef enum {
    SMODE_M1,  // Analog mode
    SMODE_M2   // ASCII Polled
} HC2A_SMode;

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
    HC2A_SMode mode;
    long output_rate; // 1-10 outputs per second, default is 4 (once every 0.25 seconds) stored as nanoseconds.
    // Timing
    trend_tracker_t rh_trend;
    trend_tracker_t temp_trend;
    struct timespec last_send_time;
    struct timespec sensor_start_time;
	struct tm sensor_time;
    bool initialized;
} HC2A_sensor;

// Command type enumeration
typedef enum {
    CMD_UNKNOWN,	// Unrecognized Command
    CMD_RDD, 	    // "RDD" received from the terminal.
    CMD_REN,        // "REN" received, Change the RS-485 address.
    CMD_HCA,        // "HCA" received, probe adjustment.
    CMD_LGC,        // "LGC" received, data recording function.
    CMD_ERD,        // "ERD" received, read recorded data.
    CMD_TID,        // "TID" received, set date and time.
    CMD_HRD	     	// "HRD" recieved, read out data memory.
} CommandType;

typedef struct {
    const char *name;
    CommandType type;
	size_t len;
} CommandMap;

#define CMD_ENTRY(str, enum_val) { str, enum_val, sizeof(str) - 1 }

static const CommandMap cmd_table[] = {
    CMD_ENTRY("RDD",		CMD_RDD),
    CMD_ENTRY("REN",    	CMD_REN),
    CMD_ENTRY("HCA",    	CMD_HCA),
    CMD_ENTRY("LGC",    	CMD_LGC),
    CMD_ENTRY("ERD",	    CMD_ERD),
    CMD_ENTRY("TID",    	CMD_TID),
    CMD_ENTRY("HRD",	    CMD_HRD)
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
int init_HC2A_sensor(HC2A_sensor **ptr);
bool HC2A_is_ready_to_send(HC2A_sensor *sensor);
char trend_tracker_update(trend_tracker_t *t, float value);
#endif
