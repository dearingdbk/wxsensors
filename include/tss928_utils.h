/*
 * File:     tss928_utils.h
 * Author:   Bruce Dearing
 * Date:     15/04/2026
 * Version:  1.0
 * Purpose:  Structures and prototypes for Vaisala TSS928 emulation.
 */

#ifndef TSS928_UTILS_H
#define TSS928_UTILS_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#define MAX_FORM_STR 128
#define MAX_SN_LEN 16
#define MAX_UNIT_STR 32
#define DATE_STRING 7
#define TIME_STRING 7
#define MAX_HEADER_STR 7
#define MAX_SELF_TEST_FLAG 6
#define MAX_FLASHES 4
#define MAX_HISTORY_MINS 30
#define RANGE_RINGS 2 // 0:NEAR, 1:DIST
#define QUADRANTS 8 //0:N, 1:NE, 2:E, 3:SE, 4:S, 5:SW, 6:W, 7:NW

typedef enum {
    SMODE_STOP,  // No output
    SMODE_POLL,  // Output only on SEND (A command)
    SMODE_RUN,   // Periodic output on startup
    SMODE_FLASH  // Output on receipt of a flash
} TSS928_SMode;

typedef struct {
	// Ground Strikes [2 range rings][8 quadrants][30 mins]
	uint32_t ground_history[RANGE_RINGS][QUADRANTS][MAX_HISTORY_MINS];
	uint32_t ground_totals[RANGE_RINGS][QUADRANTS];

	// Overhead Strikes [30 mins]
	uint32_t overhead_history[MAX_HISTORY_MINS];
	uint32_t overhead_total;

	// Cloud Strikes [30 mins]
	uint32_t cloud_history[MAX_HISTORY_MINS];
	uint32_t cloud_total;

	uint8_t current_minute_index;
	uint8_t aging_interval; // 15, 10, 5 or 30 minutes (arguments to J command are 1,2,3,4)

} StrikeBin;

typedef struct {
    // Identity
	uint8_t address;
	char serial_number[MAX_SN_LEN];

    // Configuration
    TSS928_SMode mode;
    uint16_t message_interval; 	// 0, or 2-600 seconds - 0 is polled.
	StrikeBin strikes;
	uint16_t overhead;
	uint16_t vicinity;
	uint16_t near_distant;
    uint16_t far_distant;

    // Timing
    struct timespec last_send_time;
    bool initialized;

} TSS928_sensor;

// Command type enumeration
typedef enum {
    CMD_UNKNOWN,	// Unrecognized Command
    CMD_SEND, 		// "A" received from the terminal, send a present weather message.
    CMD_STATUS,  	// "B" or "*STATUS" received from the terminal, send a status message.
    CMD_SELFTEST,  	// "C" or "*SELFTEST" recieved from the terminal, send a selftest message.
    CMD_RESET,  	// "D" or "*RESET" received from the terminal, reset the sensor.
    CMD_TYPETEST, 	// "E" recieved from the terminal, perform a type test.
    CMD_RUNTIME, 	// "F" recieved from the terminal, send a system run time message.
    CMD_VERSION, 	// "G" or "*VERSION" recieved from the terminal, send a version message.
    CMD_FORMAT, 	// "H" or "*FORMAT" recieved from the terminal, set the data output message. Arguments [0-2] 0 == Poll.
    CMD_DISTANCE, 	// "I" recieved from the terminal, set the distance unit. Arguments [1-3] 1 == miles.
    CMD_AGING, 		// "J" recieved from the terminal, set the aging interval. Arguments [1-4] 1 == 15 minutes.
    CMD_DIAGNOSTIC, // "K" recieved from the terminal, set diagnostic mode, and run a test. Arguments [1-3].
    CMD_ANGLE, 		// "L" recieved from the terminal, set the angle of rotation. Arguments [0-359] 0 default.
    CMD_TIME, 		// "N" or "*TIME" recieved from the terminal, set the current time. Arguments [0-23]:[0-59]:[0-59] 00:00:00 default.
    CMD_NOISE, 		// "P" or "*NOISE" recieved from the terminal, return the # of optical and enable crossings. Arguments [0] 0 resets the count.
    CMD_EBRATIO,	// "R" or "*EBRATIO" recieved from the terminal, return the average and standard deviation of the last 20 E/B rations.
    CMD_COMMANDS,	// "?" or "*?" recieved from the terminal, list available commands.
    CMD_RESTORE 	// "" or *DEF recieved from the terminal, restore default settings.
} CommandType;

typedef struct {
    const char *name;
    CommandType type;
	size_t len;
} CommandMap;

#define CMD_ENTRY(str, enum_val) { str, enum_val, sizeof(str) - 1 }

static const CommandMap cmd_table[] = {
    CMD_ENTRY("A",			CMD_SEND),
    CMD_ENTRY("B",			CMD_STATUS),
    CMD_ENTRY("*STATUS",	CMD_STATUS),
    CMD_ENTRY("C",			CMD_SELFTEST),
    CMD_ENTRY("*SELFTEST",	CMD_SELFTEST),
    CMD_ENTRY("D",			CMD_RESET),
    CMD_ENTRY("*RESET",		CMD_RESET),
    CMD_ENTRY("E",			CMD_TYPETEST),
    CMD_ENTRY("F",			CMD_RUNTIME),
    CMD_ENTRY("G",			CMD_VERSION),
    CMD_ENTRY("*VERSION",	CMD_VERSION),
    CMD_ENTRY("H",			CMD_FORMAT),
    CMD_ENTRY("*FORMAT",	CMD_FORMAT),
    CMD_ENTRY("I",			CMD_DISTANCE),
    CMD_ENTRY("J",			CMD_AGING),
    CMD_ENTRY("K",			CMD_DIAGNOSTIC),
    CMD_ENTRY("L",			CMD_ANGLE),
    CMD_ENTRY("N",			CMD_TIME),
    CMD_ENTRY("*TIME",		CMD_TIME),
    CMD_ENTRY("P",			CMD_NOISE),
    CMD_ENTRY("*NOISE",		CMD_NOISE),
    CMD_ENTRY("R",			CMD_EBRATIO),
    CMD_ENTRY("*EBRATIO",	CMD_EBRATIO),
    CMD_ENTRY("?",			CMD_COMMANDS),
    CMD_ENTRY("*?",			CMD_COMMANDS),
    CMD_ENTRY("*DEF",		CMD_RESTORE)
};

#define CMD_TABLE_SIZE (sizeof(cmd_table) / sizeof(CommandMap))

// Parsed command structure
typedef struct {
    CommandType type;
    uint8_t sensor_id;           // Target sensor ID (0-9)
    char raw_params[MAX_FORM_STR];
} ParsedCommand;


// Parsed message structure
typedef struct {
	char data_header[MAX_HEADER_STR];
	uint8_t site_id;
	time_t original_epoch; // This will hold our date and time in the format of epoch.
	uint8_t number_of_flashes; // The number of flashes on this particular line.
	uint8_t warning_indicator; // 0, 1, 2 or 3.
	uint8_t warning_flags; //
	char self_test_flags[MAX_SELF_TEST_FLAG];
	time_t flash_epoch_array[MAX_FLASHES];
	// FLASH INFO
		// FLASH ONE
	uint8_t time_since_flash_one; // # of 10 millisecond intervals since flash one.
	uint16_t distance_of_flash_one; // Distance to Flash one.
	uint16_t direction_of_flash_one; // Direction of flash one in degrees.
		//FLASH TWO
	uint8_t time_since_flash_two; // # of 10 millisecond intervals since flash two.
	uint16_t distance_of_flash_two; // Distance to Flash two.
	uint16_t direction_of_flash_two; // Direction of flash two in degrees.
		//FLASH THREE
	uint8_t time_since_flash_three; // # of 10 millisecond intervals since flash three.
	uint16_t distance_of_flash_three; // Distance to Flash three.
	uint16_t direction_of_flash_three; // Direction of flash three in degrees.
		// FLASH FOUR
	uint8_t time_since_flash_four; // # of 10 millisecond intervals since flash four.
	uint16_t distance_of_flash_four; // Distance to Flash four.
	uint16_t direction_of_flash_four; // Direction of flash four in degrees.

} ParsedMessage;

// Function Prototypes
int init_TSS928_sensor(TSS928_sensor **ptr);
bool TSS928_is_ready_to_send(TSS928_sensor *sensor);
//int set_dist(TSS928_sensor **ptr, int distance_id, int distance);
//int reset_flash(TSS928_sensor **ptr);
//time_t parse_to_epoch(const char *date_token, const char *time_token);
//void epoch_to_date(time_t epoch, char *buf);
//void epoch_to_time(time_t epoch, char *buf);
void record_ground_strike(StrikeBin *bin, uint8_t ring_index, uint8_t quadrant_index);
void record_overhead_strike(StrikeBin *bin);
void record_cloud_strike(StrikeBin *bin);
void advance_one_minute(StrikeBin *bin);

#endif
