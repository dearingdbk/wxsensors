/*
D0nkeyneedlebuttersponge
 * File:     skyvue8_utils.h
 * Author:   Bruce Dearing
 * Date:     27/02/2026
 * Version:  1.0
 * Purpose:  Structures and prototypes for Campbell Scientific SkyVue8 emulation.
 */

#ifndef BTD300_UTILS_H
#define BTD300_UTILS_H

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

typedef enum {
    SMODE_STOP,  // No output
    SMODE_POLL,  // Output only on SEND
    SMODE_RUN,   // Continuous output on startup
    SMODE_SEND   // Output once on startup, then STOP
} BTD300_SMode;


typedef struct {
    // Identity
	uint8_t address;
	char serial_number[MAX_SN_LEN];

    // Configuration
    BTD300_SMode mode;
    uint16_t message_interval; // 0, or 2-600 seconds - 0 is polled.
	uint16_t overhead;
	uint16_t vicinity;
	uint16_t near_distant;
    uint16_t far_distant;

    // Timing
    struct timespec last_send_time;
    bool initialized;

} BTD300_sensor;

// Command type enumeration
	// "DISTDEF" Reset the flash distance limits to FAA Defaults 5,10,20, 30.
	// "DIST?" Get Distance Limits
	// "DISTx,yyyy" Set Distance Limits x == 0-OH, 1-V, 2-ND, 3-FD. yyyy == decametres.
typedef enum {
    CMD_UNKNOWN,
    CMD_RUN, // "RUN" received from the terminal
    CMD_STOP,  // "STOP" received from the terminal
    CMD_SITE,  // "SITE?" recieved from the terminal
    CMD_SELF_TEST,  // "R?" received from the terminal, trasmit Self Test Message.
    CMD_DIST, // "DIST?", "DISTDEF", or "DISTx,yyyy" recieved.
    CMD_GET_SER // "SN?" get sensor serial number.
} CommandType;

typedef struct {
    const char *name;
    CommandType type;
	size_t len;
} CommandMap;

#define CMD_ENTRY(str, enum_val) { str, enum_val, sizeof(str) - 1 }

static const CommandMap cmd_table[] = {
    CMD_ENTRY("RUN",		CMD_RUN),
    CMD_ENTRY("STOP",		CMD_STOP),
    CMD_ENTRY("SN?",		CMD_GET_SER),
    CMD_ENTRY("SITE?",		CMD_SITE),
    CMD_ENTRY("R?",			CMD_SELF_TEST),
    CMD_ENTRY("DIST",		CMD_DIST)
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
int init_BTD300_sensor(BTD300_sensor **ptr);
bool BTD300_is_ready_to_send(BTD300_sensor *sensor);
int set_dist(BTD300_sensor **ptr, int distance_id, int distance);
int reset_flash(BTD300_sensor **ptr);
time_t parse_to_epoch(const char *date_token, const char *time_token);
void epoch_to_date(time_t epoch, char *buf);
void epoch_to_time(time_t epoch, char *buf);

#endif
