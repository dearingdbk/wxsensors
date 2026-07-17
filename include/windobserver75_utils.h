/*
 * File:     windobserver75_utils.h
 * Author:   Bruce Dearing
 * Date:     15/07/2026
 * Version:  1.0
 * Purpose:  Structures and prototypes for Gill Wind Observer 75 emulation.
 */

#ifndef WO75_UTILS_H
#define WO75_UTILS_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#define MAX_FORM_STR 128
#define MAX_SN_LEN 16
#define MAX_UNIT_STR 50
#define DATE_STRING 7
#define TIME_STRING 7
#define MAX_HEADER_STR 7
#define MAX_SELF_TEST_FLAG 6
#define NS_PER_SEC 1000000000LL

// Index 0 is unused (or set to 0 to disable continuous sending)
static const long HZ_TO_NANOSECONDS[] = {
	0,                 // 0 Hz (Polled/Stop)
    1000000000L,       // 1 Hz
    500000000L,        // 2 Hz
    333333333L,        // 3 Hz
	250000000L,        // 4 Hz (Default)
	200000000L,        // 5 Hz
	166666666L,        // 6 Hz
	142857142L,        // 7 Hz
	125000000L,        // 8 Hz
	111111111L,        // 9 Hz
	100000000L         // 10 Hz
};

static const char NUM_TO_UNITS[] = {
	'X',	// 0 = NIL
	'M',	// U1 - Metres per second (m/s)
	'N',	// U2 - Knots (knots)
	'P',	// U3 - Miles per hour (MPH)
	'K',	// U4 - Kilometres per hour (kph)
	'F'		// U5 - feet per minute (fpm)
};

typedef enum {
    SMODE_M1,  // ASCII UV Continuous
    SMODE_M2,  // ASCII POLAR Continuous
    SMODE_M3,  // ASCII UV Polled
    SMODE_M4,  // ASCII POLAR Polled
    SMODE_M5,  // NMEA Continuous
    SMODE_M14, // ASCII POLAR Polled Averaged
    SMODE_M15  // ASCII POLAR Continuous Averaged
} WO75_SMode;

typedef struct {
    // Identity
	char address; // A-Z
	char units;	  // U1 through U5, M, N, P, K, F default = m/s
    // Configuration
    WO75_SMode mode;
    long output_rate; // 1-10 outputs per second, default is 4 (once every 0.25 seconds) stored as nanoseconds.
    // Timing
    struct timespec last_send_time;
    struct timespec sensor_start_time;
	struct tm sensor_time;
    bool initialized;
} WO75_sensor;

// Command type enumeration
typedef enum {
    CMD_UNKNOWN,	// Unrecognized Command
    CMD_ENABLE, 	// "?" received from the terminal, enable polled mode.
    CMD_POLL,  		// "<A-Z>" received from the terminal, send output generated.
    CMD_DISABLE,  	// "!" recieved from the terminal, disable polled mode.
    CMD_UNIT_ID,  	// "&" received from the terminal, send unit ID.
    CMD_CONFIG	 	// "*<A-Z>" recieved from the terminal, enter configuration mode.
} CommandType;

typedef struct {
    const char *name;
    CommandType type;
	size_t len;
} CommandMap;

#define CMD_ENTRY(str, enum_val) { str, enum_val, sizeof(str) - 1 }

static const CommandMap cmd_table[] = {
    CMD_ENTRY("?",			CMD_ENABLE),
    CMD_ENTRY("A",			CMD_POLL),
    CMD_ENTRY("B",			CMD_POLL),
    CMD_ENTRY("C",			CMD_POLL),
    CMD_ENTRY("D",			CMD_POLL),
    CMD_ENTRY("E",			CMD_POLL),
    CMD_ENTRY("F",			CMD_POLL),
    CMD_ENTRY("G",			CMD_POLL),
    CMD_ENTRY("H",			CMD_POLL),
    CMD_ENTRY("I",			CMD_POLL),
    CMD_ENTRY("J",			CMD_POLL),
    CMD_ENTRY("K",			CMD_POLL),
    CMD_ENTRY("L",			CMD_POLL),
    CMD_ENTRY("M",			CMD_POLL),
    CMD_ENTRY("N",			CMD_POLL),
    CMD_ENTRY("O",			CMD_POLL),
    CMD_ENTRY("P",			CMD_POLL),
    CMD_ENTRY("Q",			CMD_POLL),
    CMD_ENTRY("R",			CMD_POLL),
    CMD_ENTRY("S",			CMD_POLL),
    CMD_ENTRY("T",			CMD_POLL),
    CMD_ENTRY("U",			CMD_POLL),
    CMD_ENTRY("V",			CMD_POLL),
    CMD_ENTRY("W",			CMD_POLL),
    CMD_ENTRY("X",			CMD_POLL),
    CMD_ENTRY("Y",			CMD_POLL),
    CMD_ENTRY("Z",			CMD_POLL),
    CMD_ENTRY("!",			CMD_DISABLE),
    CMD_ENTRY("&",			CMD_UNIT_ID),
    CMD_ENTRY("*",			CMD_CONFIG)
};

#define CMD_TABLE_SIZE (sizeof(cmd_table) / sizeof(CommandMap))

// Parsed command structure
typedef struct {
    CommandType type;
    char sensor_id;           // Target sensor ID (A-Z)
    char raw_params[MAX_FORM_STR];
} ParsedCommand;


// Parsed message structure
typedef struct {
	char msg_address;
	uint16_t wind_direction;
	float wind_speed;
	char msg_units;
	uint8_t msg_status;
} ParsedMessage;

// Function Prototypes
int init_WO75_sensor(WO75_sensor **ptr);
bool WO75_is_ready_to_send(WO75_sensor *sensor);

#endif
