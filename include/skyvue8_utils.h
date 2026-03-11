/*
 * File:     skyvue8_utils.h
 * Author:   Bruce Dearing
 * Date:     27/02/2026
 * Version:  1.0
 * Purpose:  Structures and prototypes for Campbell Scientific SkyVue8 emulation.
 */

#ifndef SKYVUE8_UTILS_H
#define SKYVUE8_UTILS_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#define MAX_FORM_STR 128
#define MAX_ADDR_LEN 4
#define MAX_SN_LEN 16
#define MAX_PROD_NAME 10
#define MAX_BATCH_NUM 64
#define MAX_LITERAL_SIZE 30
#define MAX_FORM_ITEMS 50
#define MAX_INTV_STR 16
#define MAX_DATE_STR 11
#define MAX_TIME_STR 10
#define MAX_UNIT_STR 32
#define MAX_HEIGHT_STR 6
#define SECONDS_IN_MIN 60
#define SECONDS_IN_HOUR 3600
#define SECONDS_IN_DAY 86400


typedef enum {
    SMODE_STOP,  // No output
    SMODE_POLL,  // Output only on SEND
    SMODE_RUN,   // Continuous output on startup
    SMODE_SEND   // Output once on startup, then STOP
} Skyvue8_SMode;

typedef enum {
	MSG_000 = 0,	// No Message
    MSG_001 = 1,	// No Profile / No Sky Condition
    MSG_002 = 2,  	// Profile / No Sky Condition
    MSG_003 = 3,  	// No Profile / Sky Condition
    MSG_004 = 4,   	// Deafault
	MSG_101 = 101,	// CL31 MSG 1 - 770 range bins 10 m resolution
	MSG_102 = 102,	// CL31 MSG 1 - 385 range bins 20 m resolution
	MSG_103 = 103,	// CL31 MSG 1 - 1500 range bins 5 m resolution
	MSG_104 = 104,	// CL31 MSG 1 - 770 range bins 5 m resolution
	MSG_105 = 105,	// CL31 MSG 1 - No Profile
	MSG_106 = 106,	// CL31 MSG 1 - 2048 range bins 5 m resolution
	MSG_107 = 107,	// CL31 MSG 2 - 770 range bins 10 m resolution
	MSG_108 = 108,	// CL31 MSG 2 - 385 range bins 20 m resolution
	MSG_109 = 109,	// CL31 MSG 2 - 1500 range bins 5 m resolution
	MSG_110 = 110,	// CL31 MSG 2 - 770 range bins 5 m resolution
	MSG_111 = 111,	// CL31 MSG 2 - No Profile
	MSG_112 = 112,	// CL31 MSG 2 - 2048 range bins 5 m resolution
	MSG_113 = 113,	// CT25K MSG 1
	MSG_114 = 114	// CT25K MSG 6
} Skyvue8_MessageID;

typedef enum {
	RS232_FULL = 0,
	RS232_HALF = 1,
	RS485_FULL = 2,
	RS485_HALF = 3,
	RESERVED = 4,
	RS422_FULL = 5
} Skyvue8_SerialMode;

typedef enum {
    DATA_8 = 8,  // 8 bit // Default
    DATA_7 = 7   // 7 bit
} DataFormat;


typedef enum {
    STOP_1 = 1,  // 1 bit // Default
    STOP_2 = 2   // 2 bit
} StopBits;

typedef enum {
    EVEN = 'E',
	ODD = 'O',
	NONE = 'N' // Defualt
} ParityFormat;


typedef enum {
	BAUD_300 = 0,
	BAUD_600 = 1,
    BAUD_1200 = 2,
    BAUD_2400 = 3,
	BAUD_4800 = 4,
	BAUD_9600 = 5,
    BAUD_19200 = 6,
    BAUD_38400 = 7,
    BAUD_57600 = 8,
    BAUD_76800 = 9,
	BAUD_115200 = 10 // Default
} BaudRateCode;


typedef struct {
    const uint32_t baud_num;
    BaudRateCode code;
} BaudCodeMap;

static const BaudCodeMap baud_table[] = {
	{300, 		BAUD_300},
	{600, 		BAUD_600},
	{1200, 		BAUD_1200},
	{2400, 		BAUD_2400},
	{4800, 		BAUD_4800},
	{9600, 		BAUD_9600},
	{19200, 	BAUD_19200},
	{38400, 	BAUD_38400},
	{57600, 	BAUD_57600},
	{76800,		BAUD_76800},
	{115200, 	BAUD_115200}
};

typedef struct {
    // Identity
    char serial_number[MAX_SN_LEN];
	char product_name[MAX_PROD_NAME];
    char software_version[12];
    char address; // Sensor Identification Number 0-9, a-z, or A-Z.
	char batch_num[MAX_BATCH_NUM];

    // Configuration
    Skyvue8_SMode mode;
    uint16_t send_delay;     // ms
	char date_string[MAX_DATE_STR];
	char time_string[MAX_TIME_STR];

	// Communication
	BaudRateCode baud;
	ParityFormat parity;
	DataFormat data_f;
	StopBits stop_b;
	Skyvue8_SerialMode smode;
    uint16_t measurement_period; // 0, or 2-600 seconds  - 0 is polled.
    uint16_t message_interval; // 0, or 2-600 seconds - 0 is polled.

    // Timing
    struct timespec last_send_time;
    bool initialized;
} skyvue8_sensor;


// Command type enumeration
typedef enum {
    CMD_UNKNOWN,
// General Commands
	CMD_POLL,
// Error responses
    CMD_ERROR,          // General error
    CMD_INVALID_CRC,    // CRC check failed
    CMD_INVALID_ID,     // Sensor ID mismatch
    CMD_INVALID_FORMAT  // Command format error
} CommandType;

typedef struct {
    const char *name;
    CommandType type;
	size_t len;
} CommandMap;

#define CMD_ENTRY(str, enum_val) { str, enum_val, sizeof(str) - 1 }

static const CommandMap cmd_table[] = {
    CMD_ENTRY("POLL",     CMD_POLL)
};

#define CMD_TABLE_SIZE (sizeof(cmd_table) / sizeof(CommandMap))

// Parsed command structure
typedef struct {
    CommandType type;
    uint8_t sensor_id;           // Target sensor ID (0-9)
    bool crc_valid;
    uint16_t received_crc;
    uint16_t calculated_crc;
    char raw_params[MAX_FORM_STR];
	union {

		struct {

		} poll_params;

		struct {

		} unit_params;

	} params;
} ParsedCommand;


typedef enum {
    IS_ERROR = 1,
    NO_ERROR = 0
} SensorError;


// Parsed message structure
typedef struct {
	char detection_status;
	char alarm_status;
	uint8_t win_trans_per;
	// Cloud Heights
	char first_height[MAX_HEIGHT_STR];
	char second_height[MAX_HEIGHT_STR];
	char third_height[MAX_HEIGHT_STR];
	char fourth_height[MAX_HEIGHT_STR];
	// Alarm Words
	uint16_t most_sig_alarm;
	uint16_t middle_sig_alarm;
	uint16_t least_sig_alarm;
	uint8_t sensor_id;
	uint8_t message_id;
} ParsedMessage;

// Function Prototypes
int init_skyvue8_sensor(skyvue8_sensor **ptr);
bool skyvue8_is_ready_to_send(skyvue8_sensor *sensor);

#endif
