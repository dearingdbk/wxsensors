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
#define MAX_WIN_TX 4
#define MAX_HEIGHT_STR 6
#define MAX_ALARM_WRD 5
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
/*
typedef struct {
    char serial_number[MAX_SN_LEN];
	double pressure;
	double temperature;
	char batch_num[MAX_BATCH_NUM];
} BAROModule;
*/

typedef struct {
    // Identity
    char serial_number[MAX_SN_LEN];
	char product_name[MAX_PROD_NAME];
    char software_version[12];
    uint8_t address; // Sensor Identification Number
	char batch_num[MAX_BATCH_NUM];

    // Configuration
    Skyvue8_SMode mode;
    // PTB330_Unit units;
//    char format_string[MAX_FORM_STR];
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
//    double offset;            // Linear adjustment
//    double hcp_altitude;      // Height Corrected Pressure alt
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

/*typedef struct {
    CommandType type;
    char raw_params[MAX_FORM_STR];
    int addr_target;         // For RS-485 addressing
} ptb330_command;
*/

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
	char win_tx_per[MAX_WIN_TX];
	// Cloud Heights
	char first_height[MAX_HEIGHT_STR];
	char second_height[MAX_HEIGHT_STR];
	char third_height[MAX_HEIGHT_STR];
	char fourth_height[MAX_HEIGHT_STR];
	// Alarm Words
	char most_sig_alarm[MAX_ALARM_WRD];
	char middle_sig_alarm[MAX_ALARM_WRD];
	char least_sig_alarm[MAX_ALARM_WRD];
	uint8_t address;
} ParsedMessage;

/*
typedef enum {
    FORM_LITERAL, 		// Anything user defined within quotes.
    FORM_VAR_P, 		// P
    FORM_VAR_P1, 		// P1
    FORM_VAR_P2,		// P2
    FORM_VAR_P3,		// P3
    FORM_VAR_ERR,		// ERR
    FORM_VAR_P3H,		// P3H
    FORM_VAR_DP12,		// DP12
    FORM_VAR_DP13,		// DP13
    FORM_VAR_DP23,		// DP23
    FORM_VAR_HCP,		// HCP
    FORM_VAR_QFE,		// QFE
    FORM_VAR_QNH,		// QNH
    FORM_VAR_TP1,		// TP1
    FORM_VAR_TP2,		// TP2
    FORM_VAR_TP3,		// TP3
    FORM_VAR_A3H,		// A3H Tendency
	FORM_VAR_UNIT,		// Un
	FORM_VAR_CS2,		// CS2
	FORM_VAR_CS4,		// CS4
	FORM_VAR_CSX,		// CSX
	FORM_VAR_SN,		// SN
	FORM_VAR_PSTAB,		// PSTAB
	FORM_VAR_ADDR,		// ADDR
	FORM_VAR_DATE,		// DATE
	FORM_VAR_TIME		// TIME
    // ... add others as needed
} FormItemType;

typedef struct {
    FormItemType type;
    char literal[MAX_LITERAL_SIZE]; // For things like " ", " hPa", or ":" 32 chars.
	uint8_t width; // 0 means natural length, 1-9 is fixed width.
	uint8_t precision; // y in the x.y or n.n field.
} FormItem;

// Store the compiled format here
// FormItem compiled_form[MAX_FORM_ITEMS];
// int form_item_count = 0;

*/
// Function Prototypes
int init_skyvue8_sensor(skyvue8_sensor **ptr);
bool skyvue8_is_ready_to_send(skyvue8_sensor *sensor);
//void ptb330_parse_command(const char *input, ptb330_command *cmd);
//void ptb330_format_output(ptb330_sensor *sensor, char *dest, size_t max_len);
//void parse_form_string(const char *input);
//void build_dynamic_output(ParsedMessage *live_date, char *output_buf, size_t buf_len);
//double get_hcp_pressure(double station_p, double altitude_m);
//const char* get_unit_str(PTB330_Unit unit);

#endif
