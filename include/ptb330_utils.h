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
#define MAX_BATCH_NUM 64
#define MAX_LITERAL_SIZE 32
#define MAX_FORM_ITEMS 50

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
    PTB330_Unit unit;
    const char *label;
    double multiplier; // Multiplier to convert from hPa to this unit
} UnitConversion;

static const UnitConversion unit_table[] = {
    {UNIT_HPA,   "hPa",  1.0},
    {UNIT_MBAR,  "mbar", 1.0},
    {UNIT_KPA,   "kPa",  0.1},
    {UNIT_PA,    "Pa",   100.0},
    {UNIT_INHG,  "inHg", 0.0295299},
    {UNIT_MMHG,  "mmHg", 0.750062},
    {UNIT_TORR,  "torr", 0.750062},
    {UNIT_PSI,   "psi",  0.0145038}
};


typedef enum {
    DATA_8 = 8,  // 8 bit
    DATA_7 = 7   // 7 bit
} DataFormat;


typedef enum {
    STOP_1 = 1,  // 1 bit
    STOP_2 = 2   // 2 bit
} StopBits;

typedef enum {
    EVEN = 'E',
	ODD = 'O',
	NONE = 'N'
} ParityFormat;


typedef enum {
	BAUD_110 = 0,
	BAUD_150 = 1,
	BAUD_300 = 2,
	BAUD_600 = 3,
    BAUD_1200 = 4,
    BAUD_2400 = 5,
	BAUD_4800 = 6, // Default
	BAUD_9600 = 7,
    BAUD_19200 = 8,
    BAUD_38400 = 9,
    BAUD_57600 = 10,
    BAUD_115200 = 11,
	BAUD_230400 = 12
} BaudRateCode;


typedef struct {
    const uint32_t baud_num;
    BaudRateCode code;
} BaudCodeMap;

static const BaudCodeMap baud_table[] = {
	{110, 		BAUD_110},
	{150, 		BAUD_150},
	{300, 		BAUD_300},
	{600, 		BAUD_600},
	{1200, 		BAUD_1200},
	{2400, 		BAUD_2400},
	{4800, 		BAUD_4800},
	{9600, 		BAUD_9600},
	{19200, 	BAUD_19200},
	{38400, 	BAUD_38400},
	{57600, 	BAUD_57600},
	{115200, 	BAUD_115200},
	{230400, 	BAUD_230400}
};

typedef struct {
    char serial_number[MAX_SN_LEN];
	float pressure;
	char batch_num[MAX_BATCH_NUM];

} BAROModule;

typedef struct {
    // Identity
    char serial_number[MAX_SN_LEN];
    char software_version[12];
    uint8_t address;
	char batch_num[MAX_BATCH_NUM];

    // Configuration
    PTB330_SMode mode;
    PTB330_Unit units;
    uint32_t interval;       // In seconds
    char format_string[MAX_FORM_STR];
    uint16_t send_delay;     // ms
    bool echo_enabled;

	// Communication
	BaudRateCode baud;
	ParityFormat parity;
	DataFormat data_f;
	StopBits stop_b;

    // Pressure State
    float pressure;          // Current reading from file
    float offset;            // Linear adjustment
    float hcp_altitude;      // Height Corrected Pressure alt
    // Timing
    struct timespec last_send_time;
    bool initialized;
	BAROModule module_one;
	BAROModule module_two;
	BAROModule module_three;
	BAROModule module_four;
} ptb330_sensor;


// Command type enumeration
typedef enum {
    CMD_UNKNOWN,
// General Commands
	CMD_BNUM, 	// Shows the device and module batch numbers.
	CMD_SERI, 	// Shows or sets the serial port settings for the user port.
	CMD_SNUM, 	// Shows the device and module serial numbers.
	CMD_ERRS, 	// Shows all unacknowledged errors (and clears them).
	CMD_HELP, 	// Shows the available commands.
	CMD_LOCK, 	// Shows or sets the keyboard lock.
	CMD_INFO, 	// '?' Outputs information on the device.
	CMD_ECHO, 	// Shows or sets the serial interface echoing.
	CMD_RESET, 	// Resets the device
	CMD_VERS, 	// Displays the product name and software version number.
	CMD_MODS, 	// Acknowledges added or removed modules.
	CMD_CON,	// Adjusts display contrast.

// Measurement Commands
	CMD_R, 		// Changes the serial mode to RUN and starts displaying measurement results according to the FORM string (with interval defined by INTV).
	CMD_S,
	CMD_INTV, 	// Shows or sets the continuous output interval (for RUN mode). 0–255 s/min/h/d
    CMD_SEND, 	// Shows the measurement results according to the configured form. 0-255.
	CMD_ADDR, 	// Sets the barometer address.
    CMD_SMODE, 	// Shows or sets the start mode. - STOP/POLL/RUN/SEND/PA11A, Default value STOP
	CMD_SDELAY, // 	Shows or sets the answer delay for the serial line in tens of milliseconds. 0–254 (0–2540 ms)
	CMD_OPEN, 	// Opens communications after the CLOSE command.
	CMD_CLOSE, 	// Closes communications until the OPEN command is entered.
	CMD_SCOM, 	// Shows or sets an alias (a user-specific form) for the SEND command. The given alias cannot be a command already in use.

// Measurement Setting Commands
	CMD_TQFE, 	// Shows or sets the temperature for QFE corrected pressure. −80 ... +200 °C
	CMD_DPMAX, 	// Shows or sets the maximum permissible difference pressure between barometer modules. 0–9999.99 hPa
	CMD_HHCP, 	// Shows or sets the altitude for height corrected pressure. −30 ... +30 m
	CMD_HQFE, 	// Shows or sets the altitude for the QFE corrected pressure. −30 ... +30 m
	CMD_HQNH, 	// Shows or sets the altitude for the QNH corrected pressure. −30 ... +3000 m
	CMD_ICAOQNH, // Selects the calculation formula and rounding method used for QNH and QFE calculations.
	CMD_PSTAB, 	// Shows or sets the pressure stability limits. 	0–9999.99 hPa
	CMD_AVRG, 	// Sets the barometer measurement averaging time (in seconds). 0-600 s

// Formatting Commands
	CMD_FORM, 	// Sets the custom output for the SEND command and for RUN mode.
	CMD_TIME, 	// Shows or changes the current time.
	CMD_DATE, 	// Shows or changes the current date.
	CMD_UNIT, 	// Shows or sets unit for a quantities.

// Data Recording Commands
	CMD_DSEL, 	// Selects the quantities that are displayed on the graphical user interface as well as quantities for data recording.
	CMD_DELETE, // Erases the log memory.
	CMD_UNDELETE, // Restores the erased log memory
	CMD_DIR, 	// Lists the available logs in the logging memory.
	CMD_PLAY, 	// Shows the trend, min, and max values of the given log.

// Calibration and Adjustment Commands
	CMD_CDATE, 	// Shows or sets the calibration date.
	CMD_LCP1, 	// Performs a linear adjustment for the barometer module/module.
	CMD_LCP2, 	// Performs a linear adjustment for the barometer module/module.
	CMD_LCP3, 	// Performs a linear adjustment for the barometer module/module.
	CMD_MPCP1, 	// Performs a multipoint adjustment for the barometer module/module.
	CMD_MPCP2, 	// Performs a multipoint adjustment for the barometer module/module.
	CMD_MPCP3, 	// Performs a multipoint adjustment for the barometer module/module.
	CMD_CTEXT, 	// 	Shows or sets the calibration info text.

// Setting and testing the analog output Commands
	CMD_AMODE, 	// Displays the analog output mode (if an AOUT-1 module is connected).
	CMD_ASEL, 	// Sets the analog output quantity and scaling (low/high).
	CMD_ACAL, 	// Adjusts the analog output.
	CMD_AERR, 	// Sets the analog output error value.
	CMD_ATEST, 	// Starts/ends an analog output test.

// Setting and Testing Commands
	CMD_RSEL,	// Sets the relay scaling (if a RELAY-1 module is connected)
	CMD_RTEST, 	// Starts/ends a relay output test.

// Error responses
    CMD_ERROR,          // General error
    CMD_INVALID_CRC,    // CRC check failed
    CMD_INVALID_ID,     // Sensor ID mismatch
    CMD_INVALID_FORMAT  // Command format error
} CommandType;

typedef struct {
    CommandType type;
    char raw_params[128];
    int addr_target;         // For RS-485 addressing
} ptb330_command;


typedef struct {
    const char *name;
    CommandType type;
	size_t len;
} CommandMap;

#define CMD_ENTRY(str, enum_val) { str, enum_val, sizeof(str) - 1 }

static const CommandMap cmd_table[] = {
    CMD_ENTRY("UNDELETE", CMD_UNDELETE), 	CMD_ENTRY("DELETE",	CMD_DELETE),
    CMD_ENTRY("BNUM",     CMD_BNUM),		CMD_ENTRY("SERI",     CMD_SERI),
    CMD_ENTRY("SNUM",     CMD_SNUM),    	CMD_ENTRY("ERRS",     CMD_ERRS),
    CMD_ENTRY("HELP",     CMD_HELP),    	CMD_ENTRY("?",       CMD_INFO),
    CMD_ENTRY("LOCK",     CMD_LOCK),    	CMD_ENTRY("ECHO",     CMD_ECHO),
    CMD_ENTRY("RESET",    CMD_RESET),   	CMD_ENTRY("VERS",     CMD_VERS),
    CMD_ENTRY("MODS",     CMD_MODS),    	CMD_ENTRY("CON",      CMD_CON),
    CMD_ENTRY("INTV",     CMD_INTV),    	CMD_ENTRY("SEND",     CMD_SEND),
    CMD_ENTRY("ADDR",     CMD_ADDR),		CMD_ENTRY("CDATE",	CMD_CDATE),
    CMD_ENTRY("SMODE",    CMD_SMODE),		CMD_ENTRY("SDELAY",   CMD_SDELAY),
    CMD_ENTRY("OPEN",     CMD_OPEN),		CMD_ENTRY("CLOSE",    CMD_CLOSE),
    CMD_ENTRY("SCOM",     CMD_SCOM),		CMD_ENTRY("TQFE",     CMD_TQFE),
    CMD_ENTRY("DPMAX",    CMD_DPMAX),   	CMD_ENTRY("HHCP",     CMD_HHCP),
    CMD_ENTRY("HQFE",     CMD_HQFE),    	CMD_ENTRY("HQNH",     CMD_HQNH),
    CMD_ENTRY("ICAOQNH",  CMD_ICAOQNH), 	CMD_ENTRY("PSTAB",    CMD_PSTAB),
    CMD_ENTRY("AVRG",     CMD_AVRG),    	CMD_ENTRY("FORM",     CMD_FORM),
    CMD_ENTRY("TIME",     CMD_TIME),    	CMD_ENTRY("DATE",     CMD_DATE),
    CMD_ENTRY("UNIT",     CMD_UNIT),    	CMD_ENTRY("DSEL",     CMD_DSEL),
    CMD_ENTRY("DIR",      CMD_DIR),     	CMD_ENTRY("PLAY",     CMD_PLAY),
    CMD_ENTRY("LCP1",     CMD_LCP1),    	CMD_ENTRY("LCP2",     CMD_LCP2),
    CMD_ENTRY("LCP3",     CMD_LCP3),    	CMD_ENTRY("MPCP1",    CMD_MPCP1),
    CMD_ENTRY("MPCP2",    CMD_MPCP2),   	CMD_ENTRY("MPCP3",    CMD_MPCP3),
	CMD_ENTRY("CTEXT",    CMD_CTEXT),   	CMD_ENTRY("AMODE",    CMD_AMODE),
	CMD_ENTRY("ASEL",     CMD_ASEL),		CMD_ENTRY("ACAL",     CMD_ACAL),
    CMD_ENTRY("AERR",     CMD_AERR),    	CMD_ENTRY("ATEST",    CMD_ATEST),
    CMD_ENTRY("RSEL",     CMD_RSEL),    	CMD_ENTRY("RTEST",    CMD_RTEST),
    CMD_ENTRY("R",		 CMD_R)
};

#define CMD_TABLE_SIZE (sizeof(cmd_table) / sizeof(CommandMap))

// Parsed command structure
typedef struct {
    CommandType type;
    uint8_t sensor_id;           // Target sensor ID (0-9)
    bool crc_valid;
    uint16_t received_crc;
    uint16_t calculated_crc;
    char raw_params[128];
	union {

		struct {

		} form_params;

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
	float p1_pressure;
	float p2_pressure;
	float p3_pressure;
	SensorError p1_sensor_error;
	SensorError p2_sensor_error;
	SensorError p3_sensor_error;
	float p_average;
	float trend;
} ParsedMessage;

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
	FORM_VAR_T,			// \T #T
	FORM_VAR_R,			// \R #R
	FORM_VAR_N,			// \N #N
	FORM_VAR_RN,		// \RN #RN
	FORM_VAR_UNIT,		// Un
	FORM_VAR_NN,		// n.n
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


// Function Prototypes
int init_ptb330_sensor(ptb330_sensor **ptr);
bool ptb330_is_ready_to_send(ptb330_sensor *sensor);
void ptb330_parse_command(const char *input, ptb330_command *cmd);
void ptb330_format_output(ptb330_sensor *sensor, char *dest, size_t max_len);
void parse_form_string(const char *input);

#endif
