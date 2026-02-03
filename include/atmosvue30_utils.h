/*
 * File:     atmosvue30_utils.h
 * Author:   Bruce Dearing
 * Date:     23/01/2026
 * Version:  1.0
 * Purpose:  Header file for AtmosVUE 30 Aviation Weather System emulation.
 *           Defines data structures, enumerations, and function prototypes
 *           for handling Campbell Scientific AtmosVUE 30 communication protocol.
 * Mods:
 *
 *
 */

#ifndef ATMOSVUE30_UTILS_H
#define ATMOSVUE30_UTILS_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#define MAX_INPUT_STR 512
#define MAX_SERIAL_STR 16
#define MAX_MODEL_NUM 32
#define MAX_USER_MSG 17
#define MAX_ADDRESS_NUM 9
// AtmosVUE 30 constants
#define MIN_VISIBILITY_M 5
#define MAX_VISIBILITY_M 100000
#define MIN_VISIBILITY_FT 16
#define MAX_VISIBILITY_FT 328084
#define MAX_LUMINANCE 45000
#define MAX_TEMP 70
#define MIN_TEMP -40
#define MAX_HUMIDITY 100
#define MAX_PRECIP_RATE 999.9
#define MAX_PRECIP_ACCUM 999.9
#define MAX_CONT_INTERVAL 36000
// Message format types
extern const char* message_format_names[15];

// SYNOP codes
extern const char* synop_codes[90];

// METAR codes
extern const char* metar_codes[20];

// Units of measure
typedef enum {
    UNITS_METRES = 0,
    UNITS_FEET = 1
} VisibilityUnits;

// Message formats
typedef enum {
    MSG_BASIC = 0,
    MSG_PARTIAL = 1,
    MSG_FULL = 2,
    MSG_BASIC_SYNOP = 3,
    MSG_PARTIAL_SYNOP = 4,
    MSG_FULL_SYNOP = 5,
    MSG_BASIC_METAR = 6,
    MSG_PARTIAL_METAR = 7,
    MSG_FULL_METAR = 8,
    MSG_GENERIC_BASIC_SYNOP = 9,
    MSG_GENERIC_PARTIAL_SYNOP = 10,
    MSG_GENERIC_FULL_SYNOP = 11,
    MSG_CUSTOM = 12,
    MSG_VAISALA_FD12 = 13,
    MSG_RVR_OUTPUT = 14
} MessageFormat;

// Operating modes
typedef enum {
    MODE_CONTINUOUS = 0,
    MODE_POLLING = 1
} OperatingMode;

// Communication types
typedef enum {
    COMM_RS232 = 0,
    COMM_RS485 = 1
} CommType;

// Baud rates
typedef enum {
    BAUD_1200 = 0,
    BAUD_2400 = 1,
    BAUD_38400 = 2,  // Default
    BAUD_19200 = 3,
    BAUD_57600 = 4,
    BAUD_115200 = 5
} BaudRateCode;

// Data format
typedef enum {
    DATA_8N1 = 0,  // 8 bit, no parity
    DATA_7E1 = 1   // 7 bit, even parity
} DataFormat;

// Averaging periods
typedef enum {
    AVG_1_MINUTE = 1,
    AVG_10_MINUTE = 10
} AveragingPeriod;

// System status
typedef enum {
    STATUS_NO_FAULT = 0,
    STATUS_POSSIBLE_DEGRADED = 1,
    STATUS_DEGRADED = 2,
    STATUS_MAINTENANCE_REQUIRED = 3
} SystemStatus;

// MOR format
typedef enum {
    MOR_FORMAT_MOR = 0,
    MOR_FORMAT_TMOR = 1
} MORFormat;

// System alarm levels (0-3 for each alarm type)
typedef struct {
    uint8_t emitter_failure;        // 0-2: LED output power level
    uint8_t emitter_lens_dirty;     // 0-3: Window contamination
    uint8_t emitter_temperature;    // 0-3: Temperature status
    uint8_t detector_lens_dirty;    // 0-3: Window contamination
    uint8_t detector_temperature;   // 0-3: Temperature status
    uint8_t detector_dc_saturation; // 0-1: Background light saturation
    uint8_t hood_temperature;       // 0-3: Temperature status
    uint8_t external_temperature;   // 0-3: External sensor status
    uint8_t signature_error;        // 0-4: Memory/firmware errors
    uint8_t flash_read_error;       // 0-1: Flash memory read errors
    uint8_t flash_write_error;      // 0-1: Flash memory write errors
    uint8_t particle_limit;         // 0-1: Particle processing overflow
} SystemAlarms;

// User alarms
typedef struct {
    bool alarm1_set;
    bool alarm1_active;
    uint16_t alarm1_distance;
    bool alarm2_set;
    bool alarm2_active;
    uint16_t alarm2_distance;
} UserAlarms;

// AtmosVUE-BLM (Background Luminance Meter) data
typedef struct {
    float luminance;              // 0-45000 cd/m²
    SystemStatus status;          // 0-3
    bool is_night;                // 0=day, 1=night
    uint8_t units;                // 1=cd/m², 2=fL (foot-lamberts)
    bool heater_on;
    float window_contamination;   // Percentage
} BLM_Data;

// AtmosVUE-HV (Temperature/Humidity) data
typedef struct {
    float temperature;            // -40 to +70°C
    float relative_humidity;      // 0-100% RH
    float wet_bulb_temp;          // Calculated from T and RH
    bool sensor_connected;
    SystemStatus status;
} HV_Data;

// Present weather data
typedef struct {
    uint8_t synop_code;           // 0-89 (see Table C-1 in manual)
    char metar_code[8];           // e.g., "HZ", "BR", "FG", "RA", "-SN", "+RA"
    char nws_code[8];             // e.g., "L", "R", "S", "-R", "+S"
    float particle_count;         // Particles per minute, or -99 if error
    float intensity;              // mm/hr, or -99 if error
    float accumulation;           // mm (0-999.9)
} PresentWeather;

// Calibration data
typedef struct {
    float user_gain;              // MOR calibration factor
    float user_offset;            // MOR offset
    float factory_gain;           // Factory calibration (read-only)
    float factory_offset;         // Factory calibration (read-only)
    float dirty_window_emitter;   // Dirty window baseline (emitter)
    float dirty_window_detector;  // Dirty window baseline (detector)
    float factory_dw_emitter;     // Factory dirty window baseline
    float factory_dw_detector;    // Factory dirty window baseline
    char calibration_disk_sn[16]; // Calibration disk serial number
    float calibration_disk_exco;  // Calibration disk extinction coefficient
} CalibrationData;

/// ATMOSVUE 30 WEATHER SYSTEM ///

typedef struct {
    // Sensor identification
    char serial_number[MAX_SERIAL_STR];
    char model_number[MAX_MODEL_NUM];
    uint8_t sensor_id;            // 0-9 (device address)

    // Current measurements
    uint32_t visibility;          // 5-100000 meters (or feet)
    VisibilityUnits visibility_units;
    MORFormat mor_format;         // 0=MOR, 1=TMOR
    float extinction_coeff;       // EXCO in km⁻¹
    PresentWeather present_weather;
    BLM_Data blm;
    HV_Data hv;
    float internal_temperature;   // CS125 internal temp sensor

    // Configuration
    MessageFormat message_format;
    OperatingMode mode;           // Continuous or polling
    uint16_t continuous_interval; // 0-36000 seconds
    AveragingPeriod averaging_period;
    uint8_t sample_timing;        // Samples per second (default 1)

    // Communication settings
    CommType comm_type;           // RS-232 or RS-485
    BaudRateCode baud_rate;
    DataFormat data_format;
    bool crc_checking_enabled;    // Check CRC on received commands

    // Heater control
    bool dew_heater_override;     // 0=auto, 1=manual
    bool hood_heater_override;    // 0=auto, 1=manual
    bool dew_heater_on;           // Current state
    bool hood_heater_on;          // Current state

    // Advanced settings
    bool dirty_window_compensation; // Enable/disable compensation
    float power_down_voltage;     // Voltage threshold for power down (7.0V typical)
    uint8_t rh_threshold;         // RH threshold for mist/haze (default 80%)

    // Alarms and status
    SystemStatus system_status;
    SystemAlarms system_alarms;
    UserAlarms user_alarms;

    // Calibration
    CalibrationData calibration;

    // Custom message format (bit field for MSG_CUSTOM)
    uint16_t custom_msg_bits;     // Hex bitmap of fields to include

    // Time stamping for continuous mode
    struct timespec last_send_time;

    // Internal state
    bool initialized;
    bool first_minute_elapsed;    // True after sensor powered for 1 minute
} av30_sensor;

// Command type enumeration
typedef enum {
    CMD_UNKNOWN,

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


// Parsed command structure
typedef struct {
    CommandType type;
    uint8_t sensor_id;           // Target sensor ID (0-9)
    bool crc_valid;
    uint16_t received_crc;
    uint16_t calculated_crc;

    union {
        // POLL, GET, and ACCRES usually just need the sensor_id (already in the outer struct)
        // so their internal structs can remain empty or reserved.

        // For SET and SETNC commands
        struct {
            uint8_t new_sensor_id;

            // User Alarms
            uint8_t alarm1_set;      // 0 or 1
            uint8_t alarm1_active;   // 0 or 1
            uint16_t alarm1_dist;    // distance
            uint8_t alarm2_set;
            uint8_t alarm2_active;
            uint16_t alarm2_dist;

            // System Config
            uint32_t baud_rate;      // Actual value or Code
            char serial_num[MAX_SERIAL_STR];     // Sensor serial
            uint8_t vis_units;       // 0=m, 1=ft
            uint16_t continuous_interval;       // Continuous interval
            uint8_t op_mode;         // 0=Cont, 1=Poll
            uint8_t msg_format;      // 0-14
            uint8_t comm_mode;       // 0=232, 1=485
            uint8_t averaging_period;// 1 or 10
            uint8_t sample_timing;   // default 1

            // Heaters & Logic
            uint8_t dew_heater_override;
            uint8_t hood_heater_override;
            uint8_t dirty_window_compensation;
            uint8_t crc_check_en;
            float pwr_down_volt;
            uint8_t rh_threshold;
            uint8_t data_format;     // 8N1 etc

			char full_cmd_string[MAX_INPUT_STR]; // Holds the full command string to echo back to SET command.
        } set_params;

        struct {
            uint16_t field_bitmap;   // For MSGSET
        } msgset;
    } params;
} ParsedCommand;


typedef struct {
    uint8_t msg_format;  	   		// 0-14
    uint8_t sensor_id;      		// Target sensor ID (0-9)
	uint8_t sys_status;				// 0-3
    uint16_t continuous_interval;   // Continuous interval 0-36,000
	uint32_t visibility;			// Visability 0 - 100,000
	char vis_units;					// Visability Units M or F
	MORFormat mor_format;			// MOR Format
	float exco;						// EXCO
	uint8_t avg_period;				// Averging period 1 or 10 minutes
	uint8_t emitter_failure;		//
	uint8_t e_lens_dirty;			// Emitter Lens Dirty
	uint8_t e_temp_failure;			// Emitter Temperature Failure
	uint8_t d_lens_dirty;			// Emitter
	uint8_t d_temp_failure;
	uint8_t d_saturation_lvl;
	uint8_t hood_temperature;
	uint8_t external_temperature;
	uint8_t signature_error;
	uint8_t flash_read_error;
	uint8_t flash_write_error;
	uint8_t particle_limit;
	float particle_count;
	float intensity;
	uint8_t synop_code;
	char metar_code[10];
	float temperature;
	int8_t relative_humidity;
	char blm[10];
	BLM_Data blam;
} ParsedMessage;

// Function prototypes

// Initialization and cleanup
int init_av30_sensor(av30_sensor **ptr);

// Utility functions
bool av30_is_ready_to_send(av30_sensor *sensor);

/// END ATMOSVUE 30 WEATHER SYSTEM

#endif
