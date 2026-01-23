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

// Message format types
extern const char* message_format_names[15];

// SYNOP codes
extern const char* synop_codes[90];

// METAR codes
extern const char* metar_codes[20];

// Units of measure
typedef enum {
    UNITS_METERS = 0,
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
    uint16_t visibility;          // 5-100000 meters (or feet)
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
    bool crc_valid;              // Was CRC correct?
    uint16_t received_crc;       // CRC from command
    uint16_t calculated_crc;     // Calculated CRC

    // Union for different command parameters
    union {
        // For POLL command
        struct {
            uint8_t reserved;    // Always 0
        } poll;
        // For GET command
        struct {
            uint8_t reserved;    // Always 0
        } get;
        // For SET/SETNC commands
        struct {
            uint8_t new_sensor_id;
            UserAlarms user_alarms;
            BaudRateCode baud_rate;
            VisibilityUnits visibility_units;
            uint16_t continuous_interval;
            OperatingMode mode;
            MessageFormat message_format;
            CommType comm_type;
            AveragingPeriod averaging_period;
            uint8_t sample_timing;
            bool dew_heater_override;
            bool hood_heater_override;
            bool dirty_window_compensation;
            bool crc_checking;
            float power_down_voltage;
            uint8_t rh_threshold;
            DataFormat data_format;
        } set_params;
        // For MSGSET command
        struct {
            uint16_t field_bitmap;  // Hex value of fields to include
        } msgset;
        // For ACCRES command
        struct {
            uint8_t reserved;    // Always 0
        } accres;
    } params;
} ParsedCommand;


// Function prototypes

// Initialization and cleanup
int init_sensor(av30_sensor **ptr);
void free_sensor(av30_sensor *sensor);

// Configuration management
int update_settings(av30_sensor *sensor, ParsedCommand *cmd);
int reset_to_defaults(av30_sensor *sensor);
int save_to_flash(av30_sensor *sensor);

// Measurement simulation
void update_measurements(av30_sensor *sensor);
float simulate_visibility(void);
uint8_t simulate_synop_code(void);
void simulate_metar_code(char *metar, uint8_t synop);
float simulate_luminance(void);
float simulate_temperature(void);
float simulate_humidity(void);
float calculate_wet_bulb(float temp, float rh);

// Command parsing and response
CommandType parse_command(const char *input, ParsedCommand *cmd, av30_sensor *sensor);
int generate_response(av30_sensor *sensor, ParsedCommand *cmd, char *output, size_t max_len);

// Message formatting
int format_message(av30_sensor *sensor, char *output, size_t max_len);
int format_get_response(av30_sensor *sensor, char *output, size_t max_len);
int format_poll_response(av30_sensor *sensor, char *output, size_t max_len);

// CRC calculation (CCITT CRC-16)
uint16_t calculate_crc16_ccitt(const char *data, size_t length);
bool verify_crc(const char *command);

// Utility functions
const char* get_baud_rate_string(BaudRateCode code);
uint32_t get_baud_rate_value(BaudRateCode code);
BaudRateCode get_baud_rate_code(uint32_t baud);
const char* get_message_format_name(MessageFormat format);
const char* get_synop_description(uint8_t code);
const char* get_system_status_string(SystemStatus status);
bool is_ready_to_send(av30_sensor *sensor);

// Alarm management
void update_system_alarms(av30_sensor *sensor);
void check_user_alarms(av30_sensor *sensor);
void clear_alarms(av30_sensor *sensor);

// Heater control
void update_heater_control(av30_sensor *sensor);
bool should_dew_heater_be_on(av30_sensor *sensor);
bool should_hood_heater_be_on(av30_sensor *sensor);

/// END ATMOSVUE 30 WEATHER SYSTEM

#endif
