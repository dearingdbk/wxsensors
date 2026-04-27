/*
 * File:     ceilometer.c
 * Author:   Bruce Dearing
 * Date:     27/02/2026
 * Version:  1.0
 * Purpose:  Emulates a Campbell Scientific SkyVUE 8 LIDAR Ceilometer over RS-232/RS-485/USB.
 *           This program sets up a serial connection with two threads:
 *            - Receiver thread: parses and responds to incoming commands
 *            - Sender thread: periodically transmits cloud/backscatter data in continuous mode
 *
 *           Supported commands (per Campbell Scientific SkyVUE 8 protocol):
 *             Terminal Mode Commands:
 *               OPEN <ID> [Password]          - Enter terminal mode
 *               CLOSE                         - Exit terminal mode and save settings
 *               STATUS                        - Output detailed sensor status
 *               POLL <ID> [Message_ID]        - Request specific message output
 *
 *             Configuration Commands:
 *               SERIAL <mode> <baud> <parity> <delay> - Set serial port parameters
 *               MCFG <interval> <msg1-5>      - Configure message output
 *               BS <scale> <avg> <gate> <period> <roll> <interval> <filter> - Backscatter params
 *               UNITS <units>                 - Set measurement units (m/ft, tilt correction)
 *               APPLICATION <n>               - Load application presets (Aviation/Research/Meteorology)
 *               TIME <yyyy/mm/dd> <hh:mm:ss>  - Set date and time
 *               ID <sensor_id>                - Set sensor ID (0-9, a-z, A-Z)
 *               PASSWORD [password]           - Set or clear password
 *
 *             Heater Control:
 *               HEATERS <hood> <internal> <laser> <interval> - Set heater modes
 *
 *             Calibration/Maintenance:
 *               SCCAL                         - Stratocumulus backscatter calibration
 *               SERVICE                       - Run service diagnostics
 *               HOFFSET <offset>              - Set height offset (±1000m)
 *               VIS Cap <cap>                 - Set vertical visibility max (100-10000m)
 *               ALARMS <angle>                - Set tilt alarm threshold
 *               CLOUDMODE <mode>              - Precipitation filter (0=off, 1=on)
 *
 *             System Commands:
 *               DEFAULTS                      - Restore factory defaults
 *               REBOOT                        - Force system reboot
 *               LOADOS <module>               - Load new operating system (XMODEM)
 *               LASER <mode> <power>          - Laser control (0=off, 1=on, 20-100% power)
 *               LASERON / LASEROFF            - Turn laser on/off
 *               POWEROFF                      - Prepare for power down
 *               GETUSER                       - Read all user settings as string
 *               SETUSER <string>              - Load all user settings from string
 *
 *           Output message formats:
 *             CS Messages (001-004): Campbell Scientific native format
 *               001: Cloud base heights, no profile, no sky condition
 *               002: Cloud base heights + backscatter profile, no sky condition
 *               003: Cloud base heights + sky condition, no profile
 *               004: Cloud base heights + sky condition + backscatter profile (default)
 *             CL31 Messages (101-112): CL31 compatibility format
 *             CT25K Messages (113-114): CT25K compatibility format
 *
 *             Communication: 115200 baud (default), 8 data bits, 1 stop bit, no parity
 *             Line termination: <CR><LF>
 *             Framing: SOH (0x01), STX (0x02), ETX (0x03), EOT (0x04)
 *             Checksum: CRC-16 (XOR'd with 0xFFFF)
 *
 *           Network mode:
 *             Supports addressed mode with sensor IDs 0-9, a-z, A-Z (case sensitive)
 *             Global ID: 99 (all sensors respond regardless of individual ID)
 *             Password protection: 1-10 characters (optional)
 *
 *           Data includes: cloud base heights (up to 4), vertical visibility, sky condition
 *                         (up to 5 layers in oktas), attenuated backscatter profile,
 *                         window transmission, alarms/warnings, temperatures, voltages
 *
 * Usage:    ceilometer <file_path> [serial_port] [baud_rate] [RS232|RS485|RS422]
 *           ceilometer <file_path> // Defaults: /dev/ttyUSB0, 115200 baud, RS232
 *
 * Sensor:   Campbell Scientific SkyVUE 8 (CS136) LIDAR Ceilometer
 *           - LIDAR (Light Detection And Ranging) technology
 *           - Single biaxial lens optical design
 *           - InGaAs semiconductor laser diode (Class 3B embedded, Class 1M output)
 *           - Maximum reporting range: 8 km (26,250 ft)
 *           - Minimum reporting resolution: 5 m (15 ft)
 *           - Hard target range accuracy: ±0.25% ±4.6 m (15 ft)
 *           - Reporting cycle: 2 to 600 seconds (configurable)
 *           - Cloud layers reported: Up to 4 instantaneous, up to 5 in sky condition
 *           - Sky condition: Cloud coverage in oktas (eighths)
 *           - Backscatter profile: 2048 bins × 5 characters (10,240 chars total)
 *           - Profile resolution: 5 m, 10 m, or 20 m (configurable)
 *           - Laser wavelength: 912 nm ±5 nm (near infrared)
 *           - Pulse duration: 100 ns
 *           - Pulse frequency: 10 kHz
 *           - Maximum average power: 15.0 mW (through 50 mm aperture)
 *           - Maximum pulse energy: 1500 nJ
 *           - Half-angle divergence: 0.44 mrad
 *           - Field of view: 2.0 mrad
 *           - Laser lifetime: 10 years typical
 *           - Eye safety: Class 1M (IEC/EN 60825-1:2014)
 *           - Detection status: 0-6 (no scatter, 1-4 cloud layers, obscuration, transparent)
 *           - Operating temperature: -40°C to +60°C (excluding battery)
 *           - Battery temperature: -20°C to +50°C
 *           - Relative humidity: 0 to 100%
 *           - IP rating: IP66
 *           - Maximum wind speed: 55 m/s
 *           - Output: RS-232 (full duplex), RS-485 (full/half duplex), RS-422 (full duplex)
 *           - Default baud rate: 115200 bps
 *           - Baud rates: 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 76800, 115200
 *           - Data format: 8N1 (default), 7E1, 7O1
 *           - Power (AC): 115/230 VAC ±15% (auto-switching), 50-60 Hz, 380W max
 *           - Power (DC): 10-40 VDC, 1A @ 12VDC, 0.5A @ 24VDC
 *           - Hood heater: 220W max (AC only)
 *           - Internal heater: 110W max (AC only)
 *           - Battery: Internal 12V 7Ah sealed lead-acid with deep discharge protection
 *           - USB service port: USB 1.1/2.0 compatible, fixed 115200 baud
 *           - Weight: 18 kg (40 lb) excluding cables
 *           - Dimensions: 737mm H × 294mm W × 240mm D (29" × 11.6" × 9.5")
 *           - Base plate: 316 × 316 mm (12.4" × 12.4")
 *           - Tilt angles: 0°, 6°, 12°, 18°, 24° from vertical (adjustable)
 *           - Tilt compensation: Automatic via dual-axis inclinometer
 *           - Mounting: Four 12mm diameter holes, 77mm depth
 *           - Grounding: 16mm² minimum cross-section, 10m max length
 *
 * Note:     SkyVUE 8 uses forward scatter LIDAR to detect cloud bases and vertical visibility.
 *           Pulse timing determines aerosol/cloud height; signal strength profiles identify layers.
 *           Sky condition requires 30 minutes of data (last 10 minutes weighted higher).
 *           Stratocumulus calibration requires stable cloud layer 250-2500m for 10+ minutes.
 *           Terminal mode times out after 10 minutes of inactivity.
 *           Backscatter values are signed 40-bit integers (2's complement).
 *           Profile not corrected for tilt angle (heights are corrected if enabled).
 *
 * Meteorological Parameters:
 *           - MOR: Meteorological Optical Range (derived from cloud/scatter profile)
 *           - VV: Vertical Visibility (when full obscuration but no defined cloud base)
 *           - Okta: Cloud coverage in eighths (0-8 oktas per layer)
 *           - Backscatter coefficient: Units of sr⁻¹ m⁻¹ (scaled by 10⁻⁸)
 *           - ICAO: International Civil Aviation Organization standards compliance
 *           - WMO: World Meteorological Organization recommendations compliance
 *
 * Technology:
 *           - LIDAR: Light Detection And Ranging
 *           - Biaxial optical design: Single lens split for transmitter/receiver
 *           - Avalanche PhotoDiode (APD) detector for high sensitivity
 *           - DSP (Digital Signal Processor): Main data processing and communications
 *           - TOP module: Safety shutdown, calibration, dirty window detection
 *           - PSU module: Power supply control, battery charging, deep discharge protection
 *           - Dual time-keeping circuits with cross-check alarm
 *           - Window contamination monitoring and auto-heater control
 *
 * Calibration:
 *           - Factory calibration: Pre-calibrated before shipment
 *           - Stratocumulus calibration: User-initiated backscatter calibration
 *           - Calibration plate: White test surface for service diagnostics
 *           - Window transmission: Monitored continuously, reported in messages
 *           - Dirty window detection: Automatic with 3-level alarm system
 *
 * Maintenance:
 *           - Service routine: Comprehensive self-test via SERVICE command
 *           - LED indicator: Green LED visible through window (flash pattern indicates status)
 *             1 flash/10s = OK, 2 flashes = Warning, 3 flashes = Alarm
 *           - Heater/blower test: Automatic test at configurable interval (1-168 hours, default 24h)
 *           - Laser run days: Tracked internally, reported in status
 *           - Cleaning: Hood and window maintenance as needed
 *           - Bird spike kit: Optional deterrent for avian interference
 *
 * Connectors:
 *           Base Connectors (accessible from bottom):
 *             Power Connector (4-pin):
 *               Pin 1: Live (Brown)
 *               Pin 2: Not connected
 *               Pin 3: Neutral (Blue)
 *               Pin 4: Earth (Green/Yellow)
 *
 *             Blower/Heater Connector (6-pin + Earth):
 *               Pin 1: Neutral (Black 1)
 *               Pin 2: Fan +12VDC (Black 2)
 *               Pin 3: Thermistor (Black 3)
 *               Pin 4: Thermistor 0V (Black 4)
 *               Pin 5: 230/115VAC HV Heater (Black 5)
 *               Pin 6: 230/115VAC LV Heater (Black 6)
 *               Pin E: Earth (Green/Yellow)
 *
 *             Communications Connector (6-pin + Screen):
 *               RS-232 Mode:
 *                 Pin 1 (Red): CTS output
 *                 Pin 2 (Yellow): RTS input
 *                 Pin 3 (Green): Ground
 *                 Pin 4 (Black): Ground
 *                 Pin 5 (White): RXD output
 *                 Pin 6 (Blue): TXD input
 *                 Pin E: Screen
 *
 *               RS-485/RS-422 Mode:
 *                 Pin 1: B/D+ (Y/TXD non-inverting)
 *                 Pin 2: B/RXD non-inverting
 *                 Pin 3: Ground
 *                 Pin 4: Ground
 *                 Pin 5: A/D- (Z/TXD inverting)
 *                 Pin 6: A/RXD inverting
 *                 Pin E: Screen
 *
 *           Internal Connectors:
 *             USB Port: Type B receptacle, fixed 115200 baud (service/maintenance)
 *             I/O Port: Factory use only
 *             Battery: Internal connector for 12V 7Ah sealed lead-acid
 *
 * Application Presets:
 *           Aviation (10): Default for SkyVUE 8
 *             - Units: Feet with tilt correction
 *             - Cloud mode: ON (highest cloud during precipitation)
 *             - BS temporal filter: Median mode
 *             - Tilt alarm: 30°
 *             - Message: 004 (CB, SC, BS)
 *
 *           Research (20):
 *             - Units: Meters with tilt correction
 *             - Cloud mode: OFF
 *             - BS temporal filter: Average mode
 *             - Tilt alarm: 45°
 *             - Message: 006 (CB, SC, MLH, BS)
 *
 *           Meteorology (40):
 *             - Units: Meters with tilt correction
 *             - Cloud mode: ON
 *             - BS temporal filter: Average mode
 *             - Tilt alarm: 45°
 *             - Message: 004 (CB, SC, BS)
 *
 * Alarm/Warning System:
 *           Most Significant Word (hex sum of error bits):
 *             0x8000: Units (0=feet, 1=meters)
 *             0x0800: DSP clock out of spec
 *             0x0400: Laser shutdown (temp out of range)
 *             0x0200: Battery voltage low
 *             0x0100: Mains supply failed
 *             0x0080: Heater/blower temp out of bounds
 *             0x0040: Heater/blower failure
 *             0x0020: PSU internal temp high
 *             0x0010: PSU OS signature fail
 *             0x0008: No DSP-PSU communication
 *             0x0004: Windows dirty
 *             0x0002: Tilt beyond user limit
 *             0x0001: No inclinometer communication
 *
 *           Middle Word:
 *             0x8000: Internal humidity high
 *             0x4000: T/RH chip comm failure
 *             0x2000: DSP supply voltage low
 *             0x1000: Self-test active
 *             0x0800: Watchdog updated
 *             0x0400: User settings flash fail
 *             0x0200: Factory calibration flash fail
 *             0x0100: DSP OS signature fail
 *             0x0080: DSP RAM test fail
 *             0x0040: DSP PSU out of bounds
 *             0x0020: TOP NVRAM corrupt
 *             0x0010: TOP OS signature fail
 *             0x0008: TOP ADC/DAC out of spec
 *             0x0004: TOP PSU out of bounds
 *             0x0002: TOP-DSP comm failure
 *             0x0001: APD background radiance out of range
 *
 *           Least Significant Word:
 *             0x8000: APD temperature out of range
 *             0x4000: APD saturated
 *             0x2000: Calibrator temp out of range
 *             0x1000: Calibrator failed
 *             0x0800: Gain levels not achieved
 *             0x0400: Laser runtime exceeded
 *             0x0200: Laser temp out of range
 *             0x0100: Laser thermistor failure
 *             0x0080: Laser obscured
 *             0x0040: Laser low output power
 *             0x0020: Laser max power exceeded
 *             0x0010: Laser max drive current exceeded
 *             0x0008: Laser power monitor temp out of range
 *             0x0004: Laser power monitor test fail
 *             0x0002: Laser shutdown by TOP board
 *             0x0001: Laser is off
 *
 * Mods:
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <ctype.h>
#include <poll.h>
#include "crc_utils.h"
#include "serial_utils.h"
#include "console_utils.h"
#include "file_utils.h"
#include "skyvue8_utils.h"

#define SERIAL_PORT "/dev/ttyUSB0"   // Adjust as needed, main has logic to take arguments for a new location
#define BAUD_RATE   B115200	     // Adjust as needed, main has logic to take arguments for a new baud rate
#define MAX_LINE_LENGTH 1024
#define MAX_CMD_LENGTH 256
#define MAX_MSG_LENGTH 512
#define CPU_WAIT_USEC 10000

#define DEBUG_MODE // Comment this line out to disable all debug prints

#ifdef DEBUG_MODE
    #define DEBUG_PRINT(fmt, ...) printf("DEBUG: " fmt, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...) // Becomes empty space during compilation
#endif


FILE *file_ptr = NULL; // Global File pointer
char *file_path = NULL; // path to file

// Shared state
volatile sig_atomic_t terminate = 0;
volatile sig_atomic_t kill_flag = 0;

int serial_fd = -1;
const char *program_name = "unknown";

// This needs to be freed upon exit.
skyvue8_sensor *sensor_one = NULL; // Global pointer to struct for skyvue8 sensor .

/* Synchronization primitives */
static pthread_mutex_t file_mutex = PTHREAD_MUTEX_INITIALIZER; // protects file_ptr / file access
static pthread_mutex_t sensor_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  sensor_cond; // Moved initialization down to main, to change REALTIME Clock to MONOTONIC.

// Global pointers to receiver and sender threads.
pthread_t recv_thread, send_thread, sig_thread;


/*
 * Name:         cleanup_and_exit
 * Purpose:      helper function to cleanup sensors, and arrays.
 * Arguments:    exit_code, the exit code to send on close.
 *
 * Output:       None.
 * Modifies:     Frees, sensors, sensor_map, closes file descriptors, and serial devices.
 * Returns:      None.
 * Assumptions:
 *
 * Bugs:         None known.
 * Notes:
 */
void cleanup_and_exit(int exit_code) {
	pthread_mutex_lock(&sensor_mutex);
    terminate = 1;
    pthread_cond_broadcast(&sensor_cond);
    pthread_mutex_unlock(&sensor_mutex);

	raise(SIGTERM); // To wake the signal handling thread, in the event cleanup_and_exit was called naturally.

	if (recv_thread != 0) {
        pthread_join(recv_thread, NULL);
        recv_thread = 0;
    }
    if (send_thread != 0) {
        pthread_join(send_thread, NULL);
        send_thread = 0;
    }
    if (sig_thread != 0) {
        pthread_join(sig_thread, NULL);
        sig_thread = 0;
    }

	pthread_mutex_destroy(&sensor_mutex);
    pthread_mutex_destroy(&file_mutex);
    pthread_cond_destroy(&sensor_cond);

    if (sensor_one) free(sensor_one);
    // Close resources
    if (serial_fd >= 0) close(serial_fd);
    if (file_ptr) fclose(file_ptr);
    // Cleanup utilities
    console_cleanup();
    serial_utils_cleanup();
    exit(exit_code);
}


// ---------------- Command handling ----------------


/*
 * Name:         parse_message
 * Purpose:      Tokenizes a space-delimited sensor string and populates a ParsedMessage struct.
 * Arguments:    msg: the raw input string to be parsed (modified by strtok_r).
 * 				 p_message: pointer to the struct where parsed data will be stored.
 *
 * Output:       None (internal debug prints to console only).
 * Modifies:     p_message: overwrites with new data.
 * 				 msg: the input string is modified (nulls inserted by strtok_r).
 * Returns:      None
 * Assumptions:  msg is a valid space-delimited string matching the sensor protocol.
 *               p_message has been allocated by the caller.
 *
 * Bugs:         None known.
 * Notes:        Uses a local macro NEXT_T to sequence through 32 expected fields.
 *               Ensures string fields (METAR, BLM) are safely null-terminated.
 */
void parse_message(char *msg, ParsedMessage *p_message) {
	memset(p_message, 0, sizeof(ParsedMessage)); // zero out the ParsedMessage struct.
	char *saveptr; // Our place keeper in the msg string.
	char *token; // Where we temporarily store each token.

	if ((token = strtok_r(msg, ",", &saveptr))) p_message->detection_status = (char)token[0];  // Set detection status.
   	#define NEXT_T strtok_r(NULL, ",", &saveptr) // Small macro to keep the code below cleaner.
	// These are pulled from a text file in this format:
	// 1,0,085,010000,/////,/////,/////,8000,0000,0000
	// Detection Status, Alarm Status, Window Trans %, 1st Height, 2nd Height, 3rd Height, 4th Height, Most Significant Alarm Word, Middle Alarm Word, Least Significant Alarm Word
   	if ((token = NEXT_T)) p_message->alarm_status = (char)token[0]; // Set Alarm Status.
   	if ((token = NEXT_T)) p_message->win_trans_per = atoi(token); // Set Window Transmission Percentage.
	if ((token = NEXT_T)) {
		strncpy(p_message->first_height, token, MAX_HEIGHT_STR - 1); // Set the First Cloud Height.
		p_message->first_height[MAX_HEIGHT_STR - 1] = '\0';
	}
	if ((token = NEXT_T)) {
		strncpy(p_message->second_height, token, MAX_HEIGHT_STR - 1); // Set the Second Cloud Height.
		p_message->second_height[MAX_HEIGHT_STR - 1] = '\0';
	}
	if ((token = NEXT_T)) {
		strncpy(p_message->third_height, token, MAX_HEIGHT_STR - 1); // Set the Third Cloud Height.
		p_message->third_height[MAX_HEIGHT_STR - 1] = '\0';
	}
	if ((token = NEXT_T)) {
		strncpy(p_message->fourth_height, token, MAX_HEIGHT_STR - 1); // Set the Fourth Cloud Height.
		p_message->fourth_height[MAX_HEIGHT_STR - 1] = '\0';
	}
	if ((token = NEXT_T)) p_message->most_sig_alarm = (uint16_t)strtol(token, NULL, 16);  // Set Most Significant Alarm Word "8000" -> 0x8000.
	if ((token = NEXT_T)) p_message->middle_sig_alarm = (uint16_t)strtol(token, NULL, 16);  // Set Middle Significant Alarm Word "0000" -> 0x0000.
	if ((token = NEXT_T)) p_message->least_sig_alarm = (uint16_t)strtol(token, NULL, 16);  // Set Least Significant Alarm Word "0000" -> 0x0000.
	// TODO: If we implement other message formats 002, or 004, we will need to injest the raw bin data ~10,000 chars.
	#undef NEXT_T
}

/*
 * Name:         process_and_send
 * Purpose:      Parse a data line, format the message string, and send with CRC.
 * Arguments:    msg: Pointer to the ParsedMessage struct containing the data stripped from the file/buffer.
 *
 * Output:       Prints the formatted sensor message with STX/ETX and CRC to serial.
 * Modifies:     None.
 * Returns:      None.
 * Assumptions:  sensor is initialized, and the msg has data fields filled.
 *
 * Bugs:         None known.
 * Notes:        Ensures the 32-field format matches the hardware specification.
 */
void process_and_send(ParsedMessage *msg) {

	char msg_buffer[MAX_MSG_LENGTH]; // 512
	if (msg == NULL) return;
		switch(msg->message_id) {
			case MSG_001: {
				// See wxsensors.xlsx for message breakdown.
				int length = snprintf(msg_buffer, sizeof(msg_buffer), "%s%c%s%03d\x02\r\n%c%c %03d %s %s %s %s %04X%04X%04X\r\n\x03",
    				"CS",											// 1.  char* - "CS" Always CS
					(char)sensor_one->address,						// 2.  char - Sensor ID
    				sensor_one->software_version,		   			// 3.  char* - Operating System
    				msg->message_id,								// 4.  uint8_t - Message ID
					(char)msg->detection_status,	       			// 5.  char - Detection Status
    				(char)msg->alarm_status,		  				// 6.  char - Alarm Status
    				(uint8_t)msg->win_trans_per,       				// 7.  uint8_t - Window Transmission Percent
    				msg->first_height,								// 8.  char* - First Height
		    		msg->second_height,      						// 9.  char* - Second Height
    				msg->third_height,           					// 10.  char* - Third Height
		    		msg->fourth_height,						  		// 11.  char* - Fourth Height
    				msg->most_sig_alarm,							// 12. uint16_t - Most Significant Alarm
		    		msg->middle_sig_alarm,							// 13. uint16_t - Middle Significant Alarm
    				msg->least_sig_alarm 				  			// 14. uint16_t - Least Significant Alarm
					);

				if (length > 0 && length < (int)sizeof(msg_buffer)) {
					uint16_t calculated_crc = crc16(msg_buffer, length);
					safe_serial_write(serial_fd, "\x01%s%04X\x04\r\n", msg_buffer, calculated_crc);
				}
				break;
			}
			case MSG_002:
				// TODO
				break;
			case MSG_003:
				// TODO
				break;
			case MSG_004:
				// TODO
				break;
			case MSG_101:
				// TODO
				break;
			case MSG_102:
				// TODO
				break;
			case MSG_103:
				// TODO
				break;
			case MSG_104:
				// TODO
				break;
			case MSG_105:
				// TODO
				break;
			case MSG_106:
				// TODO
				break;
			case MSG_107:
				// TODO
				break;
			case MSG_108:
				// TODO
				break;
			case MSG_109:
				// TODO
				break;
			case MSG_110:
				// TODO
				break;
			case MSG_111:
				// TODO
				break;
			case MSG_112:
				// TODO
				break;
			case MSG_113:
				// TODO
				break;
			case MSG_114:
				// TODO
				break;
			default:
				break;
	}
}

/*
 * Name:         parse_command
 * Purpose:      Translates a received string to command enum.
 * Arguments:    buf: the string to translate to a command enum.
 * 				 cmd: the ParsedMessage struct to store values in.
 *
 * Output:       None.
 * Modifies:     None.
 * Returns:      returns an enum representing the correct command, or Unknown Command as the default.
 * Assumptions:  The string recieved is a string and should be able to translate to one of the commands.
 *
 * Bugs:         None known.
 * Notes:
 */
CommandType parse_command(const char *buf, ParsedCommand *cmd) {
    if (buf == NULL || cmd == NULL) return CMD_UNKNOWN;

	memset(cmd, 0, sizeof(ParsedCommand)); // zero out the contents of our ParsedCommand
    const char *ptr = buf;

    while (*ptr && isspace((unsigned char)*ptr)) ptr++; // Skip any leading whitespace.

    for (size_t i = 0; i < CMD_TABLE_SIZE; i++) {

        if (strncasecmp(ptr, cmd_table[i].name, cmd_table[i].len) == 0) {
            // Ensure exact match (don't match "R" if the command is "RESET")
            char next = ptr[cmd_table[i].len];
            if (next == '\0' || isspace((unsigned char)next)) {
                cmd->type = cmd_table[i].type;
                ptr += cmd_table[i].len; // Jump past command

                // Skip spaces to point at arguments
                while (*ptr && isspace((unsigned char)*ptr)) ptr++;

				size_t param_len = strcspn(ptr, "\n\r"); // Get the size of the string, up to the \n \r

				if (param_len > sizeof(cmd->raw_params) - 1) { // Truncate if the remaining string is larger than our raw_params char*.
    				param_len = sizeof(cmd->raw_params) - 1;
				}

				strncpy(cmd->raw_params, ptr, param_len);
				cmd->raw_params[param_len] = '\0'; // Always null-terminate

                return cmd->type;
            }
        }
    }
    cmd->type = CMD_UNKNOWN;
    return CMD_UNKNOWN;
}


/*
 * Name:         handle_command
 * Purpose:      Handle each command and send response on serial.
 * Arguments:    cmd: the command enum we want to handle.
 *		 		 p_cmd: the parsed command string recieved to pass back as required.
 *
 * Output:       Prints to serial port the requsite response to the command.
 * Modifies:     None.
 * Returns:      None.
 * Assumptions:  None.
 *
 * Bugs:         None known.
 * Notes:
 */
void handle_command(CommandType cmd, ParsedCommand *p_cmd) {

	 switch (cmd) {
		case CMD_POLL:
			char *saveptr; // Our place keeper in the msg string.
			char *token; // Where we temporarily store each token.
			uint8_t sensor_id;
			uint8_t message_id;

			if ((token = strtok_r(p_cmd->raw_params, " \r\n", &saveptr))) sensor_id = atoi(token);  // Set sensor ID.
			if ((token = strtok_r(NULL, " \r\n", &saveptr)))  message_id = atoi(token);  // Set message ID.

        	char *line = get_next_line_copy(file_ptr, &file_mutex);

        	if (line) {
		        ParsedMessage local_msg;  // LOCAL
				parse_message(line, &local_msg);
				local_msg.sensor_id = sensor_id; // Add the sensor_id before sending. TODO: Validate address against sensor.
				local_msg.message_id = message_id; // Add the message_id before sending.
				process_and_send(&local_msg);
				fflush(NULL);
            	free(line); // caller of get_next_line_copy() must free resource.
				line = NULL;
        	}
			break;
		case CMD_ERROR:
			    // TODO: Implement a generic error handler.
			break;
		case CMD_INVALID_CRC:
			break;
		case CMD_INVALID_ID:
			safe_console_error("%s: Invalid sensor ID: %s\n", program_name, strerror(errno));
			break;
		case CMD_INVALID_FORMAT:
			safe_console_error("%s: Invalid Command Format: %s\n", program_name, strerror(errno));
			break;
		case CMD_UNKNOWN:
			safe_console_error("%s: Unknown or Bad Command:\n", program_name);
			break;
        default:
			safe_console_error("%s: Unknown or Bad Command:\n", program_name);
            break;
    }
}


// ---------------- Threads ----------------

/*
 * Name:         signal_thread
 * Purpose:      Captures any kill signals, and sets volitile bool 'terminate' and 'kill_flag' to true,
				 allowing the while loop to break, and threads to join.
 * Arguments:    None
 *
 * Output:       None.
 * Modifies:     Changes terminate to true.
 * Returns:      None.
 * Assumptions:  Terminate is set to false.
 *
 * Bugs:         None known.
 * Notes:        Signal handler: must do async-safe ops only (set sig_atomic_t flags)
 */
void* signal_thread(void* arg) {
    (void)arg;
    int sig;
    sigset_t wait_set;
    sigemptyset(&wait_set);
    sigaddset(&wait_set, SIGINT);
    sigaddset(&wait_set, SIGTERM);
    sigaddset(&wait_set, SIGQUIT); // Ctrl+backslash

    sigwait(&wait_set, &sig);     // Blocks until a signal arrives

    terminate = 1;
    kill_flag = 1;

    // safely wake threads
    pthread_mutex_lock(&sensor_mutex);
    pthread_cond_broadcast(&sensor_cond);
    pthread_mutex_unlock(&sensor_mutex);

    return NULL;
}

/*
 * Name:         receiver_thread
 * Purpose:      thread which reads from a serial port, checks if there is data, if there is data read,
 *               it parses the string as a command, and sends the command to handle_command() function.
 * Arguments:    arg: thread arguments.
 *
 * Output:       None.
 * Modifies:     None.
 * Returns:      NULL.
 * Assumptions:  serial port will have data, and that data will translate to a command.
 */
void* receiver_thread(void* arg) {
    (void)arg;
    char line[MAX_CMD_LENGTH];
    size_t len = 0;

    while (!terminate) {
        char c;
        int n = read(serial_fd, &c, 1);
        if (n > 0) {
	    	if (c == '\r' || c == '\n') {
                if (len > 0) {
                    line[len] = '\0'; // Terminate with NULL for safety.

                    ParsedCommand local_cmd;
                    CommandType cmd_type = parse_command(line, &local_cmd);

					pthread_mutex_lock(&sensor_mutex);   // <--- LOCK HERE
		    		handle_command(cmd_type, &local_cmd); // handle received command here.
                    pthread_mutex_unlock(&sensor_mutex); // <--- UNLOCK HERE
                    len = 0;
                } else { // empty line ignore
                }
            } else {
              	  if (len < sizeof(line)-1) {
				      line[len++] = c;
                  } else {
                    	len = 0;
                  }
              }
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("read");
        } else {
            // no data available n == 0, avoid busy loop
            usleep(CPU_WAIT_USEC);
        }
    }
    return NULL;
}

/*
 * Name:         sender_thread
 * Purpose:      On continuous == 1 and assuming terminate != 1 it will get the next line from a specified file, usinf
 *               get_next_line_copy() and send that line to the serial device using safe_write_response() function every 2 seconds.
 * Arguments:    arg: thread arguments.
 *
 * Output:       Error messages if encountered, prints to serial device.
 * Modifies:     None.
 * Returns:      NULL.
 * Assumptions:  serial port will have data, and that data will translate to a command.
 *
 * Bugs:         None known.
 * Notes:
 */
void* sender_thread(void* arg) {
    (void)arg;
    struct timespec ts;
	bool should_send = false;
	int interval = 0;

    while (!terminate) {
        pthread_mutex_lock(&sensor_mutex);

		// Determine if we should wait for a specific time or indefinitely
        if (sensor_one != NULL && sensor_one->mode == SMODE_RUN) {
			interval = sensor_one->message_interval; // If we are in polled mode, message_interval is zero.
            // Calculate absolute time: Last Send Time + Interval
            // Use current REALTIME + (Interval - Time Since Last Send)
            clock_gettime(CLOCK_MONOTONIC, &ts);
            // Add the continuous interval (in seconds) to the current time
            ts.tv_sec += interval;
            // Wait until that specific second arrives OR a signal interrupts us
            pthread_cond_timedwait(&sensor_cond, &sensor_mutex, &ts);
        } else {
            // If in Polling/Stop Mode, wait indefinitely for a signal from the receiver
            pthread_cond_wait(&sensor_cond, &sensor_mutex);
        }

        if (terminate) {
            pthread_mutex_unlock(&sensor_mutex);
            break;
        }

        // is_ready_to_send() handles the interval and timing logic internally, and checks if the sensor is Pollling or Continuous.
        should_send = (sensor_one != NULL && skyvue8_is_ready_to_send(sensor_one));

		pthread_mutex_unlock(&sensor_mutex);  // <-- UNLOCK BEFORE I/O

        // Do I/O operations WITHOUT holding the mutex
        if (should_send) {
            char *line = get_next_line_copy(file_ptr, &file_mutex);

            if (line) {
                ParsedMessage local_msg;  // LOCAL, not global
                parse_message(line, &local_msg);
                process_and_send(&local_msg);
                fflush(NULL);  // Flush all output streams
                free(line);
                line = NULL;
            }

            // Update timestamp with lock
            pthread_mutex_lock(&sensor_mutex);
            if (sensor_one != NULL) {
                clock_gettime(CLOCK_MONOTONIC, &sensor_one->last_send_time);
            }
            pthread_mutex_unlock(&sensor_mutex);
        }
    }
    return NULL;
}

/*
 * Name:         Main
 * Purpose:      Main funstion, which opens up serial port, and creates a receiver and transmit threads to listen, and respond to commands
 *               over that serial port. Can take two arguments or no arguments. If changing the serial device name and baud rate, you must supply both.
 *               i.e. tmp_bp_listen <file_path> [serial_device] [baud_rate]
 *		 		 uses ternary statements to set either default values for SERIAL_PORT, and BAUD_RATE which are defined above.
 * 		 		 (condition) ? (value if true) : (value if false)
 *
 * Arguments:    file_path: The location of the file we want to read from, line by line.
 *               device: the string representing the file descriptor of the serial port which should
 * 				 match the pattern ^/dev/tty(S|USB)[0-9]+$. This is tested with function is_valid_tty()
 *				 baud: the string value representing the proposed baud rate, this string is sent to get_baud_rate() which returns a speed_t value.
 *
 * Output:       Prints to stderr the appropriate error messages if encountered.
 * Modifies:     None.
 * Returns:      Returns an int 0 representing success once the program closes the fd, and joins the threads, or 1 if unable to open the serial port.
 * Assumptions:  device is a valid char * pointer and the line contains
 *               characters other than white space, and points to an FD.
 *		 		 The int provided by arguments is a valid baud rate, although B9600 is set on any errors.
 *
 * Bugs:         None known.
 * Notes:
 */
int main(int argc, char *argv[]) {

    if (argc < 2) {
        safe_console_error("Usage: %s <file_path> <serial_device> <baud_rate> <RS422|RS485>\n", argv[0]);
        return 1;
    }
	program_name = argv[0]; // Global variable to hold the program name for console errors.
    file_path = argv[1];

    file_ptr = fopen(file_path, "r");
    if (!file_ptr) {
        safe_console_error("Failed to open file: %s\n", strerror(errno));
		cleanup_and_exit(1);
    }
    //ternary statement to set SERIAL_PORT if supplied in args or the default
    const char *device = (argc >= 3 && is_valid_tty(argv[2]) == 0) ? argv[2] : SERIAL_PORT;

    // ternary statement to set BAUD_RATE if supplied in args or default
    speed_t baud = (argc >= 4) ? get_baud_rate(argv[3]) : BAUD_RATE;

    SerialMode mode = (argc >=5) ? get_mode(argv[4]) : SERIAL_RS485; // returns RS485 by default.

    serial_fd = open_serial_port(device, baud, mode);

    if (serial_fd < 0) {
        cleanup_and_exit(1);
    }

	if (init_skyvue8_sensor(&sensor_one) != 0) {
        safe_console_error("Failed to initialize sensor_one\n");
	  	cleanup_and_exit(1);
    }
    // define a signal handler, to capture kill signals and instead set our volatile bool 'terminate' to true,
    // allowing our c program, to close its loop, join threads, and close our serial device.
	sigset_t block_set;
	sigemptyset(&block_set);
	sigaddset(&block_set, SIGINT);
	sigaddset(&block_set, SIGTERM);
	sigaddset(&block_set, SIGQUIT);
	pthread_sigmask(SIG_BLOCK, &block_set, NULL);


	// Initialize the send condition to use CLOCK_MONOTONIC
	pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    pthread_cond_init(&sensor_cond, &attr); // Initialize the global variable here
    pthread_condattr_destroy(&attr);

	if (pthread_create(&sig_thread, NULL, signal_thread, NULL) != 0) {
	    safe_console_error("Failed to create signal thread: %s\n", strerror(errno));
        terminate = 1;          // <- symmetrical, but not required
		cleanup_and_exit(1);
	}

    if (pthread_create(&recv_thread, NULL, receiver_thread, NULL) != 0) {
        safe_console_error("Failed to create receiver thread: %s\n", strerror(errno));
        terminate = 1;          // <- needed because sig_thread is running
		cleanup_and_exit(1);
    }

    if (pthread_create(&send_thread, NULL, sender_thread, NULL) != 0) {
        safe_console_error("Failed to create sender thread: %s\n", strerror(errno));
        terminate = 1;          // <- needed because recv_thread is running
		cleanup_and_exit(1);
    }

    safe_console_print("Press 'q' + Enter to quit.\n");
    struct pollfd fds[1];
	fds[0].fd = STDIN_FILENO;
	fds[0].events = POLLIN;
	while (!kill_flag) {
		int ret = poll(fds, 1, 500);

		if (ret == -1) {
        	if (errno == EINTR) continue; // Interrupted by signal, check kill_flag
        	safe_console_error("%s\n", strerror(errno));
			break; // Actual error
    	}
		if (ret > 0 && (fds[0].revents & (POLLIN | POLLHUP))) {
			char input[8];
	     	if (fgets(input, sizeof(input), stdin)) {
            	if (input[0] == 'q' || input[0] == 'Q' || kill_flag == 1) {
                	kill_flag = 1;
            	}
        	} else if (feof(stdin)) {  // keep an eye on the behaviour of this check.
            	kill_flag = 1;
        	}
		}
    }
    safe_console_print("Program %s terminated.\n", program_name);
	cleanup_and_exit(0);
	return 0; // We won't get here, but it quiets verbose warnings on a no return value.
}
