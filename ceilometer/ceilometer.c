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
#include <sys/ioctl.h>
#include <regex.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdarg.h>
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
static pthread_mutex_t send_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  send_cond; // Moved initialization down to main, to change REALTIME Clock to MONOTONIC.

// Global pointers to receiver and sender threads.
pthread_t recv_thread, send_thread;

/*
 * Name:         handle_signal
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
void handle_signal(int sig) {
    (void)sig;
    terminate = 1; // Sets the atmoic var terminate to true, prompting the R & T threads to join.
    kill_flag = 1; // Sets the atomic var kill_flag to true, prompting the main loop to end.
    pthread_cond_signal(&send_cond); // Wakes up the sender thread, in the event it is waiting.
}


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
	pthread_mutex_lock(&send_mutex);
    terminate = 1;
    pthread_cond_signal(&send_cond);
    pthread_mutex_unlock(&send_mutex);

	if (recv_thread != 0) {
        pthread_join(recv_thread, NULL);
        recv_thread = 0;
    }
    if (send_thread != 0) {
        pthread_join(send_thread, NULL);
        send_thread = 0;
    }

	pthread_mutex_destroy(&send_mutex);
    pthread_mutex_destroy(&file_mutex);
    pthread_cond_destroy(&send_cond);

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
/*	memset(p_message, 0, sizeof(ParsedMessage)); // zero out the ParsedMessage struct.
	char *saveptr; // Our place keeper in the msg string.
	char *token; // Where we temporarily store each token.

	if ((token = strtok_r(msg, ",", &saveptr))) p_message->p1_pressure = atof(token);  // Set P1 pressure.
   	#define NEXT_T strtok_r(NULL, ",", &saveptr) // Small macro to keep the code below cleaner.
	// These are pulled from a text file in this format:
	// 1013.25,1013.24,1013.26,23.4,23.5,23.2,0,0,0,1013.25,0.00
	// P1 Pressure,P2 Pressure,P3 Pressure,P1 Temp,P2 Temp,P3 Temp,P1 Error,P2 Error, P3, Error,Pressure Average,Pressure Trend
   	if ((token = NEXT_T)) p_message->p2_pressure = atof(token); // Set P2 pressure.
   	if ((token = NEXT_T)) p_message->p3_pressure = atof(token); // Set P3 pressure.
	if ((token = NEXT_T)) p_message->p1_temperature = atof(token); // Set P1 temperature.
	if ((token = NEXT_T)) p_message->p2_temperature = atof(token); // Set P2 temperature.
	if ((token = NEXT_T)) p_message->p3_temperature = atof(token); // Set P3 temperature.
   	if ((token = NEXT_T)) { // Set P1 error.
		if (atoi(token) == 1) {
			p_message->p1_sensor_error = IS_ERROR;
		} else p_message->p1_sensor_error = NO_ERROR;
	}
   	if ((token = NEXT_T)) { // Set P2 error.
		if (atoi(token) == 1) {
			p_message->p2_sensor_error = IS_ERROR;
		} else p_message->p2_sensor_error = NO_ERROR;
	}
   	if ((token = NEXT_T)) { // Set P3 error.
		if (atoi(token) == 1) {
			p_message->p3_sensor_error = IS_ERROR;
		} else p_message->p3_sensor_error = NO_ERROR;
	}
	if ((token = NEXT_T)) p_message->p_average = atof(token); // Set Pressure Average.
	if ((token = NEXT_T)) p_message->trend = atof(token); // Set Pressure Trend.
	if ((token = NEXT_T)) p_message->tendency = atof(token); // Set Pressure Tendency.
	#undef NEXT_T
	p_message->altitude = sensor_one->hcp_altitude; // Update the sensor hcp_altitude.
	strncpy(p_message->serial_num, sensor_one->serial_number, MAX_SN_LEN - 1);
	p_message->serial_num[MAX_SN_LEN - 1] = '\0';
	p_message->address = sensor_one->address;
	p_message->units = sensor_one->units; */
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
	//build_dynamic_output(msg, msg_buffer, sizeof(msg_buffer));
	DEBUG_PRINT("Message Buffer after Dynamic Build holds %s\n",msg_buffer);
	safe_serial_write(serial_fd, "%s\r\n", msg_buffer);
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

    /*for (size_t i = 0; i < CMD_TABLE_SIZE; i++) {

        if (strncasecmp(ptr, cmd_table[i].name, cmd_table[i].len) == 0) {
            // Ensure exact match (don't match "R" if the command is "RESET")
            char next = ptr[cmd_table[i].len];
            if (next == '\0' || isspace((unsigned char)next) || next == '?' || next == '=') {
                cmd->type = cmd_table[i].type;
                ptr += cmd_table[i].len; // Jump past command

                // Skip spaces to point at arguments
                while (*ptr && isspace((unsigned char)*ptr)) ptr++;

				size_t param_len = strcspn(ptr, "\n\r"); // Get the size of the string, up to the \n \r

				if (param_len > sizeof(cmd->raw_params) - 1) {
    				param_len = sizeof(cmd->raw_params) - 1;
				}

				strncpy(cmd->raw_params, ptr, param_len);
				cmd->raw_params[param_len] = '\0'; // Always null-terminate

                return cmd->type;
            }
        }
    }*/
    // cmd->type = CMD_UNKNOWN;
    return CMD_UNKNOWN;
}


/*
 * Name:         handle_command
 * Purpose:      Handle each command and send response on serial.
 * Arguments:    cmd: the command enum we want to handle.
 *		 buf: the original command string recieved to pass back as required.
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

/*	 switch (cmd) {
        case CMD_BNUM:
			DEBUG_PRINT("BNUM Command Received with these params: %s\n", p_cmd->raw_params);
			//safe_serial_write(serial_fd, "PTB330 Batch Numbers:\n\tSensor: %s\n\t%s %s\n\t%s %s\n\t%s %s\r\n",
			//						sensor_one->batch_num,
			//						"Module 1:", sensor_one->module_one.batch_num,
			//						"Module 2:", sensor_one->module_two.batch_num,
			//						"Module 3:", sensor_one->module_three.batch_num);
			DEBUG_PRINT("PTB330 Batch Numbers:\n\tSensor: %s\n\t%s %s\n\t%s %s\n\t%s %s\n",
			//						sensor_one->batch_num,
			//						"Module 1:", sensor_one->module_one.batch_num,
			//						"Module 2:", sensor_one->module_two.batch_num,
			//						"Module 3:", sensor_one->module_three.batch_num);
			break;
        case CMD_SERI:
			DEBUG_PRINT("SERI Command Received with these params: %s\n", p_cmd->raw_params);
			/*	if (p_cmd->raw_params[0] != '\0') { // changes required.
    				// Tokenize the parameters (works for "9600 o 1" or "o", or any order of params)
    				char *saveptr;
    				char *token = strtok_r(p_cmd->raw_params, " ", &saveptr);
					while (token != NULL) {
    					size_t len = strlen(token);
					    if (len == 1) {
        					// Handle all single-character parameters
        					unsigned char c = (unsigned char)token[0];

        					if (isalpha(c)) {
            					char p = toupper(c);
            					// Vaisala Parity: N=None, E=Even, O=Odd
            					if (p == 'N' || p == 'E' || p == 'O') {
                					sensor_one->parity = (unsigned char)p;
            					}
        					} else if (c == '7' || c == '8') {
            						sensor_one->data_f = c - '0';
        					} else if (c == '1' || c == '2') {
            						sensor_one->stop_b = c - '0';
        					} else safe_console_error("Incorrect serial communications change request.\r\n");
    					} else if (len >= 3 && isdigit((unsigned char)token[0])) {
        					// Handle Baud rate
        					uint32_t b = (uint32_t)atoi(token);
        					for (int t = 0; t < 12; t++) {
            					if (b == baud_table[t].baud_num) {
                				sensor_one->baud = t;
                				break; // Exit loop once match is found
            					}
        					}
	    				} else safe_console_error("Incorrect serial communications change request.\r\n");
    					token = strtok_r(NULL, " ", &saveptr);
					}
				}
				DEBUG_PRINT("Baud P D S\t: %d %c %d %d\n",
								baud_table[sensor_one->baud].baud_num,
								sensor_one->parity,
								(int)sensor_one->data_f,
								(int)sensor_one->stop_b);
				safe_serial_write(serial_fd, "Baud P D S\t: %d %c %d %d\n",
								baud_table[sensor_one->baud].baud_num,
								sensor_one->parity,
								(int)sensor_one->data_f,
								(int)sensor_one->stop_b);
			break;
        case CMD_SNUM:
			DEBUG_PRINT("SNUM Command Received with these params: %s\n", p_cmd->raw_params);
			safe_serial_write(serial_fd, "PTB330 Serial Numbers:\n\tSensor: %s\n\t%s %s\n\t%s %s\n\t%s %s\r\n",
									sensor_one->serial_number,
									"Module 1:", sensor_one->module_one.serial_number,
									"Module 2:", sensor_one->module_two.serial_number,
									"Module 3:", sensor_one->module_three.serial_number);
			DEBUG_PRINT("PTB330 Serial Numbers:\n\tSensor: %s\n\t%s %s\n\t%s %s\n\t%s %s\n",
									sensor_one->serial_number,
									"Module 1:", sensor_one->module_one.serial_number,
									"Module 2:", sensor_one->module_two.serial_number,
									"Module 3:", sensor_one->module_three.serial_number);
            break;
		case CMD_ERRS:
			DEBUG_PRINT("ERRS Command Received with these params: %s\n", p_cmd->raw_params);
			break;
		case CMD_HELP:
			DEBUG_PRINT("HELP Command Received with these params: %s\n", p_cmd->raw_params);
			break;
		case CMD_LOCK:
			DEBUG_PRINT("LOCK Command Received with these params: %s\n", p_cmd->raw_params);
			break;
		case CMD_INFO:
			char current_time[20];
			time_t now = time(NULL);
			strftime(current_time, sizeof(current_time), "%H:%M:%S", localtime(&now));
			safe_serial_write(serial_fd, "\nPTB330 / %s\nSerial number\t: %s\n"
									"Batch number\t: %s\n"
									"Output format\t: %s\n"
									"Adjust. date\t: %s\n"
									"Adjust. info\t: %s\n"
									"Date\t\t: %s\n"
									"Time\t\t: %s\n"
									"Start mode\t: STOP\n"
									"Baud P D S\t: %d %c %d %d\n"
									"Output interval\t: %d %s\n"
									"Address\t\t: %hhu\n"
									"Echo\t\t: %hhu\n"
									"Module 1\t: BARO-1\n"
									"Module 2\t: BARO-1\n"
									"Module 3\t: BARO-1\n"
									"Module 4\t: EMPTY\r\n",
								sensor_one->software_version,
								sensor_one->serial_number,
								sensor_one->batch_num,
								sensor_one->format_string,
								sensor_one->date_string,
								"VAISALA",
								sensor_one->date_string,
								current_time,
								baud_table[sensor_one->baud].baud_num,
								sensor_one->parity,
								(int)sensor_one->data_f,
								(int)sensor_one->stop_b,
								(sensor_one->intv_data.interval / sensor_one->intv_data.multiplier),
								sensor_one->intv_data.interval_units,
								sensor_one->address,
								sensor_one->echo_enabled ? 1 : 0);
			break;
		case CMD_ECHO:
			if (p_cmd->raw_params[0] != '\0') {
    			char *saveptr;
    			char *token = strtok_r(p_cmd->raw_params, " \r\n", &saveptr);
				if (token != NULL) {
					char *ptr = token; // Temp pointer so we can walk through the string with toupper().
					for (; *ptr; ++ptr) *ptr = toupper((unsigned char)*ptr); // Uppercase all the strings!
					if (strncmp(token, "OFF", 3) == 0) {
						sensor_one->echo_enabled = false;
					} else if (strncmp(token, "ON", 2) == 0) {
						sensor_one->echo_enabled = true;
					}
				}
			}
			safe_serial_write(serial_fd, "Echo\t: %s\r\n", sensor_one->echo_enabled ? "ON" : "OFF");
			break;
		case CMD_RESET:
			DEBUG_PRINT("RESET Command Received with these params: %s\n", p_cmd->raw_params);
			break;
		case CMD_VERS:
			DEBUG_PRINT("VERS Command Received with these params: %s\n", p_cmd->raw_params);
			safe_serial_write(serial_fd, "PTB330 / %s\r\n", sensor_one->software_version );
			break;
		case CMD_MODS:
			DEBUG_PRINT("MODS Command Received with these params: %s\n", p_cmd->raw_params);
			break;
		case CMD_CON:
			DEBUG_PRINT("CON Command Received with these params: %s\n", p_cmd->raw_params);
			break;
		case CMD_R:
			DEBUG_PRINT("R Command Received with these params: %s\n", p_cmd->raw_params);
			sensor_one->mode = SMODE_RUN;
			pthread_cond_signal(&send_cond); // Wake our sender thread, to check if our mode has changed.
			break;
		case CMD_INTV: {
			DEBUG_PRINT("INTV Command Received with these params: %s\n", p_cmd->raw_params);
    		int val = 0;
    		char unit_str[MAX_INTV_STR] = {0};
    		long multiplier = 1;

    		// sscanf skips leading spaces automatically.
		    // %d grabs the number, %15s grabs the following word.
    		int found = sscanf(p_cmd->raw_params, "%d %15s", &val, unit_str);

    		if (found >= 1) {
        		// Handle the value limit (0-255 per Vaisala spec)
        		if (val < 0) val = 0;
        		if (val > 255) val = 255;

        		if (found == 2) {
            		// Check the unit (yyy)
            		char unit = tolower((unsigned char)unit_str[0]);
            		switch (unit) {
                		case 's':
							multiplier = 1;
							sensor_one->intv_data.interval_units[0] = 's';
							sensor_one->intv_data.interval_units[1] = '\0'; // manually terminate
							break;
                		case 'm':
							multiplier = SECONDS_IN_MIN;
							sensor_one->intv_data.interval_units[0] = 'm';
							sensor_one->intv_data.interval_units[1] = 'i';
							sensor_one->intv_data.interval_units[2] = 'n';
							sensor_one->intv_data.interval_units[3] = '\0'; // manually terminate
							break;
                		case 'h':
							multiplier = SECONDS_IN_HOUR;
							sensor_one->intv_data.interval_units[0] = 'h';
							sensor_one->intv_data.interval_units[1] = '\0'; // manually terminate
							break;
                		case 'd':
							multiplier = SECONDS_IN_DAY;
							sensor_one->intv_data.interval_units[0] = 'd';
							sensor_one->intv_data.interval_units[1] = '\0'; // manually terminate
							break;
                		default:
							multiplier = 1;
							sensor_one->intv_data.interval_units[0] = 's';
							sensor_one->intv_data.interval_units[1] = '\0'; // manually terminate
							break; // Default to seconds
            		}
        		} else {
            		// If only 'xxx' was provided without 'yyy', Vaisala defaults to seconds
            		multiplier = 1;
					sensor_one->intv_data.interval_units[0] = 's';
					sensor_one->intv_data.interval_units[1] = '\0'; // manually terminate
        		}

        		// Update the sensor state
        		sensor_one->intv_data.interval = (long)val * multiplier;
				sensor_one->intv_data.multiplier = multiplier;
				safe_serial_write(serial_fd, "Output interval %d %s\r\n", sensor_one->intv_data.interval, sensor_one->intv_data.interval_units);
    		}
			pthread_cond_signal(&send_cond); // Wake our sender thread, to check if our mode has changed.
    		break;
		}
		case CMD_SEND:
			DEBUG_PRINT("SEND Command Received with these params: %s\n", p_cmd->raw_params);
        	char *line = get_next_line_copy(file_ptr, &file_mutex);

        	if (line) {
		        ParsedMessage local_msg;  // LOCAL
				parse_message(line, &local_msg);
				process_and_send(&local_msg);
				fflush(NULL);
            	free(line); // caller of get_next_line_copy() must free resource.
				line = NULL;
        	}
			break;
		case CMD_SMODE:
			DEBUG_PRINT("SMODE Command Received with these params: %s\n", p_cmd->raw_params);
			char *saveptr;
			char *token = strtok_r(p_cmd->raw_params, " ", &saveptr);
			if (token != NULL) {
				if (strncmp(token, "STOP", 4) == 0) {
					sensor_one->mode = SMODE_STOP;
				} else if (strncmp(token, "POLL", 4) == 0) {
					sensor_one->mode = SMODE_POLL;
				} else if (strncmp(token, "RUN", 3) == 0) {
					sensor_one->mode = SMODE_RUN;
				} else if (strncmp(token, "SEND", 4) == 0) {
					sensor_one->mode = SMODE_SEND;
				} else {
					DEBUG_PRINT("No Match of mode\n");
				}
			}
			pthread_cond_signal(&send_cond); // Wake our sender thread, to check if our mode has changed.
			break;
		case CMD_SDELAY:
			DEBUG_PRINT("SDELAY Command Received with these params: %s\n", p_cmd->raw_params);
			break;
		case CMD_ADDR:
			if (p_cmd->raw_params[0] != '\0') {
				uint8_t new_address = (uint8_t)atoi(p_cmd->raw_params);
				sensor_one->address = new_address;
				safe_serial_write(serial_fd, "Address : 2 ?  %hhu\r\n",sensor_one->address);
			} else {
				safe_serial_write(serial_fd, "Address : 2 ?  %hhu\r\n",sensor_one->address);
			}
			break;
		case CMD_OPEN:
			DEBUG_PRINT("OPEN Command Received with these params: %s\n", p_cmd->raw_params);
			if (p_cmd->raw_params[0] != '\0') {
				uint8_t req_address = (uint8_t)atoi(p_cmd->raw_params);
				if (sensor_one->mode == SMODE_POLL && sensor_one->address == req_address) {
					sensor_one->mode = SMODE_STOP;
					safe_serial_write(serial_fd, "PTB330: %hhu line opened for operator commands\r\n", sensor_one->address);
					pthread_cond_signal(&send_cond); // Wake our sender thread, to check if our mode has changed.
				}
			} else {
				safe_console_error("%s: %s\n", program_name, "Open command received without an address");
			}
			break;
		case CMD_CLOSE:
			DEBUG_PRINT("CLOSE Command Received with these params: %s\n", p_cmd->raw_params);
			if (sensor_one->mode == SMODE_STOP) {
				sensor_one->mode = SMODE_POLL;
				safe_serial_write(serial_fd, "line closed\r\n");
				pthread_cond_signal(&send_cond); // Wake our sender thread, to check if our mode has changed.
			}
			break;
		case CMD_SCOM:
			DEBUG_PRINT("SCOM Command Received with these params: %s\n", p_cmd->raw_params);
			// If required this can be enabled in the handling of CMD_UNKNOWN
			break;
		case CMD_TQFE:
			DEBUG_PRINT("TQFE Command Received with these params: %s\n", p_cmd->raw_params);
			break;
		case CMD_DPMAX:
			DEBUG_PRINT("DPMAX Command Received with these params: %s\n", p_cmd->raw_params);
			break;
		case CMD_HHCP:
			DEBUG_PRINT("HHCP Command Received with these params: %s\n", p_cmd->raw_params);
			break;
		case CMD_HQFE:
			DEBUG_PRINT("HQFE Command Received with these params: %s\n", p_cmd->raw_params);
			break;
		case CMD_HQNH:
			DEBUG_PRINT("HQNH Command Received with these params: %s\n", p_cmd->raw_params);
			break;
		case CMD_PSTAB:
			DEBUG_PRINT("PSTAB Command Received with these params: %s\n", p_cmd->raw_params);
			break;
		case CMD_AVRG:
			break;
		case CMD_FORM:
			if (p_cmd->raw_params[0] == '\0') {
				DEBUG_PRINT("Output format : %s\r\n", sensor_one->format_string);
				safe_serial_write(serial_fd, "Output format : %s\r\n", sensor_one->format_string);
			} else if (p_cmd->raw_params[0] == '?') {
				if (p_cmd->raw_params[1] == '\0') {
					DEBUG_PRINT("Output format : %s\r\n", sensor_one->format_string);
					safe_serial_write(serial_fd, "Output format : %s\r\n", sensor_one->format_string);
				} else if (p_cmd->raw_params[1] == '?') {
					safe_serial_write(serial_fd, "P P3H P1 P2 P3 DP12 DP13 DP23 HCP QFE QNH TP1 TP2 TP3 A3H\n"
												 "Additional parameters\n"
												 "#T, #R, #N, #RN, Un, n.n, CS2, CS4, CSX, SN, ERR, PSTAB, ADDR, DATE, TIME\r\n");
					DEBUG_PRINT("P P3H P1 P2 P3 DP12 DP13 DP23 HCP QFE QNH TP1 TP2 TP3 A3H\n"
												 "Additional parameters\n"
												 "#T, #R, #N, #RN, Un, n.n, CS2, CS4, CSX, SN, ERR, PSTAB, ADDR, DATE, TIME\r\n");
				} else 	{
				      safe_console_error("Invalid Command Format: %s\n", strerror(errno));
				}
			} else {
				strncpy(sensor_one->format_string, p_cmd->raw_params, MAX_FORM_STR - 1); // Copy the param string to the sensor, error handling?
				sensor_one->format_string[MAX_FORM_STR - 1] = '\0';
				parse_form_string(p_cmd->raw_params); // This is going to go through the remaing string after FORM, and build the compiled_form[] fields.
			}
			break;
		case CMD_TIME:
			break;
		case CMD_DATE:
			break;
		case CMD_UNIT:
			DEBUG_PRINT("UNIT Command Received with these params: %s\n", p_cmd->raw_params);
			// Parse up to two tokens: "UNIT Pa" or "UNIT P mmHg"
		    char *sptr;
    		char *first = strtok_r(p_cmd->raw_params, " \t\r\n", &sptr);
    		char *second = strtok_r(NULL, " \t\r\n", &sptr);

    		if (!first || first[0] == '\0' || first[0] == '?') {
        		// Query current unit setting
        		safe_serial_write(serial_fd, "Unit: %s\r\n", get_unit_str(sensor_one->units));
        		break;
    		}

			PTB330_Unit new_unit = UNIT_HPA; // default

			if (!second) {
		        // Single argument: "UNIT Pa" - set global default
		        // Find matching unit in table
        		for (size_t j = 0; j < sizeof(unit_table)/sizeof(UnitConversion); j++) {
            		if (strcasecmp(first, unit_table[j].label) == 0) {
                		new_unit = unit_table[j].unit;
                		break;
            		}
        		}
		        sensor_one->units = new_unit;
        		safe_serial_write(serial_fd, "Unit: %s\r\n", get_unit_str(sensor_one->units));
    		} else {
     			// Two arguments: "UNIT P mmHg" - variable-specific (store for later)
		        // Find matching unit in table
        		for (size_t j = 0; j < sizeof(unit_table)/sizeof(UnitConversion); j++) {
            		if (strcasecmp(second, unit_table[j].label) == 0) {
                		new_unit = unit_table[j].unit;
                		break;
            		}
        		}
			    // TODO: Implement per-variable unit storage
		        sensor_one->units = new_unit;
        		safe_serial_write(serial_fd, "Unit: %s\r\n", get_unit_str(sensor_one->units));
    		}
			break;
		case CMD_DSEL:
			break;
		case CMD_DELETE:
			break;
		case CMD_UNDELETE:
			break;
		case CMD_DIR:
			break;
		case CMD_PLAY:
			break;
		case CMD_CDATE:
			break;
		case CMD_LCP1:
			break;
		case CMD_LCP2:
			break;
		case CMD_LCP3:
			break;
		case CMD_MPCP1:
			break;
		case CMD_MPCP2:
			break;
		case CMD_MPCP3:
			break;
		case CMD_CTEXT:
			break;
		case CMD_AMODE:
			break;
		case CMD_ASEL:
			break;
		case CMD_ACAL:
			break;
		case CMD_AERR:
			break;
		case CMD_ATEST:
			break;
		case CMD_RSEL:
			break;
		case CMD_RTEST:
			break;
		case CMD_ICAOQNH:
			break;
		case CMD_INVALID_CRC:
			break;
		case CMD_INVALID_ID:
			safe_console_error("%s: Failed to open file: %s\n", program_name, strerror(errno));
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
    }*/
}


// ---------------- Threads ----------------
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

					pthread_mutex_lock(&send_mutex);   // <--- LOCK HERE
		    		handle_command(cmd_type, &local_cmd); // handle received command here.
                    pthread_mutex_unlock(&send_mutex); // <--- UNLOCK HERE
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
        pthread_mutex_lock(&send_mutex);

		// Determine if we should wait for a specific time or indefinitely
        if (sensor_one != NULL) { // && sensor_one->mode == SMODE_RUN) {
			interval = 2; // sensor_one->intv_data.interval;
            // Calculate absolute time: Last Send Time + Interval
            // Use current REALTIME + (Interval - Time Since Last Send)
            clock_gettime(CLOCK_MONOTONIC, &ts);
            // Add the continuous interval (in seconds) to the current time
            ts.tv_sec += interval;
            // Wait until that specific second arrives OR a signal interrupts us
            pthread_cond_timedwait(&send_cond, &send_mutex, &ts);
        } else {
            // If in Polling/Stop Mode, wait indefinitely for a signal from the receiver
            pthread_cond_wait(&send_cond, &send_mutex);
        }

        if (terminate) {
            pthread_mutex_unlock(&send_mutex);
            break;
        }

        // is_ready_to_send() handles the interval and timing logic internally, and checks if the sensor is Pollling or Continuous.
        should_send = (sensor_one != NULL && skyvue8_is_ready_to_send(sensor_one));

		pthread_mutex_unlock(&send_mutex);  // <-- UNLOCK BEFORE I/O

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
            pthread_mutex_lock(&send_mutex);
            if (sensor_one != NULL) {
                clock_gettime(CLOCK_MONOTONIC, &sensor_one->last_send_time);
            }
            pthread_mutex_unlock(&send_mutex);
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
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_signal;
    sigaction(SIGINT, &sa, NULL);   // Ctrl-C
    sigaction(SIGTERM, &sa, NULL);  // kill, systemd, etc.

	// Initialize the send condition to use CLOCK_MONOTONIC
	pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    pthread_cond_init(&send_cond, &attr); // Initialize the global variable here
    pthread_condattr_destroy(&attr);

    if (pthread_create(&recv_thread, NULL, receiver_thread, NULL) != 0) {
        safe_console_error("Failed to create receiver thread: %s\n", strerror(errno));
        terminate = 1;          // <- symmetrical, but not required
		cleanup_and_exit(1);
    }

    if (pthread_create(&send_thread, NULL, sender_thread, NULL) != 0) {
        safe_console_error("Failed to create sender thread: %s\n", strerror(errno));
        terminate = 1;          // <- needed because recv_thread is running
        pthread_join(recv_thread, NULL);
		cleanup_and_exit(1);
    }

    ParsedCommand local_cmd;

//	handle_command(parse_command("ERRS\r\n", &p_cmd));
//	handle_command(parse_command("HELP\r\n", &p_cmd));
//	handle_command(parse_command("LOCK 2\r\n", &p_cmd));
//	handle_command(parse_command("INTV 5 s\r\n", &local_cmd), &local_cmd);

//	handle_command(parse_command("UNIT mmHG", &local_cmd), &local_cmd);
//	handle_command(parse_command("FORM P \" \" U \" \" P1  \" \" U \" :\" A3H", &local_cmd), &local_cmd);
	handle_command(parse_command("R\r\n", &local_cmd), &local_cmd);

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
    safe_console_print("Program terminated.\n");
	cleanup_and_exit(0);
	return 0; // We won't get here, but it quiets verbose warnings on a no return value.
}
