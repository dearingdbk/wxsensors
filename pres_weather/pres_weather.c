/*
 * File:     pres_weather.c
 * Author:   Bruce Dearing
 * Date:     23/01/2026
 * Version:  1.0
 * Purpose:  Emulates a Campbell Scientific AtmosVUE 30 Aviation Weather System over RS-232/RS-485.
 *           This program sets up a serial connection with two threads:
 *            - Receiver thread: parses and responds to incoming commands
 *            - Sender thread: periodically transmits weather data in continuous mode
 *
 *           Supported commands (per Campbell Scientific AtmosVUE 30 protocol):
 *             Measurement Commands:
 *               POLL:0:0:3A3B:            - Request current visibility/weather data
 *               POLL:1:0:0D0B:            - Poll device with sensor ID 1
 *               POLL:2:0:545B:            - Poll device with sensor ID 2
 *
 *             Information Commands:
 *               GET:0:0:2C67:             - Retrieve all current user settings
 *               GET:1:0:F957:             - Get settings from device with sensor ID 1
 *
 *             General Setup Commands:
 *               SET:ID:<params>:<CRC>:    - Configure settings and save to flash
 *               SETNC:ID:<params>:<CRC>:  - Configure settings (temporary, no flash save)
 *               MSGSET:ID:<hex>:<CRC>:    - Configure custom message format
 *               ACCRES:ID:0:<CRC>:        - Reset precipitation accumulation to zero
 *
 *             SET/SETNC Parameters (space-delimited):
 *               <sensor_id> <alarm1_set> <alarm1_active> <alarm1_dist>
 *               <alarm2_set> <alarm2_active> <alarm2_dist> <baud_rate>
 *               <serial_number> <units> <interval> <mode> <msg_format>
 *               <comm_type> <avg_period> <sample_timing> <dew_heater>
 *               <hood_heater> <dirty_window_comp> <crc_check> <power_down_v>
 *               <rh_threshold> <data_format>
 *
 *             Configuration Values:
 *               Sensor ID: 0-9
 *               Baud rate: 0=1200, 1=2400, 2=38400 (default), 3=19200, 4=57600, 5=115200
 *               Units: M=meters, F=feet
 *               Mode: 0=continuous, 1=polling
 *               Message format: 0-14 (14=RVR Output for AtmosVUE 30)
 *               Comm type: 0=RS-232, 1=RS-485
 *               Averaging: 1=1-minute, 10=10-minute
 *
 * Output format:
 *             Message 14 (RVR Output) - space-delimited fields:
 *             <STX><Msg_ID> <Sensor_ID> <Status> <Interval> <Visibility> <Units>
 *             <MOR_Format> <EXCO> <Avg_Period> <12_Alarms> <Particles> <Intensity>
 *             <SYNOP> <METAR> <Temp> <RH> BLM <Luminance> <BLM_Status>
 *             <Day_Night> <BLM_Units> <Checksum><ETX><CR><LF>
 *
 * Communication: 38400 baud (default), 8 data bits, 1 stop bit, no parity
 *             Line termination: <CR><LF>
 *             Framing: STX (0x02), ETX (0x03)
 *             Checksum: CCITT CRC-16 (XMODEM)
 *
 * Network mode:
 *             Supports addressed mode with sensor IDs 0-9 on RS-485 bus
 *             Address format: <command>:<sensor_id>:<reserved>:<checksum>:
 *             Example: POLL:0:0:3A3B: (polls sensor ID 0)
 *
 * Data includes: visibility (MOR), present weather codes (SYNOP/METAR),
 *                         background luminance, temperature, humidity, precipitation
 *
 * Usage:    pres_weather <file_path> [serial_port] [baud_rate] [RS232|RS485]
 *           pres_weather <file_path> // Defaults: /dev/ttyUSB0, 38400 baud, RS232
 *
 * Sensor:   Campbell Scientific AtmosVUE 30 Aviation Weather System
 *           - CS125 forward-scatter present weather and visibility sensor
 *           - AtmosVUE-BLM background luminance sensor
 *           - AtmosVUE-HV temperature and humidity sensor (optional)
 *           - Visibility range: 5 m to 100 km (16.4 ft to 62.1 miles)
 *           - Visibility accuracy: ±8% (<600m), ±10% (<10km), ±15% (<15km), ±20% (<100km)
 *           - Resolution: 1 m over entire range
 *           - Background luminance: 0 to 45,000 cd/m²
 *           - Luminance accuracy: ±0.2 cd/m² (<2), ±10% (>2)
 *           - Luminance resolution: 0.1 cd/m²
 *           - Field of view: 6° (FAA specified)
 *           - Spectral response: CIE 1931 (photopic)
 *           - LED wavelength: 850 nm ± 35 nm (near infrared)
 *           - Light pulse rate: 1 kHz
 *           - Temperature range: -40°C to +70°C (operating)
 *           - Extended range: -50°C to +70°C
 *           - Temperature accuracy: ±3°C (internal sensor)
 *           - Humidity range: 0 to 100% RH
 *           - Precipitation intensity: 0 to 999.9 mm/hr
 *           - Precipitation accumulation: 0 to 999.9 mm (±15% accuracy)
 *           - Accumulation resolution: 0.1 mm
 *           - Present weather codes: 57 SYNOP codes, METAR codes
 *           - MOR calculation: Koschmieder law (MOR = 3/EXCO)
 *           - Sample rate: 1 Hz (1-second samples)
 *           - Output averaging: 1-minute or 10-minute rolling average
 *           - IP rating: IP66 / NEMA 4X
 *           - Output: RS-232 (full duplex) or RS-485 (half duplex)
 *           - Default baud rate: 38400 bps
 *           - Baud rates: 1200, 2400, 9600, 19200, 38400, 57600, 115200 bps
 *           - Data format: 8N1 or 7E1 (8-bit no parity, 1 stop bit default)
 *           - Power (main electronics): 7-30 VDC, 12V nominal
 *           - Current: 110-248 mA (continuous sampling, RS-232)
 *           - Hood heaters: 24 VAC/DC nominal (30V max), 60W total (2x30W)
 *           - Dew heaters: 1.2W total (2x0.6W, auto-controlled)
 *           - Hood heater control: On <15°C, Off >25°C
 *           - Dew heater control: On <35°C, Off >40°C
 *           - Weight (CS125): 3 kg (6.6 lb)
 *           - Weight (BLM): 2.4 kg (5.3 lb)
 *           - Non-volatile memory: Stores configuration, calibration, user message
 *           - Update rate: Configurable (0-36000 seconds interval)
 *           - Dirty window detection: Built-in with 3 alarm levels
 *           - Dirty window compensation: Optional (configurable)
 *
 * Note:     AtmosVUE 30 combines CS125 sensor with BLM and optional HV sensors.
 *           Temperature from internal sensor used unless AtmosVUE-HV connected.
 *           Wet-bulb temperature calculated only when AtmosVUE-HV connected.
 *           Hood heater override should be OFF when heaters not used.
 *           If AtmosVUE-HV used, supply voltage must not exceed 28 VDC.
 *
 * Aeronautical Parameters (for visibility and present weather):
 *           - MOR: Meteorological Optical Range
 *           - EXCO: Extinction Coefficient (km⁻¹)
 *           - RVR: Runway Visual Range
 *           - METAR: Aviation Routine Weather Report codes
 *           - SYNOP: Surface Synoptic Observations codes
 *           - BLM: Background Luminance Measurement
 *
 * Technology:
 *           - TERPS-like forward-scatter optical technology
 *           - Particle size/speed analysis for precipitation type identification
 *           - Avalanche photodiode (APD) detector
 *           - Pulsed near-infrared LED emitter
 *           - Sample volume: Overlap of emitter beam and detector field of view
 *           - Scattering angle: Forward scatter (optimized for visibility)
 *           - Microprocessor-based digital processing
 *           - Temperature compensation: Automatic (internal and external sensors)
 *
 * Calibration:
 *           - Visibility calibration: Every 2 years with calibration disk
 *           - Dirty window zero offset: Every 2 years or after cleaning
 *           - Calibration disk required: Factory-supplied with serial number and EXCO value
 *           - Calibration bungs: Foam bungs for dark level calibration
 *
 * Maintenance:
 *           - Lens cleaning interval: 6 months (clean sites) to monthly (contaminated)
 *           - Cleaning method: Lint-free lens cloth with isopropyl alcohol only
 *           - Enclosure screw lubrication: Anti-seize grease at regular intervals
 *           - Avoid excessive pressure when cleaning (prevents scratches)
 *
 * Connectors:
 *           Connector 1 (M12 Male A-coded 8-way): Main power + RS-232/RS-485 comms
 *             Pin 1: RX (White)
 *             Pin 3: 0V comms (Green, 100Ω to 0V)
 *             Pin 5: 12V (Gray) - Main power input
 *             Pin 7: TX (Blue)
 *             Pin 8: 0V (Red) - Main power return
 *
 *           Connector 2 (RD24 Male 3+earth): 24V hood heater power input
 *             Pin 2: 24V (Red) - 16 AWG recommended
 *             Pin 3: 0V (Black) - 16 AWG recommended
 *             Pin 4: Shield/Earth
 *
 *           Connector 3 (M12 Female A-coded 8-way): AtmosVUE-HV T/RH sensor (SDI-12)
 *             Pin 5: 12V (Gray) - Power output
 *             Pin 6: SDI-12 comms (Pink)
 *             Pin 8: 0V (Red) - Ground reference
 *
 *           Connector 4 (RD24 Female 3+earth): AtmosVUE-BLM 24V heater output
 *             Pin 2: 24V (Red)
 *             Pin 3: 0V (Black)
 *             Pin 4: Shield
 *
 *           Connector 5 (M12 Female A-coded 8-way): AtmosVUE-BLM comms + 12V output
 *             Pin 1: RX (White)
 *             Pin 3: 0V comms (Green)
 *             Pin 5: 12V (Gray) - Power to BLM
 *             Pin 7: TX (Blue)
 *             Pin 8: 0V (Red)
 *
 * Internal Switches:
 *           SW1: Factory reset (ON = reset to defaults, requires stable power)
 *           SW2: Reserved (set to OFF)
 *           SW3: Temporary RS-232 @ 38400 bps (ON = override to RS-232, temporary)
 *           SW4: Must be OFF for AtmosVUE 30 operation
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
#include <linux/serial.h>
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
#include "atmosvue30_utils.h"

#define SERIAL_PORT "/dev/ttyUSB0"   // Adjust as needed, main has logic to take arguments for a new location
#define BAUD_RATE   B38400	     // Adjust as needed, main has logic to take arguments for a new baud rate
#define MAX_LINE_LENGTH 1024
#define CPU_WAIT_NANOSECONDS 10000000
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
char site_id = 'A';
uint8_t address = 0;

// This needs to be freed upon exit.
av30_sensor *sensor_one = NULL; // Global pointer to struct for atmosvue30 sensor .

ParsedCommand p_cmd;
ParsedMessage p_msg;

/* Synchronization primitives */
static pthread_mutex_t file_mutex = PTHREAD_MUTEX_INITIALIZER; // protects file_ptr / file access
static pthread_mutex_t send_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  send_cond  = PTHREAD_COND_INITIALIZER;

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


/*
 * Name:         strip_whitespace
 * Purpose:      helper function to strip all whitespace from the string.
 * Arguments:    s the destination string pointer.
 *
 * Output:       None.
 * Modifies:     s.
 * Returns:      None.
 * Assumptions:
 *
 * Bugs:         None known.
 * Notes:		 s and d point to the same string pointer, advancing s if it is a space, moves non-space chars up the string.
 */
void strip_whitespace(char* s) {
    char* d = s; // Destination pointer
    do {
        // isspace checks for ' ', \t, \n, \v, \f, \r
        while (isspace((unsigned char)*s)) {
            s++;
        }
    } while ((*d++ = *s++));
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
 * Returns:      int: 0 on completion.
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

	if ((token = strtok_r(msg, " ", &saveptr))) p_message->msg_format = atoi(token);
   	#define NEXT_T strtok_r(NULL, " ", &saveptr) // Small macro to keep the code below cleaner.

   	if ((token = NEXT_T)) p_message->sensor_id = atoi(token);
   	if ((token = NEXT_T)) p_message->sys_status = atoi(token);
   	if ((token = NEXT_T)) p_message->continuous_interval = atoi(token);
   	if ((token = NEXT_T)) p_message->visibility = atoi(token);
   	if ((token = NEXT_T)) p_message->vis_units = token[0]; // Char M or F - IGNORED
	if ((token = NEXT_T)) p_message->mor_format = (MORFormat)atoi(token);
   	if ((token = NEXT_T)) p_message->exco = atof(token);
   	if ((token = NEXT_T)) p_message->avg_period = atoi(token);
   	if ((token = NEXT_T)) p_message->sys_alarms.emitter_failure = atoi(token);
   	if ((token = NEXT_T)) p_message->sys_alarms.emitter_lens_dirty = atoi(token);
   	if ((token = NEXT_T)) p_message->sys_alarms.emitter_temperature = atoi(token);
   	if ((token = NEXT_T)) p_message->sys_alarms.detector_lens_dirty = atoi(token);
   	if ((token = NEXT_T)) p_message->sys_alarms.detector_temperature = atoi(token);
   	if ((token = NEXT_T)) p_message->sys_alarms.detector_dc_saturation = atoi(token);
   	if ((token = NEXT_T)) p_message->sys_alarms.hood_temperature = atoi(token);
   	if ((token = NEXT_T)) p_message->sys_alarms.external_temperature = atoi(token);
   	if ((token = NEXT_T)) p_message->sys_alarms.signature_error = atoi(token);
   	if ((token = NEXT_T)) p_message->sys_alarms.flash_read_error = atoi(token);
   	if ((token = NEXT_T)) p_message->sys_alarms.flash_write_error = atoi(token);
   	if ((token = NEXT_T)) p_message->sys_alarms.particle_limit = atoi(token);
   	if ((token = NEXT_T)) p_message->pres_wx.particle_count = atof(token);
   	if ((token = NEXT_T)) p_message->pres_wx.intensity = atof(token); // atof for float.
   	if ((token = NEXT_T)) p_message->pres_wx.synop_code = atoi(token);
   	if ((token = NEXT_T)) {
		strncpy(p_message->pres_wx.metar_code, token, sizeof(p_message->pres_wx.metar_code) - 1);
		p_message->pres_wx.metar_code[sizeof(p_message->pres_wx.metar_code) - 1] = '\0'; // Null terminate the string in the event the buffer was too small.
	}
   	if ((token = NEXT_T)) p_message->temperature = atof(token);
   	if ((token = NEXT_T)) p_message->relative_humidity = atoi(token);
   	if ((token = NEXT_T)) {
		strncpy(p_message->blm, token, sizeof(p_message->blm) - 1);
		p_message->blm[sizeof(p_message->blm) - 1] = '\0'; // Null terminate the string.
	}
   	if ((token = NEXT_T)) p_message->blm_data.luminance = atof(token);
   	if ((token = NEXT_T)) p_message->blm_data.status = (SystemStatus)atoi(token);
   	if ((token = NEXT_T)) p_message->blm_data.is_night = (bool)atoi(token);
   	if ((token = NEXT_T)) p_message->blm_data.units = (uint8_t)atoi(token);
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

	int length = snprintf(msg_buffer, sizeof(msg_buffer),
			"%u %u %u %u %u %c %d %.2f %u %u %u %u %u %u %u %u %u %u %u %u %u %.2f %.2f %u %s %.1f %d %s %.1f %d %d %u",
    		(uint8_t)sensor_one->message_format,			// 1.  uint8_t - Message ID
    		sensor_one->sensor_id,   			   			// 2.  uint8_t - Sensor ID
    		(uint8_t)msg->sys_status,       				// 3.  uint8_t - System Status
    		sensor_one->continuous_interval,  				// 4.  uint16_t - Continuous Interval
    		msg->visibility,           						// 5.  uint32_t (use %u if 32-bit, %lu if 64-bit) - Visability Value
    		(sensor_one->visibility_units == UNITS_METRES) ? 'M' : 'F',	// 6.  char - Viasability Units
    		(int)msg->mor_format,      						// 7.  enum (cast to int for %d) - MOR Format
    		msg->exco,                 						// 8.  float - EXCO
    		(uint8_t)sensor_one->averaging_period,  		// 9.  uint8_t - Avergaing Period
    		msg->sys_alarms.emitter_failure,				// 10. uint8_t - Emitter Failure
    		msg->sys_alarms.emitter_lens_dirty,				// 11. uint8_t - Emitter Dirty Lens
    		msg->sys_alarms.emitter_temperature,   			// 12. uint8_t - Emitter Temperature Failure
    		msg->sys_alarms.detector_lens_dirty,			// 13. uint8_t - Detector Lens Dirty
    		msg->sys_alarms.detector_temperature,			// 14. uint8_t - Detector Temperature Failure
    		msg->sys_alarms.detector_dc_saturation, 		// 15. uint8_t - Detector Daturation Level
    		msg->sys_alarms.hood_temperature,				// 16. uint8_t - Hood Temperature Failure
    		msg->sys_alarms.external_temperature,			// 17. uint8_t - External Temperature Failure
    		msg->sys_alarms.signature_error,				// 18. uint8_t - Signature Error
    		msg->sys_alarms.flash_read_error,				// 19. uint8_t - Flash Read Error
    		msg->sys_alarms.flash_write_error,				// 20. uint8_t - Flash Write Error
    		msg->sys_alarms.particle_limit,					// 21. uint8_t - Particle Limit
    		msg->pres_wx.particle_count,   					// 22. float - Particle Count
    		msg->pres_wx.intensity,         				// 23. float - Intensity
    		msg->pres_wx.synop_code,        				// 24. uint8_t - SYNOP Code
    		msg->pres_wx.metar_code,          				// 25. char[10] - METAR Code
    		msg->temperature,          						// 26. float - Temperature
    		msg->relative_humidity,    						// 27. int8_t - Relative Humidity
    		"BLM",                 		 					// 28. char[10] - BLM
    		msg->blm_data.luminance,   						// 29. float - BLM Luminosity
    		(int)msg->blm_data.status, 						// 30. enum/SystemStatus - BLM Status
    		(int)msg->blm_data.is_night,   					// 31. bool - BLM Is Day or Night
    		msg->blm_data.units            					// 32. uint8_t - BLM Units
			);

	if (length > 0 && length < (int)sizeof(msg_buffer)) {
		uint16_t calculated_crc = crc16_ccitt((uint8_t*)msg_buffer, length);
		safe_serial_write(serial_fd, "\x02%s %04X\x03\r\n", msg_buffer, calculated_crc);
	}
}

/*
 * Name:         parse_command
 * Purpose:      Translates a received string to command enum.
 * Arguments:    buf: the string to translate to a command enum.
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
	memset(cmd, 0, sizeof(ParsedCommand));
	char *stx = strchr(buf, 0x02);
    char *etx = strchr(buf, 0x03);
    if (!stx || !etx) return CMD_INVALID_FORMAT;

    // Search backward from ETX for the second colon
    // String: "... : 8AB9 : <ETX>"
    // Index:       ^      ^   ^
    //              p2     p1 etx
    char *p1 = NULL;
    char *p2 = NULL;

    for (char *p = etx - 1; p > stx; p--) {
        if (*p == ':') {
            if (!p1) p1 = p;      // the colon right before ETX
            else { p2 = p; break; } // the colon before the CRC
        }
    }

    if (!p1 || !p2) return CMD_INVALID_FORMAT; // Both pointers are NULL

    // --- CRC VALIDATION ---
    // Data: From char after <STX> up to the space before checksum. (p2 not included i.e. Colon exlcluded)
    // Length: (Pointer to p2) minus (Start)
    const char *data_start = stx + 1;

    // String: "<STX> GET : 0 : 0 : 8AB9 : <ETX>"
    // Index:     ^   ^           ^          ^
    //           stx data_start   p2        etx
    size_t data_len = (p2) - data_start;  // If the colon is included p2 + 1
    uint16_t calculated = crc16_ccitt((uint8_t*)data_start, data_len); // From data_start to p2.
    // Convert the 4 characters between p2 and p1
	// Find distance between the two colons
    // String: "<STX> GET : 0 : 0 :8AB9: <ETX>"
    // Index:     ^   ^            ^   ^   ^
    //           stx data_start  p2+1  p1 etx
	size_t hex_len = p1 - (p2 + 1); // this should be equal to 4, 6 if there are spaces.

	// Limit it to our buffer size (4 hex digits, plus 2 possible spaces)
	if (hex_len > 6) hex_len = 6;

	char hex_tmp[7] = {0}; // 7 for one extra space for '\0'

    // String: "<STX> GET : 0 : 0 :8AB9: <ETX>"
    // Index:     ^   ^            ^   ^   ^
    //           stx data_start  p2+1  p1 etx
	memcpy(hex_tmp, p2 + 1, hex_len); // Copy the 4 chars of the CRC into hex_tmp.

	// Now strip the spaces so strtol only sees "8AB9"
	strip_whitespace(hex_tmp); // This is liekly not a required step, if the string does not have spaces.
	uint16_t received = (uint16_t)strtol(hex_tmp, NULL, 16);

 	// Check if the CRC received is the same as the data sent with it.
	DEBUG_PRINT("Calculated CRC is %04X\n", calculated);
	DEBUG_PRINT("Received CRC is %04X\n", received);
	if (calculated != received && sensor_one->crc_checking_enabled) return CMD_INVALID_CRC;

    // --- IDENTIFY ENUM & PARSE CONTENT ---
    // Create a temporary work buffer for tokenization
    // This prevents strtok from mangling the original 'buf'
    char work_buf[MAX_INPUT_STR] = {0};
    if (data_len >= sizeof(work_buf)) data_len = sizeof(work_buf) - 1; // Safety check, only copy to the size of the buffer -1 for '\0'.
    memcpy(work_buf, data_start, data_len);
    work_buf[data_len] = '\0'; // technically redundant as we filled the whole buffer with '\0' when we initialized with {0}.

    char *saveptr;
    // Get the first token (The Command: GET, SET, etc.)
	// Byte Index	0	1	2	3	4	5	6	7	8	9	10	11	12	13
	// Original		G	E	T		:		0		:		0		:	\0
	// After Call	G	E	T	\0	:		0		:		0		:	\0
	// cmd_name		^
	// saveptr						^

	// strtok parses each char as a delimiter, so any space or colon are consumed.
	char *cmd_name = strtok_r(work_buf, " :", &saveptr);  // stores CMD name in cmd_name buffer, and the current location in the saveptr.
    if (!cmd_name) return CMD_UNKNOWN;

	if (cmd_name != NULL) {
    	// Map the string name to the CMD Enum
    	if (strcmp(cmd_name, "SET") == 0) {
        	cmd->type = CMD_SET;
    	} else if (strcmp(cmd_name, "SETNC") == 0) {
        	cmd->type = CMD_SETNC;
    	} else if (strcmp(cmd_name, "GET") == 0) {
        	cmd->type = CMD_GET;
    	} else if (strcmp(cmd_name, "POLL") == 0) {
			cmd->type = CMD_POLL;
		} else if (strcmp(cmd_name, "MSGSET") == 0) {
			cmd->type = CMD_MSGSET;
		} else if (strcmp(cmd_name, "ACCRES") == 0) {
			cmd->type = CMD_ACCRES;
		} else cmd->type = CMD_UNKNOWN;
	}

	// Get the sensor address from the command for future use.
	char *addr_str = strtok_r(NULL, " :", &saveptr); // Subsequent calls to strtok_r with NULL returns the next token from work_buf.
	if (addr_str) {
		if ((uint8_t)atoi(addr_str) > 9) return CMD_INVALID_ID;
    	cmd->sensor_id = (uint8_t)atoi(addr_str); // This sets the address
    } else cmd->sensor_id = 0;

    if (cmd->type == CMD_GET) {
        // GET format: GET : 0 : 0
        return CMD_GET;
    }

    if (cmd->type == CMD_POLL) {
        return CMD_POLL; // Process the POLL command.
    }

	if (cmd->type == CMD_ACCRES) {
		return CMD_ACCRES; // Process ACCRES command.
	}

	if (cmd->type == CMD_MSGSET) {
		char *t;
    	char *s = saveptr;

    	// Grab the Hex Bitmap
    	// We use " :" to skip the colon and get to the hex string
    	t = strtok_r(NULL, " :", &s);
    	if (t) {
        	// strtol(string, endptr, base)
        	// Base 16 handles "1FFF" correctly
        	cmd->params.msgset.field_bitmap = (uint16_t)strtol(t, NULL, 16);
  		}
		return CMD_MSGSET;
	}

	if (cmd->type == CMD_SET || cmd->type == CMD_SETNC) {
    	char *t;
    	char *s = saveptr;
		size_t full_length = strlen(buf); // Use strlen() here to count the full length of the buf.
		if (full_length >= MAX_INPUT_STR) full_length = MAX_INPUT_STR - 1; // Safety check, only copy to the size of the buffer -1 for '\0'.
		memcpy(cmd->params.set_params.full_cmd_string, buf, full_length); // Store the full buf CMD string to echo back to the SET command.
    	#define NEXT_T strtok_r(NULL, " ", &s) // Small macro to keep the code below cleaner.

    	// IDs and Alarms
   		if ((t = NEXT_T)) cmd->params.set_params.new_sensor_id = atoi(t);
	    if ((t = NEXT_T)) cmd->params.set_params.alarm1_set    = atoi(t);
    	if ((t = NEXT_T)) cmd->params.set_params.alarm1_active = atoi(t);
	    if ((t = NEXT_T)) cmd->params.set_params.alarm1_dist   = atoi(t);
	    if ((t = NEXT_T)) cmd->params.set_params.alarm2_set    = atoi(t);
	    if ((t = NEXT_T)) cmd->params.set_params.alarm2_active = atoi(t);
	    if ((t = NEXT_T)) cmd->params.set_params.alarm2_dist = atoi(t);

    	// Comms and Serial
    	if ((t = NEXT_T)) cmd->params.set_params.baud_rate = (BaudRateCode)atoi(t);
    	if ((t = NEXT_T)) {
			strncpy(cmd->params.set_params.serial_num, t, MAX_SERIAL_STR - 1);
       		cmd->params.set_params.serial_num[MAX_SERIAL_STR - 1] = '\0';
		}
    	// Operation Modes
	    if ((t = NEXT_T)) {
			if (t[0] == 'M' || t[0] == 'm') {
				cmd->params.set_params.vis_units = UNITS_METRES;
	    	} else {
				cmd->params.set_params.vis_units = UNITS_FEET;
			}
		}
		if ((t = NEXT_T)) cmd->params.set_params.continuous_interval = atoi(t);
	    if ((t = NEXT_T)) cmd->params.set_params.op_mode = (OperatingMode)atoi(t);
	    if ((t = NEXT_T)) cmd->params.set_params.msg_format = (MessageFormat)atoi(t);
	    if ((t = NEXT_T)) cmd->params.set_params.comm_mode = (CommType)atoi(t);
	    if ((t = NEXT_T)) cmd->params.set_params.averaging_period = (AveragingPeriod)atoi(t);
	    if ((t = NEXT_T)) cmd->params.set_params.sample_timing = atoi(t);

    	// Overrides and Advanced
	    if ((t = NEXT_T)) cmd->params.set_params.dew_heater_override = atoi(t);
	    if ((t = NEXT_T)) cmd->params.set_params.hood_heater_override = atoi(t);
	    if ((t = NEXT_T)) cmd->params.set_params.dirty_window_compensation = atoi(t);
	    if ((t = NEXT_T)) cmd->params.set_params.crc_check_en = atoi(t);
	    if ((t = NEXT_T)) cmd->params.set_params.pwr_down_volt = atof(t); // Use atof for float
	    if ((t = NEXT_T)) cmd->params.set_params.rh_threshold = atoi(t);
	    if ((t = NEXT_T)) cmd->params.set_params.data_format = (DataFormat)atoi(t);

    	#undef NEXT_T
		return cmd->type; // Returns CMD_SET or CMD_SETNC
	}
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
void handle_command(CommandType cmd) {

	 switch (cmd) {
        case CMD_POLL:
        	char *line = get_next_line_copy(file_ptr, &file_mutex);
        	if (line) {
				parse_message(line, &p_msg);
				process_and_send(&p_msg);
            	free(line); // caller of get_next_line_copy() must free resource.
				line = NULL;
        	}
			break;
        case CMD_GET:
			char crc_work_buffer[MAX_INPUT_STR];
			int length = snprintf(crc_work_buffer,
				sizeof(crc_work_buffer), "%hhu %d %d %hu %d %d %hu %d %s %c %hu %hu %d %d %d %d %d %d %d %d %.1f %hhu %d",
														sensor_one->sensor_id,
														!!sensor_one->user_alarms.alarm1_set,
														!!sensor_one->user_alarms.alarm1_active,
														sensor_one->user_alarms.alarm1_distance,
														!!sensor_one->user_alarms.alarm2_set,
														!!sensor_one->user_alarms.alarm2_active,
														sensor_one->user_alarms.alarm2_distance,
														sensor_one->baud_rate,
														sensor_one->serial_number,
														sensor_one->visibility_units ? 'F' : 'M',
														sensor_one->continuous_interval,
														sensor_one->mode,
														sensor_one->message_format,
														sensor_one->comm_type,
														sensor_one->averaging_period,
														sensor_one->sample_timing,
														sensor_one->dew_heater_override,
														sensor_one->hood_heater_override,
														sensor_one->dirty_window_compensation,
														sensor_one->crc_checking_enabled,
														sensor_one->power_down_voltage,
														sensor_one->rh_threshold,
														sensor_one->data_format);
			uint16_t calculated_crc = crc16_ccitt((uint8_t*)crc_work_buffer, length);
			safe_serial_write(serial_fd, "\x02%s %04X\x03\r\n", crc_work_buffer, calculated_crc);
			break;
        case CMD_SETNC:
            // Switch Fall-Through SET and SETNC run the same code.
        case CMD_SET:
			// Update sensor id if a change is required.
			if (p_cmd.sensor_id != p_cmd.params.set_params.new_sensor_id && p_cmd.params.set_params.new_sensor_id <= MAX_ADDRESS_NUM) {
				sensor_one->sensor_id = p_cmd.params.set_params.new_sensor_id;
			}
			// Update if alarm 1 is set TRUE/FALSE
			if (sensor_one->user_alarms.alarm1_set != p_cmd.params.set_params.alarm1_set) {
				sensor_one->user_alarms.alarm1_set = p_cmd.params.set_params.alarm1_set;
			}
			// Update if alarm 1 is active TRUE/FALSE
			if (sensor_one->user_alarms.alarm1_active != p_cmd.params.set_params.alarm1_active) {
				sensor_one->user_alarms.alarm1_active = p_cmd.params.set_params.alarm1_active;
			}
			// Update Visibility Units if required. Moved up to above the set distance evaluations to eliminate a check for current units.
			if (sensor_one->visibility_units != p_cmd.params.set_params.vis_units &&
				p_cmd.params.set_params.vis_units <= UNITS_FEET) {
				sensor_one->visibility_units = p_cmd.params.set_params.vis_units;
			}
			// Update alarm 1 distance.
			if (sensor_one->user_alarms.alarm1_distance != p_cmd.params.set_params.alarm1_dist) {
				uint32_t upper_limit = (sensor_one->visibility_units == 0) ? MAX_VISIBILITY_M : MAX_VISIBILITY_FT;
				uint32_t lower_limit = (sensor_one->visibility_units == 0) ? MIN_VISIBILITY_M : MIN_VISIBILITY_FT;
				uint32_t new_distance = p_cmd.params.set_params.alarm1_dist;
				if (new_distance <= upper_limit && new_distance >= lower_limit) {
					sensor_one->user_alarms.alarm1_distance = new_distance;
				}
			}
			// Update if alarm 2 is set TRUE/FALSE
			if (sensor_one->user_alarms.alarm2_set != p_cmd.params.set_params.alarm2_set) {
				sensor_one->user_alarms.alarm2_set = p_cmd.params.set_params.alarm2_set;
			}
			// Update if alarm 2 is active TRUE/FALSE
			if (sensor_one->user_alarms.alarm2_active != p_cmd.params.set_params.alarm2_active) {
				sensor_one->user_alarms.alarm2_active = p_cmd.params.set_params.alarm2_active;
			}
			// Update alarm 2 distance.
			if (sensor_one->user_alarms.alarm2_distance != p_cmd.params.set_params.alarm2_dist) {
				uint32_t upper_limit = (sensor_one->visibility_units == 0) ? MAX_VISIBILITY_M : MAX_VISIBILITY_FT;
				uint32_t lower_limit = (sensor_one->visibility_units == 0) ? MIN_VISIBILITY_M : MIN_VISIBILITY_FT;
				uint32_t new_distance = p_cmd.params.set_params.alarm2_dist;
				if (new_distance <= upper_limit && new_distance >= lower_limit) {
					sensor_one->user_alarms.alarm2_distance = new_distance;
				}
			}
			// Update baud rate of sensor if required. Likely will not implement changes to anything in our serial settings.
			if (sensor_one->baud_rate != p_cmd.params.set_params.baud_rate && p_cmd.params.set_params.baud_rate <= 5) {
				sensor_one->baud_rate = p_cmd.params.set_params.baud_rate;
			}
			// Update sensor serial number if required.
			if (p_cmd.params.set_params.serial_num[0] != '\0') {

		    	if (strncmp(sensor_one->serial_number, p_cmd.params.set_params.serial_num, MAX_SERIAL_STR - 1) != 0) {
		        	strncpy(sensor_one->serial_number, p_cmd.params.set_params.serial_num, MAX_SERIAL_STR - 1);
		        	sensor_one->serial_number[MAX_SERIAL_STR - 1] = '\0';
			    }
			}
			// Update the continuous sending interval.
			if (sensor_one->continuous_interval != p_cmd.params.set_params.continuous_interval &&
				p_cmd.params.set_params.continuous_interval <= MAX_CONT_INTERVAL) {
				sensor_one->continuous_interval = p_cmd.params.set_params.continuous_interval;
			}
			// Update the Operating Mode if required Polling or Continuous).
			if (sensor_one->mode != p_cmd.params.set_params.op_mode && p_cmd.params.set_params.op_mode <= MODE_POLLING) {
				sensor_one->mode = p_cmd.params.set_params.op_mode;
			}
			// Update message format if required.
			if (sensor_one->message_format != p_cmd.params.set_params.msg_format && p_cmd.params.set_params.msg_format <= MSG_RVR_OUTPUT) {
				sensor_one->message_format = p_cmd.params.set_params.msg_format;
			}
			// Update Communications Type.
			if (sensor_one->comm_type != p_cmd.params.set_params.comm_mode && p_cmd.params.set_params.comm_mode <= COMM_RS485) {
				sensor_one->comm_type = p_cmd.params.set_params.comm_mode;
			}
			// Update Averaging period if required.
			// abs(2 * 10 - 11) = 9 || abs(2 * 1 - 11) = 9
			if (sensor_one->averaging_period != p_cmd.params.set_params.averaging_period &&
				(abs(2 * p_cmd.params.set_params.averaging_period - 11) == 9)) { // mathmatic equation equal to 9 if it is either a 1 or a 10.
				sensor_one->averaging_period = p_cmd.params.set_params.averaging_period;
			}
			// Update sample timing if required
			if (sensor_one->sample_timing != p_cmd.params.set_params.sample_timing) {
				sensor_one->sample_timing = p_cmd.params.set_params.sample_timing;
			}
			// Update dew_heater_override TRUE/FALSE
			if (sensor_one->dew_heater_override != p_cmd.params.set_params.dew_heater_override) {
				sensor_one->dew_heater_override = p_cmd.params.set_params.dew_heater_override;
			}
			// Update hood_heater_override TRUE/FALSE
			if (sensor_one->hood_heater_override != p_cmd.params.set_params.hood_heater_override) {
				sensor_one->hood_heater_override = p_cmd.params.set_params.hood_heater_override;
			}
			// Update dirty window compensation TRUE/FALSE
			if (sensor_one->dirty_window_compensation != p_cmd.params.set_params.dirty_window_compensation) {
				sensor_one->dirty_window_compensation = p_cmd.params.set_params.dirty_window_compensation;
			}
			// Update CRC checking
			if (sensor_one->crc_checking_enabled != p_cmd.params.set_params.crc_check_en) {
				sensor_one->crc_checking_enabled = p_cmd.params.set_params.crc_check_en;
			}
			// Update power down voltage.
			if (sensor_one->power_down_voltage != p_cmd.params.set_params.pwr_down_volt) {
				sensor_one->power_down_voltage = p_cmd.params.set_params.pwr_down_volt;
			}
			// Update RH Threshold
			if (sensor_one->rh_threshold != p_cmd.params.set_params.rh_threshold && p_cmd.params.set_params.rh_threshold <= MAX_HUMIDITY) {
				sensor_one->rh_threshold = p_cmd.params.set_params.rh_threshold;
			}

			if (sensor_one->data_format != p_cmd.params.set_params.data_format) {
				sensor_one->data_format = p_cmd.params.set_params.data_format;
			}
			safe_serial_write(serial_fd, "%s", p_cmd.params.set_params.full_cmd_string);
			// Wake up our sender thread, to check if continuous interval changed, or our mode went from polled to continuous.
			pthread_cond_signal(&send_cond);
            break;
		case CMD_MSGSET: {
        	uint32_t requested_bits = p_cmd.params.msgset.field_bitmap;
			// Bitmask 0x3FFF (Binary: 0011 1111 1111 1111)
         	// Allows all defined fields from Bit 0 (Averaging) to Bit 13 (Humidity).
         	// Blocks Bits 14 & 15 which are "Reserved" (8000 and 4000).
			uint32_t allowed_mask = 0x3FFF;

			// requested_bits	0x121A	0001 0010 0001 1010
			// allowed_mask		0x3FFF	0011 1111 1111 1111
			// Result (&)		0x121A	0001 0010 0001 1010

        	if (sensor_one->custom_msg_bits != requested_bits) {
            	if ((requested_bits & allowed_mask) == requested_bits) {
                	sensor_one->custom_msg_bits = requested_bits;
					char hex_buf[10];
					// Convert the bits to a 4-digit uppercase hex string
					int len = sprintf(hex_buf, "%04X", requested_bits);

					// Calculate CRC of the actual characters '1', '2', '1', 'C'
					uint16_t hex_str_crc = crc16_ccitt((uint8_t*)hex_buf, len);
					//uint16_t calculated_crc = crc16_ccitt((uint8_t*)crc_work_buffer, length);
					safe_serial_write(serial_fd, "\x02%s %04X\x03\r\n", hex_buf, hex_str_crc);
					safe_console_print("\x02%s %04X\x03\r\n", hex_buf, hex_str_crc);
            	} else {
                	safe_console_error("Error: Invalid msgset bits (Mask: 0x%04X, Received: 0x%04X)\n", allowed_mask, requested_bits);
            	}
        	}
    	}
			break;
		case CMD_ACCRES:
			sensor_one->present_weather.accumulation = 0;
			safe_serial_write(serial_fd, "%s", p_cmd.params.set_params.full_cmd_string);
			break;
		case CMD_ERROR:
        	safe_console_error("Error: %s\n", strerror(errno));
			break;
		case CMD_INVALID_CRC:
			errno = EBADMSG;
			safe_console_error("CRC Check - Received CRC and Calculated CRC are not equal: %s\n", strerror(errno));
			break;
		case CMD_INVALID_ID:
			safe_console_error("Failed to open file: %s\n", strerror(errno));
			break;
		case CMD_INVALID_FORMAT:
			break;
		case CMD_UNKNOWN:
			break;
        default:
            safe_console_print("CMD: Unknown command\n");
            break;
    }
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
					pthread_mutex_lock(&send_mutex);   // <--- LOCK HERE
		    		handle_command(parse_command(line, &p_cmd)); // handle received command here.
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

    while (!terminate) {
        pthread_mutex_lock(&send_mutex);

		// 1. Determine if we should wait for a specific time or indefinitely
        if (sensor_one != NULL && sensor_one->mode == MODE_CONTINUOUS) {
            // Calculate absolute time: Last Send Time + Interval
            // We use current REALTIME + (Interval - Time Since Last Send)
            clock_gettime(CLOCK_REALTIME, &ts);
            // Add the continuous interval (in seconds) to the current time
            ts.tv_sec += sensor_one->continuous_interval;

            // 2. Wait until that specific second arrives OR a signal interrupts us
            pthread_cond_timedwait(&send_cond, &send_mutex, &ts);
        } else {
            // If in Polling Mode, wait indefinitely for a signal from the receiver
            pthread_cond_wait(&send_cond, &send_mutex);
        }


		// Calculate the next wakeup time (Current time + 10ms)
        //clock_gettime(CLOCK_REALTIME, &ts);
        //ts.tv_nsec += (CPU_WAIT_NANOSECONDS); // 10000000
        //if (ts.tv_nsec >= 1000000000L) {
        //    ts.tv_sec += 1;
        //    ts.tv_nsec -= 1000000000L;
        //}

        // Wait until signaled OR timeout reached.
        // This automatically unlocks send_mutex while waiting.
        //pthread_cond_timedwait(&send_cond, &send_mutex, &ts);

        if (terminate) {
            pthread_mutex_unlock(&send_mutex);
            break;
        }

        // is_ready_to_send() handles the interval and timing logic internally, and checks if the sensor is Pollling or Continuous.
        if (sensor_one != NULL && av30_is_ready_to_send(sensor_one)) {
        	char *line = get_next_line_copy(file_ptr, &file_mutex); // Moved this into the check for is_ready_to_send to avoid depleting the data file.

        	if (line) {
				parse_message(line, &p_msg);
				process_and_send(&p_msg);
            	free(line); // caller of get_next_line_copy() must free resource.
				line = NULL;
        	}
            // Update the last_send_time to the current monotonic clock
            clock_gettime(CLOCK_MONOTONIC, &sensor_one->last_send_time);
        }
        pthread_mutex_unlock(&send_mutex);
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

    file_path = argv[1];

    file_ptr = fopen(file_path, "r");
    if (!file_ptr) {
        safe_console_error("Failed to open file: %s\n", strerror(errno));
		cleanup_and_exit(1);
        return 1;
    }
    //ternary statement to set SERIAL_PORT if supplied in args or the default
    const char *device = (argc >= 3 && is_valid_tty(argv[2]) == 0) ? argv[2] : SERIAL_PORT;

    // ternary statement to set BAUD_RATE if supplied in args or default
    speed_t baud = (argc >= 4) ? get_baud_rate(argv[3]) : BAUD_RATE;

    SerialMode mode = (argc >=5) ? get_mode(argv[4]) : SERIAL_RS485; // returns RS485 by default.

    serial_fd = open_serial_port(device, baud, mode);

    if (serial_fd < 0) {
        cleanup_and_exit(1);
        return 1;
    }

	if (init_av30_sensor(&sensor_one) != 0) {
        safe_console_error("Failed to initialize sensor_one\n");
	  	cleanup_and_exit(1);
    }
    /* define a signal handler, to capture kill signals and instead set our volatile bool 'terminate' to true,
       allowing our c program, to close its loop, join threads, and close our serial device. */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_signal;
    sigaction(SIGINT, &sa, NULL);   // Ctrl-C
    sigaction(SIGTERM, &sa, NULL);  // kill, systemd, etc.

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

	handle_command(parse_command("\x02MSGSET:0:321C:B500:\x03\r\n", &p_cmd));
	handle_command(parse_command("\x02MSGSET:0:121C:5868:\x03\r\n", &p_cmd));

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
	return 0;
}
