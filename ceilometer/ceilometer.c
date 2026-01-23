/*
 * File:     celiometer.c
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
 *           Output format:
 *             Message 14 (RVR Output) - space-delimited fields:
 *             <STX><Msg_ID> <Sensor_ID> <Status> <Interval> <Visibility> <Units>
 *             <MOR_Format> <EXCO> <Avg_Period> <12_Alarms> <Particles> <Intensity>
 *             <SYNOP> <METAR> <Temp> <RH> BLM <Luminance> <BLM_Status>
 *             <Day_Night> <BLM_Units> <Checksum><ETX><CR><LF>
 *
 *             Communication: 38400 baud (default), 8 data bits, 1 stop bit, no parity
 *             Line termination: <CR><LF>
 *             Framing: STX (0x02), ETX (0x03)
 *             Checksum: CCITT CRC-16
 *
 *           Network mode:
 *             Supports addressed mode with sensor IDs 0-9 on RS-485 bus
 *             Address format: <command>:<sensor_id>:<reserved>:<checksum>:
 *             Example: POLL:0:0:3A3B: (polls sensor ID 0)
 *
 *           Data includes: visibility (MOR), present weather codes (SYNOP/METAR),
 *                         background luminance, temperature, humidity, precipitation
 *
 * Usage:    ceilometer <file_path> [serial_port] [baud_rate] [RS232|RS485]
 *           ceilometer <file_path> // Defaults: /dev/ttyUSB0, 38400 baud, RS232
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
#include "serial_utils.h"
#include "console_utils.h"
#include "file_utils.h"
#include "crc_utils.h"
#include "atmosvue30_utils.h"

#include <inttypes.h>

#define SERIAL_PORT "/dev/ttyUSB0"   // Adjust as needed, main has logic to take arguments for a new location
#define BAUD_RATE   B38400	     // Adjust as needed, main has logic to take arguments for a new baud rate
#define MAX_LINE_LENGTH 1024


FILE *file_ptr = NULL; // Global File pointer
char *file_path = NULL; // path to file

// Shared state
int continuous = 0;
volatile sig_atomic_t terminate = 0;
volatile sig_atomic_t kill_flag = 0;

int serial_fd = -1;
char site_id = 'A';
//uint8_t address = 0;


/* Synchronization primitives */
static pthread_mutex_t file_mutex  = PTHREAD_MUTEX_INITIALIZER; // protects file_ptr / file access
static pthread_mutex_t send_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  send_cond  = PTHREAD_COND_INITIALIZER;


/*
 * Name:         handle_signal
 * Purpose:      Captures any kill signals, and sets volitile bool 'terminate' and 'kill_flag' to true, allowing thw while loop to break, and threads to join.
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
    pthread_cond_signal(&send_cond);
}


// ---------------- Command handling ----------------



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
CommandType parse_command(const char *buf) {
    if (buf[0] == '!' && buf[1] == '\0')					return CMD_POLL;
    if (buf[0] == '?' && buf[1] == '\0')					return CMD_GET;
    if (buf[0] == '&' && buf[1] == '\0')					return CMD_SET;
    if (buf[0] == '*' && buf[1] >= 'A' && buf[1] <= 'Z' && buf[2] =='\0') 	return CMD_SETNC;
    if (buf && buf[0] >= 'A' && buf[0] <= 'Z' && buf[1] == '\0') 		return CMD_MSGSET;
    return CMD_UNKNOWN;
}


/*
 * Name:         handle_command
 * Purpose:      Handle each command and send response on serial.
 * Arguments:    cmd: the command enum we want to handle.
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
    char *resp_copy = NULL;
    switch (cmd) {
        case CMD_POLL:
		    pthread_mutex_lock(&send_mutex);
            continuous = 1; // enable continuous sending
            pthread_cond_signal(&send_cond);  // Wake sender_thread immediately
            pthread_mutex_unlock(&send_mutex);
            break;

        case CMD_GET:
            pthread_mutex_lock(&send_mutex);
		    continuous = 0; // disables continuous sending.
            pthread_cond_signal(&send_cond);   // Wake sender_thread to exit loop
            pthread_mutex_unlock(&send_mutex);
            break;

        case CMD_SET:
            // safe_console_print("CMD: SITE -> Sending site info\n");
            safe_serial_write(serial_fd, "%c\r\n", site_id);
			//safe_write_response("%c\r\n", site_id);
            break;

        case CMD_SETNC:
            resp_copy = get_next_line_copy(file_ptr, &file_mutex);
            if (resp_copy) {
                // prints <Start of Line ASCII 2>, the string of data read, <EOL ASCII 3>, Checksum of the line read
                safe_serial_write(serial_fd, "\x02%s\x03%02X\r\n", resp_copy, checksumXOR(resp_copy));
                free(resp_copy);
            } else {
                safe_console_error("ERR: Empty file\r\n");
            }
            break;
		case CMD_MSGSET:
			break;
		case CMD_ACCRES:
			break;
		case CMD_ERROR:
			break;
		case CMD_INVALID_CRC:
			break;
		case CMD_INVALID_ID:
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
 *
 * Bugs:         None known.
 * Notes:
 */
void* receiver_thread(void* arg) {
    (void)arg;
    char line[256];
    size_t len = 0;

    while (!terminate) {
        char c;
        int n = read(serial_fd, &c, 1);
        if (n > 0) {
	    if (c == '\r' || c == '\n') {
                if (len > 0) {
                    line[len] = '\0';
		    handle_command(parse_command(line));
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
            usleep(10000);
        }
    }
    return NULL;
}

/*
 * Name:         sender_thread
 * Purpose:      On continuous == 1 and assuming terminate != 1 it will get the next line from a specified file, usinf
 *               get_next_line_copy() and send that line to the serial device using safe_serial_write() function every 2 seconds.
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
    struct timespec requested_time;

    while (!terminate) {
        pthread_mutex_lock(&send_mutex);
        // wait until either terminate is set or continuous becomes 1
        while (!terminate && !continuous) {
            pthread_cond_wait(&send_cond, &send_mutex);
        }

        if (terminate) {
            pthread_mutex_unlock(&send_mutex);
            break;
        }

        while (!terminate && continuous) {
            char *line = get_next_line_copy(file_ptr, &file_mutex);
            if (line) {
                // prints <STX ASCII 2>, the string of data read, <ETX ASCII 3>, Checksum of the line read
                safe_serial_write(serial_fd, "\x02%s\x03%02X\r\n", line, checksumXOR(line));
                free(line); // caller of get_next_line_copy() must free resource.
            }

            clock_gettime(CLOCK_REALTIME, &requested_time);
            requested_time.tv_sec += 2;
            pthread_cond_timedwait(&send_cond, &send_mutex, &requested_time);
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
 *		 uses ternary statements to set either default values for SERIAL_PORT, and BAUD_RATE which are defined above.
 * 		 (condition) ? (value if true) : (value if false)
 *
 * Arguments:    file_path: The location of the file we want to read from, line by line.
 *               device: the string representing the file descriptor of the serial port which should
 * 		 match the pattern ^/dev/tty(S|USB)[0-9]+$. This is tested with function is_valid_tty()
 *		 baud: the string value representing the proposed baud rate, this string is sent to get_baud_rate() which returns a speed_t value.
 *
 * Output:       Prints to stderr the appropriate error messages if encountered.
 * Modifies:     None.
 * Returns:      Returns an int 0 representing success once the program closes the fd, and joins the threads, or 1 if unable to open the serial port.
 * Assumptions:  device is a valid char * pointer and the line contains
 *               characters other than white space, and points to an FD.
 *		 The int provided by arguments is a valid baud rate, although B9600 is set on any errors.
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
        perror("Failed to open file");
        return 1;
    }
    //ternary statement to set SERIAL_PORT if supplied in args or the default
    const char *device = (argc >= 3 && is_valid_tty(argv[2]) == 0) ? argv[2] : SERIAL_PORT;

    // ternary statement to set BAUD_RATE if supplied in args or default
    speed_t baud = (argc >= 4) ? get_baud_rate(argv[3]) : BAUD_RATE;

    SerialMode mode = (argc >=5) ? get_mode(argv[4]) : SERIAL_RS485; // returns RS485 by default.

    serial_fd = open_serial_port(device, baud, mode);

    if (serial_fd < 0) {
        fclose(file_ptr);
        return 1;
    }
    /* define a signal handler, to capture kill signals and instead set our volatile bool 'terminate' to true,
       allowing our c program, to close its loop, join threads, and close our serial device. */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_signal;
    sigaction(SIGINT, &sa, NULL);   // Ctrl-C
    sigaction(SIGTERM, &sa, NULL);  // kill, systemd, etc.

    pthread_t recv_thread, send_thread;

    if (pthread_create(&recv_thread, NULL, receiver_thread, NULL) != 0) {
        perror("Failed to create receiver thread");
        terminate = 1;          // <- symmetrical, but not required
        close(serial_fd);
        fclose(file_ptr);
        return 1;
    }

    if (pthread_create(&send_thread, NULL, sender_thread, NULL) != 0) {
        perror("Failed to create sender thread");
        terminate = 1;          // <- needed because recv_thread is running
        pthread_join(recv_thread, NULL);
        close(serial_fd);
        fclose(file_ptr);
        return 1;
    }

    safe_console_print("Press 'q' + Enter to quit.\n");
    while (!kill_flag) {
        char input[8];
        if (fgets(input, sizeof(input), stdin)) {
            if (input[0] == 'q' || input[0] == 'Q' || kill_flag == 1) {
                pthread_mutex_lock(&send_mutex);
                terminate = 1;
                kill_flag = 1;
                pthread_cond_signal(&send_cond);  // wake sender_thread
                pthread_mutex_unlock(&send_mutex);
		break;
            }
        } else if (feof(stdin)) {  // keep an eye on the behaviour of this check.
            pthread_mutex_lock(&send_mutex);
            terminate = 1;
            kill_flag = 1;
            pthread_cond_signal(&send_cond);      // wake sender_thread
            pthread_mutex_unlock(&send_mutex);
            break; // stdin closed
        } else {
            continue; // temp read error
        }
    }

    pthread_join(recv_thread, NULL);
    pthread_join(send_thread, NULL);

	pthread_mutex_destroy(&file_mutex);
	pthread_mutex_destroy(&send_mutex);
	pthread_cond_destroy(&send_cond);

    close(serial_fd);
    fclose(file_ptr);

    safe_console_print("Program terminated.\n");
	console_cleanup();
	serial_utils_cleanup();
    return 0;
}
