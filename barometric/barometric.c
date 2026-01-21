/*
 * File:     barometric.c
 * Author:   Bruce Dearing
 * Date:     16/01/2026
 * Version:  1.0
 * Purpose:  Emulates a Druck DPS8100 Barometric Pressure Sensor over RS-485/RS-422/USB.
 *           This program sets up a serial connection with two threads:
 *            - Receiver thread: parses and responds to incoming commands
 *            - Sender thread: periodically transmits pressure data when in auto-send mode
 *
 *           Supported commands (per Druck DPS8000 series protocol):
 *             Measurement Commands:
 *               R        - Get current pressure reading
 *               *R       - Get pressure reading with units
 *
 *             Information Commands:
 *               I        - Get transducer identity and setup information
 *               *I       - Get formatted identity information
 *               0:I      - Global identity request (network mode, returns serial number)
 *
 *             General Setup Commands:
 *               A,<interval>              - Set auto-send interval (0.01-9999 sec, 0=off)
 *               A,<format>,<interval>     - Set output format and interval
 *               A,?                       - Query auto-send settings
 *               N,<address>               - Set device address (0-98, 0=direct mode)
 *               N,?                       - Query device address
 *               F,<filter>                - Set filter number (0-5)
 *               F,<filter>,<prescaler>    - Set filter and measurement speed
 *               F,?                       - Query filter settings
 *               U,<unit>                  - Set pressure units (0-24)
 *               U,?                       - Query pressure units
 *               B,<wait>                  - Set global wait interval for network mode
 *               B,?                       - Query wait interval
 *               X,?                       - Status check
 *
 *             PIN-Protected Commands:
 *               C,<PIN>,1,<P1>            - First calibration point
 *               C,<PIN>,2,<P2>            - Second calibration point
 *               C,?                       - Query calibration settings
 *               H,<PIN>,<pressure>        - Change slope/span
 *               H,?                       - Query slope
 *               M,<PIN>,<message>         - Set user message (16 chars max)
 *               M,?                       - Query user message
 *               O,<PIN>,<baud>,<parity>,<databits>,<stopbits>,<termchars> - Set comm settings
 *               O,?                       - Query communication settings
 *               P,<old_PIN>,<new_PIN>     - Change PIN
 *               P,?                       - Query if PIN is set
 *               S,<PIN>,<pressure>        - Set offset
 *               S,<PIN>,X                 - Clear offset
 *               S,?                       - Query offset
 *               W,<PIN>                   - Write settings to non-volatile memory
 *
 *           Output format:
 *             Standard ASCII text: <Pressure value><CR> or <Pressure value><Units><CR>
 *             Communication: 9600 baud (default), 8 data bits, 1 stop bit, no parity
 *             Line termination: <CR> or <CR><LF>
 *
 *           Network mode:
 *             Supports addressed mode with up to 98 devices on RS-485 bus
 *             Address format: <address>:<command>
 *             Global commands: 0:R, 0:I, 0:B (all devices respond with staggered timing)
 *
 *           Data includes: barometric pressure (mbar default), status
 *
 * Usage:    barometric <file_path> [serial_port] [baud_rate] [RS422|RS485]
 *           barometric <file_path> // Defaults: /dev/ttyUSB0, 9600 baud, RS485
 *
 * Sensor:   Druck DPS8100 Barometric Pressure Sensor
 *           - TERPS (Trench Etched Resonant Pressure Sensor) technology
 *           - Digital pressure output with microprocessor
 *           - Pressure range: 600-1100 mbar (typical barometric range)
 *           - Accuracy: ±0.08 mbar (±0.008% FS typical)
 *           - Resolution: 0.001 mbar
 *           - Long-term stability: ±0.02 mbar/year
 *           - Operating temperature: -40°C to +85°C
 *           - Temperature compensation: Internal (automatic)
 *           - Output: RS-232, RS-485, or USB
 *           - Default baud rate: 9600
 *           - Default protocol: RS-485 half-duplex
 *           - Power: 5-32 VDC, 16 mA nominal (RS-485/RS-232), 20 mA (USB)
 *           - Update rate: Configurable via filter settings (10ms to 50s)
 *           - Available pressure units: 25 different units (mbar, Pa, kPa, psi, inHg, etc.)
 *           - Non-volatile memory: Stores configuration, calibration, user message
 *
 * Note:     DPS models have integrated microprocessor and provide digital pressure output only.
 *           RPS models provide raw frequency + diode voltage requiring external calculation.
 *           Temperature is used internally for compensation but not output by DPS models.
 *
 * Aeronautical Parameters (for barometric sensors):
 *           - QFE: Field elevation pressure
 *           - QNH: Nautical height (sea level pressure)
 *           - QFF: Local station pressure reduced to mean sea level
 *           - MSL: Mean sea level pressure
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
#include <regex.h>
#include "barometric_utils.h"
#include "serial_utils.h"
#include "console_utils.h"
#include "file_utils.h"
#include "crc_utils.h"

#define SERIAL_PORT "/dev/ttyUSB0"   // Adjust as needed, main has logic to take arguments for a new location
#define BAUD_RATE   B9600	     // Adjust as needed, main has logic to take arguments for a new baud rate
#define MAX_LINE_LENGTH 1024
#define MAX_CMD_LENGTH 256
#define MAX_FORMAT_NUM 12
#define MIN_TRANS_INTERVAL 0.0f
#define MAX_TRANS_INTERVAL 9999.0f
#define CPU_WAIT_MILLISECONDS 10000
#define MAX_WAIT_INTERVAL 65535

FILE *file_ptr = NULL; // Global File pointer
char *file_path = NULL; // path to file

// Shared state
volatile sig_atomic_t terminate = 0;
volatile sig_atomic_t kill_flag = 0;

// Global variables
int serial_fd = -1;
// uint8_t current_address = 0;
// int current_u_of_m = 6; // Global variable for the current units of measurement for the sensor, 6 (hPa) is the default.
ParsedCommand p_cmd;

/* Synchronization primitives */
static pthread_mutex_t file_mutex = PTHREAD_MUTEX_INITIALIZER; // protects file_ptr / file access
static pthread_mutex_t send_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  send_cond  = PTHREAD_COND_INITIALIZER;

// These need to be freed upon exit.
bp_sensor *sensor_one; // Global pointer to struct for Barometric sensor 1.
bp_sensor *sensor_two; // Global pointer to struct for Barometric sensor 2.
bp_sensor *sensor_three; // Global pointer to struct for Barometric sensor 3.

// Array of 99 pointers (0-98), initialized to NULL to use as a bp_sensor address map.
bp_sensor *sensor_map[99] = {NULL};

/*
 * Name:         handle_signal
 * Purpose:      Captures any kill signals, and sets volitile bool 'terminate' and 'kill_flag' to true, allowing thw while loop to break,
 *				 and threads to join.
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
	// see barometric_utils.h for CommandType enum.

/*
 * Name:         reassign_sensor_address
 * Purpose:      reassigns the bp_sensor address in the sensor_map if it changes.
 * Arguments:    old_addr the previous address it was stored.
 *				 new_addr the new adddress to move it to.
 *
 * Output:       None.
 * Modifies:     Changes pointer addresses of sensor_map.
 * Returns:      None.
 * Assumptions:
 *
 * Bugs:         None known.
 * Notes:
 */
void reassign_sensor_address(uint8_t old_addr, uint8_t new_addr) {

	if (sensor_map[old_addr] == NULL) return; // Nothing to move
	if (old_addr >= 99 || new_addr >= 99) return; 

    // Get the pointer
    bp_sensor *s = sensor_map[old_addr];

    // Update the internal struct value
    s->device_address = new_addr;

    // Update the Map
    sensor_map[new_addr] = s;      // Put it in the new "slot"
    sensor_map[old_addr] = NULL;   // Clear the old "slot"
}


/*
 * Name:         parse_command
 * Purpose:      Translates a received string to command enum.
 * Arguments:    buf: the string to translate to a command enum.
 *				 p_cmd: a global ParsedCommand struct that can hold values to pass to handle_command().
 *
 * Output:       None.
 * Modifies:     Sets all values of p_cmd to zero on call.
 * Returns:      returns an enum representing the correct command, or Unknown Command as the default.
 * Assumptions:  The string recieved is a string and should be able to translate to one of the commands.
 *
 * Bugs:         None known.
 * Notes:
 */
CommandType parse_command(const char *buf, ParsedCommand *p_cmd) {
	memset(p_cmd, 0, sizeof(ParsedCommand));
	const char *ptr = buf;
    // Skip leading whitespace
    while (*ptr == ' ') ptr++;
    // Check for formatter wildcard (*)
    if (*ptr == '*') {
        p_cmd->is_wildcard = true;
        p_cmd->is_formatted = true;
        ptr++;
    }

	    // Check for address (digits followed by colon)
    if (isdigit(*ptr)) {
        char addr_str[4] = {0}; // fill our address string with 0's.
        int i = 0;
        while (isdigit(*ptr) && i < 3) {
            addr_str[i++] = *ptr++;
        }
        if (*ptr == ':') {
            // This is an address
            p_cmd->is_addressed = true;
            p_cmd->address = atoi(addr_str); // convert that address string to an int and store in p_cmd struct.
            ptr++; // skip ':'
        } else {
            // Not an address - invalid format
            return CMD_UNKNOWN;
        }
    }

    // Get command character
    if (!isalpha(*ptr)) {
        return CMD_UNKNOWN; // return unknown, if we get a non-aplha char. !A-Z
    }

	char command_char = toupper(*ptr); // Ensure the command char is in uppercase.
    ptr++;

	// The rest of the string (if any) will be the payload, i.e. A,1,2,4, or A? where the payload would be ?
    const char *payload = ptr;

    // Now parse based on command
    if (*payload == ',') {
        payload++; // skip comma
        switch (command_char) {
            case 'R':
                // Check for R1, R2, etc Note: These are for an RPS sensor not DPS.
                if (isdigit(*payload)) {
                    int variant = *payload - '0';
                    if (p_cmd->is_formatted) {
                        if (variant == 1) return CMD_R1_UNITS;
                        if (variant == 2) return CMD_R2_UNITS;
                    } else {
                        if (variant == 1) return CMD_R1;
                        if (variant == 2) return CMD_R2;
                        if (variant == 3) return CMD_R3;
                        if (variant == 4) return CMD_R4;
                        if (variant == 5) return CMD_R5;
                    }
                } else if (*payload == '?') {
                    return CMD_UNKNOWN;
                }
                return CMD_UNKNOWN;

            case 'A':
                if (*payload == '?') {
	                return p_cmd->is_formatted ? CMD_A_FORMATTED : CMD_A_QUERY;
                } else {
                    // Parse interval and optional format
                    if (sscanf(payload, "%hhu,%f",
                               &p_cmd->params.auto_send.format,
                               &p_cmd->params.auto_send.interval) == 2) {
						// Both values were scanned in, nothing else to do here.
                    } else {
                        p_cmd->params.auto_send.format = 1;  // default
                        sscanf(payload, "%f", &p_cmd->params.auto_send.interval);
                    }
                    return CMD_A_SET;
                }
                break;
            case 'B':
                if (*payload == '?') {
                    return CMD_B_QUERY;
				} else {
                    p_cmd->params.bus_wait.wait_interval = atoi(payload); // update the wait interval.
					return CMD_B_SET;
				}
				break;
			case 'N':
                if (*payload == '?') {
                    return CMD_N_QUERY;
                } else {
                    p_cmd->params.set_address.address = atoi(payload); // update the new address.
                    return CMD_N_SET;
                }
                break;
            // ... more command parsing
            default:
                return CMD_UNKNOWN;
        }
    } else {
        // No payload - simple commands
        switch (command_char) {
            case 'R':
                return p_cmd->is_formatted ? CMD_R_UNITS : CMD_R;
            case 'I':
                return p_cmd->is_formatted ? CMD_I_FORMATTED : CMD_I;
            case 'X':
                return CMD_X_QUERY;
            default:
                return CMD_UNKNOWN;
        }
    }
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
    // char *resp_copy = NULL;

    switch (cmd) {
        case CMD_A_SET:
			if (p_cmd.params.auto_send.format <= MAX_FORMAT_NUM &&
				p_cmd.params.auto_send.interval >= MIN_TRANS_INTERVAL &&
				p_cmd.params.auto_send.interval <= MAX_TRANS_INTERVAL) {
				if (p_cmd.is_addressed && p_cmd.address != 0 && sensor_map[p_cmd.address] != NULL) {
					sensor_map[p_cmd.address]->output_format = p_cmd.params.auto_send.format;
					sensor_map[p_cmd.address]->transmission_interval = p_cmd.params.auto_send.interval;
				} else {
					sensor_one->output_format = p_cmd.params.auto_send.format;
					sensor_one->transmission_interval = p_cmd.params.auto_send.interval;
					sensor_two->output_format = p_cmd.params.auto_send.format;
					sensor_two->transmission_interval = p_cmd.params.auto_send.interval;
					sensor_three->output_format = p_cmd.params.auto_send.format;
					sensor_three->transmission_interval = p_cmd.params.auto_send.interval;
				}
			} else perror("error");
				// continuous = 1; // enable continuous sending
				// pthread_cond_signal(&send_cond);  // Wake sender_thread immediately
            break;
		case CMD_A_FORMATTED:
        	if (p_cmd.is_addressed && p_cmd.address != 0 && sensor_map[p_cmd.address] != NULL) {
				safe_serial_write(serial_fd, "Format = %d\r,Interval = %d\r",
											sensor_map[p_cmd.address]->output_format,
											sensor_map[p_cmd.address]->transmission_interval);
			} else {
				safe_serial_write(serial_fd, "Format = %d\r,Interval = %d\rFormat = %d\r,Interval = %d\rFormat = %d\r,Interval = %d\r",
											sensor_one->output_format,
											sensor_one->transmission_interval,
											sensor_two->output_format,
											sensor_two->transmission_interval,
											sensor_three->output_format,
											sensor_three->transmission_interval);
			}
			break;
        case CMD_A_QUERY:
		    // continuous = 0; // disables continuous sending.
            // pthread_cond_signal(&send_cond);   // Wake sender_thread to exit loop
        	if (p_cmd.is_addressed && p_cmd.address != 0 && sensor_map[p_cmd.address] != NULL) {
				safe_serial_write(serial_fd, "%d,%d\r",
											sensor_map[p_cmd.address]->output_format,
											sensor_map[p_cmd.address]->transmission_interval);
			} else {
				safe_serial_write(serial_fd, "%d,%d\r%d,%d\r%d,%d\r",sensor_one->output_format,
													sensor_one->transmission_interval,
													sensor_two->output_format,
													sensor_two->transmission_interval,
													sensor_three->output_format,
													sensor_three->transmission_interval);
			}
			break;

		case CMD_B_SET:
        		if (p_cmd.is_addressed && p_cmd.address != 0 && sensor_map[p_cmd.address] != NULL) {
					sensor_map[p_cmd.address]->wait_interval = p_cmd.params.bus_wait.wait_interval;
				} else {
					sensor_one->wait_interval = p_cmd.params.bus_wait.wait_interval;
					sensor_two->wait_interval = p_cmd.params.bus_wait.wait_interval;
					sensor_three->wait_interval = p_cmd.params.bus_wait.wait_interval;
				}
			break;
		case CMD_B_QUERY:
			if (p_cmd.is_addressed && p_cmd.address != 0 && sensor_map[p_cmd.address] != NULL) {
				safe_serial_write(serial_fd, "%d\r",sensor_map[p_cmd.address]->wait_interval);
			} else {
				safe_serial_write(serial_fd, "%d\r%d\r%d\r", sensor_one->wait_interval,
															 sensor_two->wait_interval,
															 sensor_three->wait_interval);
			}
			break;
        case CMD_R:
        	if (p_cmd.is_addressed && p_cmd.address != 0 && sensor_map[p_cmd.address] != NULL) {
        		bp_sensor *s = sensor_map[p_cmd.address];
        		safe_serial_write(serial_fd, "%.3f\r\n", s->current_pressure);
    		} else {
        			// Direct mode - send first sensor's data
        		safe_serial_write(serial_fd, "%.3f\r\n", sensor_one->current_pressure);
        		safe_serial_write(serial_fd, "%.3f\r\n", sensor_two->current_pressure);
        		safe_serial_write(serial_fd, "%.3f\r\n", sensor_three->current_pressure);
    		}
    		break;

		case CMD_R_UNITS:
			if (p_cmd.is_addressed && p_cmd.address != 0 && sensor_map[p_cmd.address] != NULL) {
        		bp_sensor *s = sensor_map[p_cmd.address];
        		safe_serial_write(serial_fd, "%.3f %s\r\n",
                         			s->current_pressure,
                         			get_pressure_units_text(s->pressure_units));
    		} else {
        		safe_serial_write(serial_fd, "%.3f %s\r\n",
                         			sensor_one->current_pressure,
                         			get_pressure_units_text(sensor_one->pressure_units));
        		safe_serial_write(serial_fd, "%.3f %s\r\n",
                         			sensor_two->current_pressure,
                         			get_pressure_units_text(sensor_one->pressure_units));
        		safe_serial_write(serial_fd, "%.3f %s\r\n",
                         			sensor_three->current_pressure,
                         			get_pressure_units_text(sensor_one->pressure_units));
    		}
			break;
        case CMD_I:
            break;
		case CMD_N_SET:
			reassign_sensor_address(p_cmd.address, p_cmd.params.set_address.address); // set the address of sensor to new address.
			break;
		case CMD_N_QUERY:
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
    char line[MAX_CMD_LENGTH];
    size_t len = 0;

    while (!terminate) {
        char c;
        int n = read(serial_fd, &c, 1);
        if (n > 0) {
	    if (c == '\r' || c == '\n') {
                if (len > 0) {
                    line[len] = '\0';
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
            usleep(10000);
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
	while (!terminate) {
        pthread_mutex_lock(&send_mutex);

        // Fetch simulated data from file to update global sensor states
        char *line = get_next_line_copy(file_ptr, &file_mutex);
        if (line) {
            int count = sscanf(line, "%f,%f,%f",
                               &sensor_one->current_pressure,
                               &sensor_two->current_pressure,
                               &sensor_three->current_pressure);
            if (count != 3) {
                // If the file line is malformed, we keep last known or zero out
                sensor_one->current_pressure = 0.0f;
                sensor_two->current_pressure = 0.0f;
                sensor_three->current_pressure = 0.0f;
            }
        }
		free(line);
        // Iterate through the sensor map and check if any sensor is "due" for a transmission
        for (int i = 0; i < 99; i++) {
            bp_sensor *s = sensor_map[i];

            // is_ready_to_send() handles the (interval > 0.0f) and timing logic internally
            if (s != NULL && is_ready_to_send(s)) {
                // Perform the serial write
                safe_serial_write(serial_fd, "%f\r\n", s->current_pressure);

                // Update the last_send_time to the current monotonic clock
                clock_gettime(CLOCK_MONOTONIC, &s->last_send_time);
            }
        }

        pthread_mutex_unlock(&send_mutex);

        // Sleep for a short duration (10ms) to prevent CPU spiking.
        // while maintaining 0.01s timing resolution.
        usleep(CPU_WAIT_MILLISECONDS);
    }
    return NULL;
}

/*
 * Name:         Main
 * Purpose:      Main funstion, which opens up serial port, and creates a receiver and transmit threads to listen, and respond to commands
 *               over that serial port. Can take two arguments or no arguments. If changing the serial device name and baud rate, you must supply both.
 *               i.e. tmp_bp_listen <file_path> [serial_device] [baud_rate]
 *				 uses ternary statements to set either default values for SERIAL_PORT, and BAUD_RATE which are defined above.
 * 				 (condition) ? (value if true) : (value if false)
 *
 * Arguments:    file_path: The location of the file we want to read from, line by line.
 *               device: the string representing the file descriptor of the serial port which should
 * 		 		 match the pattern ^/dev/tty(S|USB)[0-9]+$. This is tested with function is_valid_tty()
 *		  		 baud: the string value representing the proposed baud rate, this string is sent to get_baud_rate() which returns a speed_t value.
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
		// fprintf(stderr, "Usage: %s <file_path> <serial_device> <baud_rate> <RS422|RS485>\n", argv[0]);
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
    // define a signal handler, to capture kill signals and instead set our volatile bool 'terminate' to true,
    // allowing our c program, to close its loop, join threads, and close our serial device.
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

	// Initialize BP Sensors
    init_sensor(&sensor_one); // Calls malloc, these sensors must be freed upon exit.
    init_sensor(&sensor_two);
    init_sensor(&sensor_three);

	// Assign hardware addresses (Example addresses)
	sensor_one->device_address = 1;
	sensor_two->device_address = 2;
	sensor_three->device_address = 3;

	// Register them in the lookup table
	sensor_map[sensor_one->device_address] = sensor_one;
	sensor_map[sensor_two->device_address] = sensor_two;
	sensor_map[sensor_three->device_address] = sensor_three;

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

	// Join threads
    pthread_join(recv_thread, NULL);
    pthread_join(send_thread, NULL);

	// destroy mutexes, and conditions
	pthread_mutex_destroy(&file_mutex);
	pthread_mutex_destroy(&send_mutex);
	pthread_cond_destroy(&send_cond);

    close(serial_fd);
    fclose(file_ptr);
	free(sensor_one);
	free(sensor_two);
	free(sensor_three);
	memset(sensor_map, 0, sizeof(sensor_map));
	safe_console_print("Program terminated.\n");
	console_cleanup();
	serial_utils_cleanup();
    return 0;
}
