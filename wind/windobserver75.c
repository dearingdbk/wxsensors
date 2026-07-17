/**
 * @file     windobserver75.c
 * @author   Bruce Dearing
 * @date     07/15/2026
 * @version  1.0
 * @brief    Emulates a Gill Wind Observer 75 Ultrasonic Wind Sensor.
 * @details  This program sets up a serial connection with two threads:
 * 			 - Receiver thread: parses and responds to incoming commands
 * 			 - Sender thread: periodically transmits data strings in continuous mode (M1-M3)
 * 			 using absolute timers to prevent schedule drift.
 *
 * @section Commands Supported commands (per Gill Wind Observer ASCII Protocol):
 * 			All interactive configuration commands must be preceded by the sensor address (e.g., A).
 *			- Address Change:
 *				- A to Z - sets the sensor's network address.
 *			- Measurement Units:
 *				- M1 - sets units to Metres per Second (m/s).
 *				- M2 - sets units to Knots (knots).
 *				- M3 - sets units to Miles per Hour (mph).
 *				- M4 - sets units to Kilometres per Hour (km/h).
 *				- M5 - sets units to Feet per Minute (fpm).
 *			- Output Mode / Format:
 *				- O1 - Polar + Status (Direction, Speed, Status).
 *				- O2 - UV + Status (U-axis, V-axis, Status).
 *				- O3 - Polar + UV + Status (Direction, Speed, U-axis, V-axis, Status).
 *			- Output Rate (Continuous Modes):
 *				- H1 - set output rate to 1 Hz.
 *				- H2 - set output rate to 2 Hz.
 *				- H3 - set output rate to 3 Hz.
 *				- H4 - set output rate to 4 Hz (Default).
 *				- H5 - set output rate to 5 Hz.
 *				- H6 - set output rate to 6 Hz.
 *				- H7 - set output rate to 7 Hz.
 *				- H8 - set output rate to 8 Hz.
 *				- H9 - set output rate to 9 Hz.
 *				- H10 - set output rate to 10 Hz.
 *			- Execution Modes:
 *				- SMODE_M1 - Continuous ASCII UV output mode.
 *				- SMODE_M2 - Continuous ASCII POLAR output mode (Default).
 *			 	- SMODE_M3 - Polled ASCII UV output mode.
 *			 	- SMODE_M4 - Polled ASCII POLAR output mode.
 *				- SMODE_M5 - Continuous NMEA output mode.
 *				- SMODE_M15 - Continuous Averaged ASCII POLAR output mode.
 *				- SMODE_M14 - Polled Averaged ASCII POLAR output mode.
 *
 * @section Outputs Output Formats:
 * 			- Polar Output (Mode O1):
 *	 			Format:  <STX><Address>,<Direction>,<Speed>,<Units>,<Status>,<ETX><Checksum><CR><LF>
 * 				Example: \x02A,284,000.54,M,00,\x0301\r\n
 *
 *			- UV Output (Mode O2):
 * 				Format:  <STX><Address>,<U-Value>,<V-Value>,<Units>,<Status>,<ETX><Checksum><CR><LF>
 *				Example: \x02A,+000.12,-000.52,M,00,\x030F\r\n
 *
 *			- Polar + UV Output (Mode O3):
 *				Format:  <STX><Address>,<Direction>,<Speed>,<U-Value>,<V-Value>,<Units>,<Status>,<ETX><Checksum><CR><LF>
 *				Example: \x02A,284,000.54,+000.12,-000.52,M,00,\x030B\r\n
 *
 * @section  Status Status Codes:
 *			- 00 - OK: Normal operation, no errors detected, heating set off.
 *			- A	 - OK: NMEA data Acceptable
 *			- 01 - Wind sensor axis 1 failed (one or more transducers failed).
 *			- 02 - Wind sensor axis 2 failed (alternate error state).
 *			- 04 - Wind sensor axis 1 and 2 failed.
 *			- 08 - NVM Error.
 *			- 09 - ROM Error.
 *			- 51 - Measurement Average building.
 *			- 62 - No power to heating module.
 *			- 63 - Hardware Fault.
 *			- 65 - Warning Heater supply volts too high, or pcb too hot.
 *			- V  - NMEA data void.
 *			- 66 - OK and heating enabled.
 *			- 67 - No power to heating module.
 *			- 68 - Hardware fault.
 *			- 69 - Warning Heater Supply volts too high or pcb too hot.
 *
 * @section  Usage Usage:
 *			./windobserver75 <file_path> <serial_port_location> <baud_rate> <RS422|RS232>
 *			The serial port must match /dev/tty(S|USB)[0-9]+
 *
 * @section  Example Example:
 *			./windobserver75 data.txt /dev/ttyUSB0 9600 RS422
 *
 * @section  Spec Sensor: Gill Wind Observer 75 Ultrasonic Anemometer
 *			- High-performance, solid-state 2-axis ultrasonic wind sensor.
 *			- Wind Speed Range: 0-75 m/s (0-168 mph).
 *			- Wind Direction Range: 0-359° (no dead band).
 *			- Technology: Ultrasonic time-of-flight measurements (4 transducers).
 *			- Accuracy: Speed +/- 2% @ 12 m/s, Direction +/- 2° @ 12 m/s.
 *			- Alignment: Aligned to physical North arrow on the instrument base.
 *			- Operating Temperature: -55°C to +70°C (with optional heating).
 *			- Output: RS-422 full duplex / RS-232 / RS-485.
 *			- Default Serial Settings: 9600 Baud, 8 Data Bits, 1 Stop Bit, No Parity (8N1).
 *
 * @section  History Modifications:
 *			- 07/15/2026: Refactored sender loop to use absolute pthreads timed-waits for drift-free transmission.
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
#include "windobserver75_utils.h"

#define SERIAL_PORT "/dev/ttyUSB0"   // Adjust as needed, main has logic to take arguments for a new location
#define BAUD_RATE   B9600	     // Adjust as needed, main has logic to take arguments for a new baud rate
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
WO75_sensor *sensor_one = NULL; // Global pointer to struct for skyvue8 sensor .

// Synchronization primitives
/*	MUTEX		|	OWNS
	send_mutex	|	sensor_one->mode, sensor_one->message_interval, sensor_one->last_send_time, sensor_cond
	file_mutex	|	file_ptr
*/
static pthread_mutex_t file_mutex = PTHREAD_MUTEX_INITIALIZER; // protects file_ptr / file access
static pthread_mutex_t sensor_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  sensor_cond; // Moved initialization down to main, to change REALTIME Clock to MONOTONIC.

// Global pointers to receiver and sender threads.
pthread_t recv_thread, send_thread, sig_thread;

bool recv_thread_created = false;
bool send_thread_created = false;
bool sig_thread_created = false;

bool sensor_cond_init = false;

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
	pthread_mutex_lock(&sensor_mutex); // Lock before changing terminate to 1.
    terminate = 1;
    pthread_mutex_unlock(&sensor_mutex);
    if (sensor_cond_init) pthread_cond_broadcast(&sensor_cond);

	if (recv_thread_created) {
        pthread_join(recv_thread, NULL);
        recv_thread_created = false;
    }
    if (send_thread_created) {
        pthread_join(send_thread, NULL);
        send_thread_created = false;
    }
	if (sig_thread_created) {
		pthread_cancel(sig_thread);
		pthread_join(sig_thread, NULL);
		sig_thread_created = false;
	}

    pthread_mutex_destroy(&file_mutex);
	pthread_mutex_destroy(&sensor_mutex);
	if (sensor_cond_init) pthread_cond_destroy(&sensor_cond);

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
void parse_message(char *msg, ParsedMessage *p_msg) {
	memset(p_msg, 0, sizeof(ParsedMessage)); // zero out the ParsedMessage struct.
	char *saveptr; // Our place keeper in the msg string.
	char *token; // Where we temporarily store each token.

	// These are pulled from a text file in this format:
	// A,121,000.8,M,00
	if ((token = strtok_r(msg, ",", &saveptr))) p_msg->msg_address = (char)token[0]; // Sensor Address A-Z
   	#define NEXT_T strtok_r(NULL, ",", &saveptr) // Small macro to keep the code below cleaner.
   	if ((token = NEXT_T)) p_msg->wind_direction = (uint16_t)atoi(token); 	// Wind Direction Polar
   	if ((token = NEXT_T)) p_msg->wind_speed = (float)atof(token); 	// Wind Speed TODO: We could use strof(token, &endptr) to be more robust.
   	if ((token = NEXT_T)) p_msg->msg_status = (uint8_t)atoi(token); 	// Sensor Message Status 00, 60, or see Appendix K
	#undef NEXT_T
    pthread_mutex_lock(&sensor_mutex); // Lock before IO on sensor_one.
	p_msg->msg_units = sensor_one->units;
    pthread_mutex_unlock(&sensor_mutex); // Unlock after IO on sensor_one.
}

/*
 * Name:         process_and_send
 * Purpose:      Parse a data line, format the message string, and send.
 * Arguments:    msg: Pointer to the ParsedMessage struct containing the data stripped from the file/buffer.
 *
 * Output:       Prints the formatted sensor message to serial.
 * Modifies:     None.
 * Returns:      None.
 * Assumptions:  sensor is initialized, and the msg has data fields filled.
 *
 * Bugs:         None known.
 * Notes:
 */
void process_and_send(ParsedMessage *p_msg) {

    char final_msg[MAX_LINE_LENGTH];
    // Builds the msg string, from sensor struct values, and values read from provided file.
    snprintf(final_msg, sizeof(final_msg), "%c,%03d,%06.2f,%c,%02d,", p_msg->msg_address, p_msg->wind_direction, p_msg->wind_speed, p_msg->msg_units, p_msg->msg_status);
    safe_serial_write(serial_fd, "\x02%s\x03%02X\r\n", final_msg, checksumXOR(final_msg));
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
 * Assumptions:  The string received is a string and should be able to translate to one of the commands.
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
            if (next == '\0' || isalnum((unsigned char)next)) { // If the command is terminated, or has additional text. !! Excludes \t \n \r
				cmd->type = cmd_table[i].type;

                if (cmd_table[i].len == 1 && isalpha((unsigned char)ptr[0])) {
                    cmd->sensor_id = (char)toupper((unsigned char)ptr[0]);
                }

				ptr += cmd_table[i].len; // Jump past command
                // Skip any spaces to point at arguments
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
 *		 buf: the original command string received to pass back as required.
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
		case CMD_ENABLE:
			pthread_mutex_lock(&sensor_mutex);
			if (sensor_one->mode == SMODE_M4) {
				sensor_one->mode = SMODE_M2;
			}
			if (sensor_one->mode == SMODE_M3) {
				sensor_one->mode = SMODE_M1;
			}
			if (sensor_one->mode == SMODE_M14) {
				sensor_one->mode = SMODE_M15;
			}
		    pthread_cond_broadcast(&sensor_cond); // Wake up the thread, if it was sleeping.
			pthread_mutex_unlock(&sensor_mutex);
			break;
		case CMD_POLL:
			// TODO: Kludged solution which just sends the configured sensor id, and the next line.
			(void)p_cmd;
			char *line = get_next_line_copy(file_ptr, &file_mutex);
            if (line) {
                ParsedMessage local_msg;  // LOCAL, not global
                parse_message(line, &local_msg);
                process_and_send(&local_msg);
                fflush(NULL);  // Flush all output streams
                free(line);
                line = NULL;
            }
			break;
		case CMD_DISABLE:
			pthread_mutex_lock(&sensor_mutex);
			if (sensor_one->mode == SMODE_M2) {
				sensor_one->mode = SMODE_M4;
			}
			if (sensor_one->mode == SMODE_M1) {
				sensor_one->mode = SMODE_M3;
			}
			if (sensor_one->mode == SMODE_M15) {
				sensor_one->mode = SMODE_M14;
			}
		    pthread_cond_broadcast(&sensor_cond); // Wake up the thread, if it was sleeping.
			pthread_mutex_unlock(&sensor_mutex);
			break;
		case CMD_UNIT_ID:
		    char unit_id_msg[MAX_LINE_LENGTH];
			pthread_mutex_lock(&sensor_mutex);
		    snprintf(unit_id_msg, sizeof(unit_id_msg), "%c", sensor_one->address);
			pthread_mutex_unlock(&sensor_mutex);
    		safe_serial_write(serial_fd, "\x02%s\x03%02X\r\n", unit_id_msg, checksumXOR(unit_id_msg));
			break;
		case CMD_CONFIG:
			// TODO: Unlikely we would need to configure the sensor on the fly.
			break;
		case CMD_UNKNOWN:
			safe_serial_write(serial_fd, "Unrecognized command\r\n");
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

    // Now safely wake any threads
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
		    		handle_command(cmd_type, &local_cmd); // handle received command here.
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
	long interval = 0;

	clock_gettime(CLOCK_MONOTONIC, &ts);

    while (!terminate) {

        pthread_mutex_lock(&sensor_mutex);
		// Determine if we should wait for a specific time or indefinitely
		interval = sensor_one->output_rate; // If we are in polled mode, output_rate is zero.

		long long total_nsec = (long long)ts.tv_nsec + interval;
		ts.tv_sec  += total_nsec / NS_PER_SEC; // 1000000000LL
		ts.tv_nsec  = total_nsec % NS_PER_SEC; // 1000000000LL

        if (sensor_one != NULL && !(sensor_one->mode == SMODE_M3 || sensor_one->mode == SMODE_M4 || sensor_one->mode == SMODE_M14)) {
            // Wait until that specific interval time has passed OR a signal interrupts this thread.
            pthread_cond_timedwait(&sensor_cond, &sensor_mutex, &ts);
        } else {
            // If in Polling/Stop Mode, wait indefinitely for a signal from the receiver
            pthread_cond_wait(&sensor_cond, &sensor_mutex); // pthread_cond_wait atomically releases the mutex while it sleeps, so receiver_thread can acquire sensor_mutex.
			clock_gettime(CLOCK_MONOTONIC, &ts);
        }

        if (terminate) {
            pthread_mutex_unlock(&sensor_mutex);
            break;
        }

        // is_ready_to_send() handles the interval and timing logic internally, and checks if the sensor is Polling or Continuous.
        should_send = (sensor_one != NULL && WO75_is_ready_to_send(sensor_one));

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
			// Update timestamp with lock. Note: this is not a required step, as we removed this logic from WO75_is_ready_to_send().
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
        cleanup_and_exit(1);
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

	if (init_WO75_sensor(&sensor_one) != 0) {
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
	int ret = 0;
	if ((ret = pthread_condattr_init(&attr)) != 0) {
    	fprintf(stderr, "Fatal: pthread_condattr_init failed: %d\n", ret);
	 	cleanup_and_exit(1);
	}
    if ((ret = pthread_condattr_setclock(&attr, CLOCK_MONOTONIC)) != 0) {
    	fprintf(stderr, "Fatal: pthread_condattr_setclock failed: %d\n", ret);
    	pthread_condattr_destroy(&attr);
	  	cleanup_and_exit(1);
	}
    pthread_cond_init(&sensor_cond, &attr); // Initialize the global variable here
	sensor_cond_init = true;
    pthread_condattr_destroy(&attr);

	if (pthread_create(&sig_thread, NULL, signal_thread, NULL) != 0) {
        safe_console_error("Failed to create signal thread: %s\n", strerror(errno));
        terminate = 1;          // <- symmetrical, but not required
		cleanup_and_exit(1);
	} else sig_thread_created = true;

    if (pthread_create(&recv_thread, NULL, receiver_thread, NULL) != 0) {
        safe_console_error("Failed to create receiver thread: %s\n", strerror(errno));
        terminate = 1;          // <- needed because sig_thread is running
		cleanup_and_exit(1);
    } else recv_thread_created = true;

    if (pthread_create(&send_thread, NULL, sender_thread, NULL) != 0) {
        safe_console_error("Failed to create sender thread: %s\n", strerror(errno));
        terminate = 1;          // <- needed because recv_thread is running
		cleanup_and_exit(1);
    } else send_thread_created = true;

    safe_console_print("Press 'ctrl-c' to quit.\n");
	pthread_join(sig_thread, NULL);
	sig_thread_created = false;
    safe_console_print("Program %s terminated.\n", program_name);
	cleanup_and_exit(0);
	return 0; // We won't get here, but it quiets verbose warnings on a no return value.
}

