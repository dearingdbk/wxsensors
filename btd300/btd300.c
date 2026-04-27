/*
 * File:     btd300.c
 * Author:   Bruce Dearing
 * Date:     05/01/2026
 * Version:  1.0
 * Purpose:  Emulates a Biral BTD-300 Thunderstorm Detector over RS-422.
 *           This program sets up a serial connection with two threads:
 *            - Receiver thread: parses and responds to incoming commands
 *            - Sender thread: periodically transmits lightning/warning data when in sampling mode
 *
 *           Supported commands (per Biral BTD protocol):
 *             RUN      - Start normal sampling mode
 *             STOP     - Stop sampling, enter idle mode
 *             R?       - Request self-test/status message
 *             PV?      - Request program version
 *             SN?      - Request serial number
 *             RTC?     - Request real-time clock date/time
 *             SITE?    - Request site characterisation values
 *             LOCAL?   - Request local calibration values
 *             LEVEL?   - Request sensitivity levels
 *             DIST?    - Request distance limits
 *             RELAY?   - Request relay parameters
 *             HYST?    - Request hysteresis times
 *             W?       - Request distant flash warning parameters
 *             DOSITE   - Start site characterisation process
 *             RST      - Reset sensor
 *
 *           Output format (DATA message transmitted every 2 seconds):
 *             DATA:,ID,DDMMYY,HHMMSS,A,B,CC,DDDDD,flash1,flash2,flash3,flash4
 *           Where:
 *             ID     - Sensor identification (01-99)
 *             A      - Number of flashes detected (0-4)
 *             B      - Warning indicator (0=None, 1=Warning, 2=Alert, 3=Severe)
 *             CC     - Warning flags (corona, charged precip, lightning proximity)
 *             DDDDD  - Self-test flags (antenna status, faults)
 *             Each flash: DDMMYY,HHMMSS,CCC,XXXXX,XXX (time, centisec, distance decametres, bearing degrees)
 *
 *           Warning levels:
 *             0 - No warning: No thunderstorm activity
 *             1 - Warning: Charged precipitation or distant flash (10-30 NM)
 *             2 - Alert: Strong electric field or vicinity flash (5-10 NM)
 *             3 - Severe Alert: Overhead flash (<5 NM)
 *
 * Usage:    use case ' flash <file_path> <serial_port_location> <baud_rate> <RS422|RS485> The serial port currently must match /dev/tty(S|USB)[0-9]+
 * 	         use case ' flash <file_path> // The serial port, baud rate, and mode will be set to  defaults /dev/ttyUSB0, and B9600
 *			 use case ' flash <(socat - TCP:lightningdata.com:8080,forever,intervall=10) <serial_port_location> <baud_rate> <RS422|RS485> // in the event we use external data
 *           Serial port must match pattern: /dev/tty(S|USB)[0-9]+
 *
 * Sensor:   Biral BTD-300 Thunderstorm Detector
 *           - Standalone lightning detection and thunderstorm warning system
 *           - Technology: quasi-electrostatic field sensing (1-47 Hz)
 *           - Detection range: 0-83 km (0-45 nautical miles)
 *           - Flash types: Cloud-to-ground (CG), intra-cloud (IC), cloud-to-cloud (CC)
 *           - Detection efficiency: >95% for single flash, 99% for 2+ flashes within 56 km
 *           - Range accuracy: ±3 NM (0-10 NM), ±5.5 NM (10-45 NM)
 *           - False alarm rate: <2%
 *           - Additional sensing: charged precipitation, strong electric field detection
 *           - Corona initiator spikes for overhead thunderstorm development warning
 *           - Optional direction finder module (bearing to nearest degree)
 *           - Update period: 2 seconds
 *           - Maximum flash rate: 120 flashes per minute
 *           - Operating temperature: -55°C to +60°C
 *           - Output: RS-422 (default) or Ethernet
 *           - Default baud rate: 9600
 *           - Optional relay outputs (3x) for warning/alert/severe alert
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
#include "serial_utils.h"
#include "console_utils.h"
#include "file_utils.h"
#include "btd300_utils.h"

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
BTD300_sensor *sensor_one = NULL; // Global pointer to struct for skyvue8 sensor .

// Synchronization primitives
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

   	raise(SIGTERM);

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
	char temp_date_holder[DATE_STRING];
	char temp_time_holder[TIME_STRING];
	// These are pulled from a text file in this format:
	// DATA:,01,131125,142809,0,0,00,OOOOO,000000,000000,000,00000,000,000000,000000,000,00000,000,000000,000000,000,00000,000,000000,000000,000,00000,000
	if ((token = strtok_r(msg, ",", &saveptr))) {
        strncpy(p_message->data_header, token, MAX_HEADER_STR - 1); // Capture the message header 'DATA:' ** We could discard this.
        p_message->data_header[MAX_HEADER_STR - 1] = '\0';
    }
   	#define NEXT_T strtok_r(NULL, ",", &saveptr) // Small macro to keep the code below cleaner.
   	if ((token = NEXT_T)) p_message->site_id = (uint8_t)atoi(token); // Set site ID.
   	if ((token = NEXT_T)) {
		strncpy(temp_date_holder, token, DATE_STRING - 1);
		temp_date_holder[DATE_STRING - 1] = '\0';
	}
	if ((token = NEXT_T)) {
		strncpy(temp_time_holder, token, TIME_STRING - 1);
		temp_time_holder[TIME_STRING - 1] = '\0';
   	}
	// HANDLE EPOCH CONVERSION
	p_message->original_epoch = parse_to_epoch(temp_date_holder, temp_time_holder);
	if ((token = NEXT_T)) p_message->number_of_flashes = (uint8_t)atoi(token); // Number of Flashes in the message string.
   	if ((token = NEXT_T)) p_message->warning_indicator = (uint8_t)atoi(token); // Warning Indicator.
   	if ((token = NEXT_T)) p_message->warning_flags = (uint8_t)atoi(token); // Warning Indicator.
	if ((token = NEXT_T)) {
		strncpy(p_message->self_test_flags, token, MAX_SELF_TEST_FLAG - 1); // Set the Self Test Flags.
		p_message->self_test_flags[MAX_SELF_TEST_FLAG - 1] = '\0';
	}

	if (p_message->number_of_flashes != 0) { // The rest of the string is all zeros, if the number of flashes is zero.
		// FLASH ONE
	   	if ((token = NEXT_T)) {
			strncpy(temp_date_holder, token, DATE_STRING - 1);
			temp_date_holder[DATE_STRING - 1] = '\0';
		}
		if ((token = NEXT_T)) {
			strncpy(temp_time_holder, token, TIME_STRING - 1);
			temp_date_holder[TIME_STRING - 1] = '\0';
	   	}
		p_message->flash_epoch_array[0] = parse_to_epoch(temp_date_holder, temp_time_holder);
		if ((token = NEXT_T)) p_message->time_since_flash_one = (uint8_t)atoi(token); // # of 10 millisecond intervals since Flash one.
	   	if ((token = NEXT_T)) p_message->distance_of_flash_one = (uint16_t)atoi(token); // Distance of Flash one.
	   	if ((token = NEXT_T)) p_message->direction_of_flash_one = (uint16_t)atoi(token); // Direction of Flash one.

		if (p_message->number_of_flashes < 2) goto end_flashes; // Jump to the end if the rest of the string is zeros.
		// FLASH TWO
	   	if ((token = NEXT_T)) {
			strncpy(temp_date_holder, token, DATE_STRING - 1);
			temp_date_holder[DATE_STRING - 1] = '\0';
		}
		if ((token = NEXT_T)) {
			strncpy(temp_time_holder, token, TIME_STRING - 1);
			temp_date_holder[TIME_STRING - 1] = '\0';
		}
		p_message->flash_epoch_array[1] = parse_to_epoch(temp_date_holder, temp_time_holder);
	   	if ((token = NEXT_T)) p_message->time_since_flash_two = (uint8_t)atoi(token); // # of 10 millisecond intervals since Flash two.
	   	if ((token = NEXT_T)) p_message->distance_of_flash_two = (uint16_t)atoi(token); // Distance of Flash two.
	   	if ((token = NEXT_T)) p_message->direction_of_flash_two = (uint16_t)atoi(token); // Direction of Flash two.

		if (p_message->number_of_flashes < 3) goto end_flashes; // Jump to the end if the rest of the string is zeros.
		// FLASH THREE
   		if ((token = NEXT_T)) {
			strncpy(temp_date_holder, token, DATE_STRING - 1);
			temp_date_holder[DATE_STRING - 1] = '\0';
		}
		if ((token = NEXT_T)) {
			strncpy(temp_time_holder, token, TIME_STRING - 1);
			temp_date_holder[TIME_STRING - 1] = '\0';
		}
		p_message->flash_epoch_array[2] = parse_to_epoch(temp_date_holder, temp_time_holder);
   		if ((token = NEXT_T)) p_message->time_since_flash_three = (uint8_t)atoi(token); // # of 10 millisecond intervals since Flash three.
   		if ((token = NEXT_T)) p_message->distance_of_flash_three = (uint16_t)atoi(token); // Distance of Flash three.
   		if ((token = NEXT_T)) p_message->direction_of_flash_three = (uint16_t)atoi(token); // Direction of Flash three.

		if (p_message->number_of_flashes < 4) goto end_flashes; // Jump to the end if the rest of the string is zeros.
		// FLASH FOUR
	   	if ((token = NEXT_T)) {
			strncpy(temp_time_holder, token, TIME_STRING - 1);
			temp_date_holder[TIME_STRING - 1] = '\0';
		}
		if ((token = NEXT_T)) {
			strncpy(temp_time_holder, token, TIME_STRING - 1);
			temp_date_holder[TIME_STRING - 1] = '\0';
		}
		p_message->flash_epoch_array[3] = parse_to_epoch(temp_date_holder, temp_time_holder);
		if ((token = NEXT_T)) p_message->time_since_flash_four = (uint8_t)atoi(token); // # of 10 millisecond intervals since Flash four.
	   	if ((token = NEXT_T)) p_message->distance_of_flash_four = (uint16_t)atoi(token); // Distance of Flash four.
	   	if ((token = NEXT_T)) p_message->direction_of_flash_four = (uint16_t)atoi(token); // Direction of Flash four.
	}
	end_flashes:
	#undef NEXT_T
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
void process_and_send(ParsedMessage *msg) {
	if (msg == NULL) return;
	char current_date_string[DATE_STRING] = {0};
	char current_time_string[TIME_STRING] = {0};
	char flash_time_array[MAX_FLASHES][TIME_STRING] = {"000000","000000","000000","000000"};
	char flash_date_array[MAX_FLASHES][DATE_STRING] = {"000000","000000","000000","000000"};

	time_t now;
	time(&now);
	epoch_to_date(now, current_date_string); // Set our current date string DDMMYY.
	epoch_to_time(now, current_time_string); // Set our current time string HHMMSS.

	// Update the date and time from the data file, to match current date and time, while maintaining the delta between flash times, and the orignal time.
	for (int i = 0; i < msg->number_of_flashes; i++) {
		epoch_to_date((now - (msg->original_epoch - msg->flash_epoch_array[i])), flash_date_array[i]);  // Updates the date to current UTC date
		epoch_to_time((now - (msg->original_epoch - msg->flash_epoch_array[i])), flash_time_array[i]);  // Updates the time to current UTC time
	}

	// DATA:,01,131125,142809,0,0,00,OOOOO,000000,000000,000,00000,000,000000,000000,000,00000,000,000000,000000,000,00000,000,000000,000000,000,00000,000
	safe_serial_write(serial_fd, "%s,%02hhu,%s,%s,%hhu,%hhu,%02hhu,%s,"
								 "%s,%s,%03hhu,%05hu,%03hu," 		// Flash 1
								 "%s,%s,%03hhu,%05hu,%03hu," 		// Flash 2
								 "%s,%s,%03hhu,%05hu,%03hu," 		// Flash 3
								 "%s,%s,%03hhu,%05hu,%03hu\r\n", 	// Flash 4
									msg->data_header, 				// DATA:
									msg->site_id, 					// Site ID
									current_date_string,			// Current Date
									current_time_string,			// Current Time
									msg->number_of_flashes,			// The number of flashes on this line 0-4.
									msg->warning_indicator,			// 0, 1, 2 or 3.
									msg->warning_flags,				// Warning Flags if any
									msg->self_test_flags,			// Self Test flags
									flash_date_array[0],		 	// Updated date of Flash 1
									flash_time_array[0],			// Updated time of Flash 1
									msg->time_since_flash_one,		// # of 10 millisecond intervals since Flash 1
									msg->distance_of_flash_one,		// Distance in decametres of Flash 1
									msg->direction_of_flash_one,	// Direction in degrees of Flash 1
									flash_date_array[1],		 	// Updated date of Flash 2
									flash_time_array[1],			// Updated time of Flash 2
									msg->time_since_flash_two,		// # of 10 millisecond intervals since Flash 2
									msg->distance_of_flash_two,		// Distance in decametres of Flash 2
									msg->direction_of_flash_two,	// Direction in degrees of Flash 2
									flash_date_array[2],			// Updated date of Flash 3
									flash_time_array[2],			// Updated time of Flash 3
									msg->time_since_flash_three,	// # of 10 millisecond intervals since Flash 3
									msg->distance_of_flash_three,	// Distance in decametres of Flash 3
									msg->direction_of_flash_three,	// Direction in degrees of Flash 3
									flash_date_array[3],			// Updated date of Flash 4
									flash_time_array[3],			// Updated time of Flash 4
									msg->time_since_flash_four,		// # of 10 millisecond intervals since Flash 4
									msg->distance_of_flash_four,	// Distance in decametres of Flash 4
									msg->direction_of_flash_four);	// Direction in degrees of Flash 4
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
            if (next == '\0' || isalnum((unsigned char)next) || next == '?') { // If the command is terminated, or has additional text. !! Excludes \t \n \r
				cmd->type = cmd_table[i].type;
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
	 switch (cmd) {
		case CMD_RUN:
			DEBUG_PRINT("RUN Command Received with these params: %s\n", p_cmd->raw_params);
			if (sensor_one->mode == SMODE_RUN) {
				safe_serial_write(serial_fd, "%s\r\n", "COMMAND NOT ALLOWED");
			} else {
				sensor_one->mode = SMODE_RUN;
				pthread_cond_signal(&sensor_cond); // Wake our sender thread, to check if our mode has changed.
			}
			break;
		case CMD_STOP:
			DEBUG_PRINT("STOP Command Received with these params: %s\n", p_cmd->raw_params);
			if (sensor_one->mode == SMODE_STOP) {
				safe_serial_write(serial_fd, "%s\r\n", "COMMAND NOT ALLOWED");
			} else {
				sensor_one->mode = SMODE_STOP;
				pthread_cond_signal(&sensor_cond); // Wake our sender thread, to check if our mode has changed.
			}
			break;
		case CMD_SITE:
				safe_serial_write(serial_fd, "%hhu\r\n", sensor_one->address);
			break;
		case CMD_SELF_TEST:
			safe_console_error("%s: Invalid sensor ID: %s\n", program_name, strerror(errno));
			break;
		case CMD_DIST:
			DEBUG_PRINT("DIST Command Received with these params: %s\n", p_cmd->raw_params);
			char *saveptr; // Our place keeper in the msg string.
			char *token; // Where we temporarily store each token.
			int distance_id;
			int distance;

			if ((token = strtok_r(p_cmd->raw_params, " \r\n", &saveptr))) {
				if (strncmp(token, "DEF", 3) == 0) { // "DEFDIST" received, reset to FAA defaults.
					if (BTD300_is_ready_to_send(sensor_one)) {
						safe_serial_write(serial_fd, "%s\r\n", "COMMAND NOT ALLOWED");
					} else {
						reset_flash(&sensor_one);
                		safe_serial_write(serial_fd, "%s\r\n", "OK");
					}
				} else if (strncmp(token, "?", 1) == 0) { // "DIST?" Get Distance Limits
					safe_serial_write(serial_fd, "%s,%hu,%hu,%hu,%hu\r\n", "DIST:",
					sensor_one->overhead,
					sensor_one->vicinity,
					sensor_one->near_distant,
					sensor_one->far_distant);
				} else if (sscanf(token, "%d,%d", &distance_id, &distance) == 2) { // Both values successfully extracted
					if (BTD300_is_ready_to_send(sensor_one)) {
                		safe_serial_write(serial_fd, "%s\r\n", "COMMAND NOT ALLOWED");
					} else {
						set_dist(&sensor_one, distance_id, distance);
						safe_serial_write(serial_fd, "%s\r\n", "OK");
					}
				} // TODO: Enter a handler for a bad DIST command parameters.
			} // TODO: Enter a handler for a null token.

			break;
		case CMD_GET_SER:
            safe_serial_write(serial_fd, "%s\r\n", sensor_one->serial_number);
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
        should_send = (sensor_one != NULL && BTD300_is_ready_to_send(sensor_one));

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

	if (init_BTD300_sensor(&sensor_one) != 0) {
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

	// create the signal thread

	if (pthread_create(&sig_thread, NULL, signal_thread, NULL) != 0) {
		safe_console_error("Failed to create signal thread: %s\n", strerror(errno));
        terminate = 1;          // <- symmetrical, but not required
		cleanup_and_exit(1);
	}

    if (pthread_create(&recv_thread, NULL, receiver_thread, NULL) != 0) {
        safe_console_error("Failed to create receiver thread: %s\n", strerror(errno));
        terminate = 1;          // <- needed becaue sig_thread is running.
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
