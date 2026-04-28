/*
 * File:        tss928.c
 * Author:      Bruce Dearing
 * Date:        04/15/2026
 * Version:     1.0
 * Purpose:     Emulates a Vaisala TSS928 High-Precision Lightning Sensor.
 * 				This program sets up a serial connection with two threads:
 * 				- Receiver thread: parses and responds to incoming commands
 * 				- Sender thread: periodically transmits data strings in broadcast mode
 *
 * 				Supported commands (per Vaisala TSS928 ASCII protocol):
 * 					A - send a present weather message. (default)
 *					B - send a status message.
 *					C - send a selftest message.
 *					D - reset the sensor.
 *					E - perform a type test.
 *					F - send a system run time message.
 *					G - send a version message.
 *					H - set the data output message. Arguments [0-2] 0 == Poll.
 *					I - set the distance unit. Arguments [1-3] 1 == miles. (NOT IMPLEMENTED, only for the TSS924)
 * 					J - set the aging interval. Arguments [1-4] 1 == 15 minutes.
 *					K - set diagnostic mode, and run a test. Arguments [1-3].
 *					L - set the angle of rotation. Arguments [0-359] 0 default.
 *					N - set the current time. Arguments [0-23]:[0-59]:[0-59] 00:00:00 default.
 *					P - return the # of optical and enable crossings. Arguments [0] 0 resets the count.
 *					R - return the average and standard deviation of the last 20 E/B rations.
 *					? or *? - list available commands.
 *					*DEF - restore default settings.
 *					*STATUS - send a status message.
 *					*SELFTEST - send a selftest message.
 *					*RESET - reset the sensor.
 *					*VERSION - send a version message.
 *					*FORMAT - set the data output message. Arguments [0-2] 0 == Poll.
 *					*TIME - set the current time. Arguments [0-23]:[0-59]:[0-59] 00:00:00 default.
 *					*NOISE - return the # of optical and enable crossings. Arguments [0] 0 resets the count.
 *					*EBRATIO - return the average and standard deviation of the last 20 E/B rations.
 * Output format:	(Standard ASCII Broadcast):
 *					Present Weather Message A<Enter>:
 *						NEAR: N [0-65535] NE [0-65535] E [0-65535] SE [0-65535] S [0-65535] SW [0-65535] W [0-65535] NW [0-65535]<CR><LF>
 *						DIST: N [0-65535] NE [0-65535] E [0-65535] SE [0-65535] S [0-65535] SW [0-65535] W [0-65535] NW [0-65535]<CR><LF>
 *						OVHD [0-65535] CLOUD [0-65535] TOTAL [0-65535] [P|F] [00-FF]H [0-99] C [0-65535] [0-65535] [0-65535] [0-65535] [0-65535] [0.000-9.999]<CR><LF>
 *
 *					Status Message B<Enter>:
 *						FLASHES:<CR><LF>
 *						NEAR: N [0-65535] NE [0-65535] E [0-65535] SE [0-65535] S [0-65535] SW [0-65535] W [0-65535] NW [0-65535]<CR><LF>
 *						DIST: N [0-65535] NE [0-65535] E [0-65535] SE [0-65535] S [0-65535] SW [0-65535] W [0-65535] NW [0-65535]<CR><LF>
 *						OVHD [0-65535] CLOUD [0-65535] TOTAL [0-65535]<CR><LF>
 *						STROKES:<CR><LF>
 *						NEAR: N [0-65535] NE [0-65535] E [0-65535] SE [0-65535] S [0-65535] SW [0-65535] W [0-65535] NW [0-65535]<CR><LF>
 *						DIST: N [0-65535] NE [0-65535] E [0-65535] SE [0-65535] S [0-65535] SW [0-65535] W [0-65535] NW [0-65535]<CR><LF>
 *						OVHD [0-65535] CLOUD [0-65535] TOTAL [0-65535]<CR><LF>
 *						PASS [0-65535] FAIL [0-65535] TLOST 0 SLOST 0
 *
 *					Selftest Message C<Enter> or every 30 minutes:
 *						[P|F] [00-FF]H [-99-99] C [0-65535] [0-65535] [0-65535] [0-65535] [0-65535] [0.000-9.999]<CR><LF>
 *
 *					Reset Message D<Enter>:
 *						TSS928 Loader Version 1.5<CR><LF>
 *						TSS928 2.0 September 6, 2001
 *						Copyright (c) 2001, Global Atmosperics, Inc.
 *						[P|F] [00-FF]H [-99-99] C [0-65535] [0-65535] [0-65535] [0-65535] [0-65535] [0.000-9.999]<CR><LF>
 *
 *					Type test message E<Enter>:
 *						ABCDEFGHIJKLMNOPQRSTUVWXYZ 0123456789<CR><LF>
 *
 *					System run time message F<Enter>:
 *						D [0-65535] H [0-23] M [0-59] S [0-59]<CR><LF>
 *
 *					Version Message G<Enter>:
 *						TSS928 V2.0 September 6, 2001<CR><LF>
 *						Copyright (c) 2001, Global Atmospherics, Inc.<CR><LF>
 *
 *					Format Message H<Enter>:
 *						Modes: 0<CR><LF>
 *					Format Message H 0<Enter>:
 *						Modes: 0<CR><LF>
 *					Format Message H 1<Enter>:
 *						Modes: 1<CR><LF>
 *						NEAR: N [0-65535] NE [0-65535] E [0-65535] SE [0-65535] S [0-65535] SW [0-65535] W [0-65535] NW [0-65535]<CR><LF>
 *						DIST: N [0-65535] NE [0-65535] E [0-65535] SE [0-65535] S [0-65535] SW [0-65535] W [0-65535] NW [0-65535]<CR><LF>
 *						OVHD [0-65535] CLOUD [0-65535] TOTAL [0-65535] [P|F] [00-FF]H [0-99] C [0-65535] [0-65535] [0-65535] [0-65535] [0-65535] [0.000-9.999]<CR><LF>
 *					Format Message H 2<Enter>:
 *						Modes: 2<CR><LF>
 *						[FLASH|CLOUD] [0-4,294,967,294,999] [0-4,294,967,294,999] [0-3] [0-65535] [N|NE|E|SE|S|SW|W|NW|O|C]<CR><LF>
 *
 *					Aging Limit Message J[1|2|3|4]<Enter>:
 *						A[15|10|5|30]<CR><LF>
 *
 *					Angle of Rotation Message L[0-359]<Enter>:
 *						R[0-359]<CR><LF>
 *
 *					Current Time Message N<Enter>:
 *						N [0-23]:[0-59]:[0-59]<CR><LF>
 *					Current Time Message N [0-23]:[0-59]:[0-59]<Enter>:
 *						N [0-23]:[0-59]:[0-59]<CR><LF>
 *
 *					E/B Ration Average and Standard Deviation Message R<Enter>:
 *						AVERAGE: [0-9.999] STD: [0-9.999]<CR><LF>
 *
 *					List Commands Message ?<Enter>:
 *						*DEF<CR><LF>
 *						*EBRATIO<CR><LF>
 *						*FORMAT<CR><LF>
 *						*NOISE<CR><LF>
 *						*RESET<CR><LF>
 *						*SELFTEST<CR><LF>
 *						*STATUS<CR><LF>
 *						*TIME<CR><LF>
 *						*VERSION<CR><LF>
 *						*?<CR><LF>
 * 						A<CR><LF>
 *						B<CR><LF>
 *						C<CR><LF>
 *						D<CR><LF>
 *						E<CR><LF>
 *						F<CR><LF>
 *						G<CR><LF>
 *						H<CR><LF>
 *						I<CR><LF>
 * 						J<CR><LF>
 *						K<CR><LF>
 *						L<CR><LF>
 *						N<CR><LF>
 *						P<CR><LF>
 *						R<CR><LF>
 *						?<CR><LF>
 *
 * Threat Levels:
 * 				0 - No Activity: No strikes detected within 30 NM
 * 				1 - Distant: Lightning detected between 15-30 NM
 * 				2 - Vicinity: Lightning detected between 5-15 NM
 * 				3 - Overhead: Lightning detected within 0-5 NM
 *
 * Usage:		flash <file_path> <serial_port_location> <baud_rate> <RS422|RS232>
 *				The serial port must match /dev/tty(S|USB)[0-9]+
 *
 * Example: 	./flash data.txt /dev/ttyUSB0 9600 RS232
 *
 * Sensor:      Vaisala TSS928 Lightning Sensor
 * 				- High-precision thunderstorm detection for airport/critical infra
 *				- Technology: Combined patented magnetic direction finding and electric field pulse analysis
 *				- Detection range: 0-30 Nautical Miles (0-56 km)
 *				- Flash types: Detects Cloud-to-Ground (CG) and Intra-Cloud (IC)
 *				- Directional Accuracy: < 2 degrees RMS
 *				- Range Accuracy: Provides distance based on optical/magnetic waveform coincidence
 *				- Sensitivity: High-gain sensors for low-amplitude IC discharge detection
 *				- Operating temperature: -40°C to +55°C
 *				- Output: RS-232, RS-422, or RS-485
 *				- Default baud rate: 9600 (configurable to 19200)
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
#include "tss928_utils.h"

#define SERIAL_PORT "/dev/ttyUSB0"   // Adjust as needed, main has logic to take arguments for a new location
#define BAUD_RATE   B9600	     // Adjust as needed, main has logic to take arguments for a new baud rate
#define MAX_LINE_LENGTH 1024
#define MAX_CMD_LENGTH 256
#define MAX_MSG_LENGTH 512
#define CPU_WAIT_USEC 10000
#define MINUTE_INTERVAL 60
#define THIRTY_MIN_INTERVAL 1800

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
TSS928_sensor *sensor_one = NULL; // Global pointer to struct for skyvue8 sensor .

// Synchronization primitives
/*	MUTEX		|	OWNS
	send_mutex	|	sensor_one->mode, sensor_one->message_interval, sensor_one->last_send_time, send_cond
	data_mutex	|	sensor_one->StrikeBin, advance_one_minute()
	file_mutex	|	file_ptr
*/
static pthread_mutex_t file_mutex = PTHREAD_MUTEX_INITIALIZER; // protects file_ptr / file access
static pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t data_sleep_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  send_cond; // Moved initialization down to main, to change REALTIME Clock to MONOTONIC.
static pthread_cond_t  data_sleep_cond; // Moved initialization down to main, to change REALTIME Clock to MONOTONIC.

// Global pointers to receiver and sender threads.
pthread_t recv_thread, send_thread, data_thread, sig_thread;

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
	pthread_mutex_lock(&data_mutex); // Lock before changing terminate to 1.
    terminate = 1;
    pthread_mutex_unlock(&data_mutex);
    pthread_cond_broadcast(&send_cond);
	pthread_cond_broadcast(&data_sleep_cond);

	raise(SIGTERM);

	if (recv_thread != 0) {
        pthread_join(recv_thread, NULL);
        recv_thread = 0;
    }
    if (send_thread != 0) {
        pthread_join(send_thread, NULL);
        send_thread = 0;
    }
	if (data_thread != 0) {
		pthread_join(data_thread, NULL);
		data_thread = 0;
	}
	if (sig_thread != 0) {
		pthread_join(sig_thread, NULL);
		data_thread = 0;
	}

    pthread_mutex_destroy(&file_mutex);
	pthread_mutex_destroy(&data_mutex);
	pthread_mutex_destroy(&data_sleep_mutex);
	pthread_cond_destroy(&send_cond);
	pthread_cond_destroy(&data_sleep_cond);

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
void parse_message(char *msg) {
	// RANGE_RINGS 2 // 0:NEAR, 1:DIST
	// QUADRANTS 8 //0:N, 1:NE, 2:E, 3:SE, 4:S, 5:SW, 6:W, 7:NW
	char *saveptr; // Our place keeper in the msg string.
	char *token; // Where we temporarily store each token.
    pthread_mutex_lock(&data_mutex); // Lock before IO on sensor_one.
	// These are pulled from a text file in this format:
	//	NEAR N,NEAR NE, NEAR E,NEAR SE,NEAR S,NEAR SW,NEAR W,NEAR NW,DIST N,DIST NE,DIST E,DIST SE,DIST S,DIST SW,DIST W,DIST NW, OVHD, CLOUD
	//	  0		 0		  0		 0		 0		0		0		0	   0	  0		  0		 0		 0		0		0	   0		0	   0
	if ((token = strtok_r(msg, ",", &saveptr))) record_ground_strike(&sensor_one->strikes, NEAR, NORTH, (uint16_t)atoi(token)); // Range Ring 0 (NEAR), Quadrant 0 (N)
   	#define NEXT_T strtok_r(NULL, ",", &saveptr) // Small macro to keep the code below cleaner.
   	if ((token = NEXT_T)) record_ground_strike(&sensor_one->strikes, NEAR, NORTH_EAST, (uint16_t)atoi(token)); 	// Range Ring 0 (NEAR), Quadrant 1 (NE)
   	if ((token = NEXT_T)) record_ground_strike(&sensor_one->strikes, NEAR, EAST, (uint16_t)atoi(token)); 		// Range Ring 0 (NEAR), Quadrant 2 (E)
	if ((token = NEXT_T)) record_ground_strike(&sensor_one->strikes, NEAR, SOUTH_EAST, (uint16_t)atoi(token)); 	// Range Ring 0 (NEAR), Quadrant 3 (SE)
	if ((token = NEXT_T)) record_ground_strike(&sensor_one->strikes, NEAR, SOUTH, (uint16_t)atoi(token)); 		// Range Ring 0 (NEAR), Quadrant 4 (S)
	if ((token = NEXT_T)) record_ground_strike(&sensor_one->strikes, NEAR, SOUTH_WEST, (uint16_t)atoi(token)); 	// Range Ring 0 (NEAR), Quadrant 5 (SW)
	if ((token = NEXT_T)) record_ground_strike(&sensor_one->strikes, NEAR, WEST, (uint16_t)atoi(token)); 		// Range Ring 0 (NEAR), Quadrant 6 (W)
	if ((token = NEXT_T)) record_ground_strike(&sensor_one->strikes, NEAR, NORTH_WEST, (uint16_t)atoi(token)); 	// Range Ring 0 (NEAR), Quadrant 7 (NW)
	if ((token = NEXT_T)) record_ground_strike(&sensor_one->strikes, DIST, NORTH, (uint16_t)atoi(token)); 		// Range Ring 1 (DIST), Quadrant 0 (N)
	if ((token = NEXT_T)) record_ground_strike(&sensor_one->strikes, DIST, NORTH_EAST, (uint16_t)atoi(token));	// Range Ring 1 (DIST), Quadrant 1 (NE)
	if ((token = NEXT_T)) record_ground_strike(&sensor_one->strikes, DIST, EAST, (uint16_t)atoi(token));		// Range Ring 1 (DIST), Quadrant 2 (E)
	if ((token = NEXT_T)) record_ground_strike(&sensor_one->strikes, DIST, SOUTH_EAST, (uint16_t)atoi(token));	// Range Ring 1 (DIST), Quadrant 3 (SE)
	if ((token = NEXT_T)) record_ground_strike(&sensor_one->strikes, DIST, SOUTH, (uint16_t)atoi(token));		// Range Ring 1 (DIST), Quadrant 4 (S)
	if ((token = NEXT_T)) record_ground_strike(&sensor_one->strikes, DIST, SOUTH_WEST, (uint16_t)atoi(token));	// Range Ring 1 (DIST), Quadrant 5 (SW)
	if ((token = NEXT_T)) record_ground_strike(&sensor_one->strikes, DIST, WEST, (uint16_t)atoi(token));		// Range Ring 1 (DIST), Quadrant 6 (W)
	if ((token = NEXT_T)) record_ground_strike(&sensor_one->strikes, DIST, NORTH_WEST, (uint16_t)atoi(token));	// Range Ring 1 (DIST), Quadrant 7 (NW)
	if ((token = NEXT_T)) record_overhead_strike(&sensor_one->strikes, (uint16_t)atoi(token));					// Overhead Strikes
	if ((token = NEXT_T)) record_cloud_strike(&sensor_one->strikes, (uint16_t)atoi(token));						// Cloud Lightning
	#undef NEXT_T
    pthread_mutex_unlock(&data_mutex); // Unlock after IO on sensor_one.
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
void process_and_send(void) {
	//	NEAR: N [0-65535] NE [0-65535] E [0-65535] SE [0-65535] S [0-65535] SW [0-65535] W [0-65535] NW [0-65535]<CR><LF>
 	//	DIST: N [0-65535] NE [0-65535] E [0-65535] SE [0-65535] S [0-65535] SW [0-65535] W [0-65535] NW [0-65535]<CR><LF>
 	//	OVHD [0-65535] CLOUD [0-65535] TOTAL [0-65535] [P|F] [00-FF]H [0-99] C [0-65535] [0-65535] [0-65535] [0-65535] [0-65535] [0.000-9.999]<CR><LF>

	int temp_total = 0;

    pthread_mutex_lock(&data_mutex); // Lock before IO on sensor_one.
	for (int i = 0; i < RANGE_RINGS; i++) {
		for (int j = 0; j < QUADRANTS; j++) {
			temp_total += sensor_one->strikes.ground_totals[i][j];
		}
	}
	safe_serial_write(serial_fd, "NEAR: N %u NE %u E %u SE %u S %u SW %u W %u NW %u\r\n"
								 "DIST: N %u NE %u E %u SE %u S %u SW %u W %u NW %u\r\n"
								 "OVHD %u CLOUD %u TOTAL %u %c %02xH %d C %u %u %u %u %u %f\r\n",
									sensor_one->strikes.ground_totals[NEAR][NORTH], 		// NEAR N
									sensor_one->strikes.ground_totals[NEAR][NORTH_EAST],	// NEAR NE
									sensor_one->strikes.ground_totals[NEAR][EAST],			// NEAR E
									sensor_one->strikes.ground_totals[NEAR][SOUTH_EAST],	// NEAR SE
									sensor_one->strikes.ground_totals[NEAR][SOUTH],			// NEAR S
									sensor_one->strikes.ground_totals[NEAR][SOUTH_WEST],	// NEAR SW
									sensor_one->strikes.ground_totals[NEAR][WEST],			// NEAR W
									sensor_one->strikes.ground_totals[NEAR][NORTH_WEST],	// NEAR NW
									sensor_one->strikes.ground_totals[DIST][NORTH],			// DIST N
									sensor_one->strikes.ground_totals[DIST][NORTH_EAST],	// DIST NE
									sensor_one->strikes.ground_totals[DIST][EAST],			// DIST E
									sensor_one->strikes.ground_totals[DIST][SOUTH_EAST],	// DIST SE
									sensor_one->strikes.ground_totals[DIST][SOUTH],			// DIST S
									sensor_one->strikes.ground_totals[DIST][SOUTH_WEST],	// DIST SW
									sensor_one->strikes.ground_totals[DIST][WEST],			// DIST W
									sensor_one->strikes.ground_totals[DIST][NORTH_WEST],	// DIST NW
									sensor_one->strikes.overhead_total,						// OVHD
									sensor_one->strikes.cloud_total,						// CLOUD
									temp_total,												// TOTALS
									'P',													// char P | F Pass or Fail
									0,														// Status Code 00-FF
									27,														// Temperature TODO: build a function to get current temperature.
									sensor_one->strikes.total_strikes_since_reset,			// Total Strikes since Self Test
									0,													 	// Total rejected strokes
									0,														// Total rejected by minimum EB ratio since last selftest
									0,														// Total rejected by maximum EB ratio since last selftest
									0,														// Total rejected by minimum B amplitude since last selftest
									0.0);													// Average E/B Ration since last selftest

    pthread_mutex_unlock(&data_mutex); // Unlock after IO on sensor_one.
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
		case CMD_SEND:
			process_and_send();
			break;
		case CMD_STATUS:
			// TODO: NOT Implemented fully, STATUS message sends flashes and strokes.
			process_and_send();
			break;
		case CMD_RESET:
			pthread_mutex_lock(&data_mutex);
			reset_sensor(sensor_one);
			safe_serial_write(serial_fd, "%s\n%s\n%s\n%c %02xH %d C %u %u %u %u %u %f\r\n",
							   sensor_one->loader_version,
							   sensor_one->software_version,
							   sensor_one->copyright_information,
							   'P', 0, 27, sensor_one->strikes.total_strikes_since_reset, 0, 0, 0, 0, 0.000);
			pthread_mutex_unlock(&data_mutex);
			break;
		case CMD_SELFTEST:
			pthread_mutex_lock(&data_mutex);
			conduct_self_test(sensor_one);
			safe_serial_write(serial_fd, "%c %02xH %d C %u %u %u %u %u %f\r\n",'P', 0, 27, sensor_one->strikes.total_strikes_since_reset, 0, 0, 0, 0, 0.000);
			pthread_mutex_unlock(&data_mutex);
			break;
		case CMD_TYPETEST:
			safe_serial_write(serial_fd, "ABCDEFGHIJKLMNOPQRSTUVWXYZ 0123456789\r\n");
			break;
		case CMD_RUNTIME: {
			struct timespec current_time;
			clock_gettime(CLOCK_MONOTONIC, &current_time);
			pthread_mutex_lock(&data_mutex);
			long total_seconds = current_time.tv_sec - sensor_one->sensor_start_time.tv_sec;
			safe_serial_write(serial_fd, "D %ld H %ld M %ld S %ld\r\n",
												(total_seconds / SECONDS_IN_DAY),
												((total_seconds % SECONDS_IN_DAY) / SECONDS_IN_HOUR),
												((total_seconds % SECONDS_IN_HOUR) / SECONDS_IN_MIN),
												(total_seconds % 60));
			pthread_mutex_unlock(&data_mutex);
			break;
		}
		case CMD_VERSION:
			pthread_mutex_lock(&data_mutex);
			safe_serial_write(serial_fd,"%s\n%s\r\n", sensor_one->software_version, sensor_one->copyright_information);
			pthread_mutex_unlock(&data_mutex);
			break;
		case CMD_FORMAT:
			//TODO: Implement a handler for changing the format, currently Aero software only handles polled.
			break;
		case CMD_DIAGNOSTIC:
			//TODO: Implement a handler for handling diagnostics.
			break;
		case CMD_AGING:{
			uint8_t new_interval = (uint8_t)atoi(p_cmd->raw_params);
			pthread_mutex_lock(&data_mutex);
			switch (new_interval) {
				case 1:
					sensor_one->strikes.aging_interval = 15;
					break;
				case 2:
					sensor_one->strikes.aging_interval = 10;
					break;
				case 3:
					sensor_one->strikes.aging_interval = 5;
					break;
				case 4:
					sensor_one->strikes.aging_interval = 30;
					break;
				default:
					break;
			}
			safe_serial_write(serial_fd,"A%u\r\n", sensor_one->strikes.aging_interval);
			pthread_mutex_unlock(&data_mutex);
			break;
		}
		case CMD_ANGLE:{
			uint8_t new_rotation_angle = (uint8_t)atoi(p_cmd->raw_params);
			pthread_mutex_lock(&data_mutex);
			sensor_one->rotation_angle = (new_rotation_angle % 359); // modulus by max degrees to ensure new angle is within limits.
			safe_serial_write(serial_fd,"A%u\r\n", sensor_one->rotation_angle);
			pthread_mutex_unlock(&data_mutex);
			break;
		}
		case CMD_TIME:
			pthread_mutex_lock(&data_mutex);
			if (p_cmd->raw_params[0] == '\0') {
				safe_serial_write(serial_fd, "N %02d:%02d:%02d\r\n", sensor_one->sensor_time.tm_hour, sensor_one->sensor_time.tm_min, sensor_one->sensor_time.tm_sec);
			} else {
				update_sensor_time(p_cmd->raw_params, &sensor_one->sensor_time);
				safe_serial_write(serial_fd, "N %02d:%02d:%02d\r\n", sensor_one->sensor_time.tm_hour, sensor_one->sensor_time.tm_min, sensor_one->sensor_time.tm_sec);
			}
			pthread_mutex_unlock(&data_mutex);
			break;
		case CMD_NOISE:
			// TODO: Likely not required.
			break;
		case CMD_EBRATIO:
			// TODO: Likely not required.
			break;
		case CMD_COMMANDS:
			safe_serial_write(serial_fd,"*DEF\n"
										"*EBRATIO\n"
										"*FORMAT\n"
										"*NOISE\n"
										"*RESET\n"
										"*SELFTEST\n"
										"*STATUS\n"
										"*TIME\n"
										"*VERSION\n"
										"*?\n"
										"A\n"
										"B\n"
										"C\n"
										"D\n"
										"E\n"
										"F\n"
										"G\n"
										"H\n"
										"I\n"
										"J\n"
										"K\n"
										"L\n"
										"N\n"
										"P\n"
										"R\n"
										"?\n");
			break;
		case CMD_RESTORE:
			pthread_mutex_lock(&data_mutex);
			restore_sensor(sensor_one); // Resets the sensor to default settings.
			pthread_mutex_unlock(&data_mutex);
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
    kill_flag = 1;

    // Now safely wake any threads
    pthread_mutex_lock(&data_mutex);
    pthread_cond_broadcast(&send_cond);
    pthread_mutex_unlock(&data_mutex);

    pthread_mutex_lock(&data_sleep_mutex);
    pthread_cond_broadcast(&data_sleep_cond);
    pthread_mutex_unlock(&data_sleep_mutex);


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
	int interval = 0;

    while (!terminate) {

        pthread_mutex_lock(&data_mutex);
		// Determine if we should wait for a specific time or indefinitely
        if (sensor_one != NULL && sensor_one->mode == SMODE_RUN) {
			interval = sensor_one->message_interval; // If we are in polled mode, message_interval is zero.
            // Calculate absolute time: Last Send Time + Interval
            // Use current REALTIME + (Interval - Time Since Last Send)
            clock_gettime(CLOCK_MONOTONIC, &ts);
            // Add the continuous interval (in seconds) to the current time
            ts.tv_sec += interval;
            // Wait until that specific second arrives OR a signal interrupts us
            pthread_cond_timedwait(&send_cond, &data_mutex, &ts);
        } else {
            // If in Polling/Stop Mode, wait indefinitely for a signal from the receiver
            pthread_cond_wait(&send_cond, &data_mutex); // pthread_cond_wait atomically releases the mutex while it sleeps, so receiver_thread can actually acquire data_mutex.
        }

        if (terminate) {
            pthread_mutex_unlock(&data_mutex);
            break;
        }

        // is_ready_to_send() handles the interval and timing logic internally, and checks if the sensor is Polling or Continuous.
        should_send = (sensor_one != NULL && TSS928_is_ready_to_send(sensor_one));

		pthread_mutex_unlock(&data_mutex);  // <-- UNLOCK BEFORE I/O

        // Do I/O operations WITHOUT holding the mutex
        if (should_send) {
                process_and_send();
                fflush(NULL);  // Flush all output streams

			// Update timestamp with lock
            pthread_mutex_lock(&data_mutex);
            if (sensor_one != NULL) {
                clock_gettime(CLOCK_MONOTONIC, &sensor_one->last_send_time);
            }
            pthread_mutex_unlock(&data_mutex);
        }
    }
    return NULL;
}


/*
 * Name:         data_collection_thread
 * Purpose:      Every 10s reads one line from the data file, parses it into the strikes struct in sensor_one.
 *               Every 60s calls update_circular_buffer().
 *				 Every 30m calls conduct_self_test().
 *               Never sends data — that is sender_thread's sole responsibility.
 * Arguments:    arg: unused.
 * Returns:      NULL.
 */
void* data_collection_thread(void* arg) {
    (void)arg;
    struct timespec ts;
    time_t last_buffer_update;
	time_t last_thirty_minute_update;

    // Snapshot the start time so the 60s window begins from thread start
    clock_gettime(CLOCK_MONOTONIC, &ts);
    last_buffer_update = ts.tv_sec;
	last_thirty_minute_update = ts.tv_sec;

    while (!terminate) {
		// 10s sleep — only woken early by terminate
        clock_gettime(CLOCK_MONOTONIC, &ts);
        ts.tv_sec += 10;

        pthread_mutex_lock(&data_sleep_mutex);
        while (!terminate) {
            int rc = pthread_cond_timedwait(&data_sleep_cond, &data_sleep_mutex, &ts);
            if (rc == ETIMEDOUT) break;
        }
        pthread_mutex_unlock(&data_sleep_mutex);

        if (terminate) break;

        // --- Read one line and parse into shared_msg ---
        char *line = get_next_line_copy(file_ptr, &file_mutex);
        if (line) {
            parse_message(line);
            free(line);
            line = NULL;
        }

        // Every 60s: update circular buffer
        clock_gettime(CLOCK_MONOTONIC, &ts);
        if ((ts.tv_sec - last_buffer_update) >= MINUTE_INTERVAL) {
            pthread_mutex_lock(&data_mutex);
            advance_one_minute(&sensor_one->strikes); // Advance the circular buffer
            pthread_mutex_unlock(&data_mutex);
            last_buffer_update = ts.tv_sec;
        }

        // Every 30 minutes: update the self-test values.
		if ((ts.tv_sec - last_thirty_minute_update) >= THIRTY_MIN_INTERVAL) {
    		pthread_mutex_lock(&data_mutex);
			conduct_self_test(sensor_one);  // Conducts the resets of the sensor every 30 minutes.
		    pthread_mutex_unlock(&data_mutex);
   			last_thirty_minute_update = ts.tv_sec;
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

	if (init_TSS928_sensor(&sensor_one) != 0) {
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
    pthread_cond_init(&send_cond, &attr); // Initialize the global variable here
    pthread_cond_init(&data_sleep_cond, &attr); // Initialize the global variable here
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

	if (pthread_create(&data_thread, NULL, data_collection_thread, NULL) != 0) {
        safe_console_error("Failed to create data collection thread: %s\n", strerror(errno));
        terminate = 1;
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

