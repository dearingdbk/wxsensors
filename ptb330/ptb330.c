/*
 * File:     	ptb330.c
 * Author:   	Bruce Dearing
 * Date:     	16/01/2026
 * Version:  	1.0
 * Purpose:  	Emulates a Vaisala PTB330 Digital Barometer over RS-232/RS-485/RS-422.
 * 				This program sets up a serial connection with two threads:
 * 					- Receiver thread: parses and responds to incoming Vaisala serial commands
 * 					- Sender thread: periodically transmits data based on SMODE settings
 *
 * 				Supported commands (per Vaisala PTB330 Protocol):
 * 	Measurement Commands:
 * 				R         - Start continuous output (RUN mode)
 * 				S         - Stop continuous output (STOP mode)
 * 				SEND      - Request a single measurement message
 * 				INTV      - Set/Query continuous output interval (0-255 s/min/h/d)
 *
 * 	Information Commands:
 * 				?         - Show device information (SN, SW version, etc.)
 * 				VERS      - Display product name and software version
 * 				SNUM      - Show serial numbers for device and modules
 * 				ERRS      - List and clear unacknowledged error flags
 * 				HELP      - Display list of available serial commands
 *
 * 	Configuration Commands:
 * 				UNIT <u/q> - Set units (hPa, mbar, kPa, Pa, inHg, mmH2O, mmHg, torr, psi)
 * 				FORM <str> - Set custom output format string
 * 				SMODE <m>  - Set start mode (STOP, POLL, RUN, SEND)
 * 				ADDR <a>   - Set device address for RS-485 networking (0-255)
 * 				SERI <b p d s> - Set baud (up to 115200), parity, data bits, stop bits
 * 				SDELAY <n> - Set response delay (0-2540 ms)
 * 				ECHO <on/off> - Enable/Disable serial character echo
 *
 * 	Aeronautical/Adjustment Commands:
 * 				HHCP <m>   - Set altitude for Height Corrected Pressure (HCP)
 * 				HQNH <m>   - Set altitude for QNH corrected pressure
 * 				HQFE <m>   - Set altitude for QFE corrected pressure
 * 				TQFE <deg> - Set temperature for QFE calculation
 * 				PSTAB <hPa> - Set pressure stability indicator limit
 * 				AVRG <s>   - Set measurement averaging time (0-600 seconds)
 *
 * 	Output format:
 * 				Configurable via FORM command.
 * 				Default: P = <value> <unit> (e.g., "P = 1013.25 hPa")
 * 				Communication: 9600 8N1 (default), user-configurable
 * 				Line termination: <CR><LF>
 *
 * 	Networking (RS-485):
 * 				Supports address-based polling for up to 255 devices.
 * 				Commands: OPEN <addr>, CLOSE, SEND <addr>
 *
 * 	Sensor:    	Vaisala PTB330 BAROCAP Digital Barometer
 * 					- Sensor Tech: Vaisala BAROCAP (Silicon Capacitive Absolute Pressure)
 * 					- Redundancy: 1, 2, or 3 internal sensors (Class A or Class B)
 * 					- Range: 500-1100 hPa (Standard), 50-1100 hPa (Extended)
 * 					- Accuracy: ±0.10 hPa (Class A at 20°C)
 * 					- Resolution: 0.01 hPa (Class A)
 * 					- Long-term stability: ±0.1 hPa/year
 * 					- Operating temp: -40°C to +60°C
 * 					- Output: RS-232, RS-485, RS-422, USB, or Analog (optional)
 *
 * 	Note: 		PTB330 calculates WMO pressure trend/tendency (3-hour history).
 * 				Supports PA11A emulation mode for legacy system replacement.
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
#include "ptb330_utils.h"

#define SERIAL_PORT "/dev/ttyUSB0"   // Adjust as needed, main has logic to take arguments for a new location
#define BAUD_RATE   B9600	     // Adjust as needed, main has logic to take arguments for a new baud rate
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
ptb330_sensor *sensor_one = NULL; // Global pointer to struct for atmosvue30 sensor .

ParsedCommand p_cmd;
ParsedMessage p_msg;

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
	int length = snprintf(msg_buffer, sizeof(msg_buffer), "%u", 2);

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
	if (buf == NULL || cmd == NULL) return CMD_UNKNOWN;
	const char *ptr = buf;

	if (strncasecmp(buf, "BNUM", 4) == 0) {
		cmd->type = CMD_BNUM;
		ptr += 4; // Jump past "BNUM"
		// Do more checks / evals here to pull in required information.
	} else if (strncasecmp(ptr, "SERI", 4) == 0) {
	 	cmd->type = CMD_SERI;
		ptr += 4;
	} else if (strncasecmp(ptr, "SNUM", 4) == 0) {
		cmd->type = CMD_SNUM;
		ptr += 4;
	} else if (strncasecmp(ptr, "ERRS", 4) == 0) {
		cmd->type = CMD_ERRS;
		ptr += 4;
	} else if (strncasecmp(ptr, "HELP", 4) == 0) {
		cmd->type = CMD_HELP;
		ptr += 4;
	} else if (strncasecmp(ptr, "?", 1) == 0) {
		cmd->type = CMD_INFO;
		ptr += 1;
	} else if (strncasecmp(ptr, "LOCK", 4) == 0) {
	  	cmd->type = CMD_LOCK;
		ptr += 4;
	} else if (strncasecmp(ptr, "ECHO", 4) == 0) {
	 	cmd->type = CMD_ECHO;
		ptr += 4;
	} else if (strncasecmp(ptr, "RESET", 5) == 0) {
		cmd->type = CMD_RESET;
		ptr += 5;
	} else if (strncasecmp(ptr, "VERS", 4) == 0) {
	 	cmd->type = CMD_VERS;
		ptr += 4;
	} else if (strncasecmp(ptr, "MODS", 4) == 0) {
	 	cmd->type = CMD_MODS;
		ptr += 4;
	} else if (strncasecmp(ptr, "CON", 3) == 0) {
	 	cmd->type = CMD_CON;
		ptr += 3;
	} else if (strncasecmp(ptr, "R", 1) == 0) {
	 	cmd->type = CMD_R;
		ptr += 4;
	} else if (strncasecmp(ptr, "INTV", 4) == 0) {
	  	cmd->type = CMD_INTV;
		ptr += 4;
	} else if (strncasecmp(ptr, "SEND", 4) == 0) {
	 	cmd->type = CMD_SEND;
		ptr += 4;
	} else if (strncasecmp(ptr, "ADDR", 4) == 0) {
	 	cmd->type = CMD_ADDR;
		ptr += 4;
	} else if (strncasecmp(ptr, "SMODE", 5) == 0) {
		cmd->type = CMD_SMODE;
		ptr += 5;
	} else if (strncasecmp(ptr, "SDELAY", 6) == 0) {
		cmd->type = CMD_SDELAY;
		ptr += 6;
	} else if (strncasecmp(ptr, "OPEN", 4) == 0) {
	 	cmd->type = CMD_OPEN;
		ptr += 4;
	} else if (strncasecmp(ptr, "CLOSE", 5) == 0) {
		cmd->type = CMD_CLOSE;
		ptr += 5;
	} else if (strncasecmp(ptr, "SCOM", 4) == 0) {
	 	cmd->type = CMD_SCOM;
		ptr += 4;
	} else if (strncasecmp(ptr, "TQFE", 4) == 0) {
	 	cmd->type = CMD_TQFE;
	    ptr += 4;
	} else if (strncasecmp(ptr, "DPMAX", 5) == 0) {
		cmd->type = CMD_DPMAX;
        ptr += 5;
	} else if (strncasecmp(ptr, "HHCP", 4) == 0) {
	 	cmd->type = CMD_HHCP;
	    ptr += 4;
	} else if (strncasecmp(ptr, "HQFE", 4) == 0) {
	 	cmd->type = CMD_HQFE;
        ptr += 4;
	} else if (strncasecmp(ptr, "HQNH", 4) == 0) {
	 	cmd->type = CMD_HQNH;
        ptr += 4;
	} else if (strncasecmp(ptr, "ICAOQNH", 7) == 0) {
		cmd->type = CMD_ICAOQNH;
        ptr += 7;
	} else if (strncasecmp(ptr, "PSTAB", 5) == 0) {
		cmd->type = CMD_PSTAB;
        ptr += 5;
	} else if (strncasecmp(ptr, "AVRG", 4) == 0) {
	 	cmd->type = CMD_AVRG;
        ptr += 4;
	} else if (strncasecmp(ptr, "FORM", 4) == 0) {
	 	cmd->type = CMD_FORM;
        ptr += 4;
	} else if (strncasecmp(ptr, "TIME", 4) == 0) {
	 	cmd->type = CMD_TIME;
        ptr += 4;
	} else if (strncasecmp(ptr, "DATE", 4) == 0) {
	 	cmd->type = CMD_DATE;
        ptr += 4;
	} else if (strncasecmp(ptr, "UNIT", 4) == 0) {
	 	cmd->type = CMD_UNIT;
        ptr += 4;
	} else if (strncasecmp(ptr, "DSEL", 4) == 0) {
	 	cmd->type = CMD_DSEL;
        ptr += 4;
	} else if (strncasecmp(ptr, "DELETE", 6) == 0) {
		cmd->type = CMD_DELETE;
        ptr += 6;
	} else if (strncasecmp(ptr, "UNDELETE", 8) == 0) {
		cmd->type = CMD_UNDELETE;
        ptr += 8;
	} else if (strncasecmp(ptr, "DIR", 3) == 0) {
	 	cmd->type = CMD_DIR;
        ptr += 3;
	} else if (strncasecmp(ptr, "PLAY", 4) == 0) {
	 	cmd->type = CMD_PLAY;
        ptr += 4;
	} else if (strncasecmp(ptr, "CDATE", 5) == 0) {
		cmd->type = CMD_CDATE;
        ptr += 5;
	} else if (strncasecmp(ptr, "LCP1", 4) == 0)  {
		cmd->type = CMD_LCP1;
	    ptr += 4;
	} else if (strncasecmp(ptr, "LCP2", 4) == 0)  {
		cmd->type = CMD_LCP2;
	    ptr += 4;
	} else if (strncasecmp(ptr, "LCP3", 4) == 0)  {
		cmd->type = CMD_LCP3;
	    ptr += 4;
	} else if (strncasecmp(ptr, "MPCP1", 5) == 0)  {
		cmd->type = CMD_MPCP1;
	    ptr += 5;
	} else if (strncasecmp(ptr, "MPCP2", 5) == 0)  {
		cmd->type = CMD_MPCP2;
	    ptr += 5;
	} else if (strncasecmp(ptr, "MPCP3", 5) == 0)  {
		cmd->type = CMD_MPCP3;
	    ptr += 5;
	} else if (strncasecmp(ptr, "CTEXT", 5) == 0)  {
		cmd->type = CMD_CTEXT;
	    ptr += 5;
	} else if (strncasecmp(ptr, "AMODE", 5) == 0)  {
		cmd->type = CMD_AMODE;
	    ptr += 5;
	} else if (strncasecmp(ptr, "ASEL", 4) == 0)  {
		cmd->type = CMD_ASEL;
	    ptr += 4;
	} else if (strncasecmp(ptr, "ACAL", 4) == 0)  {
		cmd->type = CMD_ACAL;
	    ptr += 4;
	} else if (strncasecmp(ptr, "AERR", 4) == 0)  {
		cmd->type = CMD_AERR;
	    ptr += 4;
	} else if (strncasecmp(ptr, "ATEST", 5) == 0)  {
		cmd->type = CMD_ATEST;
	    ptr += 5;
	} else if (strncasecmp(ptr, "RSEL", 4) == 0)  {
		cmd->type = CMD_RSEL;
	    ptr += 4;
	} else if (strncasecmp(ptr, "RTEST", 5) == 0)  {
		cmd->type = CMD_RTEST;
	    ptr += 5;
	} else return CMD_UNKNOWN;

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
        case CMD_BNUM:
			break;
        case CMD_SERI:
			break;
        case CMD_SNUM:
            break;
		case CMD_ERRS:
			break;
		case CMD_HELP:
			break;
		case CMD_LOCK:
			break;
		case CMD_INFO:
			break;
		case CMD_ECHO:
			break;
		case CMD_RESET:
			break;
		case CMD_VERS:
			break;
		case CMD_MODS:
			break;
		case CMD_CON:
			break;
		case CMD_R:
			break;
		case CMD_INTV:
			break;
		case CMD_SEND:
			break;
		case CMD_SMODE:
			break;
		case CMD_SDELAY:
			break;
		case CMD_ADDR:
			break;
		case CMD_OPEN:
			break;
		case CMD_CLOSE:
			break;
		case CMD_SCOM:
			break;
		case CMD_TQFE:
			break;
		case CMD_DPMAX:
			break;
		case CMD_HHCP:
			break;
		case CMD_HQFE:
			break;
		case CMD_HQNH:
			break;
		case CMD_PSTAB:
			break;
		case CMD_AVRG:
			break;
		case CMD_FORM:
			break;
		case CMD_TIME:
			break;
		case CMD_DATE:
			break;
		case CMD_UNIT:
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
			safe_console_error("Failed to open file: %s\n", strerror(errno));
			break;
		case CMD_INVALID_FORMAT:
			safe_console_error("Invalid Command Format: %s\n", strerror(errno));
			break;
		case CMD_UNKNOWN:
			safe_console_error("Unknown or Bad Command: %s\n", strerror(errno));
			break;
        default:
			safe_console_error("Unknown or Bad Command: %s\n", strerror(errno));
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

		// Determine if we should wait for a specific time or indefinitely
        if (sensor_one != NULL && sensor_one->mode == SMODE_RUN) {
            // Calculate absolute time: Last Send Time + Interval
            // Use current REALTIME + (Interval - Time Since Last Send)
            clock_gettime(CLOCK_MONOTONIC, &ts);
            // Add the continuous interval (in seconds) to the current time
            ts.tv_sec += sensor_one->interval;

            // 2. Wait until that specific second arrives OR a signal interrupts us
            pthread_cond_timedwait(&send_cond, &send_mutex, &ts);
        } else {
            // If in Polling Mode, wait indefinitely for a signal from the receiver
            pthread_cond_wait(&send_cond, &send_mutex);
        }

        if (terminate) {
            pthread_mutex_unlock(&send_mutex);
            break;
        }

        // is_ready_to_send() handles the interval and timing logic internally, and checks if the sensor is Pollling or Continuous.
        if (sensor_one != NULL && ptb330_is_ready_to_send(sensor_one)) {
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

	if (init_ptb330_sensor(&sensor_one) != 0) {
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
	return 0; // We won't get here, but it quiets verbose warnings on a no return value.
}
