/*
 * File:     tmp_rh_listen.c
 * Author:   Bruce Dearing
 * Date:     19/11/2025
 * Version:  1.0
 * Purpose:  Emulates a Rotronic HC2A-S3 Temperature/Relative Humidity probe over RS-485/RS-422.
 *           The probe is housed in an Apogee TS-100 aspirated radiation shield.
 *           This program sets up a serial connection with one thread:
 *            - Receiver thread: parses and responds to incoming commands
 *
 *           Supported commands (per Rotronic HygroClip2 protocol):
 *		     Command Format { ID Adr RDD <Checksum || }> CR
 *		     Answer Format { ID Adr RDD <Checksum || }> CR
 *           Command: {F00RDD}
 * 	    	 Response: {F00rdd 001; 4.45;%RH;000;=;20.07;°C;000;=;nc;---.-;°C;000; ;001;V1.7-1;0060568338;HC2-S3 ;000;4
 *           All replies (ACKs, responses, errors) are sent out on the serial port.
 *
 *           Data output includes: relative humidity (%), temperature (°C), status, and checksum
 *
 * Usage:    use case ' tmp_rh_listen <file_path> // The serial port and baud rate will be set to  defaults /dev/ttyUSB0, and B9600
 *           use case ' tmp_rh_listen <file_path> <serial_port_location> <baud_rate> <RS422|RS485> The serial port currently must match /dev/tty(S|USB)[0-9]+
 *
 * Sensor:   Rotronic HC2A-S3 HygroClip2 Probe
 *           - Digital temperature and relative humidity probe
 *           - Temperature range: -50°C to +100°C (accuracy ±0.1°C)
 *           - Humidity range: 0-100% RH (accuracy ±0.8% RH)
 *           - Output: digital RS-485
 *           - Default baud rate: 19200
 *           - Default protocol: RS-485 half-duplex
 *
 * Housing:  Apogee TS-100 Aspirated Radiation Shield
 *           - Fan-aspirated design for accurate ambient readings
 *           - Minimizes solar radiation effects on temperature measurement
 *           - 12 VDC fan operation
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
#include "serial_utils.h"
#include "console_utils.h"
#include "file_utils.h"
#include "crc_utils.h"
#include "hc2a_utils.h"

#define SERIAL_PORT "/dev/ttyUSB0"   // Adjust as needed, main has logic to take arguments for a new location
#define BAUD_RATE   B19200	     // Adjust as needed, main has logic to take arguments for a new baud rate
#define MAX_LINE_LENGTH 1024
#define MAX_CMD_LENGTH 256
#define MAX_MSG_LENGTH 512

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
//volatile sig_atomic_t kill_flag = 0;
// volatile bool running = false;
int serial_fd = -1;
const char *program_name = "unknown";
// This needs to be freed upon exit.
HC2A_sensor *sensor_one = NULL; // Global pointer to struct for HC2A sensor.

/* Synchronization primitives */
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
    pthread_mutex_lock(&sensor_mutex);
    terminate = 1;
    if (sensor_cond_init) pthread_cond_broadcast(&sensor_cond);
    pthread_mutex_unlock(&sensor_mutex);

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

    pthread_mutex_destroy(&sensor_mutex);
    pthread_mutex_destroy(&file_mutex);
    if (sensor_cond_init) pthread_cond_destroy(&sensor_cond);

    if (sensor_one) free(sensor_one);
    // Close resources
    if (serial_fd >= 0) {
        tcflush(serial_fd, TCOFLUSH);
        close(serial_fd);
    }
    if (file_ptr) fclose(file_ptr);
    // Cleanup utilities
    console_cleanup();
    serial_utils_cleanup();
    exit(exit_code);
}

/*
 * Name:         parse_message
 * Purpose:      Tokenizes a space-delimited sensor string and populates a ParsedMessage struct.
 * Arguments:    msg: the raw input string to be parsed (modified by strtok_r).
 *               p_message: pointer to the struct where parsed data will be stored.
 *
 * Output:       None (internal debug prints to console only).
 * Modifies:     p_message: overwrites with new data.
 *               msg: the input string is modified (nulls inserted by strtok_r).
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

    // These are pulled from a text file in this format:
    // 4.45,0,20.07,0,0
    // RH, RH Alarm, Temp, Temp Alarm, Alarm Byte
    if ((token = strtok_r(msg, ",", &saveptr))) p_message->rel_humidity = (float)strtof(token, NULL);  // Set Relative Humidity.
    #define NEXT_T strtok_r(NULL, ",", &saveptr) // Small macro to keep the code below cleaner.
    if ((token = NEXT_T)) p_message->rh_alarm = (uint8_t)atoi(token);                         // Set RH Alarm Status.
    if ((token = NEXT_T)) p_message->temperature = (float)strtof(token, NULL);             // Set Temperature.
    if ((token = NEXT_T)) p_message->temp_alarm = (uint8_t)atoi(token);                       // Set Teperature Alarm.
    if ((token = NEXT_T)) p_message->alarm_byte = (uint8_t)atoi(token);                       // Set Alarm Byte.
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
    
    if (msg == NULL) return;
    char msg_buffer[MAX_MSG_LENGTH]; // 512
    HC2A_sensor local_sensor;
    pthread_mutex_lock(&sensor_mutex);
    char rh_char   = trend_tracker_update(&sensor_one->rh_trend, msg->rel_humidity);
    char temp_char = trend_tracker_update(&sensor_one->temp_trend, msg->temperature);
    local_sensor = *sensor_one;
    pthread_mutex_unlock(&sensor_mutex);

    snprintf(msg_buffer, sizeof(msg_buffer), "{%c%02urdd %03u; %.2f;%%rh;%03u;%c; %.2f;°C;%03u;%c;nc;---.-;°C;000; ;%03u;%s;%s;%s ;%03u;",
                                                            (char)local_sensor.unit_ident,  // Unit Identifier 'F'
                                                            (unsigned int)local_sensor.address,
                                                            (unsigned int)local_sensor.probe_type,
                                                            (float)msg->rel_humidity,       // RH
                                                            (unsigned int)msg->rh_alarm,
                                                            (char)rh_char,
                                                            (float)msg->temperature,
                                                            (unsigned int)msg->temp_alarm,
                                                            (char)temp_char,
                                                            (unsigned int)local_sensor.device_type,
                                                            local_sensor.firmware_version,
                                                            local_sensor.serial_number,
                                                            local_sensor.device_name,
                                                            msg->alarm_byte);
    char calc_checksum = checksum(msg_buffer);
    safe_serial_write(serial_fd, "%s%c\r\n", msg_buffer, calc_checksum);
    DEBUG_PRINT("%s%c\r\n", msg_buffer, calc_checksum);
    
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

CommandType parse_command(const char* buf, ParsedCommand *cmd) {
    memset(cmd, 0, sizeof(ParsedCommand));
    cmd->type = CMD_UNKNOWN;
    if (buf == NULL) return CMD_UNKNOWN;
    char local_buf[MAX_CMD_LENGTH]; 
    strncpy(local_buf, buf, MAX_CMD_LENGTH - 1);
    local_buf[MAX_CMD_LENGTH - 1] = '\0'; 
    
    char *p = local_buf;
    char* residual_ptr = NULL;
    char *saveptr = NULL; // Our place keeper in the msg string.
    char *token = NULL; // Where we temporarily store each token.
    
    // Commands come in this format |{F00RDD}<CR>
    if (*p == '|') p++; // Skip past the first vertical bar, this is used with multiple sensors in slave mode.
    if (*p == '{') {
        p++; // Skip past the curly bracket to start looking at the address. 
        
        if (*p == ' ') {
            p++; // Here the space represents an unknown identifier, all sensors will broadcast. 
        } else if (isalpha((unsigned char)*p)) {
            cmd->cmd_unit_ident = toupper(*p++);
        }
        
        long val = strtol(p, &residual_ptr, 10);
        if (p == residual_ptr) {
            return CMD_UNKNOWN; 
        } else {
           cmd->sensor_id = (uint8_t)val; 
        }
        if ((token = strtok_r(residual_ptr, "}\r", &saveptr))) {
                
            for (size_t i = 0; i < CMD_TABLE_SIZE; i++) {
                if (strncasecmp(token, cmd_table[i].name, cmd_table[i].len) == 0) {
                    cmd->type = cmd_table[i].type;
                    return cmd->type;
                }
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
(void) p_cmd;
    switch (cmd) {
        case CMD_RDD:
            char *line = get_next_line_copy(file_ptr, &file_mutex);
            if (line) {
                ParsedMessage local_msg;  // LOCAL, not global
                parse_message(line, &local_msg);
                process_and_send(&local_msg);
                fflush(NULL);  // Flush all output streams
                free(line);
                line = NULL;
            } else {
                safe_console_error("ERR: Empty file\r\n");
            }
            break;
        case CMD_REN:
            //  TODO:
            break;
        case CMD_HCA:
            // TODO:
            break;
        case CMD_LGC:
            // TODO:
            break;
        case CMD_ERD:
            // TODO:
            break;
        case CMD_TID:
            // TODO:
            break;
        case CMD_HRD:
            // TODO:
            break;
        default:
            safe_console_print("CMD: Unknown command\n");
            break;
    }
}


// ---------------- Threads ----------------

// Dedicated signal handling thread — replaces handle_signal
/*
 * Name:         handle_signal
 * Purpose:      Captures any kill signals, and sets volitile atomic 'terminate' & 'kill_flag' to 1, allowing the while loop to break, and threads to join.
 * Arguments:    None
 *
 * Output:       None.
 * Modifies:     Changes terminate to 1, and kill_flag to 1.
 * Returns:      None.
 * Assumptions:  Terminate is set to 0.
 *
 * Bugs:         None known.
 * Notes:
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
                    line[len] = '\0'; // Terminate with NULL for safety.
                    ParsedCommand local_cmd;
                    CommandType cmd_type = parse_command(line, &local_cmd);
                    handle_command(cmd_type, &local_cmd); // handle received command here.
                    len = 0;
                }
            } else if (len < sizeof(line)-1) {
                line[len++] = c;
            } else len = 0;
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("read");
        } else {
            // no data available, avoid busy loop
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
    struct timespec ts;
    bool should_send = false;
    int interval = 0;

    while (!terminate) {
        pthread_mutex_lock(&sensor_mutex);

        // Determine if we should wait for a specific time or indefinitely
        if (sensor_one != NULL && sensor_one->mode == SMODE_M1) {
            //interval = sensor_one->message_interval; // If we are in polled mode, message_interval is zero.
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
        should_send = (sensor_one != NULL && HC2A_is_ready_to_send(sensor_one));

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
 * Name:        Main
 * Purpose:     Main funstion, which opens up serial port, and creates a receiver and transmit threads to listen, and respond to commands 
 *              over that serial port. Can take two arguments or no arguments. If changing the serial device name and baud rate, you must supply both.
 *              i.e. tmp_bp_listen <serial_device> <baud_rate>
 *		        uses ternary statements to set either default values for SERIAL_PORT, and BAUD_RATE which are defined above.
 *              (condition) ? (value if true) : (value if false)
 *
 * Arguments:   file_path: The location of the file we want to read from, line by line.
 *              device: the string representing the file descriptor of the serial port which should
 * 		        match the pattern ^/dev/tty(S|USB)[0-9]+$. This is tested with function is_valid_tty()
 *		        baud: the string value representing the proposed baud rate, this string is sent to get_baud_rate() which returns a speed_t value.
 *
 * Output:      Prints to stderr the appropriate error messages if encountered.
 * Modifies:    None.
 * Returns:     Returns an int 0 representing success once the program closes the fd, and joins the threads, or 1 if unable to open the serial port.
 * Assumptions: device is a valid char * pointer and the line contains
 *              characters other than white space, and points to an FD.
 *		        The int provided by arguments is a valid baud rate, although B9600 is set on any errors.
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
    file_path = argv[1]; // gets the supplied file path

    file_ptr = fopen(file_path, "r");
    if (!file_ptr) {
        safe_console_error("Failed to open file: %s\n", strerror(errno));
        cleanup_and_exit(1);
    }
    //ternary statement to set SERIAL_PORT if supplied in args or the default
    const char *device = (argc >= 3 && is_valid_tty(argv[2]) == 0) ? argv[2] : SERIAL_PORT;

    // ternary statement to set BAUD_RATE if supplied in args or default
    speed_t baud = (argc >= 4) ? get_baud_rate(argv[3]) : BAUD_RATE;

    // ternary statement to set Serial Protocol if supplied in args or default
    SerialMode mode = (argc >= 5) ? get_mode(argv[4]) : SERIAL_RS485; // returns RS485 by default.

    serial_fd = open_serial_port(device, baud, mode);

    if (serial_fd < 0) {
		cleanup_and_exit(1);
    }

    if (init_HC2A_sensor(&sensor_one) != 0) {
        safe_console_error("Failed to initialize sensor_one\n");
        cleanup_and_exit(1);
    }

	// Block signals in main (inherited by all threads)
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
    pthread_join(sig_thread, NULL); // Wait until signal thread joins.
    sig_thread_created = false;
    safe_console_print("\rProgram %s terminated.\n", program_name);
    cleanup_and_exit(0);
    return 0; // We won't get here, but it quiets verbose warnings on a no return value.
}
