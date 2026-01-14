/*
 * File:     flash.c
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
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <regex.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdarg.h>
#include <time.h>
#include <ctype.h>
#include "serial_utils.h"
#include "sensor_utils.h"
#include "console_utils.h"
#include "file_utils.h"

#define SERIAL_PORT "/dev/ttyUSB0"   // Adjust as needed, main has logic to take arguments for a new location
#define BAUD_RATE   B9600	     // Adjust as needed, main has logic to take arguments for a new baud rate
#define MAX_LINE_LENGTH 1024


FILE *file_ptr = NULL; // Global File pointer
char *file_path = NULL; // path to file

// Shared state
int sampling = 1; // Is the sensor in sampling mode or not, the default for the sensor will be yes i.e. 1.
volatile sig_atomic_t terminate = 0;
volatile sig_atomic_t kill_flag = 0;

int serial_fd = -1;

flash_sensor *fl_sensor;

/* Synchronization primitives */
static pthread_mutex_t write_mutex = PTHREAD_MUTEX_INITIALIZER; // protects serial writes
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


/*
 * Name:         check_sum
 * Purpose:      Takes a '\0' delimited string, and returns a checksum of the characters XOR.
 * Arguments:    str_to_chk the string that checksum will be calculated for
 *
 * Output:       None.
 * Modifies:     None.
 * Returns:      returns an unsigned 8 bit integer of the checksum of str_to_chk.
 * Assumptions:  Terminate is set to false.
 *
 * Bugs:         None known.
 * Notes:        To print in HEX utilize dprintf(serial_fd, "%c%s%c%02X\r\n",2, str_to_chk, check_sum(str_to_chk));
 */
uint8_t check_sum(const char *str_to_chk) {

    uint8_t checksum = 0;
    if (str_to_chk == NULL) {
        return 0;
    }
    while (*str_to_chk != '\0') {
        checksum ^= (uint8_t)(*str_to_chk);
        str_to_chk++;
    }
    return checksum;
}


/*
 * Name:         safe_write_response
 * Purpose:      Serializes writes to the serial device to ensure that all writes do not
 *               interleave and create errors.
 * Arguments:    fmt -  the string representing the format you want the the function to print.
 *               ... - a list of potential unfixed arguments, that can be supplied to the format string.
 *               i.e. if you supplied safe_write_response("%s%c%d", string_var, char_var, decimal_var); it would
 *               use vdprintf to print those variables in the format specified by fmt.
 *
 * Output:       Error message, if the file is not open.
 * Modifies:     None.
 * Returns:      None.
 * Assumptions:  The file is open, and the line read is less than MAX_LINE_LENGTH
 *
 * Bugs:         None known.
 * Notes:        va_start is a C macro that initializes a variable argument list, which
                 is a list of arguments passed to a function that can have a variable
   		 number of parameters. It must be called before any other variable argument
   		 macros, such as va_arg, and requires two arguments: the va_list variable and
   		 the name of the last fixed argument in the function's parameter list in our case fmt.
 */
static void safe_write_response(const char *fmt, ...) {
    va_list var_arg;  // Declare a va_list variable
    pthread_mutex_lock(&write_mutex);
    va_start(var_arg, fmt); // Initialize var_arg with the last fixed argument 'fmt'
    vdprintf(serial_fd, fmt, var_arg); // prints to serial device the variables provided, in the fmt provided.
    va_end(var_arg);
    pthread_mutex_unlock(&write_mutex);
}


// ---------------- Command handling ----------------

typedef enum {
    CMD_UNKNOWN,
    CMD_RUN, // "RUN" received from the terminal
    CMD_STOP,  // "STOP" received from the terminal
    CMD_SITE,  // "SITE?" recieved from the terminal
    CMD_SELF_TEST,  // "R?" received from the terminal, trasmit Self Test Message.
    CMD_DEF_DIST, // "DESTDEF" Reset the flash distance limits to FAA Defaults 5,10,20, 30.
    CMD_GET_DIST, // "DIST?" Get Distance Limits
    CMD_SET_DIST, // "DISTx,yyyy" Set Distance Limits x == 0-OH, 1-V, 2-ND, 3-FD. yyyy == decametres.
    CMD_GET_SER // "SN?" get sensor serial number.
} CommandType;


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
    if (strncmp(buf, "RUN", 3) == 0)							return CMD_RUN;
    if (strncmp(buf, "STOP", 4) == 0)							return CMD_STOP;
    if (strcmp(buf, "SN?") == 0)								return CMD_GET_SER;
    if (strncmp(buf, "SITE?", 5) == 0)							return CMD_SITE;
    if (strncmp(buf, "DIST", 4) == 0) {
        if (isdigit(buf[4]) && buf[4] >= '0' && buf[4] <= '3')	return CMD_SET_DIST;
    	if (buf[4] == '?' && buf[5] == '\0')					return CMD_GET_DIST;
        if (buf[4] == 'D' && buf[5] == 'E' && buf[6] == 'F')	return CMD_DEF_DIST;
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
void handle_command(CommandType cmd, const char *buf) {
     // char *resp_copy = NULL;
    switch (cmd) {
        case CMD_RUN:
			if (sampling == 1) {
                safe_write_response("%s\r\n", "COMMAND NOT ALLOWED");
			} else {
			    pthread_mutex_lock(&send_mutex);
                sampling = 1; // enable continuous sending
                pthread_cond_signal(&send_cond);  // Wake sender_thread immediately
                pthread_mutex_unlock(&send_mutex);
				safe_write_response("%s\r\n", "OK");
			}
            break;
        case CMD_STOP:
            if (sampling == 0) {
                safe_write_response("%s\r\n", "COMMAND NOT ALLOWED");
			} else {
				   pthread_mutex_lock(&send_mutex);
		    	   sampling = 0; // disables continuous sending.
        	       pthread_cond_signal(&send_cond);   // Wake sender_thread to exit loop
            	   pthread_mutex_unlock(&send_mutex);
				   safe_write_response("%s\r\n", "OK");
			}
            break;
        case CMD_SITE:
            break;
        case CMD_SET_DIST:
            if (sampling == 1) {
                safe_write_response("%s\r\n", "COMMAND NOT ALLOWED");
			} else {
				  set_dist(&fl_sensor, buf);
	              safe_write_response("%s\r\n", "OK");
			}
			break;
		case CMD_GET_DIST:
            safe_write_response("%s,%hu,%hu,%hu,%hu\r\n", "DIST:",
								fl_sensor->overhead,
							    fl_sensor->vicinity,
								fl_sensor->near_distant,
								fl_sensor->far_distant);
			break;
		case CMD_GET_SER:
            safe_write_response("%s\r\n", fl_sensor->serial_num);
			break;
		case CMD_DEF_DIST:
			if (sampling == 1) {
                safe_write_response("%s\r\n", "COMMAND NOT ALLOWED");
			} else {
				reset_flash(&fl_sensor);
                safe_write_response("%s\r\n", "OK");
			}
            break;
        default:
            safe_console_print("BAD CMD\r\n");
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
        if (n > 0)
		{
	    	if (c == '\r' || c == '\n')
			{
        		if (len > 0)
				{
            		line[len] = '\0';
		    		handle_command(parse_command(line), line);
                	len = 0;
            	}
				else
				{ // empty line ignore. Note: Not likely to happen, as we generate the input file.
            	}
        	}
			else
			{
                if (len < sizeof(line)-1)
				{
                    line[len++] = c;
                }
				else
                {
                    len = 0;
                 }
            }
        }
        else
        {
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
            {
                safe_console_error("Read Error: %s\n", strerror(errno));
        	}
        	else
        	{
            	// no data available n == 0, avoid busy loop
               	usleep(10000);
        	}
        }
        }
    return NULL;
}

/*
 * Name:         sender_thread
 * Purpose:      On sampling == 1 and assuming terminate != 1 it will get the next line from a specified file, using
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
    struct timespec requested_time;

    while (!terminate) {
        pthread_mutex_lock(&send_mutex);
        // wait until either terminate is set or "sampling" becomes 1, by default sampling is set to 1 on startup.
        while (!terminate && !sampling) {
            pthread_cond_wait(&send_cond, &send_mutex);
        }

        if (terminate) {
            pthread_mutex_unlock(&send_mutex);
            break;
        }

        while (!terminate && sampling) {
             char *line = get_next_line_copy(file_ptr, &file_mutex);
             if (line)
			 {
                char updated_line[MAX_LINE_LENGTH];
                if (update_btd_timestamps(line, updated_line, sizeof(updated_line)) == 0)
                {
                    safe_write_response("%s\r\n", updated_line);
                } else
                {
					safe_write_response("%s\r\n", line);
				}
                free(line); // caller of get_next_line_copy() must free resource.
				line = NULL;
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
        safe_console_error("Failed to open file: %s\n", strerror(errno));
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
        safe_console_error("Failed to create receiver thread: %s\n", strerror(errno));
		// perror("Failed to create receiver thread");
        terminate = 1;          // <- symmetrical, but not required
        close(serial_fd);
        fclose(file_ptr);
        return 1;
    }

    if (pthread_create(&send_thread, NULL, sender_thread, NULL) != 0) {
        safe_console_error("Failed to create sender thread: %s\n", strerror(errno));
		// perror("Failed to create sender thread");
        terminate = 1;          // <- needed because recv_thread is running
        pthread_join(recv_thread, NULL);
        close(serial_fd);
        fclose(file_ptr);
        return 1;
    }

    if (init_flash(&fl_sensor) != 1) {
        safe_console_error("Failed to initialize sensor: %s\n", strerror(errno));
		// perror("Failed to initialize sensor");
        terminate = 1;
        pthread_join(recv_thread, NULL);
        close(serial_fd);
        fclose(file_ptr);
        free(fl_sensor);
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

	pthread_mutex_destroy(&write_mutex);
	pthread_mutex_destroy(&file_mutex);
	pthread_mutex_destroy(&send_mutex);
	pthread_cond_destroy(&send_cond);

    close(serial_fd);
    fclose(file_ptr);
	free(fl_sensor);
    safe_console_print("Program terminated.\n");
	console_cleanup();
    return 0;
}
