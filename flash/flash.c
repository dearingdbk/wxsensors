/*
 * File:     flash.c
 * Author:   Bruce Dearing
 * Date:     19/11/2025
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
char *site_config = "A0 B3 C1 E1 F1 G0000 H2 J1 K1 L1 M2 NA O1 P1 T1 U1 V1 X1 Z1";
char site_id = 'A';

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
 * Name:         get_next_line_copy
 * Purpose:      Reads a line of a file, received from a file pointer, it then iterates that pointer to the next line, or if at the EOF
 *               loops back to the beginning of the file. This version is a thread safe version of get_next_line.
 * Arguments:    None.
 *
 * Output:       Error message, if the file is not open.
 * Modifies:     None.
 * Returns:      returns a heap-allocated copy of a line of text from a file MAX_LINE_LENGTH long or NULL.
 * Assumptions:  The file is open, and the line read is less than MAX_LINE_LENGTH
 *
 * Bugs:         None known.
 * Notes:        The caller of this function, must free(), the heap-allocated copy when done processing it.
 *               i.e. if using char *line = get_next_line_copy(); in the same function, you must use 'free(line);'
 */
char *get_next_line_copy(void) {
    char temp[MAX_LINE_LENGTH];

    pthread_mutex_lock(&file_mutex);
    if (!file_ptr) {
        pthread_mutex_unlock(&file_mutex);
        return NULL;
    }

    if (!fgets(temp, sizeof(temp), file_ptr)) {
        // EOF or error; rewind if we can.
        if (fseek(file_ptr, 0 , SEEK_SET) == 0) // effectively rewinds the file ptr with a check.
		{
        	// rewind(file_ptr);
        	if (!fgets(temp, sizeof(temp), file_ptr)) {
            	pthread_mutex_unlock(&file_mutex);
            	return NULL; /* file empty or error */
        	}
		} else // This is a PIPE (socat) - It has run dry, socat should be set to try forever, so this will infinetly loop.
		  {
			  // clear the EOF status to try again, note it is going to try until we kill the program.
			  clearerr(file_ptr);
			  pthread_mutex_unlock(&file_mutex);

              // Log the error so you know why the sensor stopped
			  fprintf(stderr, "[%ld] Network stream closed. Waiting for data...\n", time(NULL));
              return NULL;
		 }
    }
    pthread_mutex_unlock(&file_mutex);

    /* Trim CR/LF safely */
    temp[strcspn(temp, "\r\n")] = '\0';
    return strdup(temp); /* caller must free, returns a copy of temp */
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
    CMD_START, // ! received from the terminal
    CMD_STOP,  // ? received from the terminal
    CMD_SITE,  // & recieved from the terminal
    CMD_POLL,  // <A-Z> received from the terminal
    CMD_SELF_TEST,  // "R?" received from the terminal, trasmit Self Test Message.
    CMD_DEF_DIST, // "DESTDEF" Reset the flash distance limits to FAA Defaults 5,10,20, 30.
    CMD_GET_DIST, // "DIST?" Get Distance Limits
    CMD_SET_DIST, // "DISTx,yyyy" Set Distance Limits x == 0-OH, 1-V, 2-ND, 3-FD. yyyy == decametres.
    CMD_CONFIG // *<A-Z> received from the terminal
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
    if (buf[0] == '!' && buf[1] == '\0')									return CMD_START;
    if (buf[0] == '?' && buf[1] == '\0')									return CMD_STOP;
    if (buf[0] == '&' && buf[1] == '\0')									return CMD_SITE;
    if (buf[0] == '*' && buf[1] >= 'A' && buf[1] <= 'Z' && buf[2] =='\0') 	return CMD_CONFIG;
    if (buf && buf[0] >= 'A' && buf[0] <= 'Z' && buf[1] == '\0') 			return CMD_POLL;
    if (strncmp(buf, "DIST", 4) == 0) {
        if (isdigit(buf[4]) && buf[4] >= '0' && buf[4] <= '3')				return CMD_SET_DIST;
	if (buf[4] == '?' && buf[5] == '\0')									return CMD_GET_DIST;
        if (buf[4] == 'D' && buf[5] == 'E' && buf[6] == 'F')				return CMD_DEF_DIST;
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
    char *resp_copy = NULL;
    switch (cmd) {
        case CMD_START:
	    pthread_mutex_lock(&send_mutex);
            sampling = 1; // enable continuous sending
            pthread_cond_signal(&send_cond);  // Wake sender_thread immediately
            pthread_mutex_unlock(&send_mutex);
            break;

        case CMD_STOP:
            pthread_mutex_lock(&send_mutex);
	    	sampling = 0; // disables continuous sending.
            pthread_cond_signal(&send_cond);   // Wake sender_thread to exit loop
            pthread_mutex_unlock(&send_mutex);
            break;

        case CMD_SITE:
            printf("CMD: SITE -> Sending site info\n");
            safe_write_response("%c\r\n", site_id);
            break;

        case CMD_POLL:
            resp_copy = get_next_line_copy();
            if (resp_copy)
            {
			    char updated_line[MAX_LINE_LENGTH];
                if (update_btd_timestamps(resp_copy, updated_line, sizeof(updated_line)) == 0)
                {
                    safe_write_response("%s\r\n", updated_line);
                } else
                {
					safe_write_response("%s\r\n", resp_copy);
				}
				free(resp_copy);
            } else {
                safe_write_response("ERR: Empty file\r\n");
            }
            break;
        case CMD_SET_DIST:
            resp_copy = get_next_line_copy();
            if (resp_copy) {
                // prints <Start of Line ASCII 2>, the string of data read, <EOL ASCII 3>, Checksum of the line read
                safe_write_response("%c%s%c%02X\r\n", 2, resp_copy, 3, check_sum(resp_copy));
                free(resp_copy);
            } else {
                safe_write_response("ERR: Empty file\r\n");
            }
            break;
        default:
            printf("CMD: Unknown command\n");
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
		    		handle_command(parse_command(line));
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
                perror("read");
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
 * Purpose:      On sampling == 1 and assuming terminate != 1 it will get the next line from a specified file, usinf
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
        // wait until either terminate is set or "sampling" becomes 1
        while (!terminate && !sampling) {
            pthread_cond_wait(&send_cond, &send_mutex);
        }

        if (terminate) {
            pthread_mutex_unlock(&send_mutex);
            break;
        }

        while (!terminate && sampling) {
             char *line = get_next_line_copy();
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
        fprintf(stderr, "Usage: %s <file_path> <serial_device> <baud_rate> <RS422|RS485>\n", argv[0]);
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

    printf("Press 'q' + Enter to quit.\n");
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
    close(serial_fd);
    fclose(file_ptr);

    printf("Program terminated.\n");
    return 0;
}
