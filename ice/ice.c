/*
 * File:     ice.c
 * Author:   Bruce Dearing
 * Date:     19/11/2025
 * Version:  1.0
 * Purpose:  Emulates a Goodrich 0872F1 Ice Detector over RS-232.
 *           This program sets up a serial connection with one thread:
 *            - Receiver thread: parses and responds to incoming commands
 *
 *           Supported commands (per Goodrich 0872 protocol):
 *             Z1       - Send routine data (probe frequency)
 *             Z3XX     - Activate de-ice heaters for XX seconds (01-60)
 *             Z4       - Perform extended diagnostics
 *             F5       - Field calibration (recalibrate probe frequency to 40,000 Hz)
 *
 *           Output format for Z1 command:
 *             ZPXXXXXCC    - Normal operation (P = Pass)
 *             ZDXXXXXCC    - De-icing cycle active (D = De-ice)
 *             ZF1XXXXXCC   - Probe failure
 *             ZF2XXXXXCC   - Heater failure
 *             ZF3XXXXXCC   - Electronics failure
 *           Where:
 *             XXXXX = Probe frequency in Hz (averaged over one minute)
 *             CC    = Two-character checksum
 *
 *           Output format for Z3 command:
 *             ZDOK51       - Confirmation of heater activation
 *
 *           Output format for Z4 command:
 *             ZP E3        - Sensor passes extended diagnostics
 *             ZD D7        - Sensor in de-ice mode
 *             ZF1 EA       - Probe failure
 *             ZF2 EB       - Heater failure
 *             ZF3 EC       - Electronics failure
 *
 *           Probe frequency indicates ice accretion:
 *             - Normal range: 38,400 - 41,500 Hz
 *             - Calibrated nominal: 40,000 Hz
 *             - Frequency decreases as ice mass accumulates on probe
 *             - Ice thickness (mm) ≈ -0.00015 × frequency + 6
 *
 * Usage:    ice_listen <data_file> [serial_port] [baud_rate] [RS422|RS485]
 *           ice_listen /path/to/ice_data.txt                          (uses defaults: /dev/ttyUSB0, 2400, RS232)
 *           ice_listen /path/to/ice_data.txt /dev/ttyUSB1 2400 RS232
 *
 *           Serial port must match pattern: /dev/tty(S|USB)[0-9]+
 *
 * Sensor:   Goodrich 0872F1 Ice Detector (formerly Rosemount)
 *           - Ultrasonic axially vibrating probe ice detector
 *           - Technology: Nickel alloy tube with 40 kHz natural resonant frequency
 *           - Ice detection sensitivity: 0.13 mm (0.005 inches) minimum
 *           - Probe frequency range: 38,400 - 41,500 Hz
 *           - Measures precipitation transitions between liquid and solid states
 *           - Differentiates rain from freezing rain as temperatures approach freezing
 *           - Self de-icing/water shedding capability (heater up to 60 seconds)
 *           - Continuous built-in test (BIT) verifies sensor functions
 *           - Output: RS-232 or digital current loop
 *           - Default baud rate: 2400
 *           - Data format: 8N1 (8 data bits, no parity, 1 stop bit)
 *           - Full duplex, asynchronous serial
 *           - Operating temperature: Designed for harsh outdoor environments
 *           - Power consumption: 10W monitoring, 385W during de-ice cycle
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
#define BAUD_RATE   B2400	     // Adjust as needed, main has logic to take arguments for a new baud rate
#define MAX_LINE_LENGTH 1024
#define MAX_PACKET_LENGTH 25

FILE *file_ptr = NULL; // Global File pointer
char *file_path = NULL; // path to file

// Shared state
int sampling = 0; // Is the sensor in sampling mode or not, the default for the sensor will be yes i.e. 1.
volatile sig_atomic_t terminate = 0;
volatile sig_atomic_t kill_flag = 0;

int serial_fd = -1;

/* Synchronization primitives */
static pthread_mutex_t write_mutex = PTHREAD_MUTEX_INITIALIZER; // protects serial writes
static pthread_mutex_t file_mutex  = PTHREAD_MUTEX_INITIALIZER; // protects file_ptr / file access


/*
 * Name:         handle_signal
 * Purpose:      Captures any kill signals, and sets volitile bool 'terminate' and 'kill_flag' to true,
 *				 allowing the while loop to break, and threads to join.
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
 * Name:         generate_check_sum
 * Purpose:      Takes a  string, and returns a checksum of the characters XOR.
 * Arguments:    str_to_chk the string that checksum will be calculated for, it is set to uint_8 to eliminate any sign errors.
 *				 length the length of the string to check.
 * Output:       None.
 * Modifies:     None.
 * Returns:      returns an unsigned 8 bit integer of the checksum of str_to_chk.
 * Assumptions:  Terminate is set to false.
 *
 * Bugs:         None known.
 * Notes:        To print in HEX utilize dprintf(serial_fd, "%s%02X\r\n", str_to_chk, check_sum(str_to_chk));
 */
uint8_t generate_check_sum(const uint8_t *str_to_chk, size_t length) {

    uint8_t checksum = 0;
    if (str_to_chk == NULL || length == 0 || length > MAX_PACKET_LENGTH) {
        return -1;
    }

	for (size_t i = 0; i < length; i++) {
		checksum += str_to_chk[i];
	}
    return (uint8_t)(checksum & 0xFF);
}

/*
 * Name:         prepend_to_buffer
 * Purpose:      Takes a destination and source string, and appends "STX \r \n" to the beginning.
 * Arguments:    str_to_chk the string that checksum will be calculated for, it is set to uint_8 to eliminate any sign errors.
 *				 length the length of the string to check.
 * Output:       None.
 * Modifies:     None.
 * Returns:      returns an unsigned 8 bit integer of the checksum of str_to_chk.
 * Assumptions:  Terminate is set to false.
 *
 * Bugs:         None known.
 * Notes:        To print in HEX utilize dprintf(serial_fd, "%s%02X\r\n", str_to_chk, check_sum(str_to_chk));
 */
char* prepend_to_buffer(const char* original) {
    // Length: 3 (for STX, \r, \n) + length of data + 1 (for null terminator)
    size_t new_len = 3 + strlen(original) + 1;
    char* new_str = malloc(new_len);
    if (new_str == NULL) return NULL;

    // \x02 is STX, \r is Carriage Return, \n is Newline
    // %s is your original data
    snprintf(new_str, new_len, "\x02\r\n%s", original);

    return new_str;
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
    CMD_Z1,	// "Z1" received from the terminal - Send Frequency Data.
    CMD_Z3, // "Z3" received from the terminal - De-ice strut and probe.
    CMD_Z4, // "Z4" recieved from the terminal - Perform extended diagnostics.
    CMD_F4  // "F4" received from the terminal - Perform Field Calibration.
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
    if (buf[0] == 'Z' && buf[1] == '1' && buf[2] == '\0')						return CMD_Z1;
    if (buf[0] == 'Z' && buf[1] == '3' && isdigit(buf[2]) && isdigit(buf[3]))	return CMD_Z3;
    if (buf[0] == 'Z' && buf[1] == '4' && buf[2] == '\0')						return CMD_Z4;
    if (buf[0] == 'F' && buf[1] == '4' && buf[2] == '\0')						return CMD_F4;
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
        case CMD_Z1: {
            resp_copy = get_next_line_copy(file_ptr, &file_mutex);
            if (resp_copy) {
				char *msg = prepend_to_buffer(resp_copy);
				uint8_t crc = generate_check_sum((const uint8_t *)msg, strlen(msg));
		    	safe_write_response("%s%02X\x03\r\n", msg, crc);
				free(msg);
				msg = NULL;
			} else {
				// safe_write_response("%s\r\n", "OK");
			}
			free(resp_copy);
            break;
			}
        case CMD_Z3: {
			char *msg = prepend_to_buffer("ZDOK51");
			uint8_t crc = generate_check_sum((const uint8_t *)msg, strlen(msg));
		    safe_write_response("%s%02X\x03\r\n", msg, crc); // Hardcoded response to Z3.
			free(msg);
			msg = NULL;
			// safe_write_response("%s\r\n", "ZDOK51"); // Hardcoded response to turning on the heater.
            break;
			}
        case CMD_Z4: {
			char* msg = prepend_to_buffer("ZP E3");
			uint8_t crc = generate_check_sum((const uint8_t *)msg, strlen(msg));
		    safe_write_response("%s%02X\x03\r\n", msg, crc); // Hardcoded response to Z4.
			free(msg);
			msg = NULL;
		    // safe_write_response("%s\r\n", "ZP E3"); // Hardcoded response to Z4.
			break;
			}
        case CMD_F4:
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
    char line[5]; // Buffer for 1 letter + 3 digits + null terminator
    size_t len = 0;

    while (!terminate) {
        char c;
        // n=0 means VTIME (0.1s) reached. n=1 means a byte arrived.
        int n = read(serial_fd, &c, 1);

        if (n > 1) { // n will be 1 for a single byte
            // Start of new command, detected by a letter (Z, F, etc.)
            if (isalpha(c)) {
                // If we have an existing command, process it before starting the new one
                if (len >= 2) {
                    line[len] = '\0';
                    handle_command(parse_command(line));
                }
                line[0] = c;
                len = 1;
            }
            // Collect up to 3 digits, i.e. Z360
            else if (isdigit(c) && len > 0) {
                line[len++] = c;

                // trigger if we hit the absolute maximum length (e.g., Z3XX)
                if (len == 4) {
                    line[len] = '\0';
                    handle_command(parse_command(line));
                    len = 0;
                }
            }
            // Ignore anything else (\r, \n, spaces, nulls)
        }
        else if (n == 0) {
            // Timeout Trigger: The line went silent (Handles Z1, Z4, F5)
            if (len >= 2) {
                line[len] = '\0';
                handle_command(parse_command(line));
                len = 0;
            }
        }
		else
		{
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("Serial Read Error");
            }
			else {
            	// avoid busy loop
               	usleep(1000);
			}
        }
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

    pthread_t recv_thread;

    if (pthread_create(&recv_thread, NULL, receiver_thread, NULL) != 0) {
        perror("Failed to create receiver thread");
        terminate = 1;          // <- symmetrical, but not required
        close(serial_fd);
        fclose(file_ptr);
        return 1;
    }

    safe_console_print("Press 'q' + Enter to quit.\n");
    while (!kill_flag) {
        char input[8];
        if (fgets(input, sizeof(input), stdin)) {
            if (input[0] == 'q' || input[0] == 'Q' || kill_flag == 1) {
                terminate = 1;
                kill_flag = 1;
		break;
            }
        } else if (feof(stdin)) {  // keep an eye on the behaviour of this check.
            terminate = 1;
            kill_flag = 1;
            break; // stdin closed
        } else {
            continue; // temp read error
        }
    }

    pthread_join(recv_thread, NULL);

	pthread_mutex_destroy(&write_mutex);
	pthread_mutex_destroy(&file_mutex);

	close(serial_fd);
    fclose(file_ptr);
    safe_console_print("Program terminated.\n");
	console_cleanup();
    return 0;
}
