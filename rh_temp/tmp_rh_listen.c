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
 *	     Command Format { ID Adr RDD <Checksum || }> CR
 *	     Answer Format { ID Adr RDD <Checksum || }> CR
 *           Command: {F00RDD}
 * 	     Response: {F00rdd 001; 4.45;%RH;000;=;20.07;°C;000;=;nc;---.-;°C;000; ;001;V1.7-1;0060568338;HC2-S3 ;000;4
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
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <regex.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdarg.h>
#include <time.h>
#include "serial_utils.h"
#include "console_utils.h"

#define SERIAL_PORT "/dev/ttyUSB0"   // Adjust as needed, main has logic to take arguments for a new location
#define BAUD_RATE   B19200	     // Adjust as needed, main has logic to take arguments for a new baud rate
#define MAX_LINE_LENGTH 1024


FILE *file_ptr = NULL; // Global File pointer
char *file_path = NULL; // path to file
// Shared state
volatile sig_atomic_t terminate = 0;
volatile sig_atomic_t kill_flag = 0;
// volatile bool running = false;
int serial_fd = -1;

/* Synchronization primitives */
static pthread_mutex_t write_mutex = PTHREAD_MUTEX_INITIALIZER; // protects serial writes
static pthread_mutex_t file_mutex  = PTHREAD_MUTEX_INITIALIZER; // protects file_ptr / file access


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
void handle_signal(int sig) {
    (void)sig;
    terminate = 1;
    kill_flag = 1;
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
        /* EOF or error; rewind and try once */
        rewind(file_ptr);
        if (!fgets(temp, sizeof(temp), file_ptr)) {
            pthread_mutex_unlock(&file_mutex);
            return NULL; /* file empty or error */
        }
    }
    pthread_mutex_unlock(&file_mutex);

    /* Trim CR/LF safely */
    temp[strcspn(temp, "\r\n")] = '\0';
    return strdup(temp); /* caller must free, returns a copy of temp */
}


/*
 * Name:         safe_write_response
 * Purpose:      Serializes writes to the serial device to ensure, that all writes do not
 *               interleave, and create errors.
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
    CMD_RDD,
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

CommandType parse_command(const char* buf) {
    if (strcmp(buf, "{F00RDD}") == 0)   	return CMD_RDD; // Expected poll request from AWI and CS systems.
    if (strncmp(buf, "{F00RDD", 7) == 0)    return CMD_RDD; // Optional command request with a checksum instead of '{' as the end.
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
        case CMD_RDD:
            resp_copy = get_next_line_copy();
            if (resp_copy) {
                safe_write_response("%s\r\n", resp_copy);
                free(resp_copy);
            } else {
                safe_write_response("ERR: Empty file\r\n");
            }
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
 * Name:         Main
 * Purpose:      Main funstion, which opens up serial port, and creates a receiver and transmit threads to listen, and respond to commands 
 *               over that serial port. Can take two arguments or no arguments. If changing the serial device name and baud rate, you must supply both.
 *               i.e. tmp_bp_listen <serial_device> <baud_rate>
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

    file_path = argv[1]; // gets the supplied file path

    file_ptr = fopen(file_path, "r");
    if (!file_ptr) {
        safe_console_error("Failed to open file: %s\n", strerror(errno));
        return 1;
    }
    //ternary statement to set SERIAL_PORT if supplied in args or the default
    const char *device = (argc >= 3 && is_valid_tty(argv[2]) == 0) ? argv[2] : SERIAL_PORT;

    // ternary statement to set BAUD_RATE if supplied in args or default
    speed_t baud = (argc >= 4) ? get_baud_rate(argv[3]) : BAUD_RATE;

    // ternary statement to set Serial Protocol if supplied in args or default
    SerialMode mode = (argc >= 5) ? get_mode(argv[4]) : SERIAL_RS485; // returns RS485 by default.

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

    pthread_t recv_thread;

    if (pthread_create(&recv_thread, NULL, receiver_thread, NULL) != 0) {
        perror("Failed to create receiver thread");
        terminate = 1;          // <- symmetrical, but not required
        close(serial_fd);
        fclose(file_ptr);
        return 1;
    }

    safe_console_print("Press 'q' + Enter to quit.\n");
    while (!kill_flag) { // Keep looping until the global kill_flag is set (user wants to quit or signal received)
        char input[8];   // Buffer to store user input (up to 7 chars + null terminator)
        // try to read a line from standard input (stdin)
        if (fgets(input, sizeof(input), stdin)) {
            if (input[0] == 'q' || input[0] == 'Q' || kill_flag == 1) {
                terminate = 1;
		break;
            }
        } else { // fgets returned NULL: could be EOF or read error
            if (feof(stdin)) { terminate = 1; kill_flag = 1; break; } // Check if end-of-file (EOF) was reached
            if (ferror(stdin)) { clearerr(stdin); continue; }  // Check if a read error occurred, re-loop
        }
    }

    pthread_join(recv_thread, NULL);
    close(serial_fd);
    fclose(file_ptr);
	safe_console_print("Program terminated.\n");
	console_cleanup();
    return 0;
}
