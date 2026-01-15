/*
 * File:     ceilomete.c
 * Author:   Bruce Dearing
 * Date:     28/11/2025
 * Version:  1.0
 * Purpose:  Program to handle setting up a serial connection and two threads
 *           one to listen, and one to respond over RS-485 / RS-422
 *           Two-thread serial handler:
 *            - Receiver thread parses and responds to commands
 *            - Sender thread periodically transmits data when active
 *           Program emulates a weather sensor - Druck DSP8100
 *           Commands supported:
 *           R - Requests current pressure reading.
 *           G - Request new pressure reading.
 *           Z - Read raw data.
 *           I - identify and setup information.
 *           A - Set automatic transmission interval, also used to set if units are output as text.
 * 	     N - Set device address.
 *           Q - Pressure measurement speed.
 *           U - Pressure Units.
 *           C - Digital output calibration.
 *           H - Set full-scale. - Disabled for sensor emulation.
 *           M - User message.
 *           O - Communication Settings
 *           P - Change PIN.
 *           S - Set offset. - Disabled for sensor emulation.
 *           <CR> <CRLF> - Command Terminators.
 *           <A|N|Q|U|C|H|M|O|P|S|E|L|T|V|W>,? Query the value of those fields.
 *           *<A-Z> provides the reading and the units of measurement.
 *           All replies (ACKs, responses, errors) are sent out on the serial port.
 *           All incoming commands follow this format:
 *           Direct Mode: <SPACE><Command>,<P1>,<P2>,...,<Pn><CR>
 *           Addressed Mode: <SPACE><Address>:<Command>,<P1>,<P2>,...,<Pn><CR>
 *	     use case ' barometric <file_path> <serial_port_location> <baud_rate> <RS422|RS485> The serial port currently must match /dev/tty(S|USB)[0-9]+
 * 	     use case ' barometric <file_path> // The serial port, baud rate, and mode will be set to  defaults /dev/ttyUSB0, and B9600
 * Mods:
 *
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
#include "crc_utils.h"
#include "serial_utils.h"
#include "console_utils.h"
#include "file_utils.h"
#include "crc_utils.h"

#define SERIAL_PORT "/dev/ttyUSB0"   // Adjust as needed, main has logic to take arguments for a new location
#define BAUD_RATE   B9600	     // Adjust as needed, main has logic to take arguments for a new baud rate
#define MAX_LINE_LENGTH 1024


FILE *file_ptr = NULL; // Global File pointer
char *file_path = NULL; // path to file

// Shared state
int continuous = 0;
volatile sig_atomic_t terminate = 0;
volatile sig_atomic_t kill_flag = 0;

int serial_fd = -1;
char site_id = 'A';
//uint8_t address = 0;


/* Synchronization primitives */
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


// ---------------- Command handling ----------------

typedef enum {
    CMD_UNKNOWN,
    CMD_START, // ! received from the terminal
    CMD_STOP,  // ? received from the terminal
    CMD_SITE,  // & recieved from the terminal
    CMD_POLL,  // <A-Z> received from the terminal
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
    if (buf[0] == '!' && buf[1] == '\0')					return CMD_START;
    if (buf[0] == '?' && buf[1] == '\0')					return CMD_STOP;
    if (buf[0] == '&' && buf[1] == '\0')					return CMD_SITE;
    if (buf[0] == '*' && buf[1] >= 'A' && buf[1] <= 'Z' && buf[2] =='\0') 	return CMD_CONFIG;
    if (buf && buf[0] >= 'A' && buf[0] <= 'Z' && buf[1] == '\0') 		return CMD_POLL;
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
            continuous = 1; // enable continuous sending
            pthread_cond_signal(&send_cond);  // Wake sender_thread immediately
            pthread_mutex_unlock(&send_mutex);
            break;

        case CMD_STOP:
            pthread_mutex_lock(&send_mutex);
		    continuous = 0; // disables continuous sending.
            pthread_cond_signal(&send_cond);   // Wake sender_thread to exit loop
            pthread_mutex_unlock(&send_mutex);
            break;

        case CMD_SITE:
            // safe_console_print("CMD: SITE -> Sending site info\n");
            safe_serial_write(serial_fd, "%c\r\n", site_id);
			//safe_write_response("%c\r\n", site_id);
            break;

        case CMD_POLL:
            resp_copy = get_next_line_copy(file_ptr, &file_mutex);
            if (resp_copy) {
                // prints <Start of Line ASCII 2>, the string of data read, <EOL ASCII 3>, Checksum of the line read
                safe_serial_write(serial_fd, "\x02%s\x03%02X\r\n", resp_copy, checksumXOR(resp_copy));
                free(resp_copy);
            } else {
                safe_console_error("ERR: Empty file\r\n");
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
 *               get_next_line_copy() and send that line to the serial device using safe_serial_write() function every 2 seconds.
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
        // wait until either terminate is set or continuous becomes 1
        while (!terminate && !continuous) {
            pthread_cond_wait(&send_cond, &send_mutex);
        }

        if (terminate) {
            pthread_mutex_unlock(&send_mutex);
            break;
        }

        while (!terminate && continuous) {
            char *line = get_next_line_copy(file_ptr, &file_mutex);
            if (line) {
                // prints <STX ASCII 2>, the string of data read, <ETX ASCII 3>, Checksum of the line read
                safe_serial_write(serial_fd, "\x02%s\x03%02X\r\n", line, checksumXOR(line));
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
        safe_console_error("Usage: %s <file_path> <serial_device> <baud_rate> <RS422|RS485>\n", argv[0]);
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

	pthread_mutex_destroy(&file_mutex);
	pthread_mutex_destroy(&send_mutex);
	pthread_cond_destroy(&send_cond);

    close(serial_fd);
    fclose(file_ptr);

    safe_console_print("Program terminated.\n");
	console_cleanup();
	serial_utils_cleanup();
    return 0;
}
