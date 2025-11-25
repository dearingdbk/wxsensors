/*
 * File:     tmp_bp_listen.c
 * Author:   Bruce Dearing
 * Date:     17/11/2025
 * Version:  1.2
 * Purpose:  Program to handle setting up a serial connection and two threads
 *           one to listen, and one to respond over RS-485 / RS-422
 *           Two-thread serial handler:
 *            - Receiver thread parses and responds to commands
 *            - Sender thread periodically transmits data when active (Not implemented in this program)
 *           Commands supported:
 *           START, STOP, {F00RDD}, SITE
 *           All replies (ACKs, responses, errors) are sent out on the serial port.
 *
 * 	     use case ' tmp_bp_listen <file_path> // The serial port and baud rate will be set to  defaults /dev/ttyUSB0, and B9600
 *           use case ' tmp_bp_listen <file_path> <serial_port_location> <baud_rate> <RS422|RS485> The serial port currently must match /dev/tty(S|USB)[0-9]+
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


#define SERIAL_PORT "/dev/ttyUSB0"   // Adjust as needed, main has logic to take arguments for a new location
#define BAUD_RATE   B9600	     // Adjust as needed, main has logic to take arguments for a new baud rate
#define MAX_LINE_LENGTH 1024


FILE *file_ptr = NULL; // Global File pointer
char *file_path = NULL; // path to file
// Shared state
volatile sig_atomic_t terminate = 0;
volatile sig_atomic_t kill_flag = 0;
volatile bool running = false;
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
 * Name:         get_next_line
 * Purpose:      Reads a line of a file, received from a file pointer, it then iterates that pointer to the next line, or if at the EOF
 *               loops back to the beginning of the file.
 * Arguments:    None.
 *
 * Output:       Error message, if the file is not open.
 * Modifies:     None.
 * Returns:      returns a line of text from a file MAX_LINE_LENGTH long or NULL.
 * Assumptions:  The file is open, and the line read is less than MAX_LINE_LENGTH
 *
 * Bugs:         None known.
 * Notes:
 */
const char* get_next_line(void) {
    static char line[MAX_LINE_LENGTH];

    if (!file_ptr) {
        fprintf(stderr, "File not opened!\n");
        return NULL;
    }

    if (!fgets(line, sizeof(line), file_ptr)) {
        // Reached EOF or error
        rewind(file_ptr);             // Go back to start
        if (!fgets(line, sizeof(line), file_ptr)) {
            // File empty
            return NULL;
        }
    }

    // Remove trailing newline
    line[strcspn(line, "\r\n")] = '\0';
    return line;
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
    CMD_START,
    CMD_STOP,
    CMD_RDD,
    CMD_SITE
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
    if (strstr(buf, "START"))      return CMD_START;
    if (strstr(buf, "STOP"))       return CMD_STOP;
    if (strstr(buf, "{F00RDD}"))   return CMD_RDD;
    if (strstr(buf, "SITE"))       return CMD_SITE;
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
            //running = true; // disable for now
            break;

        case CMD_STOP:
            //running = false; // disable for now
            break;

        case CMD_RDD:
            resp_copy = get_next_line_copy();
            if (resp_copy) {
                safe_write_response("%s\r\n", resp_copy);
                free(resp_copy);
            } else {
                safe_write_response("ERR: Empty file\r\n");
            }
            break;

        case CMD_SITE:
            printf("CMD: SITE -> Sending site info\n");
            break;

        default:
            printf("CMD: Unknown command\n");
            break;
    }
}

/*
 * Name:         get_baud_rate
 * Purpose:      Checks if the given baud rate is a standard value and returns its string name.
 * Arguments:    baud_rate: the integer value representing the baud rate provided as an argument to main.
 *
 * Output:       None.
 * Modifies:     None.
 * Returns:      returns a string representing the correct baud rate or B9600 as the default.
 * Assumptions:  baud_rate is a integer and is within the standard values
 *
 * Bugs:         None known.
 * Notes:
 */
speed_t get_baud_rate(const char *baud_rate) {
    int baud = atoi(baud_rate);
    switch (baud) {
        case 50: return B50;
        case 75: return B75;
        case 110: return B110;
        case 150: return B150;
        case 200: return B200;
        case 300: return B300;
        case 600: return B600;
        case 1200: return B1200;
        case 1800: return B1800;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return BAUD_RATE; // Default value
    }
}

/*
 * Name:         is_valid_tty
 * Purpose:      Checks if the given string matches the regular expression ^/dev/tty(S|USB)[0-9]+$
 *		 This regular expression can match /dev/ttyS0 a non-usb serial device or /dev/ttyUSB0
 * Arguments:    str: the string representing the file descriptor of the serial port which should
 * 		 match the pattern ^/dev/tty(S|USB)[0-9]+$.
 * 		   The pattern to match: ^/dev/tty(S|USB)[0-9]+$
 * 		   ^ marks the start of the string, $ marks the end.
 *		   (S|USB) checks for ttyS or ttyUSB
 *                 [0-9]+ matches one or more digits.

 * Output:       Prints to stdout the appropriate error message if one is encountered.
 * Modifies:     None.
 * Returns:      return 0 if the string matches, 1 otherwise
 * Assumptions:  str is a valid char * pointer and the line contains
 *               characters other than white space, and points to an FD.
 *
 * Bugs:         None known.
 * Notes:
 */
int is_valid_tty(const char *str) {
    regex_t regex;
    int reti;
    const char *pattern = "^/dev/tty(S|USB)[0-9]+$";

    // Compile the regular expression
    reti = regcomp(&regex, pattern, REG_EXTENDED);
    if (reti) {
        fprintf(stderr, "Could not compile regex\n");
        return 1; // Return 1 for error/no match
    }

    // Execute the regular expression
    reti = regexec(&regex, str, 0, NULL, 0);

    // Free memory allocated to the pattern buffer by regcomp
    regfree(&regex);

    if (reti == 0) {
        // Match found
        return 0;
    } else if (reti == REG_NOMATCH) {
        // No match
        return 1;
    } else {
        // Error
        char msgbuf[100];
        regerror(reti, &regex, msgbuf, sizeof(msgbuf));
        fprintf(stderr, "Regex match failed: %s\n", msgbuf);
        return 1;
    }
}

// ---------------- Serial configuration ----------------



typedef enum {
    SERIAL_RS422,  // or RS-232 fallback
    SERIAL_RS485
} SerialMode;

/*
 * Name:         get_mode
 * Purpose:      Checks if the given serial protocol is a standard value and returns its string name.
 * Arguments:    mode: the stringvalue representing the serial protocol provided as an argument to main.
 *
 * Output:       None.
 * Modifies:     None.
 * Returns:      returns a string representing the correct serial protocol or RS485 as the default.
 * Assumptions:  mode is a integer and is within the standard values
 *
 * Bugs:         None known.
 * Notes:
 */
SerialMode get_mode(const char *mode) {
    if (strcmp(mode, "RS422") == 0) {
        return SERIAL_RS422;
    } else if (strcmp(mode, "RS485") == 0) {
        return SERIAL_RS485;
    } else {
        return SERIAL_RS485;
    }
}

/*
 * Name:         open_serial_port
 * Purpose:      Takes a file descriptor to a serial device, and opens up an RS-485 or RS-422 connection
 *
 * Arguments:    portname: the string representing the file descriptor of the serial port which should
 * 		 match the pattern ^/dev/ttyUSB[0-9]+$.
 *
 * Output:       Prints to stderr the appropriate error message if one is encountered.
 * Modifies:     serial settings
 * Returns:      Returns an int representing a serial device FD if the FD opens, -1 otherwise
 * Assumptions:  portname is a valid char * pointer and the line contains
 *               characters other than white space, and points to an FD.
 *
 * Bugs:         None known.
 * Notes:
 */
int open_serial_port(const char* portname, speed_t baud_rate, SerialMode mode) {

    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Error opening serial port");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("Error from tcgetattr");
        close(fd);
        return -1;
    }

    // Set raw mode
    cfmakeraw(&tty);

    // Set baud rate
    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    // 8N1
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CLOCAL | CREAD;

    // No RTS/CTS hardware flow control
    tty.c_cflag &= ~CRTSCTS;

    // Non-blocking read with timeout
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        close(fd);
        return -1;
    }

#ifdef TIOCSRS485
    if (mode == SERIAL_RS485) {
        struct serial_rs485 rs485conf;
        memset(&rs485conf, 0, sizeof(rs485conf));

        rs485conf.flags |= SER_RS485_ENABLED;         // Enable RS-485
        rs485conf.flags |= SER_RS485_RTS_ON_SEND;     // Drive RTS high while sending
        rs485conf.flags |= SER_RS485_RTS_AFTER_SEND;  // Lower RTS after sending
        rs485conf.delay_rts_before_send = 0;
        rs485conf.delay_rts_after_send  = 0;

        if (ioctl(fd, TIOCSRS485, &rs485conf) < 0) {
            perror("Warning: RS-485 mode not enabled (driver may not support)");
            printf("Continuing in RS-422/RS-232 mode.\n");
        } else {
            printf("RS-485 half-duplex mode enabled via ioctl.\n");
        }
    } else {
        printf("RS-422 / RS-232 mode selected.\n");
    }
#else
    printf("RS-485 ioctl not supported — using RS-422 mode.\n");
#endif

    printf("Opened %s (%s, 8N1 @ baud)\n", portname, (mode == SERIAL_RS485) ? "RS-485" : "RS-422");
    return fd;
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
 * Name:         sender_thread
 * Purpose:      currently no purpose... On running=true it would send msg every 1 second.
 *               If a sensor at a later date requires continuous sending of data, in response to a START command for example
 *               we can use this thread to send that data, while still being able to listen using reciver thread.
 * Arguments:    arg: thread arguments.
 *
 * Output:       None.
 * Modifies:     None.
 * Returns:      NULL.
 * Assumptions:  serial port will have data, and that data will translate to a command.
 *
 * Bugs:         None known.
 * Notes:
 *
void* sender_thread(void* arg) {
    const char* msg = "DATA: 12345";
    while (!terminate) {
        if (running) {
            ssize_t written = write(serial_fd, msg, strlen(msg));
            // dprintf(serial_fd, "%s\r\n", msg);
            if (written > 0)
                printf("Sent (%zd bytes): %s", written, msg);
            else
                perror("Write failed");
            usleep(1000000); // 1s
        } else {
            usleep(100000);
        }
    }
    return NULL;
}*/

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

    // ternary statement to set Serial Protocol if supplied in args or default
    SerialMode mode = (argc >=5) ? get_mode(argv[4]) : SERIAL_RS485; // returns RS485 by default.

    serial_fd = open_serial_port(device, baud, mode);

    if (serial_fd < 0)
        return 1;

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

    printf("Press 'q' + Enter to quit.\n");
    while (!kill_flag) {
        char input[8];
        if (fgets(input, sizeof(input), stdin)) {
            if (input[0] == 'q' || input[0] == 'Q' || kill_flag == 1) {
                terminate = 1;
		break;
            }
        }
    }

    pthread_join(recv_thread, NULL);
    close(serial_fd);
    fclose(file_ptr);

    printf("Program terminated.\n");
    return 0;
}
