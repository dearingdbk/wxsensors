/*
 * File: tmp_bp_listen.c
 * Author: Bruce Dearing
 * Date: 17/11/2025
 * Version: 1.2
 * Purpose: Program to handle setting up a serial connection threads
 * one to listen, and one to respond over RS-485 / RS-422
 * Two-thread serial handler:
 * - Receiver thread parses and responds to commands
 * - Sender thread periodically transmits data when active
 * Commands supported:
 * START, STOP, R?, PWRSTATUS, SITE
 * All replies (ACKs, responses, errors) are sent out on the serial port.
 *
 * use case ' tmp_bp_listen <serial_port_location> <baud_rate> // The serial port currently must match /dev/tty(S|USB)[0-9]+
 * use case ' tmp_bp_listen // The serial port and baud rate will be set to defaults /dev/ttyUSB0, and B9600
 * Mods:
 * v1.2: Fixed get_baud_rate function to return speed_t instead of string
 *       Added proper error handling and cleanup
 *       Fixed regex pattern escaping
 *       Improved signal handling
 */

#include <stdio.h>
#include <stdlib.h>
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

#define SERIAL_PORT "/dev/ttyUSB0" // Adjust as needed, main has logic to take arguments for a new location
#define BAUD_RATE B9600 // Adjust as needed, main has logic to take arguments for a new baud rate

// Shared state
volatile bool running = false;
volatile bool terminate = false;
int serial_fd;

void handle_signal(int sig) {
    printf("\nReceived signal %d, terminating...\n", sig);
    terminate = true;
}

// ---------------- Command handling ----------------
typedef enum {
    CMD_UNKNOWN, CMD_START, CMD_STOP, CMD_RQUERY, CMD_PWRSTATUS, CMD_SITE
} CommandType;

// Translate received string to command enum
CommandType parse_command(const char* buf) {
    if (strstr(buf, "START")) return CMD_START;
    if (strstr(buf, "STOP")) return CMD_STOP;
    if (strstr(buf, "R?")) return CMD_RQUERY;
    if (strstr(buf, "PWRSTATUS")) return CMD_PWRSTATUS;
    if (strstr(buf, "SITE")) return CMD_SITE;
    return CMD_UNKNOWN;
}

// Handle each command and send response on serial
void handle_command(CommandType cmd) {
    const char* response = NULL;
    
    switch (cmd) {
        case CMD_START:
            running = true;
            printf("CMD: START -> Begin sending\n");
            response = "ACK: START\r\n";
            break;
        case CMD_STOP:
            running = false;
            printf("CMD: STOP -> Stop sending\n");
            response = "ACK: STOP\r\n";
            break;
        case CMD_RQUERY:
            printf("CMD: R? -> Sending OK\n");
            response = "Response: OK\r\n";
            break;
        case CMD_PWRSTATUS:
            printf("CMD: PWRSTATUS -> Sending power info\n");
            response = "PWRSTATUS: ON\r\n";
            break;
        case CMD_SITE:
            printf("CMD: SITE -> Sending site info\n");
            response = "SITE: 42A-NORTH\r\n";
            break;
        default:
            printf("CMD: Unknown command\n");
            response = "ERR: Unknown command\r\n";
            break;
    }
    
    if (response) {
        ssize_t written = write(serial_fd, response, strlen(response));
        if (written > 0) {
            printf("Sent (%zd bytes): %s", written, response);
        } else {
            perror("Write failed");
        }
    }
}

/*
 * Name: get_baud_rate
 * Purpose: Checks if the given baud rate is a standard value and returns its speed_t value.
 * Arguments: baud_rate: the string value representing the baud rate provided as an argument to main.
 *
 * Output: None.
 * Modifies: None.
 * Returns: returns a speed_t representing the correct baud rate or B9600 as the default.
 * Assumptions: baud_rate is a string and represents a valid integer
 *
 * Bugs: None known.
 * Notes: Fixed to return speed_t instead of string
 */
speed_t get_baud_rate(const char *baud_rate) {
    int baud = atoi(baud_rate);
    
    switch (baud) {
        case 50: return B50;
        case 75: return B75;
        case 110: return B110;
        case 134: return B134;
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
        default:
            printf("Invalid baud rate %d, using default 9600\n", baud);
            return BAUD_RATE; // Default value
    }
}

/*
 * Name: is_valid_tty
 * Purpose: Checks if the given string matches the regular expression ^/dev/tty(S|USB)[0-9]+$
 * This regular expression can match /dev/ttyS0 a non-usb serial device or /dev/ttyUSB0
 * Arguments: str: the string representing the file descriptor of the serial port which should
 * match the pattern ^/dev/tty(S|USB)[0-9]+$.
 * The pattern to match: ^/dev/tty(S|USB)[0-9]+$
 * ^ marks the start of the string, $ marks the end.
 * (S|USB) checks for ttyS or ttyUSB
 * [0-9]+ matches one or more digits.
 * Output: Prints to stdout the appropriate error message if one is encountered.
 * Modifies: None.
 * Returns: return 0 if the string matches, 1 otherwise
 * Assumptions: str is a valid char * pointer and the line contains
 * characters other than white space, and points to an FD.
 *
 * Bugs: None known.
 * Notes: Fixed regex pattern escaping
 */
int is_valid_tty(const char *str) {
    regex_t regex;
    int reti;
    const char *pattern = "^/dev/tty(S|USB)[0-9]+$"; // Fixed escaping
    
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
        char msgbuf,[object Object],;
        regerror(reti, &regex, msgbuf, sizeof(msgbuf));
        fprintf(stderr, "Regex match failed: %s\n", msgbuf);
        return 1;
    }
}

// ---------------- Serial configuration ----------------
/*
 * Name: open_serial_port
 * Purpose: Takes a file descriptor to a serial device, and opens up an RS-485 or RS-422 connection
 *
 * Arguments: portname: the string representing the file descriptor of the serial port which should
 * match the pattern ^/dev/tty(S|USB)[0-9]+$.
 * baud_rate: speed_t value for the baud rate
 *
 * Output: Prints to stderr the appropriate error message if one is encountered.
 * Modifies: serial settings
 * Returns: Returns an int representing a serial device FD if the FD opens, -1 otherwise
 * Assumptions: portname is a valid char * pointer and the line contains
 * characters other than white space, and points to an FD.
 *
 * Bugs: None known.
 * Notes: Added proper baud rate display
 */
int open_serial_port(const char* portname, speed_t baud_rate) {
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
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        close(fd);
        return -1;
    }

#ifdef TIOCSRS485
    struct serial_rs485 rs485conf;
    memset(&rs485conf, 0, sizeof(rs485conf));
    rs485conf.flags |= SER_RS485_ENABLED;        // Enable RS-485
    rs485conf.flags |= SER_RS485_RTS_ON_SEND;    // Drive RTS high while sending
    rs485conf.flags |= SER_RS485_RTS_AFTER_SEND; // Lower RTS after sending
    rs485conf.delay_rts_before_send = 0;
    rs485conf.delay_rts_after_send = 0;
    
    if (ioctl(fd, TIOCSRS485, &rs485conf) < 0) {
        perror("Warning: RS-485 mode not enabled (driver may not support)");
        printf("Continuing in RS-422/RS-232 mode.\n");
    } else {
        printf("RS-485 half-duplex mode enabled via ioctl.\n");
    }
#else
    printf("RS-485 ioctl not supported — using RS-422 mode.\n");
#endif

    // Convert speed_t back to readable format for display
    const char* baud_str = "Unknown";
    switch(baud_rate) {
        case B9600: baud_str = "9600"; break;
        case B19200: baud_str = "19200"; break;
        case B38400: baud_str = "38400"; break;
        case B57600: baud_str = "57600"; break;
        case B115200: baud_str = "115200"; break;
        // Add more cases as needed
    }
    
    printf("Opened %s (RS-485 or RS-422, 8N1 @ %s baud)\n", portname, baud_str);
    return fd;
}

// ---------------- Threads ----------------
void* receiver_thread(void* arg) {
    char buf,[object Object],;
    
    while (!terminate) {
        int n = read(serial_fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            // Remove trailing whitespace/newlines
            while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == '\r')) {
                buf[--n] = '\0';
            }
            if (n > 0) {  // Only process if there's actual content
                printf("Received: %s\n", buf);
                CommandType cmd = parse_command(buf);
                handle_command(cmd);
            }
        }
        usleep(10000);
    }
    return NULL;
}

void* sender_thread(void* arg) {
    const char* msg = "DATA: 12345\r\n";
    
    while (!terminate) {
        if (running) {
            ssize_t written = write(serial_fd, msg, strlen(msg));
            if (written > 0) {
                printf("Sent (%zd bytes): %s", written, msg);
            } else {
                perror("Write failed");
            }
            usleep(1000000); // 1s
        } else {
            usleep(100000);
        }
    }
    return NULL;
}

/*
 * Name: Main
 * Purpose: Main function, which opens up serial port, and creates a receiver and transmit threads to listen, and respond to commands
 * over that serial port. Can take two arguments or no arguments. If changing the serial device name and baud rate, you must supply both.
 * i.e. tmp_bp_listen <serial_device> <baud_rate>
 * uses ternary statements to set either default values for SERIAL_PORT, and BAUD_RATE which are defined above.
 * (condition) ? (value if true) : (value if false)
 *
 * Arguments: device: the string representing the file descriptor of the serial port which should
 * match the pattern ^/dev/tty(S|USB)[0-9]+$. This is tested with function is_valid_tty()
 * baud: the string value representing the proposed baud rate, this string is sent to get_baud_rate() which returns a speed_t value.
 *
 * Output: Prints to stderr the appropriate error messages if encountered.
 * Modifies: None.
 * Returns: Returns an int 0 representing success once the program closes the fd, and joins the threads, or 1 if unable to open the serial port.
 * Assumptions: device is a valid char * pointer and the line contains
 * characters other than white space, and points to an FD.
 * The int provided by arguments is a valid baud rate, although B9600 is set on any errors.
 *
 * Bugs: None known.
 * Notes: Improved error handling and cleanup
 */
int main(int argc, char *argv[]) {
    // Validate arguments
    if (argc > 1 && is_valid_tty(argv,[object Object],) != 0) {
        fprintf(stderr, "Error: Invalid serial port format '%s'\n", argv,[object Object],);
        fprintf(stderr, "Expected format: /dev/ttyUSB[0-9]+ or /dev/ttyS[0-9]+\n");
        return 1;
    }
    
    // ternary statement to set SERIAL_PORT if supplied in args or the default
    const char *device = (argc >= 2) ? argv,[object Object], : SERIAL_PORT;
    // ternary statement to set BAUD_RATE if supplied in args or default
    speed_t baud = (argc >= 3) ? get_baud_rate(argv,[object Object],) : BAUD_RATE;
    
    serial_fd = open_serial_port(device, baud);
    if (serial_fd < 0) {
        return 1;
    }
    
    /* define a signal handler, to capture kill signals and instead set our volatile bool 'terminate' to true, 
     * allowing our c program, to close its loop, join threads, and close our serial device. */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_signal;
    sigaction(SIGINT, &sa, NULL);  // Ctrl-C
    sigaction(SIGTERM, &sa, NULL); // kill, systemd, etc.
    
    pthread_t recv_thread, send_thread;
    
    if (pthread_create(&recv_thread, NULL, receiver_thread, NULL) != 0) {
        perror("Failed to create receiver thread");
        close(serial_fd);
        return 1;
    }
    
    if (pthread_create(&send_thread, NULL, sender_thread, NULL) != 0) {
        perror("Failed to create sender thread");
        terminate = true; // Signal receiver thread to exit
        pthread_join(recv_thread, NULL);
        close(serial_fd);
        return 1;
    }
    
    printf("Serial communication started. Commands: START, STOP, R?, PWRSTATUS, SITE\n");
    printf("Press 'q' + Enter to quit, or use Ctrl+C.\n");
    
    // Main loop for user input
    while (!terminate) {
        char input,[object Object],;
        if (fgets(input, sizeof(input), stdin)) {
            if (input,[object Object], == 'q' || input,[object Object], == 'Q') {
                terminate = true;
                break;
            }
        }
    }
    
    // Clean shutdown
    printf("Shutting down...\n");
    terminate = true;
    
    pthread_join(recv_thread, NULL);
    pthread_join(send_thread, NULL);
    
    close(serial_fd);
    printf("Program terminated.\n");
    return 0;
}
