/*
 * File:     serial_utils.c
 * Author:   Bruce Dearing
 * Date:     04/12/2025
 * Version:  1.0
 * Purpose:  Program to handle setting up a serial connection and two threads
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
#include "serial_utils.h"

#define SERIAL_PORT "/dev/ttyUSB0"   // Adjust as needed, main has logic to take arguments for a new location
#define BAUD_RATE   B9600	     // Adjust as needed, main has logic to take arguments for a new baud rate
#define MAX_LINE_LENGTH 1024


//FILE *file_ptr = NULL; // Global File pointer
//char *file_path = NULL; // path to file

// Shared state
//int serial_fd = -1;


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


/*typedef enum {
    SERIAL_RS422,  // or RS-232 fallback
    SERIAL_RS485
} SerialMode;*/

/*
 * Name:         get_mode
 * Purpose:      Checks if the given serial protocol is a standard value and returns its enum name.
 * Arguments:    mode: the string value representing the serial protocol provided as an argument to main.
 *
 * Output:       None.
 * Modifies:     None.
 * Returns:      returns a string representing the correct serial protocol RS485 as the default.
 * Assumptions:  None.
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
