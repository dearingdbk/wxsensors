/*
 * RS-485 / RS-422 Serial Monitor with Full Duplex Responses
 * ---------------------------------------------------------
 * Two-thread serial handler:
 *   - Receiver thread parses and responds to commands
 *   - Sender thread periodically transmits data when active
 *
 * Commands supported:
 *   START, STOP, R?, PWRSTATUS, SITE
 *
 * All replies (ACKs, responses, errors) are sent out on the serial port.
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

#define SERIAL_PORT "/dev/ttyUSB0"   // Adjust as needed
#define BAUD_RATE   B9600

// Shared state
volatile bool running   = false;
volatile bool terminate = false;
int serial_fd;

// ---------------- Command handling ----------------

typedef enum {
    CMD_UNKNOWN,
    CMD_START,
    CMD_STOP,
    CMD_RQUERY,
    CMD_PWRSTATUS,
    CMD_SITE
} CommandType;

// Translate received string to command enum
CommandType parse_command(const char* buf) {
    if (strstr(buf, "START"))      return CMD_START;
    if (strstr(buf, "STOP"))       return CMD_STOP;
    if (strstr(buf, "R?"))         return CMD_RQUERY;
    if (strstr(buf, "PWRSTATUS"))  return CMD_PWRSTATUS;
    if (strstr(buf, "SITE"))       return CMD_SITE;
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
        if (written > 0)
            printf("Sent (%zd bytes): %s", written, response);
        else
            perror("Write failed");
    }
}


/**
 * Checks if the given string matches the regular expression ^/dev/tty[0-9]+$
 *
 * @param str The input string to check
 * @return 0 if it matches, 1 otherwise
 */
int is_valid_tty(const char *str) {
    regex_t regex;
    int reti;
    // The pattern to match: ^/dev/tty[0-9]+$
    // ^ marks the start of the string, $ marks the end.
    // [0-9]+ matches one or more digits.
    const char *pattern = "^/dev/ttyUSB[0-9]+$";

    /* Compile the regular expression */
    reti = regcomp(&regex, pattern, REG_EXTENDED);
    if (reti) {
        fprintf(stderr, "Could not compile regex\n");
        return 1; // Return 1 for error/no match
    }

    /* Execute the regular expression */
    reti = regexec(&regex, str, 0, NULL, 0);

    /* Free memory allocated to the pattern buffer by regcomp */
    regfree(&regex);

    if (reti == 0) {
        /* Match found */
        return 0;
    } else if (reti == REG_NOMATCH) {
        /* No match */
        return 1;
    } else {
        /* Error */
        char msgbuf[100];
        regerror(reti, &regex, msgbuf, sizeof(msgbuf));
        fprintf(stderr, "Regex match failed: %s\n", msgbuf);
        return 1;
    }
}

// ---------------- Serial configuration ----------------

int open_serial_port(const char* portname) {

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
    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);

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
#else
    printf("RS-485 ioctl not supported — using RS-422 mode.\n");
#endif

    printf("Opened %s (RS-485 or RS-422, 8N1 @ baud)\n", portname);
    return fd;
}


// ---------------- Threads ----------------

void* receiver_thread(void* arg) {
    char buf[256];

    while (!terminate) {
        int n = read(serial_fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            printf("Received: %s\n", buf);
            CommandType cmd = parse_command(buf);
            handle_command(cmd);
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
}

// ---------------- Main ----------------

int main(int argc, char *argv[]) {
    // printf("Number of arguments: %d\n", argc);
    // printf("Arguments received:\n");
    // for (int i = 0; i < argc; i++) {
    //    printf("argv[%d]: %s\n", i, argv[i]);
    //}

        if (argc < 2) {
        printf("Usage: %s <device_string>\n", argv[0]);
        return 1;
    }

    const char *device_name = argv[1];

    if (is_valid_tty(device_name) == 0) {
        serial_fd = open_serial_port(device_name);
	printf("Argument \"%s\" matches the /dev/tty# pattern.\n", device_name);
    } else {
        serial_fd = open_serial_port(SERIAL_PORT);
	printf("Argument \"%s\" does not match the /dev/tty# pattern.\n", device_name);
    }

    // serial_fd = open_serial_port(SERIAL_PORT);
    if (serial_fd < 0) return 1;

    pthread_t recv_thread, send_thread;
    pthread_create(&recv_thread, NULL, receiver_thread, NULL);
    pthread_create(&send_thread, NULL, sender_thread, NULL);

    printf("Press 'q' + Enter to quit.\n");
    while (1) {
        char input[8];
        if (fgets(input, sizeof(input), stdin)) {
            if (input[0] == 'q' || input[0] == 'Q') {
                terminate = true;
                break;
            }
        }
    }

    pthread_join(recv_thread, NULL);
    pthread_join(send_thread, NULL);
    close(serial_fd);

    printf("Program terminated.\n");
    return 0;
}
