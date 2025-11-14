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

// ---------------- Serial configuration ----------------

int open_serial_port(const char* portname) {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Error opening serial port");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        perror("Error from tcgetattr");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; // 0.1 s timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        close(fd);
        return -1;
    }

#ifdef TIOCSRS485
    struct serial_rs485 rs485conf;
    memset(&rs485conf, 0, sizeof(rs485conf));
    rs485conf.flags |= SER_RS485_ENABLED;
    rs485conf.flags |= SER_RS485_RTS_ON_SEND; // RTS high during send
    rs485conf.flags &= SER_RS485_RTS_AFTER_SEND;  // RTS low after send
//    rs485conf.delay_rts_before_send = 0;
  //  rs485conf.delay_rts_after_send  = 0;

    if (ioctl(fd, TIOCSRS485, &rs485conf) < 0)
        perror("Warning: RS-485 mode not enabled (driver may not support)");
    else
        printf("RS-485 mode enabled via ioctl.\n");
#else
    printf("Note: RS-485 ioctl not available; assuming RS-422/RS-232 mode.\n");
#endif

    printf("Opened %s (RS-485/422, 9600 8N1)\n", portname);
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
    printf("Number of arguments: %d\n", argc);
    printf("Arguments received:\n");
    for (int i = 0; i < argc; i++) {
        printf("argv[%d]: %s\n", i, argv[i]);
    }


    serial_fd = open_serial_port(SERIAL_PORT);
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
