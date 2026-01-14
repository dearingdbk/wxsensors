/*
 * File:     sensor_utils.h
 * Author:   Bruce Dearing
 * Date:     26/11/2025
 * Version:  1.0
 * Purpose:  Program to handle setting up a serial connection.
 * Mods:
 *
 *
 */

#ifndef SERIAL_UTILS_H
#define SERIAL_UTILS_H

#include <termios.h>

typedef enum {
    SERIAL_RS422,  // or RS-232 fallback
    SERIAL_RS485
} SerialMode;

// #define MAX_INPUT_STR 256

speed_t get_baud_rate(const char *baud_rate);
int is_valid_tty(const char *str);
SerialMode get_mode(const char *mode);
int open_serial_port(const char* portname, speed_t baud_rate, SerialMode mode);

void safe_serial_write(int fd, const char *fmt, ...);
void serial_utils_cleanup(void);

#endif
