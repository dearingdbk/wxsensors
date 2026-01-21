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
    SERIAL_RS485,
	SERIAL_SDI12   // SDI-12 protocol, assuming USB adaptor.
} SerialMode;

// #define MAX_INPUT_STR 256

speed_t get_baud_rate(const char *baud_rate);
int is_valid_tty(const char *str) __attribute__((nonnull(1)));
SerialMode get_mode(const char *mode) __attribute__((nonnull(1)));
int open_serial_port(const char* portname, speed_t baud_rate, SerialMode mode)__attribute__((nonnull(1))) __attribute__((warn_unused_result));
/* Format: __attribute__((format(ARCHETYPE, STRING_INDEX, FIRST_TO_CHECK))) */
void safe_serial_write(int fd, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void serial_utils_cleanup(void);
void sdi12_wake_sensor(int fd);

#endif
