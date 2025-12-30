/*
 * File:     sensor_utils.h
 * Author:   Bruce Dearing
 * Date:     26/11/2025
 * Version:  1.0
 * Purpose:  Program to handle setting up a serial connection and two threads
 * Mods:
 *
 *
 */

#ifndef SERIAL_UTILS_H
#define SERIAL_UTILS_H

typedef enum {
    SERIAL_RS422,  // or RS-232 fallback
    SERIAL_RS485
} SerialMode;

#define MAX_INPUT_STR 256
//extern char units_of_measure[25][50];
//extern double coefficients[57];
//extern int current_u_of_m;

speed_t get_baud_rate(const char *baud_rate);
int is_valid_tty(const char *str);
SerialMode get_mode(const char *mode);
int open_serial_port(const char* portname, speed_t baud_rate, SerialMode mode);



//void init_units();
//void init_coefficients();
//int init_sensor(bp_sensor **ptr);
//int update_message(bp_sensor **ptr, char *msg);
//int update_units(bp_sensor **ptr, uint8_t unit_id);



#endif
