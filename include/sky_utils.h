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

#ifndef SKY_UTILS_H
#define SKY_UTILS_H

#define MAX_INPUT_STR 256
extern char units_of_measure[25][50];
extern double coefficients[57];
extern int current_u_of_m;

typedef enum {
    CMD_UNKNOWN,
    CMD_START, 	// ! received from the terminal
    CMD_STOP,  	// ? received from the terminal
    CMD_SITE,  	// & recieved from the terminal
    CMD_POLL,   // <A-Z> received from the terminal
    CMD_CONFIG, // *<A-Z> received from the terminal
    CMD_R	// R received from terminal
} CommandType;


//void init_units();
//void init_coefficients();
//int init_sensor(bp_sensor **ptr);
//int update_message(bp_sensor **ptr, char *msg);
//int update_units(bp_sensor **ptr, uint8_t unit_id);
unsigned short crc16(char *buffer, int length);


#endif
