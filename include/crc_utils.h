/*
 * File:     crc_utils.h
 * Author:   Bruce Dearing
 * Date:     26/11/2025
 * Version:  1.0
 * Purpose:  Program to handle setting up a serial connection and two threads
 * Mods:
 *
 *
 */

#ifndef CRC_UTILS_H
#define CRC_UTILS_H

//#define MAX_INPUT_STR 256

/*typedef enum {
    CMD_UNKNOWN,
    CMD_START, 	// ! received from the terminal
    CMD_STOP,  	// ? received from the terminal
    CMD_SITE,  	// & recieved from the terminal
    CMD_POLL,   // <A-Z> received from the terminal
    CMD_CONFIG, // *<A-Z> received from the terminal
    CMD_R	// R received from terminal
} CommandType;*/

unsigned short crc16(char *buffer, int length);
unsigned int crc_ccitt(char *line_of_data);

#endif
