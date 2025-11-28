/*
 * File:     sky_utils.c
 * Author:   Bruce Dearing
 * Date:     26/11/2025
 * Version:  1.0
 * Purpose:  Program to declare helper functions for ceilometer emulation.
 *
 * Mods:
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <regex.h>
#include "pw_utils.h"

char units_of_measure[25][50]; // Global array to hold the units of measurement available.
double coefficients[57]; // Global array to hold K coefficients of float values.


/*
 * Name:         crc16
 * Purpose:      Returns the CRC value of the buffer string provided.
 * Arguments:    buffer - the string to calculate the CRC from.
 *               length - the length of the buffer string.
 *
 * Output:       None.
 * Modifies:     The units value of the provided bp_sensor.
 * Returns:      -1 on failure, 1 on success.
 * Assumptions:
 *
 * Bugs:         None known.
 * Notes:
 */
unsigned short crc16(char *buffer, int length) {
    unsigned short crc;
    unsigned short m;
    int i, j;
    if (length < (int)sizeof(buffer)) return -1;
    crc = 0xFFFF;

    for (i=0; i < length; ++i) {
        crc ^= buffer[i] << 8;
        for (j=0; j < 8; ++j) {
            m = (crc & 0x8000) ? 0x1021 : 0;
            crc <<= 1;
            crc ^= m;
        }
    }
    crc ^= 0xFFFF;
    return crc;
}


unsigned int crc_ccitt(char *line_of_data) {
    unsigned int crc; // Returned CRC value
    unsigned int i; // counter
    crc = 0x0000;
    for (i=0; i < strlen(line_of_data); i++) {
        unsigned crc_new = (unsigned char)(crc >> 8) | (crc << 8);
        crc_new ^= line_of_data[i];
        crc_new ^= (unsigned char)(crc_new & 0xff) >> 4;
        crc_new ^= crc_new << 12;
        crc_new ^= (crc_new & 0xff) << 5;
        crc = crc_new;
    }
    return (crc);
}
