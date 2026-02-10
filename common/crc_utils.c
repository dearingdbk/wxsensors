/*
 * File:     crc_utils.c
 * Author:   Bruce Dearing
 * Date:     26/11/2025
 * Version:  1.0
 * Purpose:  Program to declare cyclic redundancy check (CRC) helper functions for sensor emulation.
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
#include "crc_utils.h"

#define MAX_PACKET_LENGTH 256

/*
 * Name:         crc16
 * Purpose:      Returns the CRC value of the buffer string provided.
 * Arguments:    buffer - the string to calculate the CRC from.
 *               length - the length of the buffer string.
 *
 * Output:       None.
 * Modifies:     unsigned short crc.
 * Returns:      the calculated CRC value of the string provided or -1 on failure.
 * Assumptions:
 *
 * Bugs:         None known.
 * Notes:
 */
unsigned short crc16(char *buffer, int length) {
	if (buffer == NULL || length <= 0) return -1;
    if (length > MAX_PACKET_LENGTH) return -1;

    unsigned short crc = 0xFFFF;
    unsigned short m;
    int i, j;

    for (i=0; i < length; ++i) {
        crc ^= (unsigned char)buffer[i] << 8;
        for (j=0; j < 8; ++j) {
            m = (crc & 0x8000) ? 0x1021 : 0;
            crc <<= 1;
            crc ^= m;
        }
    }
    crc ^= 0xFFFF;
    return crc;
}

/*
 * Name:         crc_ccitt
 * Purpose:      Returns the CRC value of the buffer string provided.
 * Arguments:    line_of_date - the string to calculate the CRC from.
 *
 * Output:       None.
 * Modifies:     unsigned int crc.
 * Returns:      the calculated crc value of the string provided.
 * Assumptions:
 *
 * Bugs:         None known.
 * Notes:
 */
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


/*
 * Name:         crc16_ccitt
 * Purpose:      Returns the CRC value of the buffer string provided.
 * Arguments:    line_of_date - the string to calculate the CRC from.
 *				 length - the length of the string.
 *
 * Output:       None.
 * Modifies:     uint16_t crc.
 * Returns:      the calculated crc value of the string provided.
 * Assumptions:
 *
 * Bugs:         None known.
 * Notes:
 */
uint16_t crc16_ccitt(const uint8_t *data, size_t length) {
    uint16_t crc = 0x0000; // Initial value

    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}


/*
 * Name:         checksum_m256
 * Purpose:      Takes a  string, and returns a checksum of the characters XOR.
 * Arguments:    str_to_chk the string that checksum will be calculated for, it is set to uint_8 to eliminate any sign errors.
 *				 length the length of the string to check.
 * Output:       None.
 * Modifies:     None.
 * Returns:      returns an unsigned 8 bit integer of the checksum of str_to_chk.
 * Assumptions:  Terminate is set to false.
 *
 * Bugs:         None known.
 * Notes:        To print in HEX utilize dprintf(serial_fd, "%s%02X\r\n", str_to_chk, check_sum(str_to_chk));
 */
uint8_t checksum_m256(const uint8_t *str_to_chk, size_t length) {

    uint8_t checksum = 0;
    if (str_to_chk == NULL || length == 0 || length > MAX_PACKET_LENGTH) {
        return -1;
    }

	for (size_t i = 0; i < length; i++) {
		checksum += str_to_chk[i];
	}
    return (uint8_t)(checksum & 0xFF);
}

/*
 * Name:         checksumXOR
 * Purpose:      Takes a '\0' delimited string, and returns a checksum of the characters XOR.
 * Arguments:    str_to_chk the string that checksum will be calculated for
 *
 * Output:       None.
 * Modifies:     None.
 * Returns:      returns an unsigned 8 bit integer of the checksum of str_to_chk.
 * Assumptions:  Terminate is set to false.
 *
 * Bugs:         None known.
 * Notes:        To print in HEX utilize dprintf(serial_fd, "%c%s%c%02X\r\n",2, str_to_chk, check_sum(str_to_chk));
 */
uint8_t checksumXOR(const char *str_to_chk) {

    uint8_t checksum = 0;
    if (str_to_chk == NULL) {
        return 0;
    }
    while (*str_to_chk != '\0') {
        checksum ^= (uint8_t)(*str_to_chk);
        str_to_chk++;
    }
    return checksum;
}

unsigned char calculate_cs2(const char *str, size_t len) {
    unsigned char sum = 0;

    for (size_t i = 0; i < len; i++) {
        sum += (unsigned char)str[i];
    }

    return sum;
}


uint16_t calculate_cs4(const char *str, size_t len) {
    uint16_t sum = 0; // uint16_t naturally wraps at 65535
    for (size_t i = 0; i < len; i++) {
        sum += (unsigned char)str[i];
    }
    return sum;
}

unsigned char calculate_csx(const char *str, size_t len) {
    unsigned char xor_result = 0;

    for (size_t i = 0; i < len; i++) {
        unsigned char b = (unsigned char)str[i];

        // Apply the rule: b' = 0 if b = 36 ($) or 42 (*)
        if (b == 36 || b == 42) {
            b = 0;
        }

        xor_result ^= b;
    }
    return xor_result;
}

