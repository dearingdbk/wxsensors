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

#include <stdint.h>

unsigned short crc16(char *buffer, int length);
unsigned int crc_ccitt(char *line_of_data);

uint16_t crc16_ccitt(const uint8_t *data, size_t length);

// Checksum 8 Modulo 256
uint8_t checksum_m256(const uint8_t *str_to_chk, size_t length);

// Checksum 8 XOR
uint8_t checksumXOR(const char *str_to_chk);

unsigned char calculate_cs2(const char *str, size_t len);

uint16_t calculate_cs4(const char *str, size_t len);

unsigned char calculate_csx(const char *str, size_t len);

#endif
