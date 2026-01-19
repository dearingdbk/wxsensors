
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

#ifndef SENSOR_UTILS_H
#define SENSOR_UTILS_H

#include <stdbool.h>

#define MAX_INPUT_STR 256

/// WIND SENSOR ///

typedef struct {
	int a_val;
	int b_val;
	int c_val;
	int e_val;
	int f_val;
	int g_val;
	int h_val;
	int j_val;
	int k_val;
	int l_val;
	int m_val;
	char n_val;
	int o_val;
	int p_val;
	int t_val;
	int u_val;
	int v_val;
	int x_val;
	int y_val;
	int z_val;
} wind_sensor;

int init_wind(wind_sensor **ptr);
char get_wind_units(int chk_val);

/// END WIND SENSOR ///

/// FLASH SENSOR ///

typedef struct {
	uint16_t overhead;
	uint16_t vicinity;
	uint16_t near_distant;
    uint16_t far_distant;
	char serial_num[13];
} flash_sensor;


time_t parse_btd_datetime(const char *date_str, const char *time_str);
int format_btd_datetime(time_t t, char *date_str, char *time_str);
int update_btd_timestamps(const char *input, char *output, size_t output_size);
int init_flash(flash_sensor **ptr);
int reset_flash(flash_sensor **ptr);
int set_dist(flash_sensor **ptr, const char *buf);

/// END FLASH SENSOR ///

#endif
