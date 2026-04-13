
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

/// END FLASH SENSOR ///

#endif
