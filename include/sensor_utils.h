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

#define MAX_INPUT_STR 256
extern char units_of_measure[25][50];
extern double coefficients[57];
extern int current_u_of_m;

typedef struct {
    char unit_type[MAX_INPUT_STR];
    char serial_number[MAX_INPUT_STR];
    char style[MAX_INPUT_STR];
    char min_pressure[MAX_INPUT_STR];
    char max_pressure[MAX_INPUT_STR];
    char manufacture_date[MAX_INPUT_STR];
    char software_version[MAX_INPUT_STR];
    int trans_interval;
    char units_sent[MAX_INPUT_STR];
    int measurement_speed;
    char filter_factor[MAX_INPUT_STR];
    char filter_step[MAX_INPUT_STR];
    char user_message[MAX_INPUT_STR];
    char units[MAX_INPUT_STR];
    char pin_set[MAX_INPUT_STR];
    char user_zero[MAX_INPUT_STR];
    char user_fs[MAX_INPUT_STR];
    char sensor_sn[MAX_INPUT_STR];
    char internal_chksum[MAX_INPUT_STR];
} bp_sensor;

typedef enum {
    CMD_UNKNOWN,
    CMD_START, 	// ! received from the terminal
    CMD_STOP,  	// ? received from the terminal
    CMD_SITE,  	// & recieved from the terminal
    CMD_POLL,   // <A-Z> received from the terminal
    CMD_CONFIG, // *<A-Z> received from the terminal
    CMD_R	// R received from terminal
} CommandType;


void init_units();
void init_coefficients();
int init_sensor(bp_sensor **ptr);
int update_message(bp_sensor **ptr, char *msg);
int update_units(bp_sensor **ptr, uint8_t unit_id);



#endif
