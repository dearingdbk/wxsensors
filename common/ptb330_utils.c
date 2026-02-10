/*
 * File:     ptb330_utils.c
 * Author:   Bruce Dearing
 * Date:     16/01/2026
 * Purpose:  Implementation of PTB330-specific logic.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "crc_utils.h"
#include "ptb330_utils.h"

FormItem compiled_form[MAX_FORM_ITEMS];
int form_item_count = 0;

int active_width = 0;
int active_precision = 0;

int init_ptb330_sensor(ptb330_sensor **ptr) {
    *ptr = malloc(sizeof(ptb330_sensor));
    if (!*ptr) return -1;

    ptb330_sensor *s = *ptr;
	//char *strncpy(char *destination, const char *source, size_t num);
    strncpy(s->serial_number, "G1234567", MAX_SN_LEN);
    strncpy(s->software_version, "1.12", 5);
    s->address = 0;
    s->mode = SMODE_STOP;
    s->units = UNIT_HPA;
    s->intv_data.interval = 1;
	s->intv_data.interval_units[0] = 's';
	s->intv_data.interval_units[1] = '\0'; // Manually terminate string.
    strncpy(s->format_string, "\" \"  P1 \" \" P2 \" \" P3 \" \" ERR \" \" P \" \" P3H \\R\\N", MAX_FORM_STR - 1); // Our default format P11A11.
	s->format_string[MAX_FORM_STR] = '\0'; // Manually terminate string.
	parse_form_string(s->format_string);
    s->pressure = 1013.25;
    s->offset = 0.0;
    s->initialized = true;
	s->baud = 6; // 4800 default.
	s->data_f = 8;
	s->parity = 'N';
	s->stop_b = 1;
	strncpy(s->batch_num, "1234", MAX_BATCH_NUM);
	memset(&s->module_one, 0, sizeof(s->module_one));
	memset(&s->module_two, 0, sizeof(s->module_two));
	memset(&s->module_three, 0, sizeof(s->module_three));
	memset(&s->module_four, 0, sizeof(s->module_four));
	strncpy(s->module_one.serial_number, "M1234567", MAX_SN_LEN);
	strncpy(s->module_two.serial_number, "M7654321", MAX_SN_LEN);
	strncpy(s->module_three.serial_number, "M4713526", MAX_SN_LEN);
	strncpy(s->module_one.batch_num, "550", MAX_BATCH_NUM);
	strncpy(s->module_two.batch_num, "550", MAX_BATCH_NUM);
	strncpy(s->module_three.batch_num, "550", MAX_BATCH_NUM);
	s->initialized = true;
	clock_gettime(CLOCK_MONOTONIC, &s->last_send_time);
    //s->last_send_time.tv_sec = 0; // Immediate first send if required, uncomment these lines.
    //s->last_send_time.tv_nsec = 0;

    return 0;
}

void ptb330_parse_command(const char *input, ptb330_command *cmd) {
    char buf[128];
    strncpy(buf, input, sizeof(buf));
    // Simple tokenizer for PTB330 commands
    char *token = strtok(buf, " \r\n");
    if (!token) {
        cmd->type = CMD_UNKNOWN;
        return;
    }

    // Convert to uppercase for comparison
    for(int i = 0; token[i]; i++) token[i] = toupper(token[i]);

    if (strcmp(token, "SEND") == 0)   cmd->type = CMD_SEND;
    else if (strcmp(token, "R") == 0) cmd->type = CMD_R;
    else if (strcmp(token, "S") == 0) cmd->type = CMD_S;
    else if (strcmp(token, "INTV") == 0) cmd->type = CMD_INTV;
    else if (strcmp(token, "SMODE") == 0) cmd->type = CMD_SMODE;
    else if (strcmp(token, "FORM") == 0) cmd->type = CMD_FORM;
    else if (strcmp(token, "UNIT") == 0) cmd->type = CMD_UNIT;
    else if (strcmp(token, "VERS") == 0) cmd->type = CMD_VERS;
    else cmd->type = CMD_UNKNOWN;

    // Capture remaining params
    char *params = strtok(NULL, "\r\n");
    if (params) strncpy(cmd->raw_params, params, sizeof(cmd->raw_params));
    else cmd->raw_params[0] = '\0';
}

bool ptb330_is_ready_to_send(ptb330_sensor *sensor) {
    if (!sensor || sensor->mode != SMODE_RUN) return false;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    long seconds = now.tv_sec - sensor->last_send_time.tv_sec;
    if (seconds >= (long)sensor->intv_data.interval) return true;

    return false;
}


/*typedef struct {
    PTB330_Unit unit;
    const char *label;
    double multiplier; // Multiplier to convert from hPa to this unit
} UnitConversion;

static const UnitConversion unit_table[] = {
    {UNIT_HPA,   "hPa",  1.0},
    {UNIT_MBAR,  "mbar", 1.0},
    {UNIT_KPA,   "kPa",  0.1},
    {UNIT_PA,    "Pa",   100.0},
    {UNIT_INHG,  "inHg", 0.0295299},
    {UNIT_MMHG,  "mmHg", 0.750062},
    {UNIT_TORR,  "torr", 0.750062},
    {UNIT_PSI,   "psi",  0.0145038}
};*/

const char* get_unit_str(PTB330_Unit unit) {
    for (int i = 0; i < (int)(sizeof(unit_table)/sizeof(UnitConversion)); i++) {
        if (unit_table[i].unit == unit) return unit_table[i].label;
    }
    return "hPa";
}

double get_scaled_pressure(float hpa_val, PTB330_Unit unit) {
    for (int i = 0; i < (int)(sizeof(unit_table)/sizeof(UnitConversion)); i++) {
        if (unit_table[i].unit == unit) return (double)hpa_val * unit_table[i].multiplier;
    }
    return (double)hpa_val;
}

/*void ptb330_format_output(ptb330_sensor *sensor, char *dest, size_t max_len) {
    // In a full implementation, you would parse the format_string (e.g., #P, #U)
    // For now, we provide the standard PTB330 default output.
    const char* unit_str = "hPa";
    switch(sensor->units) {
        case UNIT_INHG: unit_str = "inHg"; break;
        case UNIT_PSI:  unit_str = "psi"; break;
        default:        unit_str = "hPa"; break;
    }

    snprintf(dest, max_len, "P = %.2f %s\r\n", sensor->pressure + sensor->offset, unit_str);
}*/


void parse_form_string(const char *input) {
    form_item_count = 0;
    const char *p = input;

    while (*p && form_item_count < 20) {
		if (isdigit((unsigned char)p[0])) {
        	// Update the "active" format state
        	active_width = atoi(p); // Pulls out the first integer, up to . without moving pointer.
        	while (isdigit((unsigned char)*p)) p++; // Move the pointer up to the decimal.
        	if (*p == '.') {
            	p++;
            	active_precision = atoi(p); // Set the precision.
            	while (isdigit((unsigned char)*p)) p++; // Move the pointer to the next non-digit.
        	}
        	// We don't create a FormItem here; we just updated the 'rule'
    	}
    	else if (*p == '"') {
            // Handle string literals, i.e. anything in quotes.
            p++; // Skip leading quote
            int i = 0;
            while (*p && *p != '"' && i < 31) {
                compiled_form[form_item_count].literal[i++] = *p++;
            }
            compiled_form[form_item_count].literal[i] = '\0';
            compiled_form[form_item_count].type = FORM_LITERAL;
            if (*p == '"') p++; // Skip trailing quote
            form_item_count++;
        } else if (*p == '\\' || *p == '#') {
			char next = toupper((unsigned char)*(p + 1));
			if (next == 'T' || next == 'R' || next == 'N') {
            	compiled_form[form_item_count].type = FORM_LITERAL;

				if (next == 'T') {
					compiled_form[form_item_count].literal[0] = '\t';
		            compiled_form[form_item_count].literal[1] = '\0';
					p += 2;
            	}
				else if (next == 'N') {
					compiled_form[form_item_count].literal[0] = '\n';
		            compiled_form[form_item_count].literal[1] = '\0';
					p += 2;
            	}
				else if (next == 'R') {
					compiled_form[form_item_count].literal[0] = '\r';
		            compiled_form[form_item_count].literal[1] = '\0';
					next = toupper((unsigned char)*(p + 2)); // Check for RN
					if (next == 'N') {
						compiled_form[form_item_count].literal[1] = '\n';
			            compiled_form[form_item_count].literal[2] = '\0';
						p += 3;
					} else p += 2;
				}
            	form_item_count++; // Keep an eye on this, in the event \* is received.
        	} else if (1) {
				printf("We did not implement this case go to file:%s line:%d to implement\n", __FILE__, __LINE__);		// we need to implement a handler for '\205' or '\0' for data bytes with the specified value 0-255.
			}
		} else if (*p == 'U' || *p == 'u') {
			compiled_form[form_item_count].type = FORM_VAR_UNIT;
			if (isdigit((unsigned char)p[1])) {
        		compiled_form[form_item_count].width = p[1] - '0'; // Convert '3' to 3
        		p += 2; // Jump past 'U' and the digit
    		} else {
        		compiled_form[form_item_count].width = 0; // Default: no fixed width
        		p += 1; // Jump past 'U'
    		}
			form_item_count++;
		}
		else if (isgraph(*p)) {
            // Handle Variables (P, P1, ERR, etc.)
            char var_name[10];
            int i = 0;
            while (*p && !isspace(*p) && *p != '"' && *p != '\\' && i < 9) {
                var_name[i++] = *p++;
            }
            var_name[i] = '\0';
			printf("DEBUG_PRINT: What is in the VAR NAME, did it capture a \\ -> %s\n", var_name);

            if (strncmp(var_name, "P1", 2) == 0) compiled_form[form_item_count].type = FORM_VAR_P1;
            else if (strncmp(var_name, "P2", 2) == 0) compiled_form[form_item_count].type = FORM_VAR_P2;
            else if (strncmp(var_name, "P3H", 3) == 0) compiled_form[form_item_count].type = FORM_VAR_P3H;
            else if (strncmp(var_name, "P3", 2) == 0) compiled_form[form_item_count].type = FORM_VAR_P3;
            else if (strncmp(var_name, "ERR", 3) == 0) compiled_form[form_item_count].type = FORM_VAR_ERR;
            else if (strncmp(var_name, "DP12", 4) == 0) compiled_form[form_item_count].type = FORM_VAR_DP12;
            else if (strncmp(var_name, "DP13", 4) == 0) compiled_form[form_item_count].type = FORM_VAR_DP13;
            else if (strncmp(var_name, "DP23", 4) == 0) compiled_form[form_item_count].type = FORM_VAR_DP23;
            else if (strncmp(var_name, "HCP", 3) == 0) compiled_form[form_item_count].type = FORM_VAR_HCP;
            else if (strncmp(var_name, "QFE", 3) == 0) compiled_form[form_item_count].type = FORM_VAR_QFE;
            else if (strncmp(var_name, "QNH", 3) == 0) compiled_form[form_item_count].type = FORM_VAR_QNH;
            else if (strncmp(var_name, "TP1", 3) == 0) compiled_form[form_item_count].type = FORM_VAR_TP1;
            else if (strncmp(var_name, "TP2", 3) == 0) compiled_form[form_item_count].type = FORM_VAR_TP2;
            else if (strncmp(var_name, "TP3", 3) == 0) compiled_form[form_item_count].type = FORM_VAR_TP3;
            else if (strncmp(var_name, "A3H", 3) == 0) compiled_form[form_item_count].type = FORM_VAR_A3H;
            else if (strncmp(var_name, "CS2", 3) == 0) compiled_form[form_item_count].type = FORM_VAR_CS2;
            else if (strncmp(var_name, "CS4", 3) == 0) compiled_form[form_item_count].type = FORM_VAR_CS4;
            else if (strncmp(var_name, "CSX", 3) == 0) compiled_form[form_item_count].type = FORM_VAR_CSX;
            else if (strncmp(var_name, "SN", 2) == 0) compiled_form[form_item_count].type = FORM_VAR_SN;
            else if (strncmp(var_name, "PSTAB", 5) == 0) compiled_form[form_item_count].type = FORM_VAR_PSTAB;
            else if (strncmp(var_name, "ADDR", 4) == 0) compiled_form[form_item_count].type = FORM_VAR_ADDR;
            else if (strncmp(var_name, "DATE", 4) == 0) compiled_form[form_item_count].type = FORM_VAR_DATE;
            else if (strncmp(var_name, "TIME", 4) == 0) compiled_form[form_item_count].type = FORM_VAR_TIME;
            else if (strncmp(var_name, "P", 1) == 0) compiled_form[form_item_count].type = FORM_VAR_P;
            // ... and so on

			compiled_form[form_item_count].width = active_width;
        	compiled_form[form_item_count].precision = active_precision;
        	form_item_count++;
        }
        else {
            p++; // Skip spaces between tokens
        }
    }
}



// This function is called every time a measurement is requested (e.g., every 1 second)
void build_dynamic_output(ParsedMessage *p_msg, char *output_buf, size_t buf_len) {
    char *ptr = output_buf;
    size_t remaining = buf_len;
    output_buf[0] = '\0';    // Clear the buffer

	time_t t = time(NULL);
	struct tm *tm_info = localtime(&t);


    for (int i = 0; i < form_item_count; i++) {
        int written = 0;

        switch (compiled_form[i].type) {
            case FORM_LITERAL:
                // Copy the static text the user wanted (e.g., "Pressure: " or any \t \r \n chars)
                written = snprintf(ptr, remaining, "%s", compiled_form[i].literal);
                break;

            case FORM_VAR_P1:
                // Fetch the CURRENT value of P1
				if (compiled_form[i].width == 0) {
				    // 0.0 case: Revert to default sensor precision
    				written = snprintf(ptr, remaining, "%8.2f", p_msg->p1_pressure);
				} else {
    				// Custom case: Use the x.y provided by the user
    				written = snprintf(ptr, remaining, "%*.*f",
                    	   	compiled_form[i].width,
                       		compiled_form[i].precision,
                       		p_msg->p1_pressure);
				}
                break;
            case FORM_VAR_P2:
                // Fetch the CURRENT value of P2
				if (compiled_form[i].width == 0) {
				    // 0.0 case: Revert to default sensor precision
    				written = snprintf(ptr, remaining, "%8.2f", p_msg->p2_pressure);
				} else {
    				// Custom case: Use the x.y provided by the user
    				written = snprintf(ptr, remaining, "%*.*f",
                    	   	compiled_form[i].width,
                       		compiled_form[i].precision,
                       		p_msg->p2_pressure);
				}
                break;
            case FORM_VAR_P3:
                // Fetch the CURRENT value of P3
				if (compiled_form[i].width == 0) {
				    // 0.0 case: Revert to default sensor precision
    				written = snprintf(ptr, remaining, "%8.2f", p_msg->p3_pressure);
				} else {
    				// Custom case: Use the x.y provided by the user
    				written = snprintf(ptr, remaining, "%*.*f",
                    	   	compiled_form[i].width,
                       		compiled_form[i].precision,
                       		p_msg->p3_pressure);
				}
                break;

            case FORM_VAR_P:
                // Fetch the CURRENT value of P3
				if (compiled_form[i].width == 0) {
				    // 0.0 case: Revert to default sensor precision
    				written = snprintf(ptr, remaining, "%8.2f", p_msg->p_average);
				} else {
    				// Custom case: Use the x.y provided by the user
    				written = snprintf(ptr, remaining, "%*.*f",
                    	   	compiled_form[i].width,
                       		compiled_form[i].precision,
                       		p_msg->p_average);
				}
				break;
            case FORM_VAR_ERR: {
				written = snprintf(ptr, remaining, "%X%X%X",
                       p_msg->p1_sensor_error,
                       p_msg->p2_sensor_error,
                       p_msg->p3_sensor_error);
			    break;
			}
            case FORM_VAR_P3H:
                // Fetch the CURRENT 3-hour pressure trend
                written = snprintf(ptr, remaining, "%+.2f", p_msg->trend);
                break;
			case FORM_VAR_UNIT:
				const char *unit_str = "hPa"; // get_current_unit_string(sensor_one->unit_index); // e.g., "hPa"
    			int w = compiled_form[i].width;

    			if (w > 0) {
        			// Use a temporary buffer to handle the truncation/padding safely
        			char temp_unit[10];
        			// Format: Left-justified, fixed width 'w', max characters 'w'
        			snprintf(temp_unit, sizeof(temp_unit), "%-*.*s", w, w, unit_str);
        			written = snprintf(ptr, remaining, "%s", temp_unit);
    			} else {
        			// Just print the natural string
        			written = snprintf(ptr, remaining, "%s", unit_str);
    			}
				break;
			case FORM_VAR_DATE: {
    			// Format: 2026-02-09
    			written = snprintf(ptr, remaining, "%04d-%02d-%02d",
                       tm_info->tm_year + 1900,
                       tm_info->tm_mon + 1,
                       tm_info->tm_mday);
    			break;
			}
			case FORM_VAR_TIME: {
			    // Format: 13:56:55
			    written = snprintf(ptr, remaining, "%02d:%02d:%02d",
                       tm_info->tm_hour,
                       tm_info->tm_min,
                       tm_info->tm_sec);
			    break;
			}
			case FORM_VAR_DP12: { // Delta p1 - p2
				if (compiled_form[i].width == 0) {
				    // 0.0 case: Revert to default sensor precision
    				written = snprintf(ptr, remaining, "%8.2f", (p_msg->p1_pressure - p_msg->p2_pressure));
				} else {
    				// Custom case: Use the x.y provided by the user
    				written = snprintf(ptr, remaining, "%*.*f",
                    	   	compiled_form[i].width,
                       		compiled_form[i].precision,
                       		(p_msg->p1_pressure - p_msg->p2_pressure));
				}
				break;
			}
			case FORM_VAR_DP13: { // Delta p1 - p3
				if (compiled_form[i].width == 0) {
				    // 0.0 case: Revert to default sensor precision
    				written = snprintf(ptr, remaining, "%8.2f", (p_msg->p1_pressure - p_msg->p3_pressure));
				} else {
    				// Custom case: Use the x.y provided by the user
    				written = snprintf(ptr, remaining, "%*.*f",
                    	   	compiled_form[i].width,
                       		compiled_form[i].precision,
                       		(p_msg->p1_pressure - p_msg->p3_pressure));
				}
				break;
			}
			case FORM_VAR_DP23: { // Delta p2 - p3
				if (compiled_form[i].width == 0) {
				    // 0.0 case: Revert to default sensor precision
    				written = snprintf(ptr, remaining, "%8.2f", (p_msg->p2_pressure - p_msg->p3_pressure));
				} else {
    				// Custom case: Use the x.y provided by the user
    				written = snprintf(ptr, remaining, "%*.*f",
                    	   	compiled_form[i].width,
                       		compiled_form[i].precision,
                       		(p_msg->p2_pressure - p_msg->p3_pressure));
				}
				break;
			}
			case FORM_VAR_QNH: // Fall into HCP as the code is the same.
			case FORM_VAR_HCP: {
				double cor_altitude = get_hcp_pressure(p_msg->p_average, p_msg->altitude);
				if (compiled_form[i].width == 0) {
				    // 0.0 case: Revert to default sensor precision
    				written = snprintf(ptr, remaining, "%8.2lf", cor_altitude);
				} else {
    				// Custom case: Use the x.y provided by the user
    				written = snprintf(ptr, remaining, "%*.*lf",
                    	   	compiled_form[i].width,
                       		compiled_form[i].precision,
                       		cor_altitude);
				}
				break;
			}
			case FORM_VAR_QFE: {
				if (compiled_form[i].width == 0) {
				    // 0.0 case: Revert to default sensor precision
    				written = snprintf(ptr, remaining, "%8.2lf", p_msg->p_average);
				} else {
    				// Custom case: Use the x.y provided by the user
    				written = snprintf(ptr, remaining, "%*.*lf",
                    	   	compiled_form[i].width,
                       		compiled_form[i].precision,
                       		p_msg->p_average);
				}
				break;
			}
			case FORM_VAR_TP1: {
				if (compiled_form[i].width == 0) {
				    // 0.0 case: Revert to default sensor precision
    				written = snprintf(ptr, remaining, "%3.2lf", p_msg->p1_temperature);
				} else {
    				// Custom case: Use the x.y provided by the user
    				written = snprintf(ptr, remaining, "%*.*lf",
                    	   	compiled_form[i].width,
                       		compiled_form[i].precision,
                       		p_msg->p1_temperature);
				}
				break;
			}
			case FORM_VAR_TP2: {
				if (compiled_form[i].width == 0) {
				    // 0.0 case: Revert to default sensor precision
    				written = snprintf(ptr, remaining, "%3.2lf", p_msg->p2_temperature);
				} else {
    				// Custom case: Use the x.y provided by the user
    				written = snprintf(ptr, remaining, "%*.*lf",
                    	   	compiled_form[i].width,
                       		compiled_form[i].precision,
                       		p_msg->p2_temperature);
				}
				break;
			}
			case FORM_VAR_TP3: {
				if (compiled_form[i].width == 0) {
				    // 0.0 case: Revert to default sensor precision
    				written = snprintf(ptr, remaining, "%3.2lf", p_msg->p3_temperature);
				} else {
    				// Custom case: Use the x.y provided by the user
    				written = snprintf(ptr, remaining, "%*.*lf",
                    	   	compiled_form[i].width,
                       		compiled_form[i].precision,
                       		p_msg->p3_temperature);
				}
				break;
			}
			case FORM_VAR_A3H: {

				break;
			}
			case FORM_VAR_CS2: {
				// Calculate length of string built up to this exact moment
    			size_t current_len = (size_t)(ptr - output_buf);

			    // Run the sum on the existing content
			    unsigned char cs2_val = calculate_cs2(output_buf, current_len);

			    // Print the hex result into the buffer
			    written = snprintf(ptr, remaining, "%02X", cs2_val);
			    break;
			}
			case FORM_VAR_CS4: {
				// Calculate length of string built up to this exact moment
    			size_t current_len = (size_t)(ptr - output_buf);

			    // Run the sum on the existing content
			    uint16_t cs4_val = calculate_cs4(output_buf, current_len);

			    // Print the hex result into the buffer
			    written = snprintf(ptr, remaining, "%02X", cs4_val);
				break;
			}
			case FORM_VAR_CSX: {
				// Calculate length of string built up to this exact moment
    			size_t current_len = (size_t)(ptr - output_buf);

			    // Run the sum on the existing content
			    unsigned char csx_val = calculate_csx(output_buf, current_len);

			    // Print the hex result into the buffer
			    written = snprintf(ptr, remaining, "%02X", csx_val);
				break;
			}
			case FORM_VAR_PSTAB: {
			    written = snprintf(ptr, remaining, "OK");
				break;
			}
			case FORM_VAR_SN: {
			    written = snprintf(ptr, remaining, "%s", p_msg->serial_num);
				break;
			}
			case FORM_VAR_ADDR: {
			    written = snprintf(ptr, remaining, "%d", p_msg->address);
				break;
			}
        }
        // Move the pointer forward so the next item appends to the end
        if (written > 0 && (size_t)written < remaining) {
            ptr += written;
            remaining -= written;
        }
    }
}


void ptb330_format_output(ptb330_sensor *sensor, char *dest, size_t max_len) {
    char buffer[256] = {0};
    char *src = sensor->format_string;
    size_t out_idx = 0;

    while (*src && out_idx < (max_len - 1)) {
        if (*src == '#') {
            src++; // Move to the token character
            switch (*src) {
                case 'P': { // Pressure (scaled)
                    double val = get_scaled_pressure(sensor->pressure + sensor->offset, sensor->units);
                    out_idx += snprintf(&buffer[out_idx], max_len - out_idx, "%.2f", val);
                    break;
                }
                case 'U': { // Unit Label
                    out_idx += snprintf(&buffer[out_idx], max_len - out_idx, "%s", get_unit_str(sensor->units));
                    break;
                }
                case 'S': { // Serial Number
                    out_idx += snprintf(&buffer[out_idx], max_len - out_idx, "%s", sensor->serial_number);
                    break;
                }
                case 'n': { // Address/Node
                    out_idx += snprintf(&buffer[out_idx], max_len - out_idx, "%02d", sensor->address);
                    break;
                }
                default:
                    if (out_idx < max_len - 1) buffer[out_idx++] = *src;
            }
        }
        // Handle standard escape sequences used in Vaisala strings
        else if (*src == '\\') {
            src++;
            if (*src == 'r') buffer[out_idx++] = '\r';
            else if (*src == 'n') buffer[out_idx++] = '\n';
            else if (*src == 't') buffer[out_idx++] = '\t';
        }
        else {
            buffer[out_idx++] = *src;
        }
        src++;
    }
    buffer[out_idx] = '\0';
    strncpy(dest, buffer, max_len);
}
/*
void ptb330_format_output(ptb330_sensor *sensor, char *dest, size_t max_len) {
    char temp[MAX_FORM_STR] = {0};
    char *src = sensor->format_string;
    int i = 0;

    while (*src && i < (max_len - 1)) {
        if (*src == '#' && *(src + 1)) {
            src++; // Skip the '#'
            switch (*src) {
                case 'P': // Pressure
                    i += snprintf(&temp[i], max_len - i, "%.2f", sensor->pressure);
                    break;
                case 'U': // Units
                    i += snprintf(&temp[i], max_len - i, "%s", get_unit_str(sensor->units));
                    break;
                case 'S': // Serial Number
                    i += snprintf(&temp[i], max_len - i, "%s", sensor->serial_number);
                    break;
                case 't': // Current Time
                    {
                        time_t now = time(NULL);
                        struct tm *tm_info = localtime(&now);
                        i += strftime(&temp[i], max_len - i, "%H:%M:%S", tm_info);
                    }
                    break;
                default: // Handle unknown as literal
                    temp[i++] = *src;
            }
        } else if (*src == '\\' && *(src + 1)) {
            src++; // Handle escape characters
            if (*src == 'r') temp[i++] = '\r';
            else if (*src == 'n') temp[i++] = '\n';
            else if (*src == 't') temp[i++] = '\t';
        } else {
            temp[i++] = *src;
        }
        src++;
    }
    temp[i] = '\0';
    strncpy(dest, temp, max_len);
}*/


double calculate_sea_level_pressure(double station_p, double elevation_m, double temp_c) {
    double temp_k = temp_c + 273.15;
    double lapse_rate = 0.0065; // K/m
    double g = 9.80665;
    double r = 287.05;

    // The exponent part of the equation: (g / (R * L))
    double exponent = g / (r * lapse_rate);

    // The base part of the equation
    double base = 1 - (lapse_rate * elevation_m) / (temp_k + lapse_rate * elevation_m);

    return station_p / pow(base, exponent);
}


double get_hcp_pressure(double station_p, double altitude_m) {
    // Standard atmosphere constants
    const double sea_level_temp_k = 288.15; // 15 degrees C
    const double lapse_rate = 0.0065;      // K/m
    const double g = 9.80665;
    const double r = 287.05;

    // If altitude is 0, no correction needed
    if (altitude_m == 0.0f) return station_p;

    // ISA Formula: P = Ps * (1 - (L*H)/To)^(-g/RL)
    // Note: To is usually the sea-level standard temperature (288.15K)
    double exponent = g / (r * lapse_rate);
    double base = 1.0 - (lapse_rate * altitude_m) / sea_level_temp_k;

    return station_p / pow(base, exponent);
}
