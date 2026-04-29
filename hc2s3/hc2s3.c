/*
 * File:        hc2s3.c
 * Author:      Bruce Dearing
 * Date:        29/04/2026
 * Version:     1.0
 * Purpose:  	Emulates a Rotronic HC2A-S3 Temperature/Relative Humidity probe over RS-485/RS-422.
 *           	The probe is housed in an Apogee TS-100 aspirated radiation shield.
 *           	This program sets up a serial connection with one thread:
 *            	- Receiver thread: parses and responds to incoming commands
 *
 *           	Supported commands (per Rotronic HygroClip2 protocol):
 *		     	Command Format { ID Adr RDD <Checksum || }> CR
 *		     	Answer Format { ID Adr RDD <Checksum || }> CR
 *           	Command: {F00RDD}
 * 	    	 	Response: {F00rdd 001; 4.45;%RH;000;=;20.07;°C;000;=;nc;---.-;°C;000; ;001;V1.7-1;0060568338;HC2-S3 ;000;4
 *           	All replies (ACKs, responses, errors) are sent out on the serial port.
 *
 *	            Data output includes: relative humidity (%), temperature (°C), status, and checksum
 *
 * Usage:	    use case ' tmp_rh_listen <file_path> // The serial port and baud rate will be set to  defaults /dev/ttyUSB0, and B9600
 *           	use case ' tmp_rh_listen <file_path> <serial_port_location> <baud_rate> <RS422|RS485> The serial port currently must match /dev/tty(S|USB)[0-9]+
 *
 * Sensor:   	Rotronic HC2A-S3 HygroClip2 Probe
 *           	- Digital temperature and relative humidity probe
 *           	- Temperature range: -40°C to +60°C (accuracy ±0.1°C)
 *           	- Humidity range: 0-100% RH (accuracy ±0.8% RH)
 *           	- Output: Analog
 *           	- Default output 0-1 Volts
 *
 * Housing:  Apogee TS-100 Aspirated Radiation Shield
 *           - Fan-aspirated design for accurate ambient readings
 *           - Minimizes solar radiation effects on temperature measurement
 *           - 12 VDC fan operation
 *
 * Mods:
 *
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <ctype.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "console_utils.h"
#include "file_utils.h"

// I2C Configuration
#define I2C_ADDR 0x60
#define I2C_BUS "/dev/i2c-1"

// MCP4728 Single Write Commands (DAC Input Register only)
#define CMD_CH_A 0x58  // Humidity
#define CMD_CH_B 0x5A  // Temperature

// Scaling Constants
#define VREF_INT 2.048
#define MAX_DAC  4095.0

#define MAX_LINE_LENGTH 1024
#define MAX_CMD_LENGTH 256
#define MAX_MSG_LENGTH 512
#define CPU_WAIT_USEC 10000


#define NS_PER_SEC 1000000000LL
#define NS_PER_MS  1000000LL
#define SEC_PER_HOUR 3600
#define DEBUG_MODE // Comment this line out to disable all debug prints

#ifdef DEBUG_MODE
    #define DEBUG_PRINT(fmt, ...) printf("DEBUG: " fmt, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...) // Becomes empty space during compilation
#endif


// Global GPIO handles

FILE *file_ptr = NULL; // Global File pointer
char *file_path = NULL; // path to file

// Shared state
volatile sig_atomic_t terminate = 0;
volatile sig_atomic_t kill_flag = 0;

const char *program_name = "unknown";

// Shared globals
volatile long long shared_interval_ns = 0; // 0 for no rain

// Synchronization primitives
static pthread_mutex_t file_mutex = PTHREAD_MUTEX_INITIALIZER; // protects file_ptr / file access
pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER; // Protects the shared variable 'shared_interval_ns'
static pthread_mutex_t pulse_sleep_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  pulse_sleep_cond; // Moved initialization down to main, to change REALTIME Clock to MONOTONIC.
pthread_mutex_t reader_sleep_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  reader_sleep_cond; // Moved initialization down to main, to change REALTIME Clock to MONOTONIC.
// Global pointers to receiver and sender threads.
pthread_t read_thread, send_thread;


/*
 * Name:         cleanup_and_exit
 * Purpose:      helper function to cleanup sensors, and arrays.
 * Arguments:    exit_code, the exit code to send on close.
 *
 * Output:       None.
 * Modifies:     Frees, sensors, sensor_map, closes file descriptors, and serial devices.
 * Returns:      None.
 * Assumptions:
 *
 * Bugs:         None known.
 * Notes:
 */
void cleanup_and_exit(int exit_code) {
    terminate = 1;
	raise(SIGTERM);
    // Wake sender
    pthread_mutex_lock(&pulse_sleep_mutex);
    pthread_cond_broadcast(&pulse_sleep_cond);
    pthread_mutex_unlock(&pulse_sleep_mutex);

    // Wake reader
    pthread_mutex_lock(&reader_sleep_mutex);
    pthread_cond_signal(&reader_sleep_cond);
    pthread_mutex_unlock(&reader_sleep_mutex);

    if (read_thread != 0) {
		pthread_join(read_thread, NULL);
		read_thread = 0;
	}
    if (send_thread != 0) {
		pthread_join(send_thread, NULL);
		send_thread = 0;
	}

    pthread_mutex_destroy(&pulse_sleep_mutex);
    pthread_mutex_destroy(&reader_sleep_mutex);
    pthread_mutex_destroy(&file_mutex);
    pthread_mutex_destroy(&data_mutex);
    pthread_cond_destroy(&pulse_sleep_cond);
    pthread_cond_destroy(&reader_sleep_cond);

    // Close resources
    if (file_ptr) fclose(file_ptr);

	// Cleanup utilities
    console_cleanup();
    exit(exit_code);
}


/**
 * Saves a voltage to the EEPROM so it persists after power loss.
 * channel: 0 (A) or 1 (B)
 */
void save_default_to_eeprom(int fd, int channel, uint16_t dac_val) {
    uint8_t buf[3];

    // Command: 01011 [Channel] 1  (The '1' at the end triggers the EEPROM write)
    buf[0] = 0x58 | (channel << 1) | 0x01;

    // Config: VREF=1 (Internal), PD=0, G=0
    buf[1] = 0x80 | ((dac_val >> 8) & 0x0F);
    buf[2] = dac_val & 0xFF;

    if (write(fd, buf, 3) != 3) {
        perror("EEPROM Write Failed");
    } else {
        printf("Channel %d default saved to EEPROM!\n", channel);
        // Important: EEPROM writes take time (~25ms).
        // Don't send another command immediately.
        usleep(30000);
    }
}


/**
 * channel: 0 for A, 1 for B, 2 for C, 3 for D
 * dac_val: 0 to 4095
 */
void write_to_specific_channel(int fd, int channel, uint16_t dac_val) {
    uint8_t buf[3];

    // Calculate the Command Byte
    // Base is 0x58 (Channel A). Each channel shifts the bit by 2.
    buf[0] = 0x58 | (channel << 1);

    // Config: VREF=1 (Internal 2.048V), PD=0 (Normal), G=0 (Gain x1)
    // Then add the top 4 bits of our 12-bit data
    buf[1] = 0x80 | ((dac_val >> 8) & 0x0F);

    // The remaining 8 bits of data
    buf[2] = dac_val & 0xFF;

    if (write(fd, buf, 3) != 3) {
        perror("I2C Write Failed");
    }
}


/**
 * Converts a target voltage (0-1.0V) to a 12-bit DAC value
 * based on the 2.048V internal reference.
 */
uint16_t volt_to_dac(double voltage) {
    if (voltage < 0.0) voltage = 0.0;
    if (voltage > 1.0) voltage = 1.0; // Safety cap at 1V
    return (uint16_t)((voltage / VREF_INT) * MAX_DAC);
}

/**
 * Sends a single channel update to the MCP4728.
 * Uses Internal VREF, Gain x1, and Power Down = Normal.
 */
void update_dac_channel(int fd, uint8_t command, uint16_t dac_val) {
    uint8_t buf[3];

    buf[0] = command;
    // Bit 7: VREF=1 (Internal), Bits 6-5: PD=0 (Normal), Bit 4: G=0 (Gain x1)
    buf[1] = 0x80 | ((dac_val >> 8) & 0x0F);
    buf[2] = dac_val & 0xFF;

    if (write(fd, buf, 3) != 3) {
        perror("I2C Write Failed");
    }
}


// ---------------- Command handling ----------------

/*
 * Name:        sleep_ms
 * Purpose:     Sleep for a given number of milliseconds using nanosleep.
 * Arguments:   ms: milliseconds to sleep.
 */
static void sleep_ns(long long ns) {
    struct timespec ts = {.tv_nsec = ns};
    nanosleep(&ts, NULL);
}

// ---------------- Threads ----------------

// Dedicated signal handling thread — replaces handle_signal
void* signal_thread(void* arg) {
    (void)arg;
    int sig;
    sigset_t wait_set;
    sigemptyset(&wait_set);
    sigaddset(&wait_set, SIGINT);
    sigaddset(&wait_set, SIGTERM);
    sigaddset(&wait_set, SIGQUIT); // Ctrl+backslash

    sigwait(&wait_set, &sig);     // Blocks until a signal arrives

    terminate = 1;
    kill_flag = 1;

    // safely wake threads
    pthread_mutex_lock(&pulse_sleep_mutex);
    pthread_cond_broadcast(&pulse_sleep_cond);
    pthread_mutex_unlock(&pulse_sleep_mutex);

    pthread_mutex_lock(&reader_sleep_mutex);
    pthread_cond_broadcast(&reader_sleep_cond);
    pthread_mutex_unlock(&reader_sleep_mutex);

    return NULL;
}


/*
 * Name:         reader_thread
 * Purpose:      Handles reading in the data file, and setting the nanosecond interval spacing that the sender_thread should fire at.
 *
 * Arguments:    arg: thread arguments.
 *
 * Output:       NIL
 * Modifies:     shared_interval_ns.
 * Returns:      NULL.
 * Assumptions:  NIL
 *
 * Bugs:         None known.
 * Notes:
 */
void* reader_thread(void* arg) {
    (void) arg;
	char *line = NULL;
    double mm_per_hour;
	unsigned long duration_sec;

    while (!terminate) {
        line = get_next_line_copy(file_ptr, &file_mutex);
        if (line) {
			char *saveptr; // Our place keeper in the string.
			char *token; // Where we temporarily store each token.
			if ((token = strtok_r(line, " ,", &saveptr))) mm_per_hour = (double)atof(token);
			if ((token = strtok_r(NULL, " ,", &saveptr))) duration_sec = (long) atol(token);

			free(line);
			line = NULL;

            long long new_interval_ns = 0;

            if (mm_per_hour > 0) {
                // (Seconds per Hour * Nanoseconds per second) / (Target mm / mm per tip)
                //double tips_per_hour = mm_per_hour / MM_PER_TIP;
                //new_interval_ns = (long long)((SEC_PER_HOUR * NS_PER_SEC) / tips_per_hour);
            } else {
                //new_interval_ns = 0; // Flag for "dry"
            }

            pthread_mutex_lock(&data_mutex);
            shared_interval_ns = new_interval_ns; // Change the shared interval
            pthread_mutex_unlock(&data_mutex);

			pthread_mutex_lock(&pulse_sleep_mutex);
            pthread_cond_broadcast(&pulse_sleep_cond); // Alert sender of change
			pthread_mutex_unlock(&pulse_sleep_mutex);

            // Wait for the duration of this weather block
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);

            long long duration_ns = (long long)(duration_sec * NS_PER_SEC);
            ts.tv_sec += (time_t)(duration_ns / NS_PER_SEC);
            ts.tv_nsec += (long)(duration_ns % NS_PER_SEC);

            // handle nsec overflow
            if (ts.tv_nsec >= NS_PER_SEC) {
                ts.tv_sec++;
                ts.tv_nsec -= NS_PER_SEC;
            }

            pthread_mutex_lock(&reader_sleep_mutex);
            if (!terminate) pthread_cond_timedwait(&reader_sleep_cond, &reader_sleep_mutex, &ts);
            pthread_mutex_unlock(&reader_sleep_mutex);
        }
    }
    return NULL;
}


/*
 * Name:         sender_thread
 * Purpose:      On continuous == 1 and assuming terminate != 1 it will get the next line from a specified file, usinf
 *               get_next_line_copy() and send that line to the serial device using safe_write_response() function every 2 seconds.
 * Arguments:    arg: thread arguments.
 *
 * Output:       Error messages if encountered, prints to serial device.
 * Modifies:     None.
 * Returns:      NULL.
 * Assumptions:  serial port will have data, and that data will translate to a command.
 *
 * Bugs:         None known.
 * Notes:
 */
void* sender_thread(void* arg) {
    (void)arg;
	struct timespec next_pulse;
    clock_gettime(CLOCK_MONOTONIC, &next_pulse);

    while (!terminate) {
        long long current_interval_ns;

        pthread_mutex_lock(&data_mutex);
        current_interval_ns = shared_interval_ns;
        pthread_mutex_unlock(&data_mutex);

        if (current_interval_ns > 0) {
//            simulate_tip(); // Pulse the pin

            // Calculate exact time for NEXT pulse using nanoseconds
            long long next_ns = (long long)next_pulse.tv_sec * NS_PER_SEC + next_pulse.tv_nsec + current_interval_ns;

            next_pulse.tv_sec = (time_t)(next_ns / NS_PER_SEC);
            next_pulse.tv_nsec = (long)(next_ns % NS_PER_SEC);

            pthread_mutex_lock(&pulse_sleep_mutex);
            if (!terminate) {
                // Timedwait wakes up exactly at the nanosecond next_pulse is due
                pthread_cond_timedwait(&pulse_sleep_cond, &pulse_sleep_mutex, &next_pulse);
            }
            pthread_mutex_unlock(&pulse_sleep_mutex);
        } else {
            // Dry period: Just wait for the Reader to signal a change
            pthread_mutex_lock(&pulse_sleep_mutex);
            if (!terminate) pthread_cond_wait(&pulse_sleep_cond, &pulse_sleep_mutex);
            pthread_mutex_unlock(&pulse_sleep_mutex);
            // Reset reference clock after a dry spell to prevent a "burst" of old pulses
            clock_gettime(CLOCK_MONOTONIC, &next_pulse);
        }
    }
    return NULL;
}


/*
 * Name:         Main
 * Purpose:      Main funstion, which opens up serial port, and creates a receiver and transmit threads to listen, and respond to commands
 *               over that serial port. Can take two arguments or no arguments. If changing the serial device name and baud rate, you must supply both.
 *               i.e. tmp_bp_listen <file_path> [serial_device] [baud_rate]
 *		 		 uses ternary statements to set either default values for SERIAL_PORT, and BAUD_RATE which are defined above.
 * 		 		 (condition) ? (value if true) : (value if false)
 *
 * Arguments:    file_path: The location of the file we want to read from, line by line.
 *               device: the string representing the file descriptor of the serial port which should
 * 				 match the pattern ^/dev/tty(S|USB)[0-9]+$. This is tested with function is_valid_tty()
 *				 baud: the string value representing the proposed baud rate, this string is sent to get_baud_rate() which returns a speed_t value.
 *
 * Output:       Prints to stderr the appropriate error messages if encountered.
 * Modifies:     None.
 * Returns:      Returns an int 0 representing success once the program closes the fd, and joins the threads, or 1 if unable to open the serial port.
 * Assumptions:  device is a valid char * pointer and the line contains
 *               characters other than white space, and points to an FD.
 *		 		 The int provided by arguments is a valid baud rate, although B9600 is set on any errors.
 *
 * Bugs:         None known.
 * Notes:
 */
int main(int argc, char *argv[]) {

int fd;

    // 1. Open the I2C Bus
    if ((fd = open(I2C_BUS, O_RDWR)) < 0) {
        perror("Failed to open I2C bus");
        return 1;
    }

    // 2. Set the I2C Slave Address
    if (ioctl(fd, I2C_SLAVE, I2C_ADDR) < 0) {
        perror("Failed to acquire bus access/talk to slave");
        return 1;
    }

    printf("Starting HC2A-S3 Simulation on MCP4728...\n");
    printf("Channel A: Humidity (0-100%% -> 0-1V)\n");
    printf("Channel B: Temperature (-40 to 60C -> 0-1V)\n");
    printf("--------------------------------------------\n");

    // Simulation loop
    while (1) {
        // --- DATA INPUT SECTION ---
        // In your final version, these values would come from your CSV generator
        double sim_rh = 55.5;    // 55.5% Relative Humidity
        double sim_temp = 12.3;  // 12.3 Degrees Celsius

        // --- CALCULATION SECTION ---
        // RH: 0% = 0V, 100% = 1V
        double rh_volts = sim_rh / 100.0;
        // Temp: -40 = 0V, 60 = 1V (Span of 100 degrees)
        double temp_volts = (sim_temp + 40.0) / 100.0;

        uint16_t rh_dac_val = volt_to_dac(rh_volts);
        uint16_t temp_dac_val = volt_to_dac(temp_volts);

        // --- I2C OUTPUT SECTION ---
        update_dac_channel(fd, CMD_CH_A, rh_dac_val);
        update_dac_channel(fd, CMD_CH_B, temp_dac_val);

        printf("OUT -> RH: %.1f%% (%.3fV) | Temp: %.1fC (%.3fV)\n", 
                sim_rh, rh_volts, sim_temp, temp_volts);

        // Wait 2 seconds before next update
        sleep(2);
    }


    if (argc < 2) {
        safe_console_error("Usage: %s <file_path> <GPIO_chip> <GPIO_pin>\n", argv[0]);
        return 1;
    }
	program_name = argv[0]; // Global variable to hold the program name for console errors.
    file_path = argv[1];

    file_ptr = fopen(file_path, "r");
    if (!file_ptr) {
        safe_console_error("Failed to open file: %s\n", strerror(errno));
		cleanup_and_exit(1);
    }
    //ternary statement to set GPIO_CHIP if supplied in args or the default
//    const char *chip_path = (argc >= 3) ? argv[2] : GPIO_CHIP;

    // ternary statement to set GPIO_OUT_PIN if supplied in args or default
  //  offset = (argc >= 4) ? atoi(argv[3]) : GPIO_PIN;


//    if (!chip) {
//        perror("gpiod_chip_open");
//		cleanup_and_exit(1);
//    }


	// Block signals in main (inherited by all threads)
	sigset_t block_set;
	sigemptyset(&block_set);
	sigaddset(&block_set, SIGINT);
	sigaddset(&block_set, SIGTERM);
	sigaddset(&block_set, SIGQUIT);
	pthread_sigmask(SIG_BLOCK, &block_set, NULL);

	// Then create signal thread
	pthread_t sig_thread;
	pthread_create(&sig_thread, NULL, signal_thread, NULL);

	// Initialize the pulse and reader conditions to use CLOCK_MONOTONIC
	pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    pthread_cond_init(&pulse_sleep_cond, &attr); // Initialize the global variable here
    pthread_cond_init(&reader_sleep_cond, &attr); // Initialize the global variable here
    pthread_condattr_destroy(&attr);

    if (pthread_create(&read_thread, NULL, reader_thread, NULL) != 0) {
        safe_console_error("Failed to create reader thread: %s\n", strerror(errno));
        terminate = 1;          // <- symmetrical, but not required
		cleanup_and_exit(1);
    }

    if (pthread_create(&send_thread, NULL, sender_thread, NULL) != 0) {
        safe_console_error("Failed to create sender thread: %s\n", strerror(errno));
        terminate = 1;          // <- needed because read_thread is running
		cleanup_and_exit(1);
    }

    safe_console_print("Press 'q' + Enter to quit.\n");
    struct pollfd fds[1];
	fds[0].fd = STDIN_FILENO;
	fds[0].events = POLLIN;
	while (!kill_flag) {
		int ret = poll(fds, 1, 500);

		if (ret == -1) {
        	if (errno == EINTR) continue; // Interrupted by signal, check kill_flag
        	safe_console_error("%s\n", strerror(errno));
			break; // Actual error
    	}
		if (ret > 0 && (fds[0].revents & (POLLIN | POLLHUP))) {
			char input[8];
	     	if (fgets(input, sizeof(input), stdin)) {
            	if (input[0] == 'q' || input[0] == 'Q' || kill_flag == 1) {
                	kill_flag = 1;
            	}
        	} else if (feof(stdin)) {  // keep an eye on the behaviour of this check.
            	kill_flag = 1;
        	}
		}
    }
    safe_console_print("Program %s terminated.\n", program_name);
	cleanup_and_exit(0);
	return 0; // We won't get here, but it quiets verbose warnings on a no return value.
}
