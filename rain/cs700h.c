;/*
 * File:        cs700h.c
 * Author:      Bruce Dearing
 * Date:        22/04/2026
 * Version:     1.0
 * Purpose:     Simulates a CS700H Tipping Bucket Rain Gauge using a Raspberry Pi 5.
 * 				The program emulates the reed switch behavior of a physical gauge
 * 				by generating momentary active-low pulses on a GPIO pin.
 *
 * Logic:       Uses a dual-thread architecture:
 * 				- Reader Thread: Parses stochastic weather data from a CSV/text file
 * 				(mm/hr, duration) and calculates high-precision pulse intervals.
 * 				- Sender Thread: Executes GPIO toggling using libgpiod v2, utilizing
 * 				nanosecond-base absolute timing to prevent clock drift.
 *
 * Hardware:    Raspberry Pi 5 (using /dev/gpiochip4)
 * 				Default BCM Pin: 17 (Physical Pin 11)
 *
 * Wiring:      GPIO_OUT_PIN (BCM 17) ───┐
 * 										 │ [Optoisolator / NPN MOSFET]
 * 										GND ─────────────────────┘
 * 				This setup simulates the closure of the CS700.
 *
 * Sensor Specs:
 * 				- Resolution: 0.254 mm (0.01 inch) per tip.
 * 				- Pulse Width: 50 ms (emulating reed switch closure).
 * 				- Timing: Nanosecond precision via CLOCK_MONOTONIC.
 *
 * Usage:       ./cs700h <file_path> [gpio_chip] [gpio_pin]
 * 				Example: ./cs700h rain_data.txt /dev/gpiochip4 17
 *
 * Build:       Requires libgpiod v2 and pthread. (Handled by the supplied make file.)
 * 				gcc -o cs700h cs700h.c -lgpiod -lpthread
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
#include <gpiod.h>
#include "console_utils.h"
#include "file_utils.h"


#define MAX_LINE_LENGTH 1024
#define MAX_CMD_LENGTH 256
#define MAX_MSG_LENGTH 512
#define CPU_WAIT_USEC 10000

#define GPIO_CHIP		"/dev/gpiochip4"  // Pi 4 and earlier gpiochip0, Pi 5 gpiochip4
#define GPIO_PIN    	17                // BCM pin 17, physical pin 11
#define MM_PER_TIP      0.254             // CS700: 0.01 inch = 0.254mm per tip
#define PULSE_WIDTH_NS  50000000LL        // Reed switch closure duration in nanoseconds

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
struct gpiod_chip *chip = NULL;
#ifdef GPIOD_V2
    struct gpiod_line_settings *settings = NULL;
    struct gpiod_line_config   *line_cfg = NULL;
    struct gpiod_line_request  *request  = NULL;
#else
    struct gpiod_line *gpio_line = NULL;  // v1 uses a single gpio_line handle
#endif
unsigned int offset = GPIO_PIN;
int req_ret;

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

    // Wake sender
    pthread_mutex_lock(&pulse_sleep_mutex);
    pthread_cond_broadcast(&pulse_sleep_cond);
    pthread_mutex_unlock(&pulse_sleep_mutex);

    // Wake reader
    pthread_mutex_lock(&reader_sleep_mutex);
    pthread_cond_signal(&reader_sleep_cond);
    pthread_mutex_unlock(&reader_sleep_mutex);

    if (read_thread != 0) { pthread_join(read_thread, NULL); read_thread = 0; }
    if (send_thread != 0) { pthread_join(send_thread, NULL); send_thread = 0; }

    pthread_mutex_destroy(&pulse_sleep_mutex);
    pthread_mutex_destroy(&reader_sleep_mutex);
    pthread_mutex_destroy(&file_mutex);
    pthread_mutex_destroy(&data_mutex);
    pthread_cond_destroy(&pulse_sleep_cond);
    pthread_cond_destroy(&reader_sleep_cond);

    // Close resources
    if (file_ptr) fclose(file_ptr);

#ifdef GPIOD_V2
    if (request != NULL) {
		gpiod_line_request_release(request);
		request = NULL;
	}
    if (line_cfg != NULL) {
		gpiod_line_config_free(line_cfg);
		line_cfg = NULL;
	}
    if (settings) {
		gpiod_line_settings_free(settings);
		settings = NULL;
	}
#else
    if (gpio_line) {
		gpiod_line_release(gpio_line);
		gpio_line = NULL;
	}
#endif
	if (chip) { gpiod_chip_close(chip); chip = NULL; }

	// Cleanup utilities
    console_cleanup();
    exit(exit_code);
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

/*
 * Name:        simulate_tip
 * Purpose:     Drives the output pin low for PULSE_WIDTH_MS then returns
 *              high — simulating one reed switch closure (one bucket tip).
 * Arguments:   line: the open gpiod line to drive.
 */
static void simulate_tip() {
#ifdef GPIOD_V2
    gpiod_line_request_set_value(request, offset, GPIOD_LINE_VALUE_ACTIVE);
    sleep_ns(PULSE_WIDTH_NS);
    gpiod_line_request_set_value(request, offset, GPIOD_LINE_VALUE_INACTIVE);
#else
    gpiod_line_set_value(gpio_line, 1);
    sleep_ns(PULSE_WIDTH_NS);
    gpiod_line_set_value(gpio_line, 0);
#endif
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

            long long new_interval_ns;

            if (mm_per_hour > 0) {
                // (Seconds per Hour * Nanoseconds per second) / (Target mm / mm per tip)
                double tips_per_hour = mm_per_hour / MM_PER_TIP;
                new_interval_ns = (long long)((SEC_PER_HOUR * NS_PER_SEC) / tips_per_hour);
            } else {
                new_interval_ns = 0; // Flag for "dry"
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
            simulate_tip(); // Pulse the pin

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
    const char *chip_path = (argc >= 3) ? argv[2] : GPIO_CHIP;

    // ternary statement to set GPIO_OUT_PIN if supplied in args or default
    offset = (argc >= 4) ? atoi(argv[3]) : GPIO_PIN;

    chip = gpiod_chip_open(chip_path);

    if (!chip) {
        perror("gpiod_chip_open");
		cleanup_and_exit(1);
    }

#ifdef GPIOD_V2
    settings = gpiod_line_settings_new();
    if (!settings) { perror("settings"); cleanup_and_exit(1); }
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

    line_cfg = gpiod_line_config_new();
    if (!line_cfg) { perror("line_cfg"); cleanup_and_exit(1); }

    req_ret = gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings);
    if (req_ret) { perror("add line settings"); cleanup_and_exit(1); }

    request = gpiod_chip_request_lines(chip, NULL, line_cfg);
    if (!request) { perror("request lines"); cleanup_and_exit(1); }
#else
    gpio_line = gpiod_chip_get_line(chip, offset);
    if (!gpio_line) { perror("get line"); cleanup_and_exit(1); }

    req_ret = gpiod_line_request_output(gpio_line, "cs700h", 0); // 0 = initial low
    if (req_ret) { perror("request output"); cleanup_and_exit(1); }
#endif

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
