/**
 * @file        fan_rpm_sim.c
 * @author:     Bruce Dearing
 * Date:        24/07/2026
 * Version:     1.0
 * Purpose:     Simulates a PC-style fan tachometer signal using a Raspberry Pi 5.
 * 				The program emulates the open-collector tach pulses of a physical
 * 				fan by generating momentary active pulses on a GPIO pin, driving
 * 				an NPN transistor that sinks the tach line (same shield topology
 * 				used for the CS700H reed-switch emulation).
 *
 * Logic:       Uses a dual-thread architecture:
 * 				- Sender Thread: Executes GPIO toggling using libgpiod, utilizing
 * 				nanosecond-base absolute timing to prevent clock drift. Pulse
 * 				rate is derived from the current simulated RPM.
 * 				- Input Thread: Puts the terminal into raw mode and polls stdin
 * 				for Up/Down arrow key presses, adjusting RPM within
 * 				[MIN_RPM, max_rpm] and re-drawing a single, in-place status line.
 *
 * Hardware:    Raspberry Pi 5 (using /dev/gpiochip4)
 * 				Default BCM Pin: 27 (Physical Pin 13)
 *
 * Wiring:      GPIO_OUT_PIN (BCM 27) ───┐
 * 										 │ [NPN transistor]
 * 										GND ─────────────────────┘
 * 				GPIO drives the transistor base; the collector sinks the
 * 				tach line low for the pulse duration, mimicking an
 * 				open-collector fan tach pulse (active = pulse present).
 *
 * Fan Specs:
 * 				- Pulses per revolution: 2 (typical 3/4-wire PC fan tach; PULSES_PER_REV).
 * 				- Pulse Width: 200 us (must stay well under the pulse period at max RPM).
 * 				- Timing: Nanosecond precision via CLOCK_MONOTONIC.
 * 				- Starting RPM: 3000. Adjustable in RPM_STEP increments between
 * 				MIN_RPM and max_rpm (default MAX_RPM_DEFAULT, overridable via argv).
 *
 * Usage:       ./fan_rpm_sim [gpio_chip] [gpio_pin] [max_rpm]
 * 				Example: ./fan_rpm_sim /dev/gpiochip4 17 6000
 * 				Controls: Up arrow = +RPM_STEP, Down arrow = -RPM_STEP, Ctrl+C = quit.
 *
 * Build:       Requires libgpiod and pthread. (Handled by the supplied make file.)
 * 				gcc -o fan_rpm_sim fan_rpm_sim.c -lgpiod -lpthread
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

#define GPIO_CHIP           "/dev/gpiochip4"   // Pi 4 and earlier gpiochip0, Pi 5 gpiochip4
#define GPIO_PIN            27                  // BCM pin 5, physical pin 13

#define PULSES_PER_REV      2                   // Typical 3/4-wire PC fan tach output
#define PULSE_WIDTH_NS      200000LL            // 200us active pulse width

#define DEFAULT_RPM         3000                // Starting RPM
#define MIN_RPM             0                  // Floor - 0 means "stalled" (no pulses)
#define MAX_RPM_DEFAULT     6000                // Upper limit unless overridden via argv[3]
#define RPM_STEP            100                // RPM change per arrow keypress

#define NS_PER_SEC          1000000000LL
#define INPUT_POLL_MS        150                // How often input_thread rechecks 'terminate'
#define ESC_SEQ_TIMEOUT_MS    50                // Wait for the rest of an escape sequence

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

// Shared state
volatile sig_atomic_t terminate = 0;

const char *program_name = "unknown";

// RPM / timing state
int current_rpm = DEFAULT_RPM;      // Only ever written by input_thread
int max_rpm = MAX_RPM_DEFAULT;      // Set once in main() before threads start
volatile long long shared_interval_ns = 0; // ns between pulses, 0 == stalled/no pulses

// Terminal state
static struct termios orig_termios;
static bool termios_saved = false;

// Synchronization primitives
pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;        // Protects shared_interval_ns
static pthread_mutex_t pulse_sleep_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  pulse_sleep_cond;  // Initialized in main() against CLOCK_MONOTONIC

// Global pointers to threads
pthread_t input_thread_id, send_thread, sig_thread;

bool input_thread_created = false;
bool send_thread_created = false;
bool sig_thread_created = false;
bool pulse_cond_init = false;

/*
 * Name:         restore_terminal
 * Purpose:      Restores the terminal to its original (pre-raw-mode) settings.
 * Arguments:    None.
 * Output:       None.
 * Modifies:     STDIN termios settings.
 * Returns:      None.
 * Assumptions:  orig_termios was populated by enable_raw_mode() before this is called.
 * Bugs:         None known.
 * Notes:
 */
void restore_terminal(void) {
    if (termios_saved) {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
        termios_saved = false;
    }
}

/*
 * Name:         cleanup_and_exit
 * Purpose:      helper function to cleanup GPIO resources, threads, and the terminal.
 * Arguments:    exit_code, the exit code to send on close.
 * Output:       None.
 * Modifies:     Frees GPIO handles, joins threads, restores terminal state.
 * Returns:      None.
 * Assumptions:
 * Bugs:         None known.
 * Notes:
 */
void cleanup_and_exit(int exit_code) {
    terminate = 1;

    // Wake sender
    pthread_mutex_lock(&pulse_sleep_mutex);
    if (pulse_cond_init) pthread_cond_broadcast(&pulse_sleep_cond);
    pthread_mutex_unlock(&pulse_sleep_mutex);

    if (send_thread_created) {
        pthread_join(send_thread, NULL);
        send_thread_created = false;
    }

    if (input_thread_created) {
        pthread_join(input_thread_id, NULL); // input_thread polls with a timeout, so it will exit on its own
        input_thread_created = false;
    }

    if (sig_thread_created) {
        pthread_cancel(sig_thread);
        pthread_join(sig_thread, NULL);
        sig_thread_created = false;
    }

    pthread_mutex_destroy(&pulse_sleep_mutex);
    pthread_mutex_destroy(&data_mutex);
    if (pulse_cond_init) pthread_cond_destroy(&pulse_sleep_cond);

    // Close GPIO resources
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

    restore_terminal();
    printf("\n"); // Leave the status line intact and drop the shell prompt below it
    console_cleanup();
    exit(exit_code);
}


// ---------------- Helpers ----------------

/*
 * Name:        sleep_ns
 * Purpose:     Sleep for a given number of nanoseconds using nanosleep.
 * Arguments:   ns: nanoseconds to sleep.
 */
static void sleep_ns(long long ns) {
    struct timespec ts = {.tv_nsec = ns};
    nanosleep(&ts, NULL);
}

/*
 * Name:        simulate_pulse
 * Purpose:     Drives the output pin active for PULSE_WIDTH_NS then returns
 *              inactive — simulating one fan tach pulse via the NPN transistor
 *              sinking the tach line.
 * Arguments:   None.
 */
static void simulate_pulse(void) {
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

/*
 * Name:         compute_interval_ns
 * Purpose:      Converts a target RPM into the nanosecond interval between
 *               tach pulses, given PULSES_PER_REV.
 * Arguments:    rpm: target RPM (0 == stalled).
 * Output:       None.
 * Modifies:     None.
 * Returns:      Interval in nanoseconds, or 0 if rpm <= 0 (no pulses).
 * Assumptions:  rpm is non-negative.
 * Bugs:         None known.
 * Notes:
 */
static long long compute_interval_ns(int rpm) {
    if (rpm <= 0) return 0;
    double pulses_per_sec = ((double)rpm / 60.0) * PULSES_PER_REV;
    return (long long)(NS_PER_SEC / pulses_per_sec);
}

/*
 * Name:         print_rpm
 * Purpose:      Redraws the single-line RPM status display in place, without
 *               emitting a newline, so repeated calls overwrite the same line.
 * Arguments:    rpm: the RPM value to display.
 * Output:       Writes to stdout.
 * Modifies:     Cursor position on the current terminal line.
 * Returns:      None.
 * Assumptions:  Called from a terminal that supports \r and the ANSI
 *               "erase to end of line" sequence (ESC[K).
 * Bugs:         None known.
 * Notes:
 */
static void print_rpm(int rpm) {
    printf("\r\033[KFan RPM: %d  (Up/Down to adjust, range %d-%d, Ctrl+C to quit)",
           rpm, MIN_RPM, max_rpm);
    fflush(stdout);
}


// ---------------- Threads ----------------

// Dedicated signal handling thread — mirrors cs700h.c's approach
void* signal_thread(void* arg) {
    (void)arg;
    int sig;
    sigset_t wait_set;
    sigemptyset(&wait_set);
    sigaddset(&wait_set, SIGINT);
    sigaddset(&wait_set, SIGTERM);
    sigaddset(&wait_set, SIGQUIT);

    sigwait(&wait_set, &sig);     // Blocks until a signal arrives

    terminate = 1;

    pthread_mutex_lock(&pulse_sleep_mutex);
    pthread_cond_broadcast(&pulse_sleep_cond);
    pthread_mutex_unlock(&pulse_sleep_mutex);

    return NULL;
}

/*
 * Name:         sender_thread
 * Purpose:      Emits fan tach pulses on the GPIO pin at the interval set by
 *               shared_interval_ns, using absolute CLOCK_MONOTONIC timing so
 *               the rate holds steady even as pulse handling jitters slightly.
 *               When shared_interval_ns == 0 (RPM at floor), it idles until
 *               signaled by the input thread.
 * Arguments:    arg: thread arguments (unused).
 * Output:       None.
 * Modifies:     GPIO pin state.
 * Returns:      NULL.
 * Assumptions:  None.
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
            simulate_pulse();

            long long next_ns = (long long)next_pulse.tv_sec * NS_PER_SEC + next_pulse.tv_nsec + current_interval_ns;
            next_pulse.tv_sec = (time_t)(next_ns / NS_PER_SEC);
            next_pulse.tv_nsec = (long)(next_ns % NS_PER_SEC);

            pthread_mutex_lock(&pulse_sleep_mutex);
            if (!terminate) {
                // Wakes exactly at next_pulse, OR immediately if the input
                // thread changes the RPM (broadcast), so a step-change takes
                // effect on the very next pulse rather than after a stale wait.
                pthread_cond_timedwait(&pulse_sleep_cond, &pulse_sleep_mutex, &next_pulse);
            }
            pthread_mutex_unlock(&pulse_sleep_mutex);
        } else {
            // Stalled: wait for the input thread to bring RPM above 0
            pthread_mutex_lock(&pulse_sleep_mutex);
            if (!terminate) pthread_cond_wait(&pulse_sleep_cond, &pulse_sleep_mutex);
            pthread_mutex_unlock(&pulse_sleep_mutex);
            // Reset reference clock so we don't burst out stale pulses
            clock_gettime(CLOCK_MONOTONIC, &next_pulse);
        }
    }
    return NULL;
}

typedef enum { KEY_NONE, KEY_UP, KEY_DOWN, KEY_OTHER } arrow_key_t;

/*
 * Name:         read_key
 * Purpose:      Reads one key event from stdin, decoding the ESC '[' 'A'/'B'
 *               escape sequences used for the Up/Down arrow keys.
 * Arguments:    None.
 * Output:       None.
 * Modifies:     Consumes bytes from STDIN.
 * Returns:      KEY_UP, KEY_DOWN, KEY_OTHER, or KEY_NONE (nothing usable read).
 * Assumptions:  Called only after poll() reports STDIN is readable, and the
 *               terminal is in raw (non-canonical, non-echo) mode.
 * Bugs:         None known.
 * Notes:        A lone ESC (e.g. the user pressing Esc by itself) will time
 *               out waiting for '[' and is reported as KEY_OTHER.
 */
static arrow_key_t read_key(void) {
    unsigned char c;
    if (read(STDIN_FILENO, &c, 1) != 1) return KEY_NONE;

    if (c != 0x1B) return KEY_OTHER; // Not an escape sequence

    struct pollfd pfd = { .fd = STDIN_FILENO, .events = POLLIN };
    unsigned char seq0, seq1;

    if (poll(&pfd, 1, ESC_SEQ_TIMEOUT_MS) <= 0) return KEY_OTHER; // Lone ESC
    if (read(STDIN_FILENO, &seq0, 1) != 1) return KEY_NONE;
    if (seq0 != '[') return KEY_OTHER;

    if (poll(&pfd, 1, ESC_SEQ_TIMEOUT_MS) <= 0) return KEY_OTHER;
    if (read(STDIN_FILENO, &seq1, 1) != 1) return KEY_NONE;

    if (seq1 == 'A') return KEY_UP;
    if (seq1 == 'B') return KEY_DOWN;
    return KEY_OTHER;
}

/*
 * Name:         input_thread
 * Purpose:      Polls stdin for Up/Down arrow keypresses, adjusts current_rpm
 *               within [MIN_RPM, max_rpm] by RPM_STEP, pushes the recalculated
 *               interval to the sender thread, and redraws the status line.
 * Arguments:    arg: thread arguments (unused).
 * Output:       Writes the status line to stdout via print_rpm().
 * Modifies:     current_rpm, shared_interval_ns.
 * Returns:      NULL.
 * Assumptions:  Terminal has already been put into raw mode by main().
 * Bugs:         None known.
 * Notes:        Uses a bounded poll() timeout (INPUT_POLL_MS) purely so the
 *               thread wakes up periodically to check 'terminate' and can be
 *               joined promptly on shutdown.
 */
void* input_thread(void* arg) {
    (void)arg;
    struct pollfd pfd = { .fd = STDIN_FILENO, .events = POLLIN };

    print_rpm(current_rpm); // Initial draw

    while (!terminate) {
        int ret = poll(&pfd, 1, INPUT_POLL_MS);
        if (ret <= 0 || !(pfd.revents & POLLIN)) continue; // Timeout: recheck terminate

        arrow_key_t key = read_key();
        if (key != KEY_UP && key != KEY_DOWN) continue;

        int new_rpm = current_rpm + (key == KEY_UP ? RPM_STEP : -RPM_STEP);
        if (new_rpm > max_rpm) new_rpm = max_rpm;
        if (new_rpm < MIN_RPM) new_rpm = MIN_RPM;

        if (new_rpm == current_rpm) continue; // Already at a bound

        current_rpm = new_rpm;
        long long new_interval = compute_interval_ns(current_rpm);

        pthread_mutex_lock(&data_mutex);
        shared_interval_ns = new_interval;
        pthread_mutex_unlock(&data_mutex);

        pthread_mutex_lock(&pulse_sleep_mutex);
        pthread_cond_broadcast(&pulse_sleep_cond); // Wake sender so the new rate applies immediately
        pthread_mutex_unlock(&pulse_sleep_mutex);

        print_rpm(current_rpm);
    }
    return NULL;
}


/*
 * Name:         enable_raw_mode
 * Purpose:      Puts STDIN into raw mode: disables canonical (line-buffered)
 *               input and local echo so arrow-key escape sequences can be
 *               read byte-by-byte without the user needing to press Enter,
 *               and without the escape bytes being echoed to the screen.
 *               ISIG is deliberately left enabled so Ctrl+C / Ctrl+\ still
 *               raise SIGINT/SIGQUIT for signal_thread to catch.
 * Arguments:    None.
 * Output:       None.
 * Modifies:     STDIN termios settings; orig_termios/termios_saved globals.
 * Returns:      0 on success, -1 on failure.
 * Assumptions:  STDIN is an interactive terminal (isatty).
 * Bugs:         None known.
 * Notes:
 */
static int enable_raw_mode(void) {
    if (!isatty(STDIN_FILENO)) {
        safe_console_error("stdin is not a terminal; arrow-key input is unavailable.\n");
        return -1;
    }
    if (tcgetattr(STDIN_FILENO, &orig_termios) == -1) {
        safe_console_error("tcgetattr failed: %s\n", strerror(errno));
        return -1;
    }
    termios_saved = true;

    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO); // No line buffering, no local echo. ISIG stays on.
    raw.c_cc[VMIN] = 0;              // Non-blocking-style reads...
    raw.c_cc[VTIME] = 0;             // ...paired with poll() in input_thread.

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == -1) {
        safe_console_error("tcsetattr failed: %s\n", strerror(errno));
        termios_saved = false;
        return -1;
    }
    return 0;
}


/*
 * Name:         Main
 * Purpose:      Opens the GPIO line, starts the signal/sender/input threads,
 *               and blocks until a termination signal is received.
 * Arguments:    gpio_chip (optional): GPIO chip device path. Default GPIO_CHIP.
 *               gpio_pin  (optional): BCM pin offset. Default GPIO_PIN.
 *               max_rpm   (optional): upper RPM bound. Default MAX_RPM_DEFAULT.
 * Output:       Prints an in-place RPM status line; error messages to stderr.
 * Modifies:     GPIO pin state, terminal mode.
 * Returns:      0 on clean shutdown, 1 on setup failure.
 * Assumptions:  gpio_pin/max_rpm, if supplied, are valid integers.
 * Bugs:         None known.
 * Notes:
 */
int main(int argc, char *argv[]) {

	if (argc < 2) {
        safe_console_error("Usage: %s <file_path> <GPIO_chip> <GPIO_pin>\n", argv[0]);
        return 1;
    }

    program_name = argv[0];

    const char *chip_path = (argc >= 2) ? argv[1] : GPIO_CHIP;
    offset = (argc >= 3) ? (unsigned int)atoi(argv[2]) : GPIO_PIN;
    max_rpm = (argc >= 4) ? atoi(argv[3]) : MAX_RPM_DEFAULT;

    if (max_rpm < MIN_RPM) {
        safe_console_error("max_rpm must be >= %d\n", MIN_RPM);
        return 1;
    }
    if (current_rpm > max_rpm) current_rpm = max_rpm;

    // Sanity check: the pulse must fully complete within the tightest (max RPM) period.
    long long tightest_interval_ns = compute_interval_ns(max_rpm);
    if (tightest_interval_ns > 0 && PULSE_WIDTH_NS >= tightest_interval_ns) {
        safe_console_error(
            "PULSE_WIDTH_NS (%lld ns) does not fit inside the pulse period at max_rpm=%d "
            "(%lld ns). Lower max_rpm, reduce PULSE_WIDTH_NS, or reduce PULSES_PER_REV.\n",
            PULSE_WIDTH_NS, max_rpm, tightest_interval_ns);
        return 1;
    }

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

    req_ret = gpiod_line_request_output(gpio_line, "fan_rpm_sim", 0); // 0 = initial low
    if (req_ret) { perror("request output"); cleanup_and_exit(1); }
#endif

    if (enable_raw_mode() != 0) {
        cleanup_and_exit(1);
    }

    // Block signals in main (inherited by all threads)
    sigset_t block_set;
    sigemptyset(&block_set);
    sigaddset(&block_set, SIGINT);
    sigaddset(&block_set, SIGTERM);
    sigaddset(&block_set, SIGQUIT);
    pthread_sigmask(SIG_BLOCK, &block_set, NULL);

    // Initialize the pulse condition to use CLOCK_MONOTONIC
    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    pthread_cond_init(&pulse_sleep_cond, &attr);
    pulse_cond_init = true;
    pthread_condattr_destroy(&attr);

    // Set the initial pulse interval before any thread reads it
    shared_interval_ns = compute_interval_ns(current_rpm);

    if (pthread_create(&sig_thread, NULL, signal_thread, NULL) != 0) {
        safe_console_error("Failed to create signal thread: %s\n", strerror(errno));
        terminate = 1;
        cleanup_and_exit(1);
    } else sig_thread_created = true;

    if (pthread_create(&send_thread, NULL, sender_thread, NULL) != 0) {
        safe_console_error("Failed to create sender thread: %s\n", strerror(errno));
        terminate = 1;
        cleanup_and_exit(1);
    } else send_thread_created = true;

    if (pthread_create(&input_thread_id, NULL, input_thread, NULL) != 0) {
        safe_console_error("Failed to create input thread: %s\n", strerror(errno));
        terminate = 1;
        cleanup_and_exit(1);
    } else input_thread_created = true;

    pthread_join(sig_thread, NULL);
    cleanup_and_exit(0);
    return 0; // Unreachable, quiets no-return-value warnings
}
