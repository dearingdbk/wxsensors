/*
 * File:     ice.c
 * Author:   Bruce Dearing
 * Date:     19/11/2025
 * Version:  1.1
 * Purpose:  Emulates a Goodrich 0872F1 Ice Detector over RS-232.
 *           Threads:
 *            - Receiver thread: parses and responds to incoming commands
 *            - Tick thread (sender_thread): 1 s simulation clock. Reads one CSV row per
 *              simulated minute, accumulates ice, runs the heater / auto de-ice logic.
 *              It never transmits; the 0872 is polled, so replies come from the receiver.
 *
 *           Supported commands (per Goodrich 0872 protocol):
 *             Z1       - Send routine data (probe frequency)
 *             Z3XX     - Activate de-ice heaters for XX seconds (01-60)
 *             Z4       - Perform extended diagnostics
 *             F4       - Field calibration (recalibrate probe frequency to 40,000 Hz)
 *
 *           Output format for Z1 command:
 *             ZPXXXXXCC    - Normal operation (P = Pass)
 *             ZDXXXXXCC    - De-icing cycle active (D = De-ice)
 *             ZF1XXXXXCC   - Probe failure
 *             ZF2XXXXXCC   - Heater failure
 *             ZF3XXXXXCC   - Electronics failure
 *           Where:
 *             XXXXX = Probe frequency in Hz (averaged over one minute)
 *             CC    = Two-character checksum
 *                     (8-bit sum of every byte from STX through the last data byte, STX/CR/LF included)
 *
 *           Output format for Z3 command:   ZDOK51   ("ZDOK" + checksum 51)
 *
 *           Output format for Z4 command:   (3 data characters + checksum)
 *             ZP E3        - Sensor passes extended diagnostics  ("ZP " + E3)
 *             ZD D7        - Sensor in de-ice mode               ("ZD " + D7)
 *             ZF1EA        - Probe failure
 *             ZF2EB        - Heater failure
 *             ZF3EC        - Electronics failure
 *
 *           Probe frequency indicates ice accretion:
 *             - Normal range: 38,400 - 41,500 Hz
 *             - Calibrated nominal: 40,000 Hz
 *             - Frequency decreases as ice mass accumulates on probe
 *             - Ice thickness (mm) ~= (40000 - f) / 262.47   (0.00015 in/Hz, 130 Hz ~= 0.5 mm)
 *
 * Usage:    ice_listen <data_file> [serial_port] [baud_rate] [RS422|RS485]
 *           ice_listen /path/to/ice_data.txt                          (uses defaults: /dev/ttyUSB0, 2400, RS485)
 *           ice_listen /path/to/ice_data.txt /dev/ttyUSB1 2400 RS232
 *
 *           Serial port must match pattern: /dev/tty(S|USB)[0-9]+
 *
 *           CSV row format: RH, temp_C, precip_phase (0-4), precip_rate_mm_per_h
 *
 * Sensor:   Goodrich 0872F1 Ice Detector (formerly Rosemount)
 *           - Ultrasonic axially vibrating probe ice detector
 *           - Technology: Nickel alloy tube with 40 kHz natural resonant frequency
 *           - Ice detection sensitivity: 0.13 mm (0.005 inches) minimum
 *           - Probe frequency range: 38,400 - 41,500 Hz
 *           - Self de-icing/water shedding capability (heater up to 60 seconds)
 *           - Continuous built-in test (BIT) verifies sensor functions
 *           - Output: RS-232 or digital current loop
 *           - Default baud rate: 2400, 8N1, full duplex, asynchronous serial
 *           - Power consumption: 10W monitoring, 385W during de-ice cycle
 *
 * Mods:     1.1 - Tick thread rewritten (inverted terminate test, integer division, missing
 *                 sh_state plumbing). Z1 no longer consumes CSV rows. Z3/Z4 double checksum
 *                 fixed. Z3 duration parsed, heater implemented, F4 implemented.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <ctype.h>
#include "serial_utils.h"
#include "console_utils.h"
#include "file_utils.h"
#include "crc_utils.h"
#include "g0872f1_utils.h"

#define SERIAL_PORT "/dev/ttyUSB0"   // Adjust as needed, main has logic to take arguments for a new location
#define BAUD_RATE   B2400	     // Adjust as needed, main has logic to take arguments for a new baud rate
#define MAX_LINE_LENGTH 1024
#define MAX_CMD_LENGTH 256

// Simulation timing
#define TICK_SECONDS       1     // simulation clock period
#define ROW_PERIOD_TICKS   60    // one CSV row per simulated minute

// Model constants. TODO: label each by source (manufacturer / sibling-sensor approximation / study).
#define AUTO_DEICE_SECONDS 30    // ASSUMPTION: heater run time for an automatic cycle, verify against manual.
#define ILR_COLD           0.80  // ice-liquid ratio when wet-bulb < ILR_WB_SPLIT_C
#define ILR_WARM           0.66  // ice-liquid ratio otherwise
#define ILR_WB_SPLIT_C    (-2.8)

#define DEBUG_MODE // Comment this line out to disable all debug prints

#ifdef DEBUG_MODE
    #define DEBUG_PRINT(fmt, ...) printf("DEBUG: " fmt, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...) // Becomes empty space during compilation
#endif

FILE *file_ptr = NULL; // Global File pointer
char *file_path = NULL; // path to file

// Shared state
atomic_bool terminate = ATOMIC_VAR_INIT(false);
int serial_fd = -1;
const char *program_name = "unknown";
// This needs to be freed upon exit.
G0872F1_sensor *sensor_one = NULL; // Global pointer to struct for G0872F1 sensor.

/* Synchronization primitives */
static pthread_mutex_t file_mutex = PTHREAD_MUTEX_INITIALIZER; // protects file_ptr / file access
static pthread_mutex_t sensor_mutex = PTHREAD_MUTEX_INITIALIZER; // only guards sensor_cond
static pthread_cond_t  sensor_cond; // Initialized in main, to change REALTIME Clock to MONOTONIC.

pthread_t recv_thread, send_thread, sig_thread;

bool recv_thread_created = false;
bool send_thread_created = false;
bool sig_thread_created = false;
bool sensor_cond_init = false;

// Ice/heater/calibration state. Every field is protected by sh_state.rwlock.
SharedState sh_state = {
    .ice_accum_mm = 0.0,
    .ilr = 0.0,
    .precip_rate = 0.0,
    .baseline_hz = BASELINE_HZ,
    .is_icing = false,
    .heater_end = {0, 0},
    .rwlock = PTHREAD_RWLOCK_INITIALIZER
};


/*
 * Name:         cleanup_and_exit
 * Purpose:      helper function to cleanup sensors, and arrays.
 * Arguments:    exit_code, the exit code to send on close.
 *
 * Output:       None.
 * Modifies:     Frees, sensors, closes file descriptors, and serial devices.
 * Returns:      None.
 */
void cleanup_and_exit(int exit_code) {
    pthread_mutex_lock(&sensor_mutex);
    atomic_store(&terminate, true);
    if (sensor_cond_init) pthread_cond_broadcast(&sensor_cond);
    pthread_mutex_unlock(&sensor_mutex);

    if (recv_thread_created) {
        pthread_join(recv_thread, NULL);
        recv_thread_created = false;
    }
    if (send_thread_created) {
        pthread_join(send_thread, NULL);
        send_thread_created = false;
    }
    if (sig_thread_created) {
        pthread_cancel(sig_thread);
        pthread_join(sig_thread, NULL);
        sig_thread_created = false;
    }

    pthread_mutex_destroy(&sensor_mutex);
    pthread_mutex_destroy(&file_mutex);
    if (sensor_cond_init) pthread_cond_destroy(&sensor_cond);

    if (sensor_one) free(sensor_one);
    if (serial_fd >= 0) {
        tcflush(serial_fd, TCOFLUSH);
        close(serial_fd);
    }
    if (file_ptr) fclose(file_ptr);
    console_cleanup();
    serial_utils_cleanup();
    exit(exit_code);
}

/*
 * Name:         prepend_to_buffer
 * Purpose:      Returns a malloc'd copy of original with "STX \r \n" prepended.
 * Arguments:    original: the payload string.
 * Returns:      Heap string (caller frees), or NULL on allocation failure.
 */
char* prepend_to_buffer(const char* original) {
    size_t new_len = 3 + strlen(original) + 1;
    char* new_str = malloc(new_len);
    if (new_str == NULL) return NULL;
    snprintf(new_str, new_len, "\x02\r\n%s", original);
    return new_str;
}

/*
 * Name:         send_framed
 * Purpose:      Frames a payload as STX CR LF <payload> <CC> ETX CR LF and writes it to serial.
 * Arguments:    payload: data characters only, e.g. "ZDOK", "ZP ", "ZP40000". NO checksum.
 * Notes:        The checksum covers STX, CR and LF as well as the payload.
 */
static void send_framed(const char *payload) {
    char *msg = prepend_to_buffer(payload);
    if (msg == NULL) return;
    uint8_t crc = checksum_m256((const uint8_t *)msg, strlen(msg));
    safe_serial_write(serial_fd, "%s%02X\x03\r\n", msg, crc);
    DEBUG_PRINT("TX: %s%02X\r\n", payload, crc);
    free(msg);
}

// ---------------- Heater helpers (caller must hold sh_state.rwlock unless noted) ----------------

static bool ts_before(const struct timespec *a, const struct timespec *b) {
    return (a->tv_sec < b->tv_sec) || (a->tv_sec == b->tv_sec && a->tv_nsec < b->tv_nsec);
}

// Caller holds rwlock (read or write).
static bool heater_active_locked(const struct timespec *now) {
    return ts_before(now, &sh_state.heater_end);
}

// Takes its own lock.
static bool heater_is_active(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    pthread_rwlock_rdlock(&sh_state.rwlock);
    bool active = heater_active_locked(&now);
    pthread_rwlock_unlock(&sh_state.rwlock);
    return active;
}

// Takes its own lock. A new Z3 replaces any running cycle.
static void start_heater(unsigned seconds) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    pthread_rwlock_wrlock(&sh_state.rwlock);
    sh_state.heater_end = now;
    sh_state.heater_end.tv_sec += seconds;
    pthread_rwlock_unlock(&sh_state.rwlock);
}

// ---------------- Weather row handling ----------------

/*
 * Name:         parse_message
 * Purpose:      Parses one CSV row (RH, temp C, precip phase, precip rate mm/h) and
 *               publishes the result to sh_state.
 * Arguments:    msg: the raw row (modified by strtok_r).
 *               p_message: struct to fill in.
 * Modifies:     p_message, msg, sh_state (precip_rate, is_icing, ilr).
 * Notes:        Called only from the tick thread.
 */
void parse_message(char *msg, ParsedMessage *p_message) {
    memset(p_message, 0, sizeof(ParsedMessage));
    char *saveptr = NULL;
    char *token;

    if ((token = strtok_r(msg, ",", &saveptr))) p_message->rel_humidity = strtod(token, NULL);
    #define NEXT_T strtok_r(NULL, ",", &saveptr)
    if ((token = NEXT_T)) p_message->temperature = strtod(token, NULL);
    if ((token = NEXT_T)) p_message->msg_phase = (Precip_Phase)atoi(token); // 0=none 1=rain 2=freezing_rain 3=ice 4=snow
    if ((token = NEXT_T)) p_message->precip_rate = strtod(token, NULL);
    #undef NEXT_T

    p_message->wb_temp = wet_bulb_temp_c(p_message->temperature, p_message->rel_humidity);
    p_message->is_icing = (p_message->msg_phase == PRECIP_FREEZE) && (p_message->wb_temp <= 0.0);
    p_message->ilr = (p_message->wb_temp < ILR_WB_SPLIT_C) ? ILR_COLD : ILR_WARM;

    pthread_rwlock_wrlock(&sh_state.rwlock);
    sh_state.precip_rate = p_message->precip_rate;
    sh_state.is_icing    = p_message->is_icing;
    sh_state.ilr         = p_message->ilr;
    pthread_rwlock_unlock(&sh_state.rwlock);

    DEBUG_PRINT("ROW: RH=%.1f T=%.1fC phase=%d rate=%.2fmm/h wb=%.2fC icing=%d ilr=%.2f\n",
                p_message->rel_humidity, p_message->temperature, (int)p_message->msg_phase,
                p_message->precip_rate, p_message->wb_temp, p_message->is_icing, p_message->ilr);
}

/*
 * Name:         send_z1_response
 * Purpose:      Reply to Z1 from the current simulated state. Does NOT touch the CSV.
 */
static void send_z1_response(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    pthread_rwlock_rdlock(&sh_state.rwlock);
    double freq_hz = sh_state.baseline_hz - (sh_state.ice_accum_mm * SLOPE_HZ_PER_MM);
    bool deicing = heater_active_locked(&now);
    pthread_rwlock_unlock(&sh_state.rwlock);

    if (freq_hz < 0.0) freq_hz = 0.0;
    if (freq_hz > 99999.0) freq_hz = 99999.0;

    char payload[32];
    snprintf(payload, sizeof(payload), "Z%c%05.0f", deicing ? 'D' : 'P', freq_hz);
    send_framed(payload);
}

// ---------------- Command handling ----------------

/*
 * Name:         parse_command
 * Purpose:      Translates a received string to a command enum and fills in cmd.
 * Arguments:    buf: NUL-terminated command text. cmd: output struct.
 * Returns:      The command type (also stored in cmd->type); CMD_UNKNOWN on anything else.
 * Notes:        Z3 must be exactly Z3 + two digits, 01-60. Sets cmd->deice_seconds.
 */
CommandType parse_command(const char *buf, ParsedCommand *cmd) {
    memset(cmd, 0, sizeof(ParsedCommand));
    cmd->type = CMD_UNKNOWN;
    if (buf == NULL) return CMD_UNKNOWN;

    if (strcmp(buf, "Z1") == 0)      cmd->type = CMD_Z1;
    else if (strcmp(buf, "Z4") == 0) cmd->type = CMD_Z4;
    else if (strcmp(buf, "F4") == 0) cmd->type = CMD_F4;
    else if (buf[0] == 'Z' && buf[1] == '3' &&
             isdigit((unsigned char)buf[2]) && isdigit((unsigned char)buf[3]) && buf[4] == '\0') {
        int secs = (buf[2] - '0') * 10 + (buf[3] - '0');
        if (secs >= 1 && secs <= 60) {
            cmd->type = CMD_Z3;
            cmd->deice_seconds = (uint8_t)secs;
        }
    }
    return cmd->type;
}

/*
 * Name:         handle_command
 * Purpose:      Handle each command and send the response on serial.
 */
void handle_command(CommandType cmd, ParsedCommand *p_cmd) {
    switch (cmd) {
        case CMD_Z1:
            send_z1_response();
            break;
        case CMD_Z3:
            start_heater(p_cmd->deice_seconds);
            send_framed("ZDOK");                       // checksum 51 is appended by send_framed
            break;
        case CMD_Z4:
            send_framed(heater_is_active() ? "ZD " : "ZP "); // 3 data chars: D7 / E3
            break;
        case CMD_F4:
            // Field calibration: reset the probe baseline to nominal. TODO: check manual for a reply, if any.
            pthread_rwlock_wrlock(&sh_state.rwlock);
            sh_state.baseline_hz = BASELINE_HZ;
            pthread_rwlock_unlock(&sh_state.rwlock);
            break;
        default:
            safe_console_print("BAD CMD:%d\r\n", (int)cmd);
            break;
    }
}


// ---------------- Threads ----------------

/*
 * Name:         signal_thread
 * Purpose:      Waits for SIGINT/SIGTERM/SIGQUIT, sets terminate and wakes the tick thread.
 */
void* signal_thread(void* arg) {
    (void)arg;
    int sig;
    sigset_t wait_set;
    sigemptyset(&wait_set);
    sigaddset(&wait_set, SIGINT);
    sigaddset(&wait_set, SIGTERM);
    sigaddset(&wait_set, SIGQUIT);

    sigwait(&wait_set, &sig);

    atomic_store(&terminate, true);
    pthread_mutex_lock(&sensor_mutex);
    pthread_cond_broadcast(&sensor_cond);
    pthread_mutex_unlock(&sensor_mutex);
    return NULL;
}

/*
 * Name:         receiver_thread
 * Purpose:      Reads the serial port, assembles a command line, and dispatches it.
 * Notes:        The AERO server does not send CR/LF after G0872F1 commands, so a termios
 *               VTIME expiry (n == 0) with a non-empty buffer also ends a command.
 */
void* receiver_thread(void* arg) {
    (void)arg;
    char line[MAX_CMD_LENGTH];
    size_t len = 0;

    while (!atomic_load(&terminate)) {
        char c;
        int n = read(serial_fd, &c, 1);
        if (n > 0) {
            if (c == '\r' || c == '\n') {
                if (len > 0) {
                    line[len] = '\0';
                    ParsedCommand local_cmd;
                    CommandType cmd_type = parse_command(line, &local_cmd);
                    handle_command(cmd_type, &local_cmd);
                    len = 0;
                }
            } else if (len < sizeof(line) - 1) {
                line[len++] = c;
            } else len = 0;
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("read");
        } else {
            if (len > 0) {
                line[len] = '\0';
                ParsedCommand local_cmd;
                CommandType cmd_type = parse_command(line, &local_cmd);
                handle_command(cmd_type, &local_cmd);
                len = 0;
            } else {
                usleep(10000);
            }
        }
    }
    return NULL;
}

/*
 * Name:         sender_thread  (simulation tick)
 * Purpose:      1 s simulation clock. Every ROW_PERIOD_TICKS ticks it reads the next CSV row into
 *               sh_state. Every tick it runs the heater/ice model:
 *                 - heater on           -> probe is cleared (ice_accum_mm = 0)
 *                 - icing and heater off-> ice_accum_mm += rate/3600 * TICK * ILR
 *                 - ice_accum_mm >= DEICE_THRESHOLD_MM -> automatic de-ice cycle
 * Notes:        Uses an absolute CLOCK_MONOTONIC deadline that advances by a fixed step, so
 *               there is no drift, and it always wakes within one tick of terminate being set.
 */
void* sender_thread(void* arg) {
    (void)arg;
    struct timespec next;
    unsigned long ticks = 0;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (!atomic_load(&terminate)) {
        next.tv_sec += TICK_SECONDS;

        pthread_mutex_lock(&sensor_mutex);
        int rc = 0;
        while (!atomic_load(&terminate) && rc != ETIMEDOUT) {
            rc = pthread_cond_timedwait(&sensor_cond, &sensor_mutex, &next);
        }
        pthread_mutex_unlock(&sensor_mutex);
        if (atomic_load(&terminate)) break;

        // Weather row: file I/O with no locks held except the file mutex inside get_next_line_copy.
        if (ticks % ROW_PERIOD_TICKS == 0) {
            char *line = get_next_line_copy(file_ptr, &file_mutex);
            if (line) {
                ParsedMessage row;
                parse_message(line, &row);
                free(line);
            } else {
                safe_console_error("ERR: Empty file\r\n");
            }
        }
        ticks++;

        // Ice model
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        pthread_rwlock_wrlock(&sh_state.rwlock);
        if (heater_active_locked(&now)) {
            sh_state.ice_accum_mm = 0.0;
        } else {
            if (sh_state.is_icing) {
                sh_state.ice_accum_mm += sh_state.precip_rate * ((double)TICK_SECONDS / 3600.0) * sh_state.ilr;
            }
            if (sh_state.ice_accum_mm >= DEICE_THRESHOLD_MM) {
                sh_state.heater_end = now;
                sh_state.heater_end.tv_sec += AUTO_DEICE_SECONDS; // probe clears on next tick
            }
        }
        pthread_rwlock_unlock(&sh_state.rwlock);
    }
    return NULL;
}

/*
 * Name:         Main
 * Purpose:      Opens the serial port, then starts the signal, receiver and tick threads.
 *               ice_listen <file_path> [serial_device] [baud_rate] [RS422|RS485]
 * Returns:      0 on clean shutdown, 1 if setup failed.
 */
int main(int argc, char *argv[]) {

    if (argc < 2) {
        safe_console_error("Usage: %s <file_path> <serial_device> <baud_rate> <RS422|RS485>\n", argv[0]);
        cleanup_and_exit(1);
    }

    program_name = argv[0];
    file_path = argv[1];
    file_ptr = fopen(file_path, "r");
    if (!file_ptr) {
        safe_console_error("Failed to open file: %s\n", strerror(errno));
        cleanup_and_exit(1);
    }
    const char *device = (argc >= 3 && is_valid_tty(argv[2]) == 0) ? argv[2] : SERIAL_PORT;
    speed_t baud = (argc >= 4) ? get_baud_rate(argv[3]) : BAUD_RATE;
    SerialMode mode = (argc >= 5) ? get_mode(argv[4]) : SERIAL_RS485;

    serial_fd = open_serial_port(device, baud, mode);
    if (serial_fd < 0) {
        cleanup_and_exit(1);
    }

    if (init_G0872F1_sensor(&sensor_one) != 0) {
        safe_console_error("Failed to initialize sensor_one\n");
        cleanup_and_exit(1);
    }

    sigset_t block_set;
    sigemptyset(&block_set);
    sigaddset(&block_set, SIGINT);
    sigaddset(&block_set, SIGTERM);
    sigaddset(&block_set, SIGQUIT);
    pthread_sigmask(SIG_BLOCK, &block_set, NULL);

    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    pthread_cond_init(&sensor_cond, &attr);
    sensor_cond_init = true;
    pthread_condattr_destroy(&attr);

    if (pthread_create(&sig_thread, NULL, signal_thread, NULL) != 0) {
        safe_console_error("Failed to create signal thread: %s\n", strerror(errno));
        atomic_store(&terminate, true);
        cleanup_and_exit(1);
    } else sig_thread_created = true;

    if (pthread_create(&recv_thread, NULL, receiver_thread, NULL) != 0) {
        safe_console_error("Failed to create receiver thread: %s\n", strerror(errno));
        atomic_store(&terminate, true);
        cleanup_and_exit(1);
    } else recv_thread_created = true;

    if (pthread_create(&send_thread, NULL, sender_thread, NULL) != 0) {
        safe_console_error("Failed to create sender thread: %s\n", strerror(errno));
        atomic_store(&terminate, true);
        cleanup_and_exit(1);
    } else send_thread_created = true;

    safe_console_print("Press 'ctrl-c' to quit.\n");

    pthread_join(sig_thread, NULL);
    sig_thread_created = false;
    safe_console_print("\rProgram %s terminated.\n", program_name);
    cleanup_and_exit(0);
    return 0;
}
