#include <stdio.h>
#include <stdarg.h>
#include <pthread.h>
#include "console_utils.h"

static pthread_mutex_t console_mutex = PTHREAD_MUTEX_INITIALIZER;

void safe_console_print(const char *fmt, ...) {
    va_list args;
    pthread_mutex_lock(&console_mutex);
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    fflush(stdout);
    pthread_mutex_unlock(&console_mutex);
}

void safe_console_error(const char *fmt, ...) {
    va_list args;
    pthread_mutex_lock(&console_mutex);
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fflush(stderr);
    pthread_mutex_unlock(&console_mutex);
}

void console_init(void) {
    // Currently nothing needed, but useful for future extensions
}

void console_cleanup(void) {
    pthread_mutex_destroy(&console_mutex);
}
