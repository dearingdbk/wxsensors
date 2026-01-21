#ifndef CONSOLE_UTILS_H
#define CONSOLE_UTILS_H

// Thread-safe console output functions
void safe_console_print(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void safe_console_error(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Optional: Initialize/destroy if needed (though mutex can be static)
void console_init(void);
void console_cleanup(void);

#endif // CONSOLE_UTILS_H
