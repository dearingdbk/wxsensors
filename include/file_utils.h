/*
 * File:     file_utils.h
 * Author:   Bruce Dearing
 * Date:     13/01/2026
 * Version:  1.0
 * Purpose:  Thread-safe file reading utilities for sensor data files
 */

#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <stdio.h>
#include <pthread.h>

/*
 * Name:         get_next_line_copy
 * Purpose:      Thread-safe reading of next line from file with automatic rewind.
 *               Returns a heap-allocated copy of the line.
 * Arguments:    file_ptr - Pointer to the file to read from
 *               file_mutex - Mutex to protect file access
 *
 * Returns:      Heap-allocated string containing the line (caller must free)
 *               NULL on error or empty file
 *
 * Notes:        - Automatically rewinds to start of file on EOF
 *               - Handles pipes/streams that cannot be rewound
 *               - Strips CR/LF from end of line
 *               - Caller MUST free() the returned pointer when done
 */
char *get_next_line_copy(FILE *file_ptr, pthread_mutex_t *file_mutex);

/*
 * Name:         file_utils_cleanup
 * Purpose:      Cleanup function (currently unused, reserved for future use)
 */
void file_utils_cleanup(void);

#endif // FILE_UTILS_H
