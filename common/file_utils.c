/*
 * File:     file_utils.c
 * Author:   Bruce Dearing
 * Date:     13/01/2026
 * Version:  1.0
 * Purpose:  Thread-safe file reading utilities for sensor data files.
 *           Provides functions to read lines from files with proper thread
 *           synchronization and automatic file rewinding/cycling.
 *
 * Mods:
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include "file_utils.h"

#define MAX_LINE_LENGTH 1024

/*
 * Name:         get_next_line_copy
 * Purpose:      Reads a line from a file with thread-safe access. Automatically
 *               handles EOF by rewinding (for regular files) or waiting (for pipes).
 *               Returns a heap-allocated copy of the line.
 *
 * Arguments:    file_ptr - FILE pointer to read from (must be already opened)
 *               file_mutex - Mutex to serialize access to the file
 *
 * Output:       Error messages to stderr if file operations fail
 * Modifies:     file_ptr position (advances to next line or rewinds)
 * Returns:      Heap-allocated string containing the line (without CR/LF)
 *               NULL if file is empty, closed, or on error
 *
 * Assumptions:  - file_ptr is valid and open
 *               - file_mutex is properly initialized
 *               - Lines are shorter than MAX_LINE_LENGTH
 *
 * Bugs:         None known.
 *
 * Notes:        IMPORTANT: Caller must free() the returned pointer!
 *
 *               For regular files: Automatically rewinds to beginning on EOF
 *               For pipes/streams: Clears EOF and returns NULL (caller should retry)
 *               The function handles both:
 *               1. Regular files (can seek) - loops the data
 *               2. Pipes/streams (cannot seek) - waits for more data
 *
 *               This makes it suitable for both:
 *               - Static data files that should loop
 *               - Network streams that may temporarily run dry
 */
char *get_next_line_copy(FILE *file_ptr, pthread_mutex_t *file_mutex) {
    char temp[MAX_LINE_LENGTH];

    pthread_mutex_lock(file_mutex);
    // Validate input
    if (!file_ptr) {
        pthread_mutex_unlock(file_mutex);
        fprintf(stderr, "Error: NULL file pointer passed to get_next_line_copy\n");
        return NULL;
    }

    // Try to read a line
    if (!fgets(temp, sizeof(temp), file_ptr)) {
        // EOF or error encountered

        // Try to rewind the file (works for regular files, not pipes)
        if (fseek(file_ptr, 0, SEEK_SET) == 0) {
            // Successfully rewound - this is a regular file
            // Try reading again from the beginning
            if (!fgets(temp, sizeof(temp), file_ptr)) {
                // File is empty or error on second read
                pthread_mutex_unlock(file_mutex);
                fprintf(stderr, "Error: File is empty or unreadable\n");
                return NULL;
            }
            // Successfully read after rewind - fall through to return
        }
        else {
            // fseek failed - this is likely a pipe or non-seekable stream
            // Clear the EOF indicator so future reads can try again
            clearerr(file_ptr);
            pthread_mutex_unlock(file_mutex);

            // Log to help diagnose network/pipe issues
            fprintf(stderr, "[%ld] Stream EOF reached. Waiting for more data...\n", time(NULL));
            return NULL;
        }
    }

    pthread_mutex_unlock(file_mutex);

    // Successfully read a line - now process it

    // Strip trailing CR/LF characters safely
    // strcspn finds the first occurrence of \r or \n and returns its position
    temp[strcspn(temp, "\r\n")] = '\0';

    // Create a heap-allocated copy to return
    char *result = strdup(temp);
    if (!result) {
        fprintf(stderr, "Error: Memory allocation failed in get_next_line_copy\n");
        return NULL;
    }

    return result;
}

/*
 * Name:         file_utils_cleanup
 * Purpose:      Cleanup function for file utilities module.
 *               Currently no resources to clean up, but provided for
 *               consistency with other utility modules and future expansion.
 *
 * Arguments:    None
 * Returns:      None
 *
 * Notes:        Should be called before program exit for consistency.
 *               Future versions might add cleanup logic here.
 */
void file_utils_cleanup(void) {
    // Currently nothing to clean up
    // This function is provided for:
}
