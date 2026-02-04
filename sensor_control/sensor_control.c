/*
 * File:     sensor_control.c
 * Author:   Bruce Dearing
 * Date:     07-01-2026
 * Version:  1.0
 * Purpose:  GTK 3 GUI application to control wxsensors emulator programs.
 *           Provides start/stop buttons, status LEDs, and flag entry for each sensor.
 *
 * Compile:  gcc -o sensor_control sensor_control.c $(pkg-config --cflags --libs gtk+-3.0) -Wall -Wextra
 *
 * Usage:    ./sensor_control
 */

#include <gtk/gtk.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

/* AddressSanitizer suppression for GTK/GLib internal leaks.
 * This function is automatically called by the LeakSanitizer at startup.
 */
#if defined(__has_feature)
#  if __has_feature(address_sanitizer) || defined(__SANITIZE_ADDRESS__)
extern const char* __lsan_default_suppressions() {
    return "leak:libgtk-3.so\n"
           "leak:libglib-2.0.so\n"
           "leak:libgobject-2.0.so\n"
           "leak:libfontconfig.so\n"
           "leak:libdbus-1.so\n";
}
#endif
#endif


#define MAX_SENSORS 10
#define MAX_PATH_LEN 256
#define MAX_FLAGS_LEN 512

// Sensor definition structure
typedef struct {
    const char *name;
    const char *display_name;
    const char *default_flags;
} SensorDef;

// Runtime sensor state structure
typedef struct {
    SensorDef *def;
    GtkWidget *led_area;
    GtkWidget *flags_entry;
    GtkWidget *start_button;
    GtkWidget *stop_button;
    pid_t pid;
    gboolean running;
    guint timeout_id;
} SensorState;

// Global sensor definitions *** Make all changes and additions here, the code will handle the rest.
// {<NAME_OF_EXECUTABLE>, <GUI_SENSOR_LABEL>, <FLAGS_PROVIDED_TO_SENSOR>}
static SensorDef sensor_defs[] = {
    {"wind",        "Gill WindObserver 75",     "./data_files/wind/wind_data_P.txt /dev/ttyUSB0 9600 RS422"},
    {"rh_temp",     "Rotronic HC2A-S3",         "./data_files/rh_temp/rh_temp_data.txt /dev/ttyUSB1 9600 RS485"},
    {"pres_weather","Campbell AtmosVue30",      "./data_files/pres_weather/pres_weather.txt /dev/ttyUSB2 38400 RS485"},
    {"barometric",  "Barometric Sensor",        "./data_files/barometric/barometric_data.txt /dev/ttyUSB3 9600 RS485"},
    {"ceilometer",  "Ceilometer",               "./data_files/ceilometer/ceil_data.txt /dev/ttyUSB4 9600 RS422"},
    {"flash",       "Biral BTD-300",            "./data_files/flash/flash_data.txt /dev/ttyUSB5 9600 RS422"},
    {"ice",         "Goodrich 0872F1",          "./data_files/ice/ice_data.txt /dev/ttyUSB6 2400 RS232"},
    {"rain",        "Campbell CS700H",          "./data_files/rain/rain_data.txt /dev/ttyUSB7 1200 SDI-12"},
    {NULL, NULL, NULL}
};

// Global array of sensor states
static SensorState sensors[MAX_SENSORS];
static int num_sensors = 0;

/*
 * Name:         draw_led
 * Purpose:      Draw the status LED circle (green=running, red=stopped)
 */
static gboolean draw_led(GtkWidget *widget, cairo_t *cr, gpointer data) {
    SensorState *sensor = (SensorState *)data;
    GtkAllocation allocation;
    double radius, center_x, center_y;

    gtk_widget_get_allocation(widget, &allocation);

    center_x = allocation.width / 2.0;
    center_y = allocation.height / 2.0;
    radius = (MIN(allocation.width, allocation.height) / 2.0) - 2;

    // Draw filled circle
    cairo_arc(cr, center_x, center_y, radius, 0, 2 * G_PI);

    if (sensor->running) {
        cairo_set_source_rgb(cr, 0.2, 0.8, 0.2);  // Green
    } else {
        cairo_set_source_rgb(cr, 0.8, 0.2, 0.2);  // Red
    }
    cairo_fill_preserve(cr);

    // Draw border
    cairo_set_source_rgb(cr, 0.3, 0.3, 0.3);
    cairo_set_line_width(cr, 1.5);
    cairo_stroke(cr);

    return FALSE;
}

/*
 * Name:         check_process
 * Purpose:      Periodically check if the sensor process is still running
 */
static gboolean check_process(gpointer data) {
    SensorState *sensor = (SensorState *)data;
    int status;
    pid_t result;

    if (sensor->pid <= 0) {
        return FALSE;
    }

    result = waitpid(sensor->pid, &status, WNOHANG);

    if (result == sensor->pid) {
        // Process has exited
        sensor->running = FALSE;
        sensor->pid = 0;
        sensor->timeout_id = 0;

        gtk_widget_queue_draw(sensor->led_area);
        gtk_widget_set_sensitive(sensor->start_button, TRUE);
        gtk_widget_set_sensitive(sensor->stop_button, FALSE);
        gtk_widget_set_sensitive(sensor->flags_entry, TRUE);

        return FALSE;  // Stop the timeout
    }

    return TRUE;  // Continue checking
}

/*
 * Name:         show_error_dialog
 * Purpose:      Display an error message dialog
 */
static void show_error_dialog(GtkWidget *parent, const char *message) {
    GtkWidget *dialog;

    dialog = gtk_message_dialog_new(GTK_WINDOW(gtk_widget_get_toplevel(parent)),
                                    GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
                                    GTK_MESSAGE_ERROR,
                                    GTK_BUTTONS_OK,
                                    "%s", message);
    gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);
}

/*
 * Name:         on_start_clicked
 * Purpose:      Handle start button click - spawn the sensor process
 */
static void on_start_clicked(GtkWidget *widget, gpointer data) {
    SensorState *sensor = (SensorState *)data;
    const gchar *flags;
    char executable[MAX_PATH_LEN];
    char command[MAX_PATH_LEN + MAX_FLAGS_LEN];
    pid_t pid;

    (void)widget;

    flags = gtk_entry_get_text(GTK_ENTRY(sensor->flags_entry));

    if (strlen(flags) == 0) {
        show_error_dialog(sensor->start_button,
                         "Please enter flags (at minimum: data file path)");
        return;
    }

    // Build executable path
    snprintf(executable, sizeof(executable), "./bin/%s/%s",
             sensor->def->name, sensor->def->name);

    // Build full command for display/debug
    snprintf(command, sizeof(command), "%s %s", executable, flags);
    g_print("Starting: %s\n", command);

    pid = fork();

    if (pid == -1) {
        show_error_dialog(sensor->start_button, "Failed to fork process");
        return;
    }
    if (pid == 0) {
        // Child process
        char *args[32];
        char flags_copy[MAX_FLAGS_LEN];
        int argc = 0;
        char *token;
        char *saveptr;

        // Create new process group
        setsid();

        // Build argument array
        args[argc++] = executable;
        strncpy(flags_copy, flags, sizeof(flags_copy) - 1);
        flags_copy[sizeof(flags_copy) - 1] = '\0';
        token = strtok_r(flags_copy, " ", &saveptr);
        while (token != NULL && argc < 31) {
            args[argc++] = token;
            token = strtok_r(NULL, " ", &saveptr);
        }
        args[argc] = NULL;

        execvp(executable, args);
        // If exec fails
        perror("execvp failed");
        _exit(1);
    }
    // Parent process
    sensor->pid = pid;
    sensor->running = TRUE;

    gtk_widget_queue_draw(sensor->led_area);
    gtk_widget_set_sensitive(sensor->start_button, FALSE);
    gtk_widget_set_sensitive(sensor->stop_button, TRUE);
    gtk_widget_set_sensitive(sensor->flags_entry, FALSE);

    // Start monitoring the process
    sensor->timeout_id = g_timeout_add(1000, check_process, sensor);
}

/*
 * Name:         on_stop_clicked
 * Purpose:      Handle stop button click - terminate the sensor process
 */
static void on_stop_clicked(GtkWidget *widget, gpointer data) {
    SensorState *sensor = (SensorState *)data;

    (void)widget;

    if (sensor->pid > 0) {
        g_print("Stopping %s (PID: %d)\n", sensor->def->name, sensor->pid);
        // Send SIGTERM to process group
        kill(-sensor->pid, SIGTERM);
        // Wait for process to terminate
        waitpid(sensor->pid, NULL, 0);
        sensor->pid = 0;
    }
    if (sensor->timeout_id > 0) {
        g_source_remove(sensor->timeout_id);
        sensor->timeout_id = 0;
    }

    sensor->running = FALSE;

    gtk_widget_queue_draw(sensor->led_area);
    gtk_widget_set_sensitive(sensor->start_button, TRUE);
    gtk_widget_set_sensitive(sensor->stop_button, FALSE);
    gtk_widget_set_sensitive(sensor->flags_entry, TRUE);
}

/*
 * Name:         on_start_all_clicked
 * Purpose:      Start all sensors that are not currently running
 */
static void on_start_all_clicked(GtkWidget *widget, gpointer data) {
    int i;

    (void)widget;
    (void)data;

    for (i = 0; i < num_sensors; i++) {
        if (!sensors[i].running) {
            on_start_clicked(NULL, &sensors[i]);
        }
    }
}

/*
 * Name:         on_stop_all_clicked
 * Purpose:      Stop all running sensors
 */
static void on_stop_all_clicked(GtkWidget *widget, gpointer data) {
    int i;

    (void)widget;
    (void)data;

    for (i = 0; i < num_sensors; i++) {
        if (sensors[i].running) {
            on_stop_clicked(NULL, &sensors[i]);
        }
    }
}

/*
 * Name:         on_window_destroy
 * Purpose:      Clean up when window is closed - stop all sensors
 */
static void on_window_destroy(GtkWidget *widget, gpointer data) {
    (void)widget;
    (void)data;

    on_stop_all_clicked(NULL, NULL);
    gtk_main_quit();
}

/*
 * Name:         create_sensor_row
 * Purpose:      Create a GTK box containing all widgets for one sensor
 */
static GtkWidget *create_sensor_row(SensorState *sensor) {
    GtkWidget *hbox;
    GtkWidget *label;

    hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
    gtk_widget_set_margin_start(hbox, 10);
    gtk_widget_set_margin_end(hbox, 10);
    gtk_widget_set_margin_top(hbox, 5);
    gtk_widget_set_margin_bottom(hbox, 5);

    // Status LED
    sensor->led_area = gtk_drawing_area_new();
    gtk_widget_set_size_request(sensor->led_area, 24, 24);
    g_signal_connect(sensor->led_area, "draw", G_CALLBACK(draw_led), sensor);
    gtk_box_pack_start(GTK_BOX(hbox), sensor->led_area, FALSE, FALSE, 5);

    // Sensor name label
    label = gtk_label_new(sensor->def->display_name);
    gtk_widget_set_size_request(label, 180, -1);
    gtk_label_set_xalign(GTK_LABEL(label), 0);
    gtk_box_pack_start(GTK_BOX(hbox), label, FALSE, FALSE, 5);

    // Flags entry
    sensor->flags_entry = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(sensor->flags_entry), sensor->def->default_flags);
    gtk_entry_set_placeholder_text(GTK_ENTRY(sensor->flags_entry), 
                                   "<data_file> [port] [baud] [mode]");
    gtk_widget_set_hexpand(sensor->flags_entry, TRUE);
    gtk_box_pack_start(GTK_BOX(hbox), sensor->flags_entry, TRUE, TRUE, 5);

    // Start button
    sensor->start_button = gtk_button_new_with_label("Start");
    gtk_widget_set_size_request(sensor->start_button, 80, -1);
    g_signal_connect(sensor->start_button, "clicked", G_CALLBACK(on_start_clicked), sensor);
    gtk_box_pack_start(GTK_BOX(hbox), sensor->start_button, FALSE, FALSE, 5);

    // Stop button
    sensor->stop_button = gtk_button_new_with_label("Stop");
    gtk_widget_set_size_request(sensor->stop_button, 80, -1);
    gtk_widget_set_sensitive(sensor->stop_button, FALSE);
    g_signal_connect(sensor->stop_button, "clicked", G_CALLBACK(on_stop_clicked), sensor);
    gtk_box_pack_start(GTK_BOX(hbox), sensor->stop_button, FALSE, FALSE, 5);

    return hbox;
}

/*
 * Name:         apply_css
 * Purpose:      Apply CSS styling to the application
 */
static void apply_css(void) {
    GtkCssProvider *provider;
    const char *css =
        "button.start-btn { background: #4CAF50; color: white; }"
        "button.start-btn:hover { background: #45a049; }"
        "button.stop-btn { background: #f44336; color: white; }"
        "button.stop-btn:hover { background: #da190b; }"
        "button:disabled { opacity: 0.5; }"
        ".header-label { font-size: 18px; font-weight: bold; }"
        ".sensor-row { border-bottom: 1px solid #ddd; }";

    provider = gtk_css_provider_new();
    gtk_css_provider_load_from_data(provider, css, -1, NULL);
    gtk_style_context_add_provider_for_screen(gdk_screen_get_default(),
                                              GTK_STYLE_PROVIDER(provider),
                                              GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    g_object_unref(provider);
}

/*
 * Name:         create_main_window
 * Purpose:      Build the main application window with all widgets
 */
static GtkWidget *create_main_window(void) {
    GtkWidget *window;
    GtkWidget *main_vbox;
    GtkWidget *header_label;
    GtkWidget *separator;
    GtkWidget *header_hbox;
    GtkWidget *button_hbox;
    GtkWidget *start_all_btn;
    GtkWidget *stop_all_btn;
    GtkWidget *scrolled_window;
    GtkWidget *sensor_vbox;
    int i;

    // Create main window
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "wxsensors Control Panel");
    gtk_window_set_default_size(GTK_WINDOW(window), 950, 650);
    gtk_container_set_border_width(GTK_CONTAINER(window), 10);
    g_signal_connect(window, "destroy", G_CALLBACK(on_window_destroy), NULL);

    // Main vertical box
    main_vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);
    gtk_container_add(GTK_CONTAINER(window), main_vbox);

    // Header
    header_label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(header_label),
                         "<span size='large' weight='bold'>Weather Sensor Emulator Control</span>");
    gtk_box_pack_start(GTK_BOX(main_vbox), header_label, FALSE, FALSE, 10);

    // Separator
    separator = gtk_separator_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_box_pack_start(GTK_BOX(main_vbox), separator, FALSE, FALSE, 5);

    // Column headers
    header_hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
    gtk_widget_set_margin_start(header_hbox, 10);
    gtk_widget_set_margin_end(header_hbox, 10);

    GtkWidget *status_lbl = gtk_label_new("Status");
    gtk_widget_set_size_request(status_lbl, 34, -1);
    gtk_box_pack_start(GTK_BOX(header_hbox), status_lbl, FALSE, FALSE, 5);

    GtkWidget *sensor_lbl = gtk_label_new("Sensor");
    gtk_widget_set_size_request(sensor_lbl, 180, -1);
    gtk_label_set_xalign(GTK_LABEL(sensor_lbl), 0);
    gtk_box_pack_start(GTK_BOX(header_hbox), sensor_lbl, FALSE, FALSE, 5);

    GtkWidget *flags_lbl = gtk_label_new("Flags");
    gtk_box_pack_start(GTK_BOX(header_hbox), flags_lbl, TRUE, TRUE, 5);

    GtkWidget *controls_lbl = gtk_label_new("Controls");
    gtk_widget_set_size_request(controls_lbl, 170, -1);
    gtk_box_pack_start(GTK_BOX(header_hbox), controls_lbl, FALSE, FALSE, 5);

    gtk_box_pack_start(GTK_BOX(main_vbox), header_hbox, FALSE, FALSE, 5);

    // Scrolled window for sensor list: Note This should be sized appropriately for all sensors.
    scrolled_window = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled_window),
                                   GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    gtk_widget_set_vexpand(scrolled_window, TRUE);
    gtk_box_pack_start(GTK_BOX(main_vbox), scrolled_window, TRUE, TRUE, 5);

    // Sensor list container
    sensor_vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_container_add(GTK_CONTAINER(scrolled_window), sensor_vbox);

    // Create sensor rows
    num_sensors = 0;
    for (i = 0; sensor_defs[i].name != NULL && num_sensors < MAX_SENSORS; i++) {
        sensors[num_sensors].def = &sensor_defs[i];
        sensors[num_sensors].pid = 0;
        sensors[num_sensors].running = FALSE;
        sensors[num_sensors].timeout_id = 0;
        GtkWidget *row = create_sensor_row(&sensors[num_sensors]);
        gtk_box_pack_start(GTK_BOX(sensor_vbox), row, FALSE, FALSE, 0);

        // Add separator between rows
        if (sensor_defs[i + 1].name != NULL) {
            separator = gtk_separator_new(GTK_ORIENTATION_HORIZONTAL);
            gtk_box_pack_start(GTK_BOX(sensor_vbox), separator, FALSE, FALSE, 0);
        }
        num_sensors++;
    }

    // Bottom separator
    separator = gtk_separator_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_box_pack_start(GTK_BOX(main_vbox), separator, FALSE, FALSE, 5);

    // Bottom button bar
    button_hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
    gtk_box_pack_start(GTK_BOX(main_vbox), button_hbox, FALSE, FALSE, 5);

    // Spacer to push buttons to the right
    GtkWidget *spacer = gtk_label_new("");
    gtk_widget_set_hexpand(spacer, TRUE);
    gtk_box_pack_start(GTK_BOX(button_hbox), spacer, TRUE, TRUE, 0);

    // Start All button
    start_all_btn = gtk_button_new_with_label("Start All");
    gtk_widget_set_size_request(start_all_btn, 100, -1);
    g_signal_connect(start_all_btn, "clicked", G_CALLBACK(on_start_all_clicked), NULL);
    gtk_box_pack_start(GTK_BOX(button_hbox), start_all_btn, FALSE, FALSE, 5);

    // Stop All button
    stop_all_btn = gtk_button_new_with_label("Stop All");
    gtk_widget_set_size_request(stop_all_btn, 100, -1);
    g_signal_connect(stop_all_btn, "clicked", G_CALLBACK(on_stop_all_clicked), NULL);
    gtk_box_pack_start(GTK_BOX(button_hbox), stop_all_btn, FALSE, FALSE, 5);

    return window;
}

/*
 * Name:         main
 * Purpose:      Application entry point
 */
int main(int argc, char *argv[]) {
    GtkWidget *window;

    gtk_init(&argc, &argv);
    apply_css();
    window = create_main_window();
    gtk_widget_show_all(window);
    gtk_main();

    return 0;
}
