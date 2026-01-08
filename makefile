CC = gcc
CFLAGS = -Wall -Wextra -Iinclude -MMD -MP -ffunction-sections -fdata-sections
LDFLAGS = -Wl,--gc-sections
SRC_DIR = ../wxsensors
BIN_DIR = bin
OBJ_DIR = obj

# GTK flags
GTK_CFLAGS = $(shell pkg-config --cflags gtk+-3.0)
GTK_LIBS = $(shell pkg-config --libs gtk+-3.0)

# Common sources and objects
COMMON_SRC = $(wildcard common/*.c)
COMMON_OBJ = $(patsubst common/%.c, $(OBJ_DIR)/%.o, $(COMMON_SRC))

# Get all immediate subfolders of SRC_DIR that contain .c files, excluding common and sensor_control
FOLDERS := $(shell find $(SRC_DIR) -mindepth 1 -maxdepth 1 -type d ! -name 'common' ! -name 'sensor_control' ! -name 'cldn_parser' -exec sh -c 'ls $$0/*.c >/dev/null 2>&1 && echo $$0' {} \;)
FOLDER_NAMES := $(notdir $(FOLDERS))
EXES := $(addprefix $(BIN_DIR)/, $(addsuffix /%, $(FOLDER_NAMES)))

# Default target - build sensors only
all: $(EXES)

# Build everything including GUI
full: all gui

# Build common object files first
$(OBJ_DIR)/%.o: common/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

# Build one executable per folder, linking common objects
$(BIN_DIR)/%/%: $(COMMON_OBJ)
	@folder_name=$*; \
	src_folder=$(SRC_DIR)/$$folder_name; \
	srcs=$$(find $$src_folder -maxdepth 1 -name "*.c"); \
	if [ -z "$$srcs" ]; then \
		echo "Skipping $$folder_name, no C files"; \
	else \
		echo "Building $$folder_name ..."; \
		mkdir -p $(BIN_DIR)/$$folder_name; \
		$(CC) $(CFLAGS) $$srcs $(COMMON_OBJ) $(LDFLAGS) -o $(BIN_DIR)/$$folder_name/$$folder_name; \
		echo "OK: built $(BIN_DIR)/$$folder_name/$$folder_name"; \
	fi

# Build GTK GUI control panel
gui: $(BIN_DIR)/sensor_control

$(BIN_DIR)/sensor_control: $(SRC_DIR)/sensor_control/sensor_control.c | $(BIN_DIR)
	$(CC) $(CFLAGS) $(GTK_CFLAGS) $< -o $@ $(GTK_LIBS)
	@echo "OK: built $(BIN_DIR)/sensor_control"

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

clean:
	rm -rf $(BIN_DIR) $(OBJ_DIR)

.PHONY: all full gui clean
