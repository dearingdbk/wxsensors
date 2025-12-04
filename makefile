CC = gcc
CFLAGS = -Wall -Wextra -Iinclude

SRC_DIR = ../wxsensors
BIN_DIR = bin
COMMON_SRC = common/serial_utils.c

# Get all immediate subfolders of SRC_DIR that contain .c files
FOLDERS := $(shell find $(SRC_DIR) -mindepth 1 -maxdepth 1 -type d -exec sh -c 'ls $$0/*.c >/dev/null 2>&1 && echo $$0' {} \;)

# Strip SRC_DIR prefix to get folder names
FOLDER_NAMES := $(notdir $(FOLDERS))

# Executable paths: bin/<folder>/<folder>
EXES := $(addprefix $(BIN_DIR)/, $(addsuffix /%, $(FOLDER_NAMES)))

# Default target
all: $(EXES)

# Build one executable per folder
$(BIN_DIR)/%/%:
	@folder_name=$*; \
	src_folder=$(SRC_DIR)/$$folder_name; \
	srcs=$$(find $$src_folder -maxdepth 1 -name "*.c"); \
	if [ -z "$$srcs" ]; then \
		echo "Skipping $$folder_name, no C files"; \
	else \
		echo "Building $$folder_name ..."; \
		mkdir -p $(BIN_DIR)/$$folder_name; \
		$(CC) $(CFLAGS) $$srcs -o $(BIN_DIR)/$$folder_name/$$folder_name; \
		echo "OK: built $(BIN_DIR)/$$folder_name/$$folder_name"; \
	fi

# Clean
clean:
	rm -rf $(BIN_DIR)
