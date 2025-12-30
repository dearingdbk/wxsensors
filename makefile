CC = gcc
CFLAGS = -Wall -Wextra -Iinclude -MMD -MP
SRC_DIR = ../wxsensors
BIN_DIR = bin
OBJ_DIR = obj

# Common sources and objects
COMMON_SRC = $(wildcard common/*.c)
COMMON_OBJ = $(patsubst common/%.c, $(OBJ_DIR)/%.o, $(COMMON_SRC))

# Get all immediate subfolders of SRC_DIR that contain .c files, excluding common
FOLDERS := $(shell find $(SRC_DIR) -mindepth 1 -maxdepth 1 -type d ! -name 'common' -exec sh -c 'ls $$0/*.c >/dev/null 2>&1 && echo $$0' {} \;)
FOLDER_NAMES := $(notdir $(FOLDERS))
EXES := $(addprefix $(BIN_DIR)/, $(addsuffix /%, $(FOLDER_NAMES)))

# Default target
all: $(EXES)

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
		$(CC) $(CFLAGS) $$srcs $(COMMON_OBJ) -o $(BIN_DIR)/$$folder_name/$$folder_name; \
		echo "OK: built $(BIN_DIR)/$$folder_name/$$folder_name"; \
	fi

clean:
	rm -rf $(BIN_DIR) $(OBJ_DIR)
