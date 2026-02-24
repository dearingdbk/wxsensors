# ============================================================================
# wxsensors Makefile with Optional Sanitizers (ASan + UBSan)
# ============================================================================

# List of required packages
REQUIRED_PKGS = gtk+-3.0

# Check if packages exist
PKG_CHECK := $(shell pkg-config --exists $(REQUIRED_PKGS) || echo "missing")
ifeq ($(PKG_CHECK), missing)
$(error "Error: Required libraries not found. Please install: $(REQUIRED_PKGS)")
endif

# ============================================================================
# BUILD MODE CONFIGURATION
# ============================================================================

# Default: Sanitizers ENABLED (safe for development)
# To disable: make sanitize=0
sanitize ?= 1

ifeq ($(sanitize),1)
    # AddressSanitizer + UndefinedBehaviorSanitizer
    SANITIZE_FLAGS = -fsanitize=address,undefined -fno-omit-frame-pointer
    $(info >>> Building with ASan + UBSan ENABLED (sanitize=1))
else
    # Production/performance build (no sanitizers)
    SANITIZE_FLAGS =
    $(info >>> Building WITHOUT sanitizers (sanitize=0))
endif

# ============================================================================
# COMPILER CONFIGURATION
# ============================================================================

CC = gcc

# Base flags (always applied)
CFLAGS_BASE = -Wall -Wextra -Iinclude -MMD -MP -ffunction-sections -fdata-sections

# Debug symbols (always enabled)
DEBUG_FLAGS = -g

# Combine all C flags
CFLAGS = $(CFLAGS_BASE) $(SANITIZE_FLAGS) $(DEBUG_FLAGS)

# Linker flags
LDFLAGS_BASE = -Wl,--gc-sections -lm
LDFLAGS = $(LDFLAGS_BASE) $(SANITIZE_FLAGS)

# Directories (single output location)
SRC_DIR = ../wxsensors
BIN_DIR = bin
OBJ_DIR = obj

# GTK flags
GTK_CFLAGS = $(shell pkg-config --cflags gtk+-3.0)
GTK_LIBS = $(shell pkg-config --libs gtk+-3.0)

# ============================================================================
# SOURCE FILES
# ============================================================================

# Common sources and objects
COMMON_SRC = $(wildcard common/*.c)
COMMON_OBJ = $(patsubst common/%.c, $(OBJ_DIR)/%.o, $(COMMON_SRC))

# Get all immediate subfolders of SRC_DIR that contain .c files
FOLDERS := $(shell find $(SRC_DIR) -mindepth 1 -maxdepth 1 -type d ! -name 'common' ! -name 'sensor_control' ! -name 'cldn_parser' -exec sh -c 'ls $$0/*.c >/dev/null 2>&1 && echo $$0' {} \;)
FOLDER_NAMES := $(notdir $(FOLDERS))
EXES := $(addprefix $(BIN_DIR)/, $(addsuffix /%, $(FOLDER_NAMES)))

# ============================================================================
# BUILD TARGETS
# ============================================================================

.PHONY: all full gui clean sanitize nosanitize

# Default target - build sensors only
all: $(EXES)

# Convenience targets
sanitize:
	$(MAKE) sanitize=1 all

nosanitize:
	$(MAKE) sanitize=0 all

# Build everything including GUI
full: all gui

# Full build variants
full-sanitize:
	$(MAKE) sanitize=1 full

full-nosanitize:
	$(MAKE) sanitize=0 full

# ============================================================================
# COMPILATION RULES
# ============================================================================

# Build common object files
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
	$(CC) $(CFLAGS) $(GTK_CFLAGS) $< -o $@ $(GTK_LIBS) $(LDFLAGS)
	@echo "OK: built $(BIN_DIR)/sensor_control"

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# Clean all build artifacts
clean:
	rm -rf $(BIN_DIR) $(OBJ_DIR)

# ============================================================================
# DEBUGGING HELPERS
# ============================================================================

# Show current configuration
info:
	@echo "Build configuration:"
	@echo "  Sanitizers enabled: $(sanitize)"
	@echo "  CFLAGS: $(CFLAGS)"
	@echo "  LDFLAGS: $(LDFLAGS)"
	@echo "  BIN_DIR: $(BIN_DIR)"
	@echo "  OBJ_DIR: $(OBJ_DIR)"
	@echo "  FOLDER_NAMES: $(FOLDER_NAMES)"

# Run with sanitizer options preset (for testing)
test-wind: $(BIN_DIR)/wind/wind
	@echo "Running wind sensor with sanitizer options..."
	ASAN_OPTIONS=detect_leaks=1:print_stats=1:verbosity=1 \
	UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
	./$(BIN_DIR)/wind/wind data_files/wind_sample.txt 2>&1 | head -100
