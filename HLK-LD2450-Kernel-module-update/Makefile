# Makefile for Hi-Link LD2450 radar module driver
# Purpose: Builds kernel module, Device Tree overlay, user-space apps, and documentation for Raspberry Pi
# Targets:
#   - all: Builds module, Device Tree, apps, and README
#   - module: Compiles kernel module
#   - dt: Compiles Device Tree overlay (dtbo)
#   - app: Builds user-space application (LD2450_app)
#   - test: Builds test suite (ld2450_test)
#   - readme: Copies README to build directory
#   - install: Installs module and Device Tree overlay
#   - test-driver: Tests module loading
#   - test-race: Tests race condition handling
#   - backup: Backs up existing files
#   - clean: Cleans build artifacts
#   - check-tools: Verifies required tools (gcc, dtc)
#   - version: Generates version file
# Changelog:
#   - 2.3.2: Optimized for Raspberry Pi
#   - 2.4.0: Added block device, race condition test, network driver, tracepoints, test suite

# Define kernel modules to build
obj-m += ld2450.o
ld2450-y := ld2450_core.o ld2450_serial.o ld2450_power.o ld2450_sysfs.o ld2450_input.o ld2450_debugfs.o ld2450_block.o ld2450_net.o ld2450_trace.o

# Define variables
KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
CC := gcc
DTC := dtc
APP := LD2450_app
TEST := ld2450_test
DTS_FILES := ld2450.dts
DTBO_FILES := ld2450.dtbo
VERSION := 2.4.0
CFLAGS := -g -O2 -Wall -pthread
KCFLAGS := -DCONFIG_LD2450_DEBUG=$(LD2450_DEBUG)
LD2450_DEBUG ?= 0
LDFLAGS := -pthread -lrt

MAKEFLAGS += -j$(shell nproc)

all: check-tools dt app test module readme

module:
	@echo "Building kernel module..."
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules EXTRA_CFLAGS="$(KCFLAGS)"

dt: $(DTBO_FILES)
$(DTBO_FILES): %.dtbo: %.dts
	@echo "Compiling Device Tree Overlay $<..."
	$(DTC) -@ -I dts -O dtb -o $@ $<

app: $(APP).c
	@echo "Building user application $(APP)..."
	$(CC) $(CFLAGS) -o $(APP) $(APP).c $(LDFLAGS) -lm

test: $(TEST).c
	@echo "Building test suite $(TEST)..."
	$(CC) $(CFLAGS) -o $(TEST) $(TEST).c $(LDFLAGS)

readme:
	@echo "Copying README..."
	@cp README.md $(PWD)/README.md

install: module dt
	@echo "Installing kernel module and Device Tree overlay..."
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install
	@mkdir -p /boot/overlays
	@for dtbo in $(DTBO_FILES); do \
		cp $$dtbo /boot/overlays/; \
		echo "Installed $$dtbo to /boot/overlays/"; \
	done
	@depmod -a
	@echo "Add 'dtoverlay=ld2450' to /boot/config.txt for Raspberry Pi overlay"

test-driver: install
	@echo "Testing kernel module..."
	@if lsmod | grep -q ld2450; then \
		echo "Unloading existing ld2450 module..."; \
		sudo rmmod ld2450; \
	fi
	@echo "Loading ld2450 module..."
	@sudo insmod $(PWD)/ld2450.ko
	@echo "Module loaded. Check 'dmesg' for logs."
	@echo "Run './$(APP)' or './$(TEST)' to test applications."

test-race:
	@echo "Testing race condition handling..."
	@sudo sh -c "echo 1 > /sys/devices/platform/ld2450/race_test"

backup:
	@echo "Backing up existing files..."
	@mkdir -p backup
	@for dtbo in $(DTBO_FILES); do \
		[ -f /boot/overlays/$$dtbo ] && cp /boot/overlays/$$dtbo backup/$$dtbo-$(shell date +%F-%H%M%S); \
	done
	@[ -f /lib/modules/$(shell uname -r)/kernel/drivers/misc/ld2450.ko ] && \
		cp /lib/modules/$(shell uname -r)/kernel/drivers/misc/ld2450.ko backup/ld2450.ko-$(shell date +%F-%H%M%S)

clean:
	@echo "Cleaning up..."
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -f $(DTBO_FILES) $(APP) $(TEST) README.md
	rm -f *.o *.ko *.mod *.mod.c *.symvers *.order .*.cmd
	rm -rf .tmp_versions

check-tools:
	@echo "Checking for required tools..."
	@which $(CC) > /dev/null || (echo "Error: gcc not found"; exit 1)
	@which $(DTC) > /dev/null || (echo "Error: dtc not found"; exit 1)
	@[ -d $(KERNEL_DIR) ] || (echo "Error: Kernel directory $(KERNEL_DIR) not found"; exit 1)

version:
	@echo "Generating version file..."
	@echo "$(VERSION)" > version.txt
	@echo "Driver version: $(VERSION)"

.PHONY: all module dt app test readme install test-driver test-race backup clean check-tools version