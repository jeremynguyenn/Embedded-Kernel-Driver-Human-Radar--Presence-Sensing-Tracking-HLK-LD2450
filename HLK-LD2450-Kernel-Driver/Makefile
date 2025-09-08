# Define kernel modules to build
obj-m += LD2450.o
# obj-m += LD2450_gpt.o  # Uncomment to include additional module

# Define variables
KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
CC := gcc
DTC := dtc
APP := LD2450_app
DTS_FILES := serdev_overlay.dts serdev_driver.dts
DTBO_FILES := $(DTS_FILES:.dts=.dtbo)
# Default platform: raspberry (or stm32)
PLATFORM ?= raspberry  
# Driver version, matching LD2450.c
VERSION := 2.1.0  

# Compiler flags - Cờ gỡ lỗi cho ứng dụng người dùng
CFLAGS := -g -O2 -Wall
# Cờ gỡ lỗi kernel (nếu LD2450_DEBUG được bật)
KCFLAGS := -DCONFIG_LD2450_DEBUG=$(LD2450_DEBUG)
LD2450_DEBUG ?= 0  # Debug flag for kernel module (0=off, 1=on)

# Parallel build support
MAKEFLAGS += -j$(shell nproc)

# Default target
all: check-tools dt app module

# Build kernel module
module:
	@echo "Building kernel module..."
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules EXTRA_CFLAGS="$(KCFLAGS)"

# Build Device Tree overlays
dt: $(DTBO_FILES)
$(DTBO_FILES): %.dtbo: %.dts
	@echo "Compiling Device Tree Overlay $<..."
	$(DTC) -@ -I dts -O dtb -o $@ $<

# Build user application
app: $(APP).c
	@echo "Building user application $(APP)..."
	$(CC) $(CFLAGS) -o $(APP) $(APP).c -lm

# Install kernel module and Device Tree overlays
install: module dt
	@echo "Installing kernel module and Device Tree overlays..."
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install
	@mkdir -p /boot/overlays
	@for dtbo in $(DTBO_FILES); do \
		cp $$dtbo /boot/overlays/; \
		echo "Installed $$dtbo to /boot/overlays/"; \
	done
	@depmod -a
	@echo "Add 'dtoverlay=serdev_overlay' to /boot/config.txt for Raspberry Pi"

# Test module (load/unload)
test: install
	@echo "Testing kernel module..."
	@if lsmod | grep -q LD2450; then \
		echo "Unloading existing LD2450 module..."; \
		sudo rmmod LD2450; \
	fi
	@echo "Loading LD2450 module..."
	@sudo insmod $(PWD)/LD2450.ko
	@echo "Module loaded. Check 'dmesg' for logs."
	@echo "Run './$(APP)' to test user application."

# Backup existing files before installation
backup:
	@echo "Backing up existing files..."
	@mkdir -p backup
	@for dtbo in $(DTBO_FILES); do \
		[ -f /boot/overlays/$$dtbo ] && cp /boot/overlays/$$dtbo backup/$$dtbo-$(shell date +%F-%H%M%S); \
	done
	@[ -f /lib/modules/$(shell uname -r)/kernel/drivers/misc/LD2450.ko ] && \
		cp /lib/modules/$(shell uname -r)/kernel/drivers/misc/LD2450.ko backup/LD2450.ko-$(shell date +%F-%H%M%S)

# Clean up generated files
clean:
	@echo "Cleaning up..."
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -f $(DTBO_FILES) $(APP)
	rm -f *.o *.ko *.mod *.mod.c *.symvers *.order .*.cmd
	rm -rf .tmp_versions

# Check for required tools
check-tools:
	@echo "Checking for required tools..."
	@which $(CC) > /dev/null || (echo "Error: gcc not found"; exit 1)
	@which $(DTC) > /dev/null || (echo "Error: dtc not found"; exit 1)
	@[ -d $(KERNEL_DIR) ] || (echo "Error: Kernel directory $(KERNEL_DIR) not found"; exit 1)

# Generate version file
version:
	@echo "Generating version file..."
	@echo "$(VERSION)" > version.txt
	@echo "Driver version: $(VERSION)"

# Platform-specific configuration
ifeq ($(PLATFORM),raspberry)
    DTS_FILES := serdev_overlay.dts serdev_driver.dts
else ifeq ($(PLATFORM),stm32)
    DTS_FILES := serdev_driver.dts
endif

# Phony targets to avoid conflicts
.PHONY: all module dt app install test backup clean check-tools version
