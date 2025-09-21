# LD2450 Radar Module Driver

<img width="500" height="300" alt="image" src="https://github.com/user-attachments/assets/7173d589-83c8-4ce1-a64b-1fbb72111f8d" />
<img width="225" height="225" alt="image" src="https://github.com/user-attachments/assets/03454da8-fded-42ca-87dc-85a3640dd0f3" />


This project provides an enhanced Linux kernel driver for the Hi-Link LD2450 radar module, along with supporting Device Tree files, a user application, and a comprehensive `Makefile` for building and deployment. The driver supports UART communication via `serdev`, power management, and advanced features like workqueues, debugfs, sysfs, and ioctls.

## Project Components
<img width="244" height="206" alt="image" src="https://github.com/user-attachments/assets/d30c8725-130e-4171-a738-0ef40bae2ba3" />

1. **Kernel Driver (LD2450.c, LD2450.h)**:
   - **Purpose**: Controls the LD2450 radar module via UART, processing tracking data (x, y, velocity, distance) and reporting via input events.
   - **Improvements**:
     - Added **workqueues** for asynchronous tracking data processing.
     - Implemented **Runtime PM** for power efficiency.
     - Added **debugfs** for raw data access and debugging.
     - Introduced **sysfs** attributes for real-time tracking data and module info (firmware version, MAC address).
     - Supported custom **ioctls** for mode switching and data retrieval.
     - Enhanced error handling and multi-device support readiness.

2. **Device Tree Files**:
   - **`serdev_driver.dts`**: Defines the LD2450 device for UART0 with power GPIO (GPIO18), baud rate (256000), and wake-up support. Compatible with Raspberry Pi (`brcm,bcm2835`) or STM32 (`st,stm32`).
   - **`serdev_overlay.dts`**: A Device Tree Overlay for Raspberry Pi, enabling LD2450 on UART0 with the same configuration as `serdev_driver.dts`.
   - **Improvements**:
     - Updated `compatible` to `"hilink,ld2450"` to match the driver.
     - Added `power-gpios`, `baud-rate`, and `wakeup-source` properties.

3. **Makefile**:
   - **Purpose**: Builds the kernel module (`LD2450.ko`), user application (`LD2450_app`), and Device Tree overlays (`serdev_overlay.dtbo`, `serdev_driver.dtbo`).
   - **Improvements**:
     - Added support for multiple platforms (Raspberry Pi, STM32) via `PLATFORM` variable.
     - Enabled parallel compilation with `MAKEFLAGS += -j$(nproc)`.
     - Introduced `test`, `backup`, and `version` targets for easier testing and deployment.
     - Enhanced tool checking and user feedback.

4. **User Application (LD2450_app.c)**:
   - A simple application to interact with the driver, reading tracking data via ioctls or device files.

## Build and Installation

### Prerequisites
- Linux kernel source (`/lib/modules/$(uname -r)/build`)
- `gcc` and `dtc` (Device Tree Compiler)
- Raspberry Pi or STM32 hardware with LD2450 module connected via UART

### Flowchart Structure
```
[Start]
   |
   v
[Probe Function (ld2450_probe)]
   | Initialize data structures (ld2450_data, mutex, kfifo, input_dev)
   | Allocate GPIO (power-gpios)
   | Open serdev (UART)
   | Enable Runtime PM
   | Call ld2450_init_module
   |
   v
[Initialize Module (ld2450_init_module)]
   | Power on (ld2450_power_on via GPIO)
   | Configure module (ld2450_setup_module)
   |   - Enter command mode (CMD_SET_ENABLE_CONFIG)
   |   - Set baud rate (256000)
   |   - Get firmware version
   |   - Get MAC address
   |   - Disable Bluetooth
   |   - Set single-target tracking
   |   - Exit command mode (reset)
   | Create sysfs attributes
   | Create debugfs entries
   | Initialize workqueue
   |
   v
[UART Data Reception (ld2450_recv)]
   | Receive UART data into kfifo
   | If mode == LD2450_MODE_TRACKING:
   |   Check for valid tracking frame (0xAAFF0300...0x55CC)
   |   Queue tracking_work if valid
   | Else:
   |   Store data in kfifo for command mode
   | Signal data_ready completion
   |
   v
[Process Tracking Data (ld2450_tracking_work)]
   | Acquire Runtime PM
   | Read frame from kfifo
   | If valid frame:
   |   Parse frame (ld2450_parse_tracking_frame)
   |   Update x_pos, y_pos, velocity, distance
   |   Report to input_dev (ABS_X, ABS_Y, ABS_RX)
   | Release Runtime PM
   |
   v
[User-Space Interaction]
   | Read tracking data via:
   |   - sysfs (/sys/.../tracking_data)
   |   - ioctl (LD2450_IOC_GET_TRACKING)
   |   - debugfs (/sys/kernel/debug/ld2450/raw_data)
   | Set mode via ioctl (LD2450_IOC_SET_MODE)
   |
   v
[Power Management]
   | If idle for 5s (LD2450_RPM_AUTOSUSPEND_DELAY):
   |   Runtime suspend (ld2450_runtime_suspend)
   |     - Power off (ld2450_power_off via GPIO)
   | On activity:
   |   Runtime resume (ld2450_runtime_resume)
   |     - Power on (ld2450_power_on)
   | System suspend/resume:
   |   - ld2450_suspend: Close serdev, power off
   |   - ld2450_resume: Power on, reopen serdev, reconfigure
   |
   v
[End]
```

### Build Commands
```bash
# Build everything (kernel module, Device Tree overlays, user app)
make

# Build with debug enabled
make LD2450_DEBUG=1

# Build for STM32 platform
make PLATFORM=stm32

# Install kernel module and Device Tree overlays
sudo make install

# Test the kernel module
sudo make test

# Clean up generated files
make clean
```

### Deployment
- For Raspberry Pi:
  - Copy `serdev_overlay.dtbo` to `/boot/overlays/`.
  - Add `dtoverlay=serdev_overlay` to `/boot/config.txt`.
- For STM32:
  - Use `serdev_driver.dtbo` or integrate `serdev_driver.dts` into the main Device Tree.
- Run `./LD2450_app` to test the user application.

## Key Features
- **Asynchronous Processing**: Uses workqueues to handle tracking data, reducing latency.
- **Power Management**: Supports Runtime PM and wake-up functionality.
- **Debugging**: Provides debugfs (`/sys/kernel/debug/ld2450/raw_data`) for raw frame data.
- **User Interface**: Sysfs attributes (`/sys/devices/.../fw_version`, `mac_addr`, `tracking_data`) and ioctls for user-space interaction.
- **Error Handling**: Detailed error reporting and retry mechanisms for UART communication.

## Directory Structure
```
├── LD2450.c           # Kernel driver source
├── LD2450.h           # Kernel driver header
├── LD2450_app.c       # User application source
├── serdev_driver.dts  # Device Tree for LD2450
├── serdev_overlay.dts # Device Tree Overlay for Raspberry Pi
├── Makefile           # Build script
└── README.md          # This file
```

## Notes
- GPIO18 is used as the default power GPIO; adjust in `serdev_driver.dts` or `serdev_overlay.dts` based on your hardware.
- The driver is compatible with Linux kernel versions supporting `serdev` and has been tested on Raspberry Pi.
- For STM32, update the `compatible` string and GPIO pin in `serdev_driver.dts`.

## License
- Licensed under GPL-2.0-or-later.
