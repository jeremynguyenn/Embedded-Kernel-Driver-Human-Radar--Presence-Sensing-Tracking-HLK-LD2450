/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ld2450_input.c - Input device handling for Hi-Link LD2450 radar module
 * Author: Nguyen Nhan
 * Date: 1st September 2025
 * Version: 2.3.2
 * Description: Manages input device for tracking data on Raspberry Pi
 * Input Subsystem:
 *   - Uses Linux input subsystem to report tracking data (X, Y, velocity) as absolute coordinates
 *   - Registers input device with ABS_X, ABS_Y, ABS_RX for tracking data
 *   - Accessible via /dev/input/eventX
 * Changelog:
 *   - 2.2.0: Modularized input handling
 *   - 2.3.0: Added explanatory comments
 *   - 2.3.1: Added detailed comments about input subsystem
 *   - 2.3.2: Optimized for Raspberry Pi
 */

#include <linux/input.h>
#include "LD2450.h"

int ld2450_create_input(struct ld2450_data *data)
{
    struct input_dev *input_dev;
    int error;

    input_dev = devm_input_allocate_device(data->dev);
    if (!input_dev)
        return -ENOMEM;

    input_dev->name = LD2450_DEVICE_NAME;
    input_dev->phys = "ld2450/input0";
    input_dev->id.bustype = BUS_HOST;
    input_set_abs_params(input_dev, ABS_X, -32768, 32767, 0, 0);
    input_set_abs_params(input_dev, ABS_Y, -32768, 32767, 0, 0);
    input_set_abs_params(input_dev, ABS_RX, -32768, 32767, 0, 0);
    data->input_dev = input_dev;

    error = input_register_device(input_dev);
    if (error) {
        dev_err(data->dev, "Failed to register input device: %d\n", error);
        return error;
    }

    return 0;
}

void ld2450_remove_input(struct ld2450_data *data)
{
    if (data->input_dev)
        input_unregister_device(data->input_dev);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Input device handling for Hi-Link LD2450 radar module on Raspberry Pi");
MODULE_VERSION(LD2450_VERSION);