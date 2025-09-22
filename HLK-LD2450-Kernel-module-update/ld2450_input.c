/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ld2450_input.c - Input subsystem for Hi-Link LD2450 radar module
 * Author: Nguyen Nhan
 * Date: 22nd September 2025
 * Version: 2.4.0
 * Description: Registers input device for tracking data on Raspberry Pi
 * Input Events:
 *   - ABS_X, ABS_Y: Absolute position coordinates
 *   - ABS_SPEED: Velocity
 *   - ABS_DISTANCE: Distance
 *   - REL_X, REL_Y: Relative motion (new)
 * Changelog:
 *   - 2.3.2: Optimized for Raspberry Pi
 *   - 2.4.0: Added relative motion events, enhanced error handling
 */

#include <linux/input.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include "LD2450.h"

int ld2450_create_input(struct ld2450_data *data)
{
    struct input_dev *input_dev;
    int ret;

    input_dev = devm_input_allocate_device(data->dev);
    if (!input_dev) {
        dev_err(data->dev, "Failed to allocate input device\n");
        data->error_count++;
        return -ENOMEM;
    }

    data->input_dev = input_dev;
    input_dev->name = LD2450_DEVICE_NAME;
    input_dev->id.bustype = BUS_HOST;
    input_dev->dev.parent = data->dev;

    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(EV_REL, input_dev->evbit); /* Added for relative motion */
    __set_bit(ABS_X, input_dev->absbit);
    __set_bit(ABS_Y, input_dev->absbit);
    __set_bit(ABS_SPEED, input_dev->absbit);
    __set_bit(ABS_DISTANCE, input_dev->absbit);
    __set_bit(REL_X, input_dev->relbit);
    __set_bit(REL_Y, input_dev->relbit);

    input_set_abs_params(input_dev, ABS_X, -32768, 32767, 0, 0);
    input_set_abs_params(input_dev, ABS_Y, -32768, 32767, 0, 0);
    input_set_abs_params(input_dev, ABS_SPEED, -32768, 32767, 0, 0);
    input_set_abs_params(input_dev, ABS_DISTANCE, 0, 65535, 0, 0);

    ret = input_register_device(input_dev);
    if (ret) {
        dev_err(data->dev, "Failed to register input device: %d\n", ret);
        data->error_count++;
        return ret;
    }

    return 0;
}

void ld2450_remove_input(struct ld2450_data *data)
{
    if (data->input_dev) {
        input_unregister_device(data->input_dev);
        data->input_dev = NULL;
    }
}

static void ld2450_report_relative_motion(struct ld2450_data *data)
{
    static s16 prev_x, prev_y;
    s16 rel_x = data->x_pos - prev_x;
    s16 rel_y = data->y_pos - prev_y;

    input_report_rel(data->input_dev, REL_X, rel_x);
    input_report_rel(data->input_dev, REL_Y, rel_y);
    input_sync(data->input_dev);

    prev_x = data->x_pos;
    prev_y = data->y_pos;
}

void ld2450_tracking_work(struct work_struct *work)
{
    struct ld2450_data *data = container_of(work, struct ld2450_data, tracking_work);
    u8 buffer[LD2450_FRAME_SIZE];
    size_t len;

    while (kfifo_len(&data->fifo) >= LD2450_FRAME_SIZE) {
        len = kfifo_out(&data->fifo, buffer, LD2450_FRAME_SIZE);
        if (len != LD2450_FRAME_SIZE)
            continue;

        if (ld2450_parse_tracking_frame(buffer, data) == LD2450_SUCCESS) {
            if (down_trylock(&data->data_sem)) {
                dev_err(data->dev, "Semaphore contention detected\n");
                data->error_count++;
                continue;
            }
            input_report_abs(data->input_dev, ABS_X, data->x_pos);
            input_report_abs(data->input_dev, ABS_Y, data->y_pos);
            input_report_abs(data->input_dev, ABS_SPEED, data->velocity);
            input_report_abs(data->input_dev, ABS_DISTANCE, data->distance);
            ld2450_report_relative_motion(data);
            input_sync(data->input_dev);
            up(&data->data_sem);
            complete(&data->data_ready);
        } else {
            data->error_count++;
        }
    }
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Input subsystem for Hi-Link LD2450 radar module on Raspberry Pi");
MODULE_VERSION("2.4.0");