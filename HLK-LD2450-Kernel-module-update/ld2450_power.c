/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ld2450_power.c - Power management for Hi-Link LD2450 radar module
 * Author: Nguyen Nhan
 * Date: 1st September 2025
 * Version: 2.3.2
 * Description: Handles power on/off and reset via GPIO on Raspberry Pi
 * Power Management:
 *   - Uses GPIO for power control (power-gpios in Device Tree)
 *   - Supports runtime PM (autosuspend) and system suspend/resume
 *   - Wakeup support via device_init_wakeup and wakeup_enabled flag
 * Changelog:
 *   - 2.2.0: Modularized power management
 *   - 2.3.0: Added explanatory comments
 *   - 2.3.1: Added detailed comments about power management and wakeup
 *   - 2.3.2: Optimized for Raspberry Pi
 */

#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include "LD2450.h"

int ld2450_power_on(struct ld2450_data *data)
{
    if (LD2450_DEBUG)
        dev_info(data->dev, "Powering on module\n");

    gpiod_set_value_cansleep(data->power_gpio, 1);
    data->powered = true;
    msleep(1000);
    return LD2450_SUCCESS;
}

int ld2450_power_off(struct ld2450_data *data)
{
    if (LD2450_DEBUG)
        dev_info(data->dev, "Powering off module\n");

    gpiod_set_value_cansleep(data->power_gpio, 0);
    data->powered = false;
    msleep(1000);
    return LD2450_SUCCESS;
}

int ld2450_reset_module(struct ld2450_data *data)
{
    u8 msg[16], ack[16];
    u16 msg_len, ack_len;
    int ret;

    if (LD2450_DEBUG)
        dev_info(data->dev, "Resetting module\n");

    ret = ld2450_write_cmd(data, CMD_SET_RESTART, msg, &msg_len);
    if (ret)
        return ret;

    ret = ld2450_get_ack(data, CMD_SET_RESTART, ack, &ack_len);
    if (ret)
        return ret;

    return LD2450_SUCCESS;
}

int ld2450_suspend(struct device *dev)
{
    struct ld2450_data *data = dev_get_drvdata(dev);

    if (device_may_wakeup(dev) && data->wakeup_enabled) {
        /* Enable wakeup if GPIO supports IRQ */
    }

    serdev_device_close(data->serdev);
    ld2450_power_off(data);
    flush_workqueue(data->workqueue);

    dev_dbg(dev, "Suspended\n");
    return 0;
}

int ld2450_resume(struct device *dev)
{
    struct ld2450_data *data = dev_get_drvdata(dev);
    int ret;

    ret = ld2450_power_on(data);
    if (ret)
        return ret;

    ret = serdev_device_open(data->serdev);
    if (ret) {
        dev_err(dev, "Failed to reopen serdev: %d\n", ret);
        return ret;
    }

    ret = ld2450_setup_module(data);
    if (ret)
        dev_err(dev, "Resume setup failed: %d\n", ret);

    if (device_may_wakeup(dev) && data->wakeup_enabled) {
        /* Disable wakeup IRQ */
    }

    dev_dbg(dev, "Resumed\n");
    return 0;
}

int ld2450_runtime_suspend(struct device *dev)
{
    struct ld2450_data *data = dev_get_drvdata(dev);
    return ld2450_power_off(data);
}

int ld2450_runtime_resume(struct device *dev)
{
    struct ld2450_data *data = dev_get_drvdata(dev);
    return ld2450_power_on(data);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Power management for Hi-Link LD2450 radar module on Raspberry Pi");
MODULE_VERSION(LD2450_VERSION);