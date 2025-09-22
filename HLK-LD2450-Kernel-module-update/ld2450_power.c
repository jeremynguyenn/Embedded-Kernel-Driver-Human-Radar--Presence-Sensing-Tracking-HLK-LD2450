/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ld2450_power.c - Power management for Hi-Link LD2450 radar module
 * Author: Nguyen Nhan
 * Date: 22nd September 2025
 * Version: 2.4.0
 * Description: Handles power on/off, reset, and suspend/resume via GPIO on Raspberry Pi
 * Power Management:
 *   - Uses GPIO for power control
 *   - Supports runtime PM and system suspend/resume
 *   - Wakeup support with IRQ handling
 * Virtual Memory:
 *   - Demonstrates vmalloc for large buffers with OOM handling
 * Changelog:
 *   - 2.3.2: Optimized for Raspberry Pi
 *   - 2.4.0: Added wakeup IRQ, vmalloc demo, completed resume
 */

#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include "LD2450.h"

int ld2450_power_on(struct ld2450_data *data)
{
    void *vmalloc_buf;

    if (LD2450_DEBUG)
        dev_info(data->dev, "Powering on module\n");

    /* Demo vmalloc with OOM handling */
    vmalloc_buf = vmalloc(1024 * 1024); /* 1MB buffer */
    if (!vmalloc_buf) {
        dev_err(data->dev, "vmalloc failed\n");
        data->error_count++;
        return -ENOMEM;
    }
    memset(vmalloc_buf, 0, 1024 * 1024); /* Simulate usage */
    vfree(vmalloc_buf);

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
    if (ret) {
        dev_err(data->dev, "Failed to send restart command: %d\n", ret);
        data->error_count++;
        return ret;
    }

    ret = ld2450_get_ack(data, CMD_SET_RESTART, ack, &ack_len);
    if (ret) {
        dev_err(data->dev, "Failed to get restart ACK: %d\n", ret);
        data->error_count++;
        return ret;
    }

    return LD2450_SUCCESS;
}

static irqreturn_t ld2450_wakeup_irq(int irq, void *dev_id)
{
    struct ld2450_data *data = dev_id;
    dev_info(data->dev, "Wakeup IRQ triggered\n");
    pm_request_resume(data->dev);
    return IRQ_HANDLED;
}

int ld2450_suspend(struct device *dev)
{
    struct ld2450_data *data = dev_get_drvdata(dev);
    int irq;

    if (device_may_wakeup(dev) && data->wakeup_enabled) {
        irq = gpiod_to_irq(data->power_gpio);
        if (irq >= 0) {
            enable_irq_wake(irq);
            dev_info(dev, "Enabled wakeup IRQ %d\n", irq);
        } else {
            dev_err(dev, "Failed to get IRQ for wakeup: %d\n", irq);
            data->error_count++;
        }
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
    int ret, irq;

    ret = ld2450_power_on(data);
    if (ret) {
        dev_err(dev, "Failed to power on during resume: %d\n", ret);
        data->error_count++;
        return ret;
    }

    ret = serdev_device_open(data->serdev);
    if (ret) {
        dev_err(dev, "Failed to open serdev during resume: %d\n", ret);
        data->error_count++;
        return ret;
    }

    serdev_device_set_baudrate(data->serdev, LD2450_DEFAULT_BAUD_RATE);
    serdev_device_set_flow_control(data->serdev, false);
    serdev_device_set_parity(data->serdev, SERDEV_PARITY_NONE);

    ret = ld2450_setup_module(data);
    if (ret) {
        dev_err(dev, "Failed to setup module during resume: %d\n", ret);
        serdev_device_close(data->serdev);
        data->error_count++;
        return ret;
    }

    if (device_may_wakeup(dev) && data->wakeup_enabled) {
        irq = gpiod_to_irq(data->power_gpio);
        if (irq >= 0) {
            disable_irq_wake(irq);
            dev_info(dev, "Disabled wakeup IRQ %d\n", irq);
        }
    }

    dev_dbg(dev, "Resumed\n");
    return 0;
}

static const struct dev_pm_ops ld2450_pm_ops = {
    .suspend = ld2450_suspend,
    .resume = ld2450_resume,
};

int ld2450_init_module(struct ld2450_data *data)
{
    int ret, irq;

    ret = ld2450_power_on(data);
    if (ret) {
        dev_err(data->dev, "Failed to power on module: %d\n", ret);
        return ret;
    }

    ret = ld2450_setup_module(data);
    if (ret) {
        dev_err(data->dev, "Failed to setup module: %d\n", ret);
        ld2450_power_off(data);
        return ret;
    }

    if (data->wakeup_enabled) {
        irq = gpiod_to_irq(data->power_gpio);
        if (irq >= 0) {
            ret = devm_request_irq(data->dev, irq, ld2450_wakeup_irq, IRQF_TRIGGER_RISING,
                                   "ld2450_wakeup", data);
            if (ret) {
                dev_err(data->dev, "Failed to request wakeup IRQ: %d\n", ret);
                data->error_count++;
                return ret;
            }
        }
    }

    return 0;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Power management for Hi-Link LD2450 radar module on Raspberry Pi");
MODULE_VERSION("2.4.0");