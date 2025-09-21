/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ld2450_core.c - Core driver for Hi-Link LD2450 radar module
 * Author: Nguyen Nhan
 * Date: 1st September 2025
 * Version: 2.3.3
 * Description: Core probe, remove, and initialization logic for Raspberry Pi
 * Workflow:
 *   - Probe: Allocates resources, sets up serdev, GPIO, input device, sysfs, debugfs
 *   - File Operations: Provides /dev/ld2450 interface for user-space (blocking and non-blocking)
 *   - Power Management: Handles suspend/resume and runtime PM
 *   - Serial Communication: Manages UART via serdev
 *   - IPC: Uses POSIX message queue for tracking data
 * File Operations:
 *   - open: Initializes file access, non-blocking, limits concurrent opens
 *   - read: Reads tracking data, protected by spinlock for atomicity
 *   - write: Sends commands to device, uses mutex for blocking synchronization
 *   - poll: Supports asynchronous I/O by waiting for new data
 * Changelog:
 *   - 2.2.0: Modularized driver, added POSIX message queue
 *   - 2.3.0: Added module_param, full file operations, spinlock
 *   - 2.3.1: Added retry logic, poll, string module param, blocking/non-blocking comments
 *   - 2.3.2: Removed STM32 support, optimized for Raspberry Pi
 *   - 2.3.3: Added poll() implementation, enhanced release(), added open counter
 */

#include <linux/module.h>
#include <linux/serdev.h>
#include <linux/of_device.h>
#include <linux/miscdevice.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include "LD2450.h"

#define LD2450_VERSION "2.3.3"
#define LD2450_MAX_OPENS 5 /* Maximum concurrent file opens */

static int ld2450_baud_rate = LD2450_DEFAULT_BAUD_RATE;
module_param(ld2450_baud_rate, int, 0644);
MODULE_PARM_DESC(ld2450_baud_rate, "Baud rate for LD2450 UART communication");

static char *ld2450_device_name = LD2450_DEVICE_NAME;
module_param(ld2450_device_name, charp, 0644);
MODULE_PARM_DESC(ld2450_device_name, "Device name for LD2450 driver");

static struct ld2450_data *ld2450_global_data;
static DEFINE_SPINLOCK(ld2450_lock);

static int ld2450_open(struct inode *inode, struct file *file)
{
    struct ld2450_data *data = ld2450_global_data;
    int ret;

    spin_lock(&ld2450_lock);
    if (data->open_count >= LD2450_MAX_OPENS) {
        spin_unlock(&ld2450_lock);
        return -EBUSY;
    }
    data->open_count++;
    spin_unlock(&ld2450_lock);

    file->private_data = data;
    ret = nonseekable_open(inode, file);
    if (ret) {
        spin_lock(&ld2450_lock);
        data->open_count--;
        spin_unlock(&ld2450_lock);
    }
    return ret;
}

static int ld2450_release(struct inode *inode, struct file *file)
{
    struct ld2450_data *data = file->private_data;

    spin_lock(&ld2450_lock);
    data->open_count--;
    spin_unlock(&ld2450_lock);

    file->private_data = NULL;
    return 0;
}

static unsigned int ld2450_poll(struct file *file, struct poll_table_struct *wait)
{
    struct ld2450_data *data = file->private_data;
    unsigned int mask = 0;

    poll_wait(file, &data->data_ready.wait, wait);

    spin_lock(&ld2450_lock);
    if (data->data_ready.done)
        mask |= POLLIN | POLLRDNORM;
    spin_unlock(&ld2450_lock);

    return mask;
}

static ssize_t ld2450_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    struct ld2450_data *data = file->private_data;
    struct ld2450_tracking_data tracking;
    unsigned long flags;
    int ret;

    if (count < sizeof(struct ld2450_tracking_data))
        return -EINVAL;

    spin_lock_irqsave(&ld2450_lock, flags);
    tracking.x_pos = data->x_pos;
    tracking.y_pos = data->y_pos;
    tracking.velocity = data->velocity;
    tracking.distance = data->distance;
    spin_unlock_irqrestore(&ld2450_lock, flags);

    if (copy_to_user(buf, &tracking, sizeof(tracking)))
        return -EFAULT;

    return sizeof(tracking);
}

static ssize_t ld2450_write(struct file *file, const char __user *buf, size_t count, loff_t *pos)
{
    struct ld2450_data *data = file->private_data;
    u8 buffer[LD2450_MAX_MESSAGE_SIZE];
    u16 msg_len = min_t(size_t, count, LD2450_MAX_MESSAGE_SIZE);
    int ret;

    if (copy_from_user(buffer, buf, msg_len))
        return -EFAULT;

    mutex_lock(&data->fifo_lock);
    ret = serdev_device_write(data->serdev, buffer, msg_len, msecs_to_jiffies(LD2450_READ_TIMEOUT_MS));
    mutex_unlock(&data->fifo_lock);

    if (ret < 0)
        return ret;

    return msg_len;
}

static loff_t ld2450_llseek(struct file *file, loff_t offset, int whence)
{
    return no_llseek(file, offset, whence);
}

static long ld2450_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct ld2450_data *data = file->private_data;
    struct ld2450_tracking_data tracking;
    int mode;
    int ret;

    switch (cmd) {
    case LD2450_IOC_SET_MODE:
        if (copy_from_user(&mode, (int __user *)arg, sizeof(int)))
            return -EFAULT;
        data->mode = mode;
        break;
    case LD2450_IOC_GET_TRACKING:
        spin_lock(&ld2450_lock);
        tracking.x_pos = data->x_pos;
        tracking.y_pos = data->y_pos;
        tracking.velocity = data->velocity;
        tracking.distance = data->distance;
        spin_unlock(&ld2450_lock);
        if (copy_to_user((void __user *)arg, &tracking, sizeof(tracking)))
            return -EFAULT;
        break;
    default:
        return -ENOTTY;
    }

    return 0;
}

static const struct file_operations ld2450_fops = {
    .owner = THIS_MODULE,
    .open = ld2450_open,
    .release = ld2450_release,
    .read = ld2450_read,
    .write = ld2450_write,
    .llseek = ld2450_llseek,
    .unlocked_ioctl = ld2450_ioctl,
    .poll = ld2450_poll,
};

static struct miscdevice ld2450_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = LD2450_DEVICE_NAME,
    .fops = &ld2450_fops,
};

static int ld2450_probe(struct serdev_device *serdev)
{
    struct ld2450_data *data;
    int ret;

    data = devm_kzalloc(&serdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    data->serdev = serdev;
    data->dev = &serdev->dev;
    data->miscdevice = ld2450_misc;
    data->open_count = 0; /* Initialize open counter */
    serdev_device_set_drvdata(serdev, data);
    ld2450_global_data = data;

    INIT_KFIFO(data->fifo);
    spin_lock_init(&ld2450_lock);
    mutex_init(&data->fifo_lock);
    init_completion(&data->data_ready);

    data->power_gpio = devm_gpiod_get(data->dev, "power", GPIOD_OUT_LOW);
    if (IS_ERR(data->power_gpio)) {
        dev_err(data->dev, "Failed to get power GPIO: %ld\n", PTR_ERR(data->power_gpio));
        return PTR_ERR(data->power_gpio);
    }

    data->workqueue = create_singlethread_workqueue(ld2450_device_name);
    if (!data->workqueue) {
        dev_err(data->dev, "Failed to create workqueue\n");
        return -ENOMEM;
    }

    INIT_WORK(&data->tracking_work, ld2450_tracking_work);

    ret = ld2450_create_input(data);
    if (ret) {
        dev_err(data->dev, "Failed to create input device: %d\n", ret);
        destroy_workqueue(data->workqueue);
        return ret;
    }

    ret = ld2450_init_mq(data);
    if (ret) {
        dev_err(data->dev, "Failed to initialize message queue: %d\n", ret);
        ld2450_remove_input(data);
        destroy_workqueue(data->workqueue);
        return ret;
    }

    ret = serdev_device_open(serdev);
    if (ret) {
        dev_err(data->dev, "Failed to open serdev: %d\n", ret);
        ld2450_cleanup_mq(data);
        ld2450_remove_input(data);
        destroy_workqueue(data->workqueue);
        return ret;
    }

    serdev_device_set_baudrate(serdev, ld2450_baud_rate);
    serdev_device_set_flow_control(serdev, false);
    serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);

    pm_runtime_enable(data->dev);
    pm_runtime_set_autosuspend_delay(data->dev, LD2450_READ_TIMEOUT_MS);
    pm_runtime_use_autosuspend(data->dev);

    device_init_wakeup(data->dev, true);
    data->wakeup_enabled = device_get_wakeup_enable(data->dev);

    ret = ld2450_init_module(data);
    if (ret) {
        serdev_device_close(serdev);
        ld2450_remove_input(data);
        pm_runtime_disable(data->dev);
        return ret;
    }

    ret = ld2450_create_sysfs(data);
    if (ret) {
        ld2450_cleanup_mq(data);
        serdev_device_close(serdev);
        ld2450_remove_input(data);
        pm_runtime_disable(data->dev);
        return ret;
    }

    ret = ld2450_create_debugfs(data);
    if (ret) {
        ld2450_remove_sysfs(data);
        ld2450_cleanup_mq(data);
        serdev_device_close(serdev);
        ld2450_remove_input(data);
        pm_runtime_disable(data->dev);
        return ret;
    }

    ld2450_misc.name = ld2450_device_name;
    ret = misc_register(&ld2450_misc);
    if (ret) {
        ld2450_remove_sysfs(data);
        ld2450_cleanup_mq(data);
        serdev_device_close(serdev);
        ld2450_remove_input(data);
        pm_runtime_disable(data->dev);
        return ret;
    }

    return 0;
}

static void ld2450_remove(struct serdev_device *serdev)
{
    struct ld2450_data *data = serdev_device_get_drvdata(serdev);

    misc_deregister(&ld2450_misc);
    destroy_workqueue(data->workqueue);
    ld2450_remove_sysfs(data);
    ld2450_remove_debugfs(data);
    ld2450_cleanup_mq(data);
    ld2450_power_off(data);
    ld2450_remove_input(data);
    serdev_device_close(serdev);
    pm_runtime_disable(data->dev);
}

static const struct of_device_id ld2450_of_match[] = {
    { .compatible = "hilink,ld2450" },
    { }
};
MODULE_DEVICE_TABLE(of, ld2450_of_match);

static struct serdev_device_driver ld2450_driver = {
    .probe = ld2450_probe,
    .remove = ld2450_remove,
    .driver = {
        .name = LD2450_DEVICE_NAME,
        .of_match_table = ld2450_of_match,
    },
};

static int __init ld2450_init(void)
{
    int ret, retries = LD2450_RETRY_COUNT;

    while (retries--) {
        ret = serdev_device_driver_register(&ld2450_driver);
        if (!ret)
            break;
        pr_err("Failed to register serdev driver: %d, retrying...\n", ret);
        msleep(100);
    }
    if (ret) {
        pr_err("Failed to register serdev driver after retries\n");
        return ret;
    }

    pr_info("LD2450 driver v%s loaded\n", LD2450_VERSION);
    return 0;
}

static void __exit ld2450_exit(void)
{
    serdev_device_driver_unregister(&ld2450_driver);
    pr_info("LD2450 driver unloaded\n");
}

module_init(ld2450_init);
module_exit(ld2450_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Core driver for Hi-Link LD2450 radar module on Raspberry Pi");
MODULE_VERSION(LD2450_VERSION);