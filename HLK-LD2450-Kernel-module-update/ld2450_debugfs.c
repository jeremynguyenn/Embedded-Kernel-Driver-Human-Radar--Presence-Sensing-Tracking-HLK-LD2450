/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ld2450_debugfs.c - Debugfs interfaces for Hi-Link LD2450 radar module
 * Author: Nguyen Nhan
 * Date: 1st September 2025
 * Version: 2.3.2
 * Description: Manages debugfs entries for raw data access on Raspberry Pi
 * Debugfs:
 *   - raw_data: Provides raw UART data for debugging via /sys/kernel/debug/ld2450/raw_data
 *   - Uses mutex for thread-safe data access
 * Changelog:
 *   - 2.3.0: Separated debugfs from sysfs, added raw_data read implementation
 *   - 2.3.1: Added detailed comments about debugfs
 *   - 2.3.2: Optimized for Raspberry Pi
 */

#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include "LD2450.h"

static ssize_t ld2450_debugfs_raw_data_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    struct ld2450_data *data = file->private_data;
    u8 buffer[LD2450_FRAME_SIZE];
    int ret;

    if (*ppos != 0)
        return 0; /* Only one read allowed */

    mutex_lock(&data->fifo_lock);
    ret = serdev_device_read(data->serdev, buffer, LD2450_FRAME_SIZE, msecs_to_jiffies(LD2450_READ_TIMEOUT_MS));
    if (ret < 0) {
        mutex_unlock(&data->fifo_lock);
        return ret;
    }

    if (copy_to_user(buf, buffer, ret)) {
        mutex_unlock(&data->fifo_lock);
        return -EFAULT;
    }

    mutex_unlock(&data->fifo_lock);
    *ppos += ret;
    return ret;
}

static const struct file_operations debugfs_raw_data_fops = {
    .owner = THIS_MODULE,
    .open = simple_open,
    .read = ld2450_debugfs_raw_data_read,
};

int ld2450_create_debugfs(struct ld2450_data *data)
{
    data->debugfs_root = debugfs_create_dir(LD2450_DEVICE_NAME, NULL);
    if (IS_ERR(data->debugfs_root)) {
        dev_err(data->dev, "Failed to create debugfs directory\n");
        return PTR_ERR(data->debugfs_root);
    }

    if (!debugfs_create_file("raw_data", 0444, data->debugfs_root, data, &debugfs_raw_data_fops)) {
        dev_err(data->dev, "Failed to create debugfs raw_data\n");
        debugfs_remove_recursive(data->debugfs_root);
        return -ENOMEM;
    }

    return 0;
}

void ld2450_remove_debugfs(struct ld2450_data *data)
{
    debugfs_remove_recursive(data->debugfs_root);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Debugfs interfaces for Hi-Link LD2450 radar module on Raspberry Pi");
MODULE_VERSION(LD2450_VERSION);