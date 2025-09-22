/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ld2450_debugfs.c - Debugfs interfaces for Hi-Link LD2450 radar module
 * Author: Nguyen Nhan
 * Date: 22nd September 2025
 * Version: 2.4.0
 * Description: Manages debugfs entries for raw data and stats on Raspberry Pi
 * Debugfs:
 *   - raw_data: Provides raw UART data
 *   - error_count: Displays error count
 *   - fifo_usage: Displays FIFO usage
 *   - thread_status: Displays kernel thread status
 *   - race_stats: Displays race condition test stats
 * Changelog:
 *   - 2.3.2: Optimized for Raspberry Pi
 *   - 2.4.0: Added thread_status and race_stats
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
        return 0;

    mutex_lock(&data->fifo_lock);
    ret = serdev_device_read(data->serdev, buffer, LD2450_FRAME_SIZE, msecs_to_jiffies(LD2450_READ_TIMEOUT_MS));
    if (ret < 0) {
        mutex_unlock(&data->fifo_lock);
        data->error_count++;
        return ret;
    }

    if (copy_to_user(buf, buffer, ret)) {
        mutex_unlock(&data->fifo_lock);
        data->error_count++;
        return -EFAULT;
    }

    mutex_unlock(&data->fifo_lock);
    *ppos += ret;
    return ret;
}

static ssize_t thread_status_show(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    struct ld2450_data *data = file->private_data;
    char status[32];
    int len;

    if (*ppos != 0)
        return 0;

    len = scnprintf(status, sizeof(status), "Thread running: %d\n", !kthread_should_stop());
    if (copy_to_user(buf, status, len))
        return -EFAULT;

    *ppos += len;
    return len;
}

static const struct file_operations debugfs_raw_data_fops = {
    .owner = THIS_MODULE,
    .open = simple_open,
    .read = ld2450_debugfs_raw_data_read,
};

static const struct file_operations debugfs_thread_status_fops = {
    .owner = THIS_MODULE,
    .open = simple_open,
    .read = thread_status_show,
};

int ld2450_create_debugfs(struct ld2450_data *data)
{
    data->debugfs_root = debugfs_create_dir(LD2450_DEVICE_NAME, NULL);
    if (IS_ERR(data->debugfs_root)) {
        dev_err(data->dev, "Failed to create debugfs directory\n");
        data->error_count++;
        return PTR_ERR(data->debugfs_root);
    }

    if (!debugfs_create_file("raw_data", 0444, data->debugfs_root, data, &debugfs_raw_data_fops)) {
        dev_err(data->dev, "Failed to create debugfs raw_data\n");
        debugfs_remove_recursive(data->debugfs_root);
        data->error_count++;
        return -ENOMEM;
    }

    if (!debugfs_create_file("thread_status", 0444, data->debugfs_root, data, &debugfs_thread_status_fops)) {
        dev_err(data->dev, "Failed to create debugfs thread_status\n");
        debugfs_remove_recursive(data->debugfs_root);
        data->error_count++;
        return -ENOMEM;
    }

    debugfs_create_u32("error_count", 0444, data->debugfs_root, &data->error_count);
    debugfs_create_u32("fifo_usage", 0444, data->debugfs_root, &data->fifo_usage);
    debugfs_create_u32("race_stats", 0444, data->debugfs_root, &data->race_test_result);

    return 0;
}

void ld2450_remove_debugfs(struct ld2450_data *data)
{
    debugfs_remove_recursive(data->debugfs_root);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Debugfs interfaces for Hi-Link LD2450 radar module on Raspberry Pi");
MODULE_VERSION("2.4.0");