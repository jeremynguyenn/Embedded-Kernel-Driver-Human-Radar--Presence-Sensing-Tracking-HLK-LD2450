/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ld2450_sysfs.c - Sysfs and debugfs interfaces for Hi-Link LD2450 radar module
 * Author: Nguyen Nhan
 * Date: 1st September 2025
 * Version: 2.3.2
 * Description: Manages sysfs attributes and debugfs entries for Raspberry Pi
 * Sysfs Attributes:
 *   - fw_version: Displays firmware version of the LD2450 module
 *   - mac_addr: Displays MAC address of the module
 *   - tracking_data: Displays current tracking data (X, Y, velocity, distance)
 *   - Note: Provides user-space access to device attributes via /sys/devices/.../ld2450
 * Debugfs:
 *   - raw_data: Provides raw UART data for debugging
 * Changelog:
 *   - 2.2.0: Added memory allocation debugging
 *   - 2.3.0: Separated debugfs from sysfs
 *   - 2.3.1: Added detailed comments about sysfs attributes
 *   - 2.3.2: Optimized for Raspberry Pi
 */

#include <linux/sysfs.h>
#include <linux/debugfs.h>
#include "LD2450.h"

static ssize_t fw_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct ld2450_data *data = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%s\n", data->fw_version);
}

static DEVICE_ATTR_RO(fw_version);

static ssize_t mac_addr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct ld2450_data *data = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%s\n", data->mac_addr);
}

static DEVICE_ATTR_RO(mac_addr);

static ssize_t tracking_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct ld2450_data *data = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "X=%d, Y=%d, Vel=%d, Dist=%u\n",
                    data->x_pos, data->y_pos, data->velocity, data->distance);
}

static DEVICE_ATTR_RO(tracking_data);

static const struct file_operations debugfs_raw_data_fops = {
    .owner = THIS_MODULE,
    .open = simple_open,
    .read = ld2450_debugfs_raw_data_read,
};

int ld2450_create_sysfs(struct ld2450_data *data)
{
    int ret;

    ret = device_create_file(data->dev, &dev_attr_fw_version);
    if (ret) {
        dev_err(data->dev, "Failed to create fw_version sysfs\n");
        return ret;
    }

    ret = device_create_file(data->dev, &dev_attr_mac_addr);
    if (ret) {
        dev_err(data->dev, "Failed to create mac_addr sysfs\n");
        device_remove_file(data->dev, &dev_attr_fw_version);
        return ret;
    }

    ret = device_create_file(data->dev, &dev_attr_tracking_data);
    if (ret) {
        dev_err(data->dev, "Failed to create tracking_data sysfs\n");
        device_remove_file(data->dev, &dev_attr_fw_version);
        device_remove_file(data->dev, &dev_attr_mac_addr);
        return ret;
    }

    return 0;
}

void ld2450_remove_sysfs(struct ld2450_data *data)
{
    device_remove_file(data->dev, &dev_attr_fw_version);
    device_remove_file(data->dev, &dev_attr_mac_addr);
    device_remove_file(data->dev, &dev_attr_tracking_data);
}

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
MODULE_DESCRIPTION("Sysfs and debugfs for Hi-Link LD2450 radar module on Raspberry Pi");
MODULE_VERSION(LD2450_VERSION);