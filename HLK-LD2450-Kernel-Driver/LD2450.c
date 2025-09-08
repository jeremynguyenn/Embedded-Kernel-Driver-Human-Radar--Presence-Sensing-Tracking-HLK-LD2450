/**
 * Autor: Nguyen Nhan;
 * Device Driver para o Módulo radar Hi-Link LD2450; 
 */

/*___________________________________________________________________________*/
/*___________________________INFORMACOES ADICIONAIS__________________________*/
/*
 * Este modulo foi feito com base na documentacao:
 * "Serial Communication Protocol" do fabricante Hi-Link, versao 1.03.
 *
 * DEFAULT BAUD RATE = 256000
 *
 * Command protocol frame format
 *
 * SEND COMMAND FRAME FORMAT = ( Header + In-frame data length + In-frame data + End of frame )
 * Header:                  (4 Bytes)
 * In-frame data length:    (2 Bytes)
 * In-frame data:           ( Command word (2 bytes) + Command value (N bytes) )
 * End of frame:            (4 Bytes)
 *
 * ACK COMMAND FRAME FORMAT = ( Header + In-frame data length + In-frame data + End of frame )
 * Header:                  (4 Bytes)
 * In-frame data length:    (2 Bytes)
 * In-frame data:           ( Send Command Word | 0x0100 (2 bytes) + Return value (N bytes) )
 * End of frame:            (4 Bytes)
 *
 *
 * EXAMPLE: Read firmware version command
 * Command word: 0x00A0
 * Command value: none
 * Return Value: 2 bytes ACK status (0 success, 1 failure) + 2 bytes firmware type (0x0000) + 2 bytes
 * major version number + 4 bytes minor version number
 *              Header          Data Length     Command word + Command value        End of frame
 * Send data: FD FC FB FA   +   02 00       +   A0 00                           +   04 03 02 01
*/


/*___________________________________________________________________________*/
/*_________________________HISTORICO DE REVISOES_____________________________*/



/*___________________________________________________________________________*/
/*_______________________________INCLUDES____________________________________*/
/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * LD2450.c - Advanced Kernel driver for Hi-Link LD2450 radar module
 * Author: Nguyen Nhan
 * Date: 1st September 2025 
 * Description: Full-featured driver with input events, sysfs, PM support.
 * Based on Hi-Link Serial Communication Protocol v1.03
 */
#include <linux/module.h>
#include <linux/serdev.h>
#include <linux/of_device.h>
#include <linux/miscdevice.h>
#include <linux/gpio/consumer.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/sysfs.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include "LD2450.h"

#define LD2450_VERSION "2.1.0"  /* Updated version */
#define LD2450_GPIO_POWER "power"
#define LD2450_INPUT_NAME "ld2450-radar"
#define LD2450_INPUT_PHYS "ld2450/input0"
#define LD2450_RPM_AUTOSUSPEND_DELAY 5000 /* 5 seconds */

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Advanced Hi-Link LD2450 radar module driver with workqueues and debugfs");
MODULE_VERSION(LD2450_VERSION);

static struct ld2450_data *ld2450_global_data;

/**
 * ld2450_tracking_work - Workqueue handler for processing tracking data
 * @work: Work structure
 */
static void ld2450_tracking_work(struct work_struct *work)
{
	struct ld2450_data *data = container_of(work, struct ld2450_data, tracking_work);
	u8 buffer[LD2450_FRAME_SIZE];
	int ret;

	pm_runtime_get_sync(data->dev);
	if (data->mode != LD2450_MODE_TRACKING)
		goto out;

	ret = ld2450_read_data(data, buffer, LD2450_FRAME_SIZE);
	if (ret == LD2450_FRAME_SIZE) {
		if (buffer[0] == 0xAA && buffer[1] == 0xFF &&
		    buffer[2] == 0x03 && buffer[3] == 0x00 &&
		    buffer[28] == 0x55 && buffer[29] == 0xCC) {
			if (ld2450_parse_tracking_frame(buffer, data) == LD2450_SUCCESS) {
				input_report_abs(data->input_dev, ABS_X, data->x_pos);
				input_report_abs(data->input_dev, ABS_Y, data->y_pos);
				input_report_abs(data->input_dev, ABS_RX, data->velocity);
				input_sync(data->input_dev);
			}
		}
	}

out:
	pm_runtime_put_autosuspend(data->dev);
}

/**
 * ld2450_parse_tracking_frame - Parse 30-byte tracking frame
 * @frame: Raw frame buffer
 * @data: Device data to store parsed values
 * Return: 0 on success, negative error on failure
 */
int ld2450_parse_tracking_frame(const u8 *frame, struct ld2450_data *data)
{
	u16 checksum = 0;
	int i;

	for (i = 4; i < 28; i++)
		checksum += frame[i];
	if (checksum != ((frame[28] << 8) | frame[27]))
		return LD2450_ERR_CRC;

	data->x_pos = (s16)((frame[5] << 8) | frame[4]);
	data->y_pos = (s16)((frame[7] << 8) | frame[6]);
	data->velocity = (s16)((frame[9] << 8) | frame[8]);
	data->distance = (u16)((frame[11] << 8) | frame[10]);

	if (LD2450_DEBUG)
		dev_dbg(data->dev, "Parsed: X=%d, Y=%d, Vel=%d, Dist=%u\n",
			data->x_pos, data->y_pos, data->velocity, data->distance);

	return LD2450_SUCCESS;
}

/**
 * ld2450_power_on - Power on the LD2450 module via GPIO
 * @data: LD2450 device data
 * Return: 0 on success, negative error code on failure
 */
static int ld2450_power_on(struct ld2450_data *data)
{
	if (LD2450_DEBUG)
		dev_info(data->dev, "Powering on module\n");

	gpiod_set_value_cansleep(data->power_gpio, 1);
	data->powered = true;
	msleep(1000);
	return LD2450_SUCCESS;
}

/**
 * ld2450_power_off - Power off the LD2450 module
 * @data: LD2450 device data
 * Return: 0 on success, negative error code on failure
 */
static int ld2450_power_off(struct ld2450_data *data)
{
	if (LD2450_DEBUG)
		dev_info(data->dev, "Powering off module\n");

	gpiod_set_value_cansleep(data->power_gpio, 0);
	data->powered = false;
	msleep(1000);
	return LD2450_SUCCESS;
}

/**
 * ld2450_reset_module - Perform logical reset of the module
 * @data: LD2450 device data
 * Return: 0 on success, negative error code on failure
 */
static int ld2450_reset_module(struct ld2450_data *data)
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

	return ld2450_write_cmd_wait_ack(data, msg, msg_len, ack, ack_len);
}

/**
 * ld2450_write_cmd - Construct command message for LD2450
 * @data: LD2450 device data
 * @cmd: Command to send
 * @msg: Buffer to store constructed message
 * @msg_len: Pointer to store message length
 * Return: 0 on success, negative error code on failure
 */
static int ld2450_write_cmd(struct ld2450_data *data, u32 cmd, u8 *msg, u16 *msg_len)
{
	u16 index = 0;

	msg[index++] = 0xFD;
	msg[index++] = 0xFC;
	msg[index++] = 0xFB;
	msg[index++] = 0xFA;

	switch (cmd) {
	case CMD_SET_ENABLE_CONFIG:
		msg[index++] = 0x04; msg[index++] = 0x00;
		msg[index++] = 0xFF; msg[index++] = 0x00;
		msg[index++] = 0x01; msg[index++] = 0x00;
		break;
	case CMD_SET_BAUD_256000:
		msg[index++] = 0x04; msg[index++] = 0x00;
		msg[index++] = 0xA1; msg[index++] = 0x00;
		msg[index++] = 0x07; msg[index++] = 0x00;
		break;
	case CMD_SET_OFF_BLUETOOTH:
		msg[index++] = 0x04; msg[index++] = 0x00;
		msg[index++] = 0xA4; msg[index++] = 0x00;
		msg[index++] = 0x00; msg[index++] = 0x00;
		break;
	case CMD_GET_FW_VERSION:
		msg[index++] = 0x02; msg[index++] = 0x00;
		msg[index++] = 0xA0; msg[index++] = 0x00;
		break;
	case CMD_GET_MAC_ADDRESS:
		msg[index++] = 0x04; msg[index++] = 0x00;
		msg[index++] = 0xA5; msg[index++] = 0x00;
		msg[index++] = 0x01; msg[index++] = 0x00;
		break;
	case CMD_SET_ONE_TARGET_TRACKING:
		msg[index++] = 0x02; msg[index++] = 0x00;
		msg[index++] = 0x80; msg[index++] = 0x00;
		break;
	case CMD_SET_RESTART:
		msg[index++] = 0x02; msg[index++] = 0x00;
		msg[index++] = 0xA3; msg[index++] = 0x00;
		break;
	default:
		return LD2450_ERR_PROTOCOL;
	}

	msg[index++] = 0x04;
	msg[index++] = 0x03;
	msg[index++] = 0x02;
	msg[index++] = 0x01;

	*msg_len = index;
	return LD2450_SUCCESS;
}

/**
 * ld2450_get_ack - Construct expected ACK message
 * @data: LD2450 device data
 * @cmd: Command for which ACK is expected
 * @ack: Buffer to store ACK message
 * @ack_len: Pointer to store ACK length
 * Return: 0 on success, negative error code on failure
 */
static int ld2450_get_ack(struct ld2450_data *data, u32 cmd, u8 *ack, u16 *ack_len)
{
	u16 index = 0;

	ack[index++] = 0xFD;
	ack[index++] = 0xFC;
	ack[index++] = 0xFB;
	ack[index++] = 0xFA;

	switch (cmd) {
	case CMD_SET_ENABLE_CONFIG:
		ack[index++] = 0x08; ack[index++] = 0x00;
		ack[index++] = 0xFF; ack[index++] = 0x01;
		ack[index++] = 0x00; ack[index++] = 0x00;
		ack[index++] = 0x01; ack[index++] = 0x00;
		ack[index++] = 0x40; ack[index++] = 0x00;
		break;
	case CMD_SET_RESTART:
		ack[index++] = 0x04; ack[index++] = 0x00;
		ack[index++] = 0xA3; ack[index++] = 0x01;
		ack[index++] = 0x00; ack[index++] = 0x00;
		break;
	case CMD_GET_FW_VERSION:
		ack[index++] = 0x0C; ack[index++] = 0x00;
		ack[index++] = 0xA0; ack[index++] = 0x01;
		ack[index++] = 0x00; ack[index++] = 0x00;
		ack[index++] = 0x00; /* Firmware type */
		break;
	default:
		return LD2450_ERR_PROTOCOL;
	}

	ack[index++] = 0x04;
	ack[index++] = 0x03;
	ack[index++] = 0x02;
	ack[index++] = 0x01;

	*ack_len = index;
	return LD2450_SUCCESS;
}

/**
 * ld2450_write_cmd_wait_ack - Send command and wait for ACK
 * @data: LD2450 device data
 * @cmd: Command buffer
 * @cmd_len: Command length
 * @ack: Expected ACK buffer
 * @ack_len: Expected ACK length
 * Return: 0 on success, negative error code on failure
 */
static int ld2450_write_cmd_wait_ack(struct ld2450_data *data, const u8 *cmd, u16 cmd_len, const u8 *ack, u16 ack_len)
{
	int ret, retries = LD2450_RETRY_COUNT;

	while (retries--) {
		mutex_lock(&data->fifo_lock);
		kfifo_reset(&data->fifo);
		mutex_unlock(&data->fifo_lock);

		pm_runtime_get_sync(data->dev);
		ret = serdev_device_write_buf(data->serdev, cmd, cmd_len);
		pm_runtime_put(data->dev);
		if (ret < 0) {
			dev_err(data->dev, "Failed to write command: %d\n", ret);
			continue;
		}

		ret = wait_event_timeout(data->data_ready,
					 !kfifo_is_empty(&data->fifo),
					 msecs_to_jiffies(LD2450_READ_TIMEOUT_MS));
		if (!ret) {
			dev_err(data->dev, "Timeout waiting for ACK\n");
			return LD2450_ERR_TIMEOUT;
		}

		u8 recv[LD2450_MAX_MESSAGE_SIZE];
		size_t recv_len = kfifo_out(&data->fifo, recv, ack_len);
		if (recv_len != ack_len || memcmp(recv, ack, ack_len)) {
			dev_err(data->dev, "ACK mismatch\n");
			continue;
		}

		return LD2450_SUCCESS;
	}

	return LD2450_ERR_UART_RECV;
}

/**
 * ld2450_read_data - Read tracking data from FIFO
 * @data: LD2450 device data
 * @buffer: Buffer to store data
 * @size: Size of buffer
 * Return: Number of bytes read or negative error code
 */
static int ld2450_read_data(struct ld2450_data *data, u8 *buffer, size_t size)
{
	int ret;

	if (data->mode != LD2450_MODE_TRACKING)
		return -EINVAL;

	mutex_lock(&data->fifo_lock);
	ret = wait_event_timeout(data->data_ready,
				 !kfifo_is_empty(&data->fifo),
				 msecs_to_jiffies(LD2450_READ_TIMEOUT_MS));
	if (!ret) {
		mutex_unlock(&data->fifo_lock);
		return LD2450_ERR_TIMEOUT;
	}

	ret = kfifo_out(&data->fifo, buffer, size);
	mutex_unlock(&data->fifo_lock);

	return ret;
}

/**
 * ld2450_setup_module - Configure the LD2450 module
 * @data: LD2450 device data
 * Return: 0 on success, negative error code on failure
 */
static int ld2450_setup_module(struct ld2450_data *data)
{
	u8 msg[16], ack[16];
	u16 msg_len, ack_len;
	int ret;

	dev_info(data->dev, "Setting up LD2450 module\n");

	pm_runtime_get_sync(data->dev);

	/* Enter command mode */
	ret = ld2450_write_cmd(data, CMD_SET_ENABLE_CONFIG, msg, &msg_len);
	if (ret)
		goto out;
	ret = ld2450_get_ack(data, CMD_SET_ENABLE_CONFIG, ack, &ack_len);
	if (ret)
		goto out;
	ret = ld2450_write_cmd_wait_ack(data, msg, msg_len, ack, ack_len);
	if (ret) {
		dev_err(data->dev, "Failed to enter command mode: %d\n", ret);
		goto out;
	}

	/* Set baudrate to 256000 */
	ret = ld2450_write_cmd(data, CMD_SET_BAUD_256000, msg, &msg_len);
	if (ret)
		goto out;
	serdev_device_set_baudrate(data->serdev, LD2450_DEFAULT_BAUD_RATE);

	/* Get firmware version */
	ret = ld2450_write_cmd(data, CMD_GET_FW_VERSION, msg, &msg_len);
	if (ret)
		goto out;
	ret = ld2450_get_ack(data, CMD_GET_FW_VERSION, ack, &ack_len);
	if (ret)
		goto out;
	ret = ld2450_write_cmd_wait_ack(data, msg, msg_len, ack, ack_len);
	if (ret) {
		dev_err(data->dev, "Failed to get firmware version: %d\n", ret);
		goto out;
	}
	u8 fw_buf[16];
	size_t fw_len = kfifo_out(&data->fifo, fw_buf, 16);
	memcpy(data->fw_version, fw_buf, fw_len);

	/* Get MAC address */
	ret = ld2450 difficiles_write_cmd(data, CMD_GET_MAC_ADDRESS, msg, &msg_len);
	if (ret)
		goto out;
	u8 mac_buf[12];
	size_t mac_len = kfifo_out(&data->fifo, mac_buf, 12);
	memcpy(data->mac_addr, mac_buf, mac_len);

	/* Disable Bluetooth */
	ret = ld2450_write_cmd(data, CMD_SET_OFF_BLUETOOTH, msg, &msg_len);
	if (ret)
		goto out;

	/* Set single target tracking */
	ret = ld2450_write_cmd(data, CMD_SET_ONE_TARGET_TRACKING, msg, &msg_len);
	if (ret)
		goto out;
	data->tracking_mode = 1;

	/* Exit command mode with restart */
	ret = ld2450_reset_module(data);
	if (ret) {
		dev_err(data->dev, "Failed to restart module: %d\n", ret);
		goto out;
	}

	data->mode = LD2450_MODE_TRACKING;
	dev_info(data->dev, "LD2450 setup complete, entering tracking mode\n");

out:
	pm_runtime_put(data->dev);
	return ret;
}

/**
 * ld2450_init_module - Initialize the LD2450 module
 * @data: LD2450 device data
 * Return: 0 on success, negative error code on failure
 */
static int ld2450_init_module(struct ld2450_data *data)
{
	int ret;

	ret = ld2450_power_on(data);
	if (ret) {
		dev_err(data->dev, "Failed to power on: %d\n", ret);
		return ret;
	}

	ret = ld2450_setup_module(data);
	if (ret) {
		dev_err(data->dev, "Failed to setup module: %d\n", ret);
		ld2450_power_off(data);
		return ret;
	}

	ret = ld2450_create_sysfs(data);
	if (ret) {
		dev_err(data->dev, "Failed to create sysfs: %d\n", ret);
		ld2450_power_off(data);
		return ret;
	}

	ret = ld2450_create_debugfs(data);
	if (ret) {
		dev_err(data->dev, "Failed to create debugfs: %d\n", ret);
		ld2450_remove_sysfs(data);
		ld2450_power_off(data);
		return ret;
	}

	INIT_WORK(&data->tracking_work, ld2450_tracking_work);
	data->workqueue = create_singlethread_workqueue("ld2450_wq");
	if (!data->workqueue) {
		dev_err(data->dev, "Failed to create workqueue\n");
		ld2450_remove_sysfs(data);
		ld2450_remove_debugfs(data);
		ld2450_power_off(data);
		return -ENOMEM;
	}

	return LD2450_SUCCESS;
}

/**
 * ld2450_recv - Callback for receiving UART data
 * @serdev: Serdev device
 * @buffer: Received data
 * @size: Size of received data
 * Return: Number of bytes processed
 */
static int ld2450_recv(struct serdev_device *serdev, const u8 *buffer, size_t size)
{
	struct ld2450_data *data = serdev_device_get_drvdata(serdev);
	size_t copied;

	mutex_lock(&data->fifo_lock);
	if (data->mode == LD2450_MODE_TRACKING) {
		if (size >= LD2450_FRAME_SIZE &&
		    buffer[0] == 0xAA && buffer[1] == 0xFF &&
		    buffer[2] == 0x03 && buffer[3] == 0x00 &&
		    buffer[28] == 0x55 && buffer[29] == 0xCC) {
			copied = kfifo_in(&data->fifo, buffer, LD2450_FRAME_SIZE);
			if (copied == LD2450_FRAME_SIZE)
				queue_work(data->workqueue, &data->tracking_work);
			else
				dev_warn(data->dev, "FIFO overflow, lost %zu bytes\n", LD2450_FRAME_SIZE - copied);
		}
	} else {
		copied = kfifo_in(&data->fifo, buffer, size);
		if (copied != size)
			dev_warn(data->dev, "FIFO overflow, lost %zu bytes\n", size - copied);
	}
	mutex_unlock(&data->fifo_lock);
	complete(&data->data_ready);

	return size;
}

static const struct serdev_device_ops ld2450_serdev_ops = {
	.receive_buf = ld2450_recv,
};

/* Sysfs show functions */
static ssize_t fw_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ld2450_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", data->fw_version);
}

static ssize_t mac_addr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ld2450_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", data->mac_addr);
}

static ssize_t tracking_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ld2450_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "X=%d Y=%d Vel=%d Dist=%u\n",
		       data->x_pos, data->y_pos, data->velocity, data->distance);
}

static DEVICE_ATTR_RO(fw_version);
static DEVICE_ATTR_RO(mac_addr);
static DEVICE_ATTR_RO(tracking_data);

/* Debugfs functions */
static int debugfs_raw_data_show(void *priv, u64 *val)
{
	struct ld2450_data *data = priv;
	u8 buffer[LD2450_FRAME_SIZE];
	int ret;

	ret = ld2450_read_data(data, buffer, LD2450_FRAME_SIZE);
	if (ret < 0)
		return ret;

	*val = *(u64 *)buffer; /* Simplified for demo; adjust for actual raw data */
	return 0;
}

static int debugfs_raw_data_open(struct inode *inode, struct file *file)
{
	return single_open(file, debugfs_raw_data_show, inode->i_private);
}

static const struct file_operations debugfs_raw_data_fops = {
	.owner = THIS_MODULE,
	.open = debugfs_raw_data_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/**
 * ld2450_create_sysfs - Create sysfs attributes
 * @data: Device data
 * Return: 0 on success, negative error
 */
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

/**
 * ld2450_remove_sysfs - Remove sysfs attributes
 * @data: Device data
 */
void ld2450_remove_sysfs(struct ld2450_data *data)
{
	device_remove_file(data->dev, &dev_attr_fw_version);
	device_remove_file(data->dev, &dev_attr_mac_addr);
	device_remove_file(data->dev, &dev_attr_tracking_data);
}

/**
 * ld2450_create_debugfs - Create debugfs entries
 * @data: Device data
 * Return: 0 on success, negative error
 */
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

/**
 * ld2450_remove_debugfs - Remove debugfs entries
 * @data: Device data
 */
void ld2450_remove_debugfs(struct ld2450_data *data)
{
	debugfs_remove_recursive(data->debugfs_root);
}

/* Power Management */
static int ld2450_suspend(struct device *dev)
{
	struct ld2450_data *data = dev_get_drvdata(dev);
	int ret = 0;

	if (device_may_wakeup(dev) && data->wakeup_enabled) {
		/* Enable wakeup if GPIO supports irq */
	}

	serdev_device_close(data->serdev);
	ld2450_power_off(data);
	flush_workqueue(data->workqueue);

	dev_dbg(dev, "Suspended\n");
	return ret;
}

static int ld2450_resume(struct device *dev)
{
	struct ld2450_data *data = dev_get_drvdata(dev);
	int ret;

	ld2450_power_on(data);
	ret = serdev_device_open(data->serdev);
	if (ret) {
		dev_err(dev, "Failed to reopen serdev: %d\n", ret);
		return ret;
	}

	ret = ld2450_setup_module(data);
	if (ret)
		dev_err(dev, "Resume setup failed: %d\n", ret);

	if (device_may_wakeup(dev) && data->wakeup_enabled) {
		/* disable_irq_wake(gpio_to_irq(desc_to_gpio(data->power_gpio))); */
	}

	dev_dbg(dev, "Resumed\n");
	return 0;
}

static int ld2450_runtime_suspend(struct device *dev)
{
	struct ld2450_data *data = dev_get_drvdata(dev);
	ld2450_power_off(data);
	return 0;
}

static int ld2450_runtime_resume(struct device *dev)
{
	struct ld2450_data *data = dev_get_drvdata(dev);
	return ld2450_power_on(data);
}

static const struct dev_pm_ops ld2450_pm_ops = {
	.suspend = ld2450_suspend,
	.resume = ld2450_resume,
	.runtime_suspend = ld2450_runtime_suspend,
	.runtime_resume = ld2450_runtime_resume,
};

/**
 * ld2450_ioctl - Handle ioctl commands
 * @file: File structure
 * @cmd: Ioctl command
 * @arg: Ioctl argument
 * Return: 0 on success, negative error code on failure
 */
static long ld2450_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ld2450_data *data = ld2450_global_data;
	int ret = 0;

	switch (cmd) {
	case LD2450_IOC_SET_MODE:
	{
		int mode;
		if (copy_from_user(&mode, (int __user *)arg, sizeof(int)))
			return -EFAULT;
		if (mode != LD2450_MODE_COMMAND && mode != LD2450_MODE_TRACKING)
			return -EINVAL;
		data->mode = mode;
		break;
	}
	case LD2450_IOC_GET_TRACKING:
	{
		struct ld2450_tracking_data tracking = {
			.x_pos = data->x_pos,
			.y_pos = data->y_pos,
			.velocity = data->velocity,
			.distance = data->distance,
		};
		if (copy_to_user((void __user *)arg, &tracking, sizeof(tracking)))
			return -EFAULT;
		break;
	}
	default:
		return -ENOTTY;
	}

	return ret;
}

static const struct file_operations ld2450_fops = {
	.owner = THIS_MODULE,
	.open = ld2450_open,
	.release = ld2450_release,
	.read = ld2450_read,
	.write = ld2450_write,
	.unlocked_ioctl = ld2450_ioctl,
};

/**
 * ld2450_probe - Probe function for serdev device
 * @serdev: Serdev device
 * Return: 0 on success, negative error code on failure
 */
static int ld2450_probe(struct serdev_device *serdev)
{
	struct ld2450_data *data;
	struct input_dev *input_dev;
	int ret, error;

	data = devm_kzalloc(&serdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->serdev = serdev;
	data->dev = &serdev->dev;
	mutex_init(&data->fifo_lock);
	init_completion(&data->data_ready);
	serdev_device_set_drvdata(serdev, data);
	ld2450_global_data = data;

	data->power_gpio = devm_gpiod_get(data->dev, LD2450_GPIO_POWER, GPIOD_OUT_LOW);
	if (IS_ERR(data->power_gpio)) {
		dev_err(data->dev, "Failed to get power GPIO: %ld\n", PTR_ERR(data->power_gpio));
		return PTR_ERR(data->power_gpio);
	}

	ret = devm_kfifo_alloc(data->dev, &data->fifo, LD2450_FIFO_SIZE, GFP_KERNEL);
	if (ret) {
		dev_err(data->dev, "Failed to allocate FIFO: %d\n", ret);
		return ret;
	}

	input_dev = devm_input_allocate_device(data->dev);
	if (!input_dev)
		return -ENOMEM;

	input_dev->name = LD2450_INPUT_NAME;
	input_dev->phys = LD2450_INPUT_PHYS;
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

	serdev_device_set_client_ops(serdev, &ld2450_serdev_ops);
	ret = serdev_device_open(serdev);
	if (ret) {
		dev_err(data->dev, "Failed to open serdev: %d\n", ret);
		input_unregister_device(input_dev);
		return ret;
	}

	serdev_device_set_baudrate(serdev, LD2450_DEFAULT_BAUD_RATE);
	serdev_device_set_flow_control(serdev, false);
	serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);

	/* Enable runtime PM */
	pm_runtime_enable(data->dev);
	pm_runtime_set_autosuspend_delay(data->dev, LD2450_RPM_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(data->dev);

	/* Enable wakeup if supported */
	device_init_wakeup(data->dev, true);
	data->wakeup_enabled = device_get_wakeup_enable(data->dev);

	ret = ld2450_init_module(data);
	if (ret) {
		serdev_device_close(serdev);
		input_unregister_device(input_dev);
		pm_runtime_disable(data->dev);
		return ret;
	}

	return 0;
}

/**
 * ld2450_remove - Remove function for serdev device
 * @serdev: Serdev device
 */
static void ld2450_remove(struct serdev_device *serdev)
{
	struct ld2450_data *data = serdev_device_get_drvdata(serdev);

	destroy_workqueue(data->workqueue);
	ld2450_remove_sysfs(data);
	ld2450_remove_debugfs(data);
	ld2450_power_off(data);
	input_unregister_device(data->input_dev);
	serdev_device_close(serdev);
	pm_runtime_disable(data->dev);
}

static const struct of_device_id ld2450_of_match[] = {
	{ .compatible = "hilink,ld2450" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ld2450_of_match);

static struct serdev_device_driver ld2450_driver = {
	.probe = ld2450_probe,
	.remove = ld2450_remove,
	.driver = {
		.name = "ld2450",
		.of_match_table = ld2450_of_match,
		.pm = &ld2450_pm_ops,
	},
};

static struct miscdevice ld2450_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ld2450",
	.fops = &ld2450_fops,
};

static int __init ld2450_init(void)
{
	int ret;

	ret = serdev_device_driver_register(&ld2450_driver);
	if (ret) {
		pr_err("Failed to register serdev driver: %d\n", ret);
		return ret;
	}

	ret = misc_register(&ld2450_misc);
	if (ret) {
		pr_err("Failed to register misc device: %d\n", ret);
		serdev_device_driver_unregister(&ld2450_driver);
		return ret;
	}

	pr_info("LD2450 driver v%s loaded\n", LD2450_VERSION);
	return 0;
}

static void __exit ld2450_exit(void)
{
	misc_deregister(&ld2450_misc);
	serdev_device_driver_unregister(&ld2450_driver);
	pr_info("LD2450 driver unloaded\n");
}

module_init(ld2450_init);
module_exit(ld2450_exit);