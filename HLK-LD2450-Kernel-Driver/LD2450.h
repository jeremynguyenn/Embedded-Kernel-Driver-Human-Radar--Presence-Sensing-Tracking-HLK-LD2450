/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * LD2450.h - Header file for Hi-Link LD2450 radar module driver
 * Author: Nguyen Nhan
 * Date: 1st September 2025
 * Description: Advanced kernel driver for HLK-LD2450 radar module over UART (serdev)
 * Features: Input events, sysfs, PM, parsing, workqueues, debugfs, ioctls
 */

#ifndef __LD2450_H__
#define __LD2450_H__

#include <linux/types.h>
#include <linux/device.h>
#include <linux/kfifo.h>
#include <linux/serdev.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/input.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/gpio/consumer.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/ioctl.h>

#define STM32       1
#define RASPBERRY   2
#define SYSTEM_PLATFORM RASPBERRY

/* Debug flags */
#define LD2450_DEBUG 0

/* Constants */
#define LD2450_DEFAULT_BAUD_RATE 256000
#define LD2450_FIFO_SIZE 512
#define LD2450_READ_TIMEOUT_MS 1000
#define LD2450_RETRY_COUNT 3
#define LD2450_TRACKING_BUFF_SIZE 32
#define LD2450_MAX_MESSAGE_SIZE 64
#define LD2450_FRAME_SIZE 30  /* Tracking frame size */
#define LD2450_DEVICE_NAME "ld2450"

/* Protocol definitions */
#define PROTOCOL_START 0xFDFCFBFA
#define PROTOCOL_END 0x04030201
#define PROT_DATA_OUT_START 0xAAFF0300
#define PROT_DATA_OUT_END 0x55CC

/* Command definitions */
#define CMD_SET_ENABLE_CONFIG 0x0400FF000100
#define CMD_SET_DISABLE_CONFIG 0x0200FE00
#define CMD_SET_RESTART 0x0200A300
#define CMD_GET_FW_VERSION 0x0200A000
#define CMD_SET_BAUD_256000 0x0400A1000700
#define CMD_SET_OFF_BLUETOOTH 0x0400A4000000
#define CMD_GET_MAC_ADDRESS 0x0400A5000100
#define CMD_SET_ONE_TARGET_TRACKING 0x02008000

/* Command ACKs */
#define CMD_SET_ENABLE_CONFIG_ACK 0x0800FF010001004000
#define CMD_RESTART_MODULE_ACK 0x0400A3010000
#define CMD_GET_FW_VERSION_ACK 0x0C00A001000000

/* Ioctl commands */
#define LD2450_IOC_MAGIC 'L'
#define LD2450_IOC_SET_MODE _IOW(LD2450_IOC_MAGIC, 1, int)
#define LD2450_IOC_GET_TRACKING _IOR(LD2450_IOC_MAGIC, 2, struct ld2450_tracking_data)

/* Enums */
enum ld2450_error {
	LD2450_SUCCESS = 0,
	LD2450_ERR_FAIL = -1,
	LD2450_ERR_UART_SEND = -2,
	LD2450_ERR_UART_RECV = -3,
	LD2450_ERR_TIMEOUT = -4,
	LD2450_ERR_PROTOCOL = -5,
	LD2450_ERR_CRC = -6,
};

enum ld2450_mode {
	LD2450_MODE_COMMAND = 0,
	LD2450_MODE_TRACKING,
};

/* Tracking data structure for ioctl */
struct ld2450_tracking_data {
	s16 x_pos;
	s16 y_pos;
	s16 velocity;
	u16 distance;
};

/* Data structure for LD2450 device */
struct ld2450_data {
	struct serdev_device *serdev;
	struct device *dev;
	struct kfifo fifo;
	struct mutex fifo_lock;
	struct completion data_ready;
	struct input_dev *input_dev;
	struct gpio_desc *power_gpio;
	struct workqueue_struct *workqueue;
	struct work_struct tracking_work;
	struct dentry *debugfs_root;
	u8 mode;
	u8 fw_version[16];
	u8 mac_addr[12];
	u8 tracking_mode;
	bool powered;
	bool wakeup_enabled;
	/* Parsed tracking data */
	s16 x_pos, y_pos, velocity;
	u16 distance;
};

/* Function prototypes */
int ld2450_init_module(struct ld2450_data *data);
int ld2450_power_on(struct ld2450_data *data);
int ld2450_power_off(struct ld2450_data *data);
int ld2450_reset_module(struct ld2450_data *data);
int ld2450_setup_module(struct ld2450_data *data);
int ld2450_write_cmd(struct ld2450_data *data, u32 cmd, u8 *msg, u16 *msg_len);
int ld2450_get_ack(struct ld2450_data *data, u32 cmd, u8 *ack, u16 *ack_len);
int ld2450_read_data(struct ld2450_data *data, u8 *buffer, size_t size);
int ld2450_parse_tracking_frame(const u8 *frame, struct ld2450_data *data);
int ld2450_create_sysfs(struct ld2450_data *data);
void ld2450_remove_sysfs(struct ld2450_data *data);
int ld2450_create_debugfs(struct ld2450_data *data);
void ld2450_remove_debugfs(struct ld2450_data *data);

/* PM callbacks */
int ld2450_suspend(struct device *dev);
int ld2450_resume(struct device *dev);

#endif /* __LD2450_H__ */