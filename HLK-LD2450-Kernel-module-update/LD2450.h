/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * LD2450.h - Header file for Hi-Link LD2450 radar module driver
 * Author: Nguyen Nhan
 * Date: 1st September 2025
 * Version: 2.3.3
 * Description: Advanced kernel driver for HLK-LD2450 radar module over UART (serdev) on Raspberry Pi
 * Features: Input events, sysfs, PM, parsing, workqueues, debugfs, ioctls, POSIX IPC
 * Driver Types:
 *   - Character device: Implemented via miscdevice (/dev/ld2450) for user-space interaction
 *   - Input device: Reports tracking data (X, Y, velocity) via input subsystem
 *   - Serdev driver: Handles UART communication for the LD2450 module
 *   - Other types (not used): Block devices (for storage), Network devices (for networking), USB/PCI (for specific hardware)
 * Memory Management:
 *   - Kernel: Uses kmalloc/devm_kzalloc for small, contiguous memory allocations
 *   - User-space: Uses malloc/calloc/realloc for dynamic memory (see LD2450_app.c)
 *   - Virtual Memory: Kernel allocations are mapped to virtual address space via MMU
 *   - Note: No swapping or complex page table management in this driver due to small memory footprint
 * Changelog:
 *   - 2.2.0: Added POSIX message queue, thread synchronization, and modularized structure
 *   - 2.3.0: Added comments for driver types and memory management
 *   - 2.3.1: Added comments for other driver types and virtual memory
 *   - 2.3.2: Removed STM32 support, optimized for Raspberry Pi
 *   - 2.3.3: Added open_count for concurrent open limiting, updated file operations
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
#include <linux/mqueue.h>

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
#define LD2450_MQ_NAME "/ld2450_mq"
#define LD2450_MQ_MAXMSG 10
#define LD2450_MQ_MSGSIZE sizeof(struct ld2450_tracking_data)

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

/* IOCTL commands */
#define LD2450_IOC_MAGIC 'L'
#define LD2450_IOC_SET_MODE _IOW(LD2450_IOC_MAGIC, 1, int)
#define LD2450_IOC_GET_TRACKING _IOR(LD2450_IOC_MAGIC, 2, struct ld2450_tracking_data)

/* Error codes */
#define LD2450_SUCCESS 0
#define LD2450_ERR_UART_RECV -EIO
#define LD2450_ERR_PROTOCOL -EINVAL

/* Tracking data structure */
struct ld2450_tracking_data {
    short x_pos;
    short y_pos;
    short velocity;
    unsigned short distance;
};

/* Driver data structure */
struct ld2450_data {
    struct serdev_device *serdev;
    struct device *dev;
    struct miscdevice miscdevice;
    DECLARE_KFIFO(fifo, u8, LD2450_FIFO_SIZE);
    spinlock_t ld2450_lock;
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
    /* POSIX message queue */
    struct mq_attr mq_attr;
    mqd_t mq;
    /* Open counter */
    unsigned int open_count;
};

/* Function prototypes - Core */
int ld2450_init_module(struct ld2450_data *data);
int ld2450_setup_module(struct ld2450_data *data);

/* Function prototypes - Power Management */
int ld2450_power_on(struct ld2450_data *data);
int ld2450_power_off(struct ld2450_data *data);
int ld2450_reset_module(struct ld2450_data *data);

/* Function prototypes - Serial Communication */
int ld2450_write_cmd(struct ld2450_data *data, u32 cmd, u8 *msg, u16 *msg_len);
int ld2450_get_ack(struct ld2450_data *data, u32 cmd, u8 *ack, u16 *ack_len);
int ld2450_read_data(struct ld2450_data *data, u8 *buffer, size_t size);
int ld2450_parse_tracking_frame(const u8 *frame, struct ld2450_data *data);

/* Function prototypes - Sysfs and Debugfs */
int ld2450_create_sysfs(struct ld2450_data *data);
void ld2450_remove_sysfs(struct ld2450_data *data);
int ld2450_create_debugfs(struct ld2450_data *data);
void ld2450_remove_debugfs(struct ld2450_data *data);

/* Function prototypes - Input */
int ld2450_create_input(struct ld2450_data *data);
void ld2450_remove_input(struct ld2450_data *data);

/* Function prototypes - POSIX IPC */
int ld2450_init_mq(struct ld2450_data *data);
void ld2450_cleanup_mq(struct ld2450_data *data);

/* Function prototypes - File Operations */
int ld2450_open(struct inode *inode, struct file *file);
int ld2450_release(struct inode *inode, struct file *file);
ssize_t ld2450_read(struct file *file, char __user *buf, size_t count, loff_t *pos);
ssize_t ld2450_write(struct file *file, const char __user *buf, size_t count, loff_t *pos);
loff_t ld2450_llseek(struct file *file, loff_t offset, int whence);
long ld2450_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
unsigned int ld2450_poll(struct file *file, struct poll_table_struct *wait);

#endif /* __LD2450_H__ */