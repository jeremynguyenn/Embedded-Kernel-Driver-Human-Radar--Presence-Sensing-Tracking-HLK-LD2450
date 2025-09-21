/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ld2450_serial.c - Serial communication for Hi-Link LD2450 radar module
 * Author: Nguyen Nhan
 * Date: 1st September 2025
 * Version: 2.3.3
 * Description: Handles UART communication via serdev for the LD2450 module on Raspberry Pi
 * Workflow:
 *   - Receive: Processes incoming UART data, parses tracking frames
 *   - Write: Sends commands to the LD2450 module
 *   - POSIX IPC: Sends tracking data to message queue with priority
 * Changelog:
 *   - 2.2.0: Added message queue support
 *   - 2.3.0: Added spinlock for atomic operations
 *   - 2.3.1: Optimized for Raspberry Pi
 *   - 2.3.3: Added message queue priority support
 */

#include <linux/serdev.h>
#include <linux/kfifo.h>
#include <linux/mqueue.h>
#include "LD2450.h"

static int ld2450_receive_buf(struct serdev_device *serdev, const u8 *data, size_t count)
{
    struct ld2450_data *ld2450 = serdev_device_get_drvdata(serdev);
    size_t i = 0;

    while (i < count) {
        if (kfifo_in(&ld2450->fifo, &data[i], 1) != 1) {
            dev_err(ld2450->dev, "FIFO overflow\n");
            return -ENOMEM;
        }
        i++;
    }

    schedule_work(&ld2450->tracking_work);
    return count;
}

static void ld2450_tracking_work(struct work_struct *work)
{
    struct ld2450_data *data = container_of(work, struct ld2450_data, tracking_work);
    u8 buffer[LD2450_FRAME_SIZE];
    size_t len;

    while (kfifo_len(&data->fifo) >= LD2450_FRAME_SIZE) {
        len = kfifo_out(&data->fifo, buffer, LD2450_FRAME_SIZE);
        if (len != LD2450_FRAME_SIZE)
            continue;

        if (ld2450_parse_tracking_frame(buffer, data) == LD2450_SUCCESS) {
            unsigned int priority = (data->distance < 100) ? 10 : 0; /* High priority for close distances */
            if (data->mq != -1) {
                if (mq_send(data->mq, (const char *)&(struct ld2450_tracking_data){
                    .x_pos = data->x_pos,
                    .y_pos = data->y_pos,
                    .velocity = data->velocity,
                    .distance = data->distance
                }, sizeof(struct ld2450_tracking_data), priority) == -1) {
                    dev_err(data->dev, "Failed to send to message queue: %d\n", errno);
                }
            }
            complete(&data->data_ready); /* Signal for poll */
            input_report_abs(data->input_dev, ABS_X, data->x_pos);
            input_report_abs(data->input_dev, ABS_Y, data->y_pos);
            input_report_abs(data->input_dev, ABS_SPEED, data->velocity);
            input_report_abs(data->input_dev, ABS_DISTANCE, data->distance);
            input_sync(data->input_dev);
        }
    }
}

int ld2450_write_cmd(struct ld2450_data *data, u32 cmd, u8 *msg, u16 *msg_len)
{
    u8 buffer[LD2450_MAX_MESSAGE_SIZE];
    u16 len = 0;

    switch (cmd) {
    case CMD_SET_ENABLE_CONFIG:
        memcpy(buffer, "\xFD\xFC\xFB\xFA\x06\x00\xFF\x00\x01\x00\x04\x03\x02\x01", 14);
        len = 14;
        break;
    case CMD_SET_DISABLE_CONFIG:
        memcpy(buffer, "\xFD\xFC\xFB\xFA\x04\x00\xFE\x00\x04\x03\x02\x01", 12);
        len = 12;
        break;
    case CMD_SET_RESTART:
        memcpy(buffer, "\xFD\xFC\xFB\xFA\x04\x00\xA3\x00\x04\x03\x02\x01", 12);
        len = 12;
        break;
    case CMD_GET_FW_VERSION:
        memcpy(buffer, "\xFD\xFC\xFB\xFA\x04\x00\xA0\x00\x04\x03\x02\x01", 12);
        len = 12;
        break;
    case CMD_SET_BAUD_256000:
        memcpy(buffer, "\xFD\xFC\xFB\xFA\x06\x00\xA1\x00\x07\x00\x04\x03\x02\x01", 14);
        len = 14;
        break;
    case CMD_SET_OFF_BLUETOOTH:
        memcpy(buffer, "\xFD\xFC\xFB\xFA\x06\x00\xA4\x00\x00\x00\x04\x03\x02\x01", 14);
        len = 14;
        break;
    default:
        return -EINVAL;
    }

    mutex_lock(&data->fifo_lock);
    *msg_len = serdev_device_write(data->serdev, buffer, len, msecs_to_jiffies(LD2450_READ_TIMEOUT_MS));
    mutex_unlock(&data->fifo_lock);

    if (*msg_len < 0)
        return *msg_len;

    memcpy(msg, buffer, *msg_len);
    return 0;
}

int ld2450_get_ack(struct ld2450_data *data, u32 cmd, u8 *ack, u16 *ack_len)
{
    u8 buffer[LD2450_MAX_MESSAGE_SIZE];
    int ret;

    ret = serdev_device_read(data->serdev, buffer, LD2450_MAX_MESSAGE_SIZE, msecs_to_jiffies(LD2450_READ_TIMEOUT_MS));
    if (ret < 0)
        return ret;

    *ack_len = ret;
    memcpy(ack, buffer, *ack_len);
    return 0;
}

int ld2450_read_data(struct ld2450_data *data, u8 *buffer, size_t size)
{
    int ret;

    mutex_lock(&data->fifo_lock);
    ret = serdev_device_read(data->serdev, buffer, size, msecs_to_jiffies(LD2450_READ_TIMEOUT_MS));
    mutex_unlock(&data->fifo_lock);

    return ret;
}

int ld2450_parse_tracking_frame(const u8 *frame, struct ld2450_data *data)
{
    if (frame[0] != 0xAA || frame[1] != 0xFF || frame[2] != 0x03 || frame[3] != 0x00)
        return LD2450_ERR_PROTOCOL;

    spin_lock(&data->ld2450_lock);
    data->x_pos = (s16)(frame[6] | (frame[7] << 8));
    data->y_pos = (s16)(frame[8] | (frame[9] << 8));
    data->velocity = (s16)(frame[10] | (frame[11] << 8));
    data->distance = (u16)(frame[12] | (frame[13] << 8));
    spin_unlock(&data->ld2450_lock);

    return LD2450_SUCCESS;
}

static const struct serdev_device_ops ld2450_serdev_ops = {
    .receive_buf = ld2450_receive_buf,
};

int ld2450_init_mq(struct ld2450_data *data)
{
    data->mq_attr.mq_maxmsg = LD2450_MQ_MAXMSG;
    data->mq_attr.mq_msgsize = LD2450_MQ_MSGSIZE;
    data->mq = mq_open(LD2450_MQ_NAME, O_WRONLY | O_CREAT | O_NONBLOCK, 0644, &data->mq_attr);
    if (data->mq == -1) {
        dev_err(data->dev, "Failed to open message queue: %d\n", errno);
        return -errno;
    }
    return 0;
}

void ld2450_cleanup_mq(struct ld2450_data *data)
{
    if (data->mq != -1) {
        mq_close(data->mq);
        mq_unlink(LD2450_MQ_NAME);
        data->mq = -1;
    }
}

int ld2450_setup_module(struct ld2450_data *data)
{
    u8 msg[LD2450_MAX_MESSAGE_SIZE];
    u16 msg_len;
    int ret;

    ret = ld2450_write_cmd(data, CMD_SET_ENABLE_CONFIG, msg, &msg_len);
    if (ret) {
        dev_err(data->dev, "Failed to enable config: %d\n", ret);
        return ret;
    }

    ret = ld2450_write_cmd(data, CMD_SET_BAUD_256000, msg, &msg_len);
    if (ret) {
        dev_err(data->dev, "Failed to set baud rate: %d\n", ret);
        return ret;
    }

    ret = ld2450_write_cmd(data, CMD_SET_OFF_BLUETOOTH, msg, &msg_len);
    if (ret) {
        dev_err(data->dev, "Failed to disable Bluetooth: %d\n", ret);
        return ret;
    }

    return 0;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Serial communication for Hi-Link LD2450 radar module");
MODULE_VERSION("2.3.3");