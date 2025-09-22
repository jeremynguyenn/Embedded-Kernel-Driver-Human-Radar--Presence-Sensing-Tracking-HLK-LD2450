/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ld2450_block.c - Simulated block device for Hi-Link LD2450 radar module
 * Author: Nguyen Nhan
 * Date: 22nd September 2025
 * Version: 2.4.0
 * Description: Implements a simulated block device for learning purposes on Raspberry Pi
 * Block Device:
 *   - Uses in-memory buffer to simulate block storage
 *   - Supports read/write operations via block request queue
 * Changelog:
 *   - 2.4.0: Initial implementation of block device
 */

#include <linux/blkdev.h>
#include <linux/vmalloc.h>
#include "LD2450.h"

#define LD2450_BLOCK_SIZE (4 * 1024) /* 4KB blocks */
#define LD2450_NUM_BLOCKS 1024 /* 4MB total storage */
#define LD2450_SECTOR_SIZE 512

static void ld2450_block_request(struct request_queue *q)
{
    struct ld2450_data *data = q->queuedata;
    struct request *req;
    void *buffer = vmalloc(LD2450_BLOCK_SIZE * LD2450_NUM_BLOCKS);
    if (!buffer) {
        dev_err(data->dev, "Failed to allocate block buffer\n");
        data->error_count++;
        return;
    }

    while ((req = blk_fetch_request(q)) != NULL) {
        unsigned long start = blk_rq_pos(req) * LD2450_SECTOR_SIZE;
        unsigned long len = blk_rq_bytes(req);

        if (start + len > LD2450_BLOCK_SIZE * LD2450_NUM_BLOCKS) {
            dev_err(data->dev, "Request out of bounds\n");
            __blk_end_request_all(req, -EIO);
            data->error_count++;
            continue;
        }

        if (rq_data_dir(req) == READ) {
            memcpy(req->buffer, buffer + start, len);
        } else {
            memcpy(buffer + start, req->buffer, len);
        }

        __blk_end_request_all(req, 0);
    }

    vfree(buffer);
}

int ld2450_create_block_dev(struct ld2450_data *data)
{
    struct gendisk *disk;
    int ret;

    data->block_dev.queue = blk_init_queue(ld2450_block_request, &data->ld2450_lock);
    if (!data->block_dev.queue) {
        dev_err(data->dev, "Failed to initialize block queue\n");
        data->error_count++;
        return -ENOMEM;
    }

    blk_queue_logical_block_size(data->block_dev.queue, LD2450_SECTOR_SIZE);
    data->block_dev.queue->queuedata = data;

    disk = alloc_disk(LD2450_BLOCK_MINORS);
    if (!disk) {
        dev_err(data->dev, "Failed to allocate disk\n");
        blk_cleanup_queue(data->block_dev.queue);
        data->error_count++;
        return -ENOMEM;
    }

    data->block_dev.disk = disk;
    disk->major = LD2450_BLOCK_MAJOR;
    disk->first_minor = 0;
    disk->fops = &(struct block_device_operations){
        .owner = THIS_MODULE,
    };
    disk->queue = data->block_dev.queue;
    disk->private_data = data;
    snprintf(disk->disk_name, 32, "ld2450_block");
    set_capacity(disk, LD2450_NUM_BLOCKS * (LD2450_BLOCK_SIZE / LD2450_SECTOR_SIZE));

    ret = add_disk(disk);
    if (ret) {
        dev_err(data->dev, "Failed to add disk: %d\n", ret);
        put_disk(disk);
        blk_cleanup_queue(data->block_dev.queue);
        data->error_count++;
        return ret;
    }

    dev_info(data->dev, "Block device created: /dev/%s\n", disk->disk_name);
    return 0;
}

void ld2450_remove_block_dev(struct ld2450_data *data)
{
    if (data->block_dev.disk) {
        del_gendisk(data->block_dev.disk);
        put_disk(data->block_dev.disk);
    }
    if (data->block_dev.queue)
        blk_cleanup_queue(data->block_dev.queue);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Simulated block device for Hi-Link LD2450 radar module on Raspberry Pi");
MODULE_VERSION("2.4.0");