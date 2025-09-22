/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ld2450_net.c - Simulated network driver for Hi-Link LD2450 radar module
 * Author: Nguyen Nhan
 * Date: 22nd September 2025
 * Version: 2.4.0
 * Description: Implements a simulated network device for learning purposes on Raspberry Pi
 * Network Device:
 *   - Registers a netdev interface (ld2450_net) to send/receive tracking data
 *   - Uses in-memory buffer to simulate packet transmission
 *   - Supports basic netdev operations (open, stop, transmit)
 * Changelog:
 *   - 2.4.0: Initial implementation of network driver
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include "LD2450.h"

#define LD2450_NET_NAME "ld2450_net"
#define LD2450_MTU 1500

struct ld2450_net_priv {
    struct net_device *netdev;
    struct ld2450_data *data;
    u32 tx_packets;
    u32 rx_packets;
    u32 tx_errors;
};

static int ld2450_net_open(struct net_device *dev)
{
    struct ld2450_net_priv *priv = netdev_priv(dev);
    netif_start_queue(dev);
    dev_info(&dev->dev, "Network device %s opened\n", dev->name);
    return 0;
}

static int ld2450_net_stop(struct net_device *dev)
{
    netif_stop_queue(dev);
    dev_info(&dev->dev, "Network device %s stopped\n", dev->name);
    return 0;
}

static netdev_tx_t ld2450_net_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct ld2450_net_priv *priv = netdev_priv(dev);
    struct ld2450_tracking_data tracking;

    if (skb->len != sizeof(struct ld2450_tracking_data)) {
        dev_err(&dev->dev, "Invalid packet size\n");
        priv->tx_errors++;
        dev_kfree_skb(skb);
        return NETDEV_TX_OK;
    }

    if (copy_from_user(&tracking, skb->data, sizeof(tracking))) {
        dev_err(&dev->dev, "Failed to copy packet data\n");
        priv->tx_errors++;
        dev_kfree_skb(skb);
        return NETDEV_TX_OK;
    }

    spin_lock(&priv->data->ld2450_lock);
    priv->data->x_pos = tracking.x_pos;
    priv->data->y_pos = tracking.y_pos;
    priv->data->velocity = tracking.velocity;
    priv->data->distance = tracking.distance;
    spin_unlock(&priv->data->ld2450_lock);

    priv->tx_packets++;
    dev_kfree_skb(skb);
    return NETDEV_TX_OK;
}

static const struct net_device_ops ld2450_net_ops = {
    .ndo_open = ld2450_net_open,
    .ndo_stop = ld2450_net_stop,
    .ndo_start_xmit = ld2450_net_xmit,
};

int ld2450_create_netdev(struct ld2450_data *data)
{
    struct net_device *netdev;
    struct ld2450_net_priv *priv;
    int ret;

    netdev = alloc_etherdev(sizeof(struct ld2450_net_priv));
    if (!netdev) {
        dev_err(data->dev, "Failed to allocate netdev\n");
        data->error_count++;
        return -ENOMEM;
    }

    priv = netdev_priv(netdev);
    priv->netdev = netdev;
    priv->data = data;

    netdev->netdev_ops = &ld2450_net_ops;
    netdev->mtu = LD2450_MTU;
    strscpy(netdev->name, LD2450_NET_NAME, IFNAMSIZ);
    eth_random_addr(netdev->dev_addr);

    ret = register_netdev(netdev);
    if (ret) {
        dev_err(data->dev, "Failed to register netdev: %d\n", ret);
        free_netdev(netdev);
        data->error_count++;
        return ret;
    }

    dev_info(data->dev, "Network device %s registered\n", netdev->name);
    return 0;
}

void ld2450_remove_netdev(struct ld2450_data *data)
{
    struct ld2450_net_priv *priv = netdev_priv(data->netdev);
    if (priv->netdev) {
        unregister_netdev(priv->netdev);
        free_netdev(priv->netdev);
    }
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Simulated network device for Hi-Link LD2450 radar module on Raspberry Pi");
MODULE_VERSION("2.4.0");