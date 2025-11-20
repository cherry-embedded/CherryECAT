/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_NETDEV_H
#define EC_NETDEV_H

#include "phy/chry_phy.h"

typedef struct ec_master ec_master_t;

typedef struct ec_netdev {
    ec_master_t *master;
    struct chry_phy_device phydev;
    ec_netdev_index_t index;
    char name[24];
    uint8_t mac_addr[6];
    bool link_state;
    uint8_t tx_frame_index;
    unsigned long jiffies_poll;

    // Frame statistics
    uint64_t tx_count;                     /**< Number of frames sent. */
    uint64_t last_tx_count;                /**< Number of frames sent of last statistics cycle.*/
    uint64_t rx_count;                     /**< Number of frames received. */
    uint64_t last_rx_count;                /**< Number of frames received of last statistics cycle.*/
    uint64_t tx_bytes;                     /**< Number of bytes sent. */
    uint64_t last_tx_bytes;                /**< Number of bytes sent of last statistics cycle.*/
    uint64_t rx_bytes;                     /**< Number of bytes received. */
    uint64_t last_rx_bytes;                /**< Number of bytes received of last statistics cycle.*/
    uint64_t tx_errors;                    /**< Number of transmit errors. */
    int32_t tx_frame_rates[EC_RATE_COUNT]; /**< Transmit rates in frames/s for different statistics cycle periods.*/
    int32_t rx_frame_rates[EC_RATE_COUNT]; /**< Receive rates in frames/s for different statistics cycle periods.*/
    int32_t tx_byte_rates[EC_RATE_COUNT];  /**< Transmit rates in byte/s for different statistics cycle periods.*/
    int32_t rx_byte_rates[EC_RATE_COUNT];  /**< Receive rates in byte/s for different statistics cycle periods.*/

} ec_netdev_t;

void ec_netdev_clear_stats(ec_netdev_t *netdev);
void ec_netdev_update_stats(ec_netdev_t *netdev);

ec_netdev_t *ec_netdev_init(uint8_t netdev_index);
void ec_netdev_poll_link_state(ec_netdev_t *netdev);
uint8_t *ec_netdev_get_txbuf(ec_netdev_t *netdev);
int ec_netdev_send(ec_netdev_t *netdev, uint32_t size);
void ec_netdev_receive(ec_netdev_t *netdev, uint8_t *frame, uint32_t size);

#endif