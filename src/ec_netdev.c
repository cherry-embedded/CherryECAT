/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_master.h"

extern void ec_master_receive_datagrams(ec_master_t *master,
                                        ec_netdev_index_t netdev_index,
                                        const uint8_t *frame_data,
                                        size_t size);

/** List of intervals for statistics [s].
 */
const unsigned int netdev_rate_intervals[] = {
    1, 10, 60
};

EC_FAST_CODE_SECTION void ec_netdev_clear_stats(ec_netdev_t *netdev)
{
    unsigned int i;

    // zero frame statistics
    netdev->tx_count = 0;
    netdev->last_tx_count = 0;
    netdev->rx_count = 0;
    netdev->last_rx_count = 0;
    netdev->tx_bytes = 0;
    netdev->last_tx_bytes = 0;
    netdev->rx_bytes = 0;
    netdev->last_rx_bytes = 0;
    netdev->tx_errors = 0;

    for (i = 0; i < EC_RATE_COUNT; i++) {
        netdev->tx_frame_rates[i] = 0;
        netdev->rx_frame_rates[i] = 0;
        netdev->tx_byte_rates[i] = 0;
        netdev->rx_byte_rates[i] = 0;
    }
}

EC_FAST_CODE_SECTION void ec_netdev_update_stats(ec_netdev_t *netdev)
{
    unsigned int i;

    int32_t tx_frame_rate = (netdev->tx_count - netdev->last_tx_count) * 1000;
    int32_t rx_frame_rate = (netdev->rx_count - netdev->last_rx_count) * 1000;
    int32_t tx_byte_rate = (netdev->tx_bytes - netdev->last_tx_bytes);
    int32_t rx_byte_rate = (netdev->rx_bytes - netdev->last_rx_bytes);

    /* Low-pass filter:
     *      Y_n = y_(n - 1) + T / tau * (x - y_(n - 1))   | T = 1
     *   -> Y_n += (x - y_(n - 1)) / tau
     */
    for (i = 0; i < EC_RATE_COUNT; i++) {
        int32_t n = netdev_rate_intervals[i];
        netdev->tx_frame_rates[i] +=
            (tx_frame_rate - netdev->tx_frame_rates[i]) / n;
        netdev->rx_frame_rates[i] +=
            (rx_frame_rate - netdev->rx_frame_rates[i]) / n;
        netdev->tx_byte_rates[i] +=
            (tx_byte_rate - netdev->tx_byte_rates[i]) / n;
        netdev->rx_byte_rates[i] +=
            (rx_byte_rate - netdev->rx_byte_rates[i]) / n;
    }

    netdev->last_tx_count = netdev->tx_count;
    netdev->last_rx_count = netdev->rx_count;
    netdev->last_tx_bytes = netdev->tx_bytes;
    netdev->last_rx_bytes = netdev->rx_bytes;
}

ec_netdev_t *ec_netdev_init(uint8_t netdev_index)
{
    ec_netdev_t *netdev;

    netdev = ec_netdev_low_level_init(netdev_index);
    if (netdev) {
        ec_netdev_clear_stats(netdev);
        netdev->index = netdev_index;
        netdev->tx_frame_index = 0;
        netdev->link_state = false;

        snprintf(netdev->name, sizeof(netdev->name), "ec-netdev%d(%s)", netdev_index, netdev_index == 0 ? "main" : "backup");
#ifndef CONFIG_EC_PHY_CUSTOM
        struct chry_phy_config config;

        config.auto_negotiation = true;
        config.loopback = false;
        netdev->phydev.mdio_read = ec_mdio_low_level_read;
        netdev->phydev.mdio_write = ec_mdio_low_level_write;
        netdev->phydev.user_data = netdev;

        // Delay to ensure PHY power up and access to mac
        ec_osal_msleep(200);
        EC_ASSERT_MSG(chry_phy_init(&netdev->phydev, &config) == 0, "PHY init failed for netdev %d\n", netdev_index);

        EC_LOG_INFO("PHY info: \n");
        EC_LOG_INFO("  ID: 0x%08x\n", netdev->phydev.phy_id);
        EC_LOG_INFO("  Name: %s\n", netdev->phydev.driver->phy_name);
        EC_LOG_INFO("  Description: %s\n", netdev->phydev.driver->phy_desc);
#endif
    }

    return netdev;
}

void ec_netdev_poll_link_state(ec_netdev_t *netdev)
{
#ifndef CONFIG_EC_PHY_CUSTOM
    struct chry_phy_status status = { 0 };
    static struct chry_phy_status current_status = { 0 };

    chry_phy_get_status(&netdev->phydev, &status);

    EC_LOG_DBG("PHY link: %d, speed: %d, duplex: %d\n", status.link, status.speed, status.duplex);

    if (memcmp(&current_status, &status, sizeof(struct chry_phy_status)) != 0) {
        ec_memcpy(&current_status, &status, sizeof(struct chry_phy_status));

        if (status.link) {
            netdev->link_state = true;
            ec_netdev_low_level_link_up(netdev, &status);
        } else {
            netdev->link_state = false;
            ec_netdev_low_level_link_up(netdev, &status);
        }
    }
#else
    ec_netdev_low_level_poll_link_state(netdev);
#endif
}

EC_FAST_CODE_SECTION uint8_t *ec_netdev_get_txbuf(ec_netdev_t *netdev)
{
    return (ec_netdev_low_level_get_txbuf(netdev) + ETH_HLEN);
}

EC_FAST_CODE_SECTION int ec_netdev_send(ec_netdev_t *netdev, uint32_t size)
{
    if (ec_netdev_low_level_output(netdev, size + ETH_HLEN) == 0) {
        netdev->tx_count++;
        netdev->master->netdev_stats.tx_count++;
        netdev->tx_bytes += ETH_HLEN + size;
        netdev->master->netdev_stats.tx_bytes += ETH_HLEN + size;

        return 0;
    } else {
        netdev->tx_errors++;
        return -1;
    }
}

EC_FAST_CODE_SECTION void ec_netdev_receive(ec_netdev_t *netdev, uint8_t *frame, uint32_t size)
{
    const uint8_t *ec_data = frame + ETH_HLEN;
    uint32_t ec_size = size - ETH_HLEN;

    netdev->rx_count++;
    netdev->master->netdev_stats.rx_count++;
    netdev->rx_bytes += size;
    netdev->master->netdev_stats.rx_bytes += size;

    ec_master_receive_datagrams(netdev->master, netdev->index, ec_data, ec_size);
}