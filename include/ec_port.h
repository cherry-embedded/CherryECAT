/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_PORT_H
#define EC_PORT_H

#include "ec_netdev.h"

typedef void (*ec_htimer_cb)(void *arg);

ec_netdev_t *ec_netdev_low_level_init(uint8_t netdev_index);
#ifndef CONFIG_EC_PHY_CUSTOM
void ec_netdev_low_level_link_up(ec_netdev_t *netdev, struct chry_phy_status *status);
#else
void ec_netdev_low_level_poll_link_state(ec_netdev_t *netdev);
#endif
uint8_t *ec_netdev_low_level_get_txbuf(ec_netdev_t *netdev);
int ec_netdev_low_level_output(ec_netdev_t *netdev, uint32_t size);

void ec_mdio_low_level_write(struct chry_phy_device *phydev, uint16_t phy_addr, uint16_t regnum, uint16_t val);
uint16_t ec_mdio_low_level_read(struct chry_phy_device *phydev, uint16_t phy_addr, uint16_t regnum);

void ec_htimer_start(uint32_t us, ec_htimer_cb cb, void *arg);
void ec_htimer_stop(void);
void ec_htimer_update(uint32_t us);

uint32_t ec_get_cpu_frequency(void);

#endif