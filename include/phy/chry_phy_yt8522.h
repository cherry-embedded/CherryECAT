/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "chry_phy.h"

#define YT8522_PHYSTS 0x11 /* PHY Status Register */

#define YT8522_PHYSTS_LINK_STATUS (0x400U)
/*
 * SPEED (RO)
 *
 * Link Speed.
 * 11: Reserved  10: 1000Mbps
 * 01: 100Mbps  00: 10Mbps
 */
#define YT8522_PHYSTS_SPEED_STATUS  (0xC000U)
#define YT8522_PHYSTS_DUPLEX_STATUS (0x2000U)

void yt8522_phy_init(struct chry_phy_device *phydev, struct chry_phy_config *config)
{
    (void)phydev;
    (void)config;
}

void yt8522_phy_get_status(struct chry_phy_device *phydev, struct chry_phy_status *status)
{
    uint16_t regval;

    regval = phydev->mdio_read(phydev, phydev->phy_addr, YT8522_PHYSTS);

    status->link = regval & YT8522_PHYSTS_LINK_STATUS;

    if (status->link) {
        status->duplex = regval & YT8522_PHYSTS_DUPLEX_STATUS;

        switch ((regval & YT8522_PHYSTS_SPEED_STATUS) >> 14) {
            case 0:
                status->speed = 10;
                break;
            case 1:
                status->speed = 100;
                break;
            case 2:
                status->speed = 1000;
                break;

            default:
                break;
        }
    }
}

const struct chry_phy_driver yt8522_driver = {
    .phy_id = 0x4F51E900,
    .phy_id_mask = 0xFFFFFFC0,
    .phy_name = "YT8512/YT8522/YT8531",
    .phy_desc = "MOTOR COMM YT8512/YT8522/YT8531 Ethernet PHY",
    .phy_init = yt8522_phy_init,
    .phy_get_status = yt8522_phy_get_status,
};