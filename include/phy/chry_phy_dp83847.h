/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "chry_phy.h"

#define DP83847_PHYSTS 0x11 /* PHY Status Register */

#define DP83847_PHYSTS_LINK_STATUS (0x400U)
/*
 * SPEED (RO)
 *
 * Link Speed.
 * 11: Reserved  10: 1000Mbps
 * 01: 100Mbps  00: 10Mbps
 */
#define DP83847_PHYSTS_SPEED_STATUS  (0xC000U)
#define DP83847_PHYSTS_DUPLEX_STATUS (0x2000U)

void dp83847_phy_init(struct chry_phy_device *phydev, struct chry_phy_config *config)
{
    (void)phydev;
    (void)config;
}

void dp83847_phy_get_status(struct chry_phy_device *phydev, struct chry_phy_status *status)
{
    uint16_t regval;

    regval = phydev->mdio_read(phydev, phydev->phy_addr, DP83847_PHYSTS);

    status->link = regval & DP83847_PHYSTS_LINK_STATUS;

    if (status->link) {
        status->duplex = regval & DP83847_PHYSTS_DUPLEX_STATUS;

        switch ((regval & DP83847_PHYSTS_SPEED_STATUS) >> 14) {
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

const struct chry_phy_driver dp83847_driver = {
    .phy_id = 0x2000A230,
    .phy_id_mask = 0xFFFFFFF0,
    .phy_name = "DP83847",
    .phy_desc = "TI DP83847 Ethernet PHY",
    .phy_init = dp83847_phy_init,
    .phy_get_status = dp83847_phy_get_status,
};