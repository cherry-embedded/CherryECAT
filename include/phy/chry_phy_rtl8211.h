/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "chry_phy.h"

#define RTL8211_PHYSR 17 /* PHY Specific Status Register */

#define RTL8211_PHYSTS_LINK_STATUS (0x400U)
/*
 * SPEED (RO)
 *
 * Link Speed.
 * 11: Reserved  10: 1000Mbps
 * 01: 100Mbps  00: 10Mbps
 */
#define RTL8211_PHYSTS_SPEED_STATUS  (0xC000U)
#define RTL8211_PHYSTS_DUPLEX_STATUS (0x2000U)

void rtl8211_phy_init(struct chry_phy_device *phydev, struct chry_phy_config *config)
{
    (void)phydev;
    (void)config;
}

void rtl8211_phy_get_status(struct chry_phy_device *phydev, struct chry_phy_status *status)
{
    uint16_t regval;

    regval = phydev->mdio_read(phydev, phydev->phy_addr, RTL8211_PHYSR);

    status->link = regval & RTL8211_PHYSTS_LINK_STATUS;

    if (status->link) {
        status->duplex = regval & RTL8211_PHYSTS_DUPLEX_STATUS;

        switch ((regval & RTL8211_PHYSTS_SPEED_STATUS) >> 14) {
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

const struct chry_phy_driver rtl8211_driver = {
    .phy_id = 0x001CC910,
    .phy_id_mask = 0xFFFFFFF0,
    .phy_name = "RTL8211",
    .phy_desc = "REALTEK RTL8211 Ethernet PHY",
    .phy_init = rtl8211_phy_init,
    .phy_get_status = rtl8211_phy_get_status,
};