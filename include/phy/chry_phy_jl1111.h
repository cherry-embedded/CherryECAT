/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "chry_phy.h"

void jl1111_phy_init(struct chry_phy_device *phydev, struct chry_phy_config *config)
{
    (void)phydev;
    (void)config;
}

void jl1111_phy_get_status(struct chry_phy_device *phydev, struct chry_phy_status *status)
{
    uint16_t regval;

    regval = phydev->mdio_read(phydev, phydev->phy_addr, MII_BMSR);
    status->link = regval & BMSR_LINKSTATUS;

    if (status->link) {
        regval = phydev->mdio_read(phydev, phydev->phy_addr, MII_ANAR) & phydev->mdio_read(phydev, phydev->phy_addr, MII_ANLPAR);

        if (regval & ANAR_100HALF) {
            if (regval & ANAR_100FULL) {
                status->speed = 100;
                status->duplex = true;
            } else {
                status->speed = 100;
                status->duplex = false;
            }
        } else if (regval & ANAR_10HALF) {
            if (regval & ANAR_10FULL) {
                status->speed = 10;
                status->duplex = true;
            } else {
                status->speed = 10;
                status->duplex = false;
            }
        }
    }
}

const struct chry_phy_driver jl1111_driver = {
    .phy_id = 0x937C4020,
    .phy_id_mask = 0xFFFFFFF0,
    .phy_name = "JL1111",
    .phy_desc = "JLSemi JL1111 Ethernet PHY",
    .phy_init = jl1111_phy_init,
    .phy_get_status = jl1111_phy_get_status,
};