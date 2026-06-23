/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "chry_phy.h"

#define KSZ8081_PHY_CONTROL1 0x1EU /*!< The PHY control one register. */
#define KSZ8081_PHY_CONTROL2 0x1FU /*!< The PHY control two register. */

#define KSZ8081_PHY_CTL2_REFCLK_SELECT_MASK 0x0080U /*!< The PHY RMII reference clock select. */

void ksz8081_phy_init(struct chry_phy_device *phydev, struct chry_phy_config *config)
{
    (void)phydev;
    (void)config;

    uint16_t regval;

    regval = phydev->mdio_read(phydev, phydev->phy_addr, KSZ8081_PHY_CONTROL2);
    phydev->mdio_write(phydev, phydev->phy_addr, KSZ8081_PHY_CONTROL2, (regval | KSZ8081_PHY_CTL2_REFCLK_SELECT_MASK));
}

void ksz8081_phy_get_status(struct chry_phy_device *phydev, struct chry_phy_status *status)
{
    uint16_t regval;

    regval = phydev->mdio_read(phydev, phydev->phy_addr, KSZ8081_PHY_CONTROL1);
    status->link = regval & (1 << 8);

    if (status->link) {
        regval &= 0x07;

        switch (regval) {
            case 0x01:
                status->speed = 10;
                status->duplex = false;
                break;
            case 0x02:
                status->speed = 100;
                status->duplex = false;
                break;
            case 0x05:
                status->speed = 10;
                status->duplex = true;
                break;
            case 0x06:
                status->speed = 100;
                status->duplex = true;
                break;

            default:
                break;
        }
    }
}

const struct chry_phy_driver ksz8081_driver = {
    .phy_id = 0x00221560,
    .phy_id_mask = 0xFFFFFFF0,
    .phy_name = "KSZ8081",
    .phy_desc = "MICROCHIP KSZ8081 Ethernet PHY",
    .phy_init = ksz8081_phy_init,
    .phy_get_status = ksz8081_phy_get_status,
};