/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "chry_phy.h"

#define DP83825_PHYSTS 0x10 /* PHY Status Register */
#define DP83825_PHYCR 0x19

#define DP83825_PHYSTS_LINK_STATUS (0x0001U)
/*
 * SPEED_STATUS (RO)
 *
 * Speed10:
 * This bit indicates the status of the speed and is determined from Auto-Negotiation or Forced
 * Modes.
 * 1 = 10 Mb/s mode.
 * 0 = 100 Mb/s mode.
 * Note: This bit is only valid if Auto-Negotiation is enabled and complete and there is a valid
 * link or if Auto-Negotiation is disabled and there is a valid link.
 */
#define DP83825_PHYSTS_SPEED_STATUS  (0x0002U)
#define DP83825_PHYSTS_DUPLEX_STATUS (0x0004U)

void dp83825_phy_init(struct chry_phy_device *phydev, struct chry_phy_config *config)
{
    (void)phydev;
    (void)config;
//    uint16_t data = 0;
//
//    data |= config->auto_negotiation ? 0x1000U : 0;
//
//    if (config->auto_negotiation == false) {
//        data |= 0x2000U;      /* Set port speed */
//        data |= 0x100U;     /* Set duplex mode */
//    }
//
//
//    phydev->mdio_write(phydev, phydev->phy_addr, MII_BMCR, data);
//    data = phydev->mdio_read(phydev, phydev->phy_addr, DP83825_PHYCR); //DP83825_PHYCR
//    data &= 0x3f;
//    phydev->mdio_write(phydev, phydev->phy_addr, DP83825_PHYCR, data); //disable auto MDIX
}

void dp83825_phy_get_status(struct chry_phy_device *phydev, struct chry_phy_status *status)
{
    uint16_t regval;

    regval = phydev->mdio_read(phydev, phydev->phy_addr, DP83825_PHYSTS);

    status->link = regval & DP83825_PHYSTS_LINK_STATUS;

    if (status->link) {
        status->duplex = regval & DP83825_PHYSTS_DUPLEX_STATUS;
        status->speed = (regval & DP83825_PHYSTS_SPEED_STATUS) ? 10 : 100;
    }
}

const struct chry_phy_driver dp83825_driver = {
    .phy_id = 0x2000A000,
    .phy_id_mask = 0xFFFFFC00,
    .phy_name = "DP83825",
    .phy_desc = "TI DP83825 Ethernet PHY",
    .phy_init = dp83825_phy_init,
    .phy_get_status = dp83825_phy_get_status,
};