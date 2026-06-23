/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "chry_phy.h"

#define DP83848_PHYSTS 0x10 /* PHY Status Register */

#define DP83848_PHYSTS_LINK_STATUS (0x0001U)
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
#define DP83848_PHYSTS_SPEED_STATUS  (0x0002U)
#define DP83848_PHYSTS_DUPLEX_STATUS (0x0004U)

void dp83848_phy_init(struct chry_phy_device *phydev, struct chry_phy_config *config)
{
    (void)phydev;
    (void)config;
}

void dp83848_phy_get_status(struct chry_phy_device *phydev, struct chry_phy_status *status)
{
    uint16_t regval;

    regval = phydev->mdio_read(phydev, phydev->phy_addr, DP83848_PHYSTS);

    status->link = regval & DP83848_PHYSTS_LINK_STATUS;

    if (status->link) {
        status->duplex = regval & DP83848_PHYSTS_DUPLEX_STATUS;
        status->speed = (regval & DP83848_PHYSTS_SPEED_STATUS) ? 10 : 100;
    }
}

const struct chry_phy_driver dp83848_driver = {
    .phy_id = 0x20005C90,
    .phy_id_mask = 0xFFFFFFF0,
    .phy_name = "DP83848",
    .phy_desc = "TI DP83848 Ethernet PHY",
    .phy_init = dp83848_phy_init,
    .phy_get_status = dp83848_phy_get_status,
};