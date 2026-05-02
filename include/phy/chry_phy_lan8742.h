/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "chry_phy.h"

/* LAN8742A PHY Special Control/Status Register (Register 0x1F / 31)
 * Reference: STM32 LAN8742 BSP driver */
#define LAN8742_PHYSCSR 31 /* PHY Special Control/Status Register */

/* PHYSCSR Register bit definitions */
#define LAN8742_PHYSCSR_AUTONEGO_DONE   (0x1000U)  /* Auto-negotiation complete */
#define LAN8742_PHYSCSR_HCDSPEEDMASK    (0x001CU)  /* Speed/Duplex mask (bits [4:2]) */

/* Speed and Duplex status values (read from PHYSCSR & HCDSPEEDMASK) */
#define LAN8742_PHYSCSR_10BT_HD         (0x0004U)  /* 10Base-T Half Duplex */
#define LAN8742_PHYSCSR_10BT_FD         (0x0014U)  /* 10Base-T Full Duplex */
#define LAN8742_PHYSCSR_100BTX_HD       (0x0008U)  /* 100Base-TX Half Duplex */
#define LAN8742_PHYSCSR_100BTX_FD       (0x0018U)  /* 100Base-TX Full Duplex */

void lan8742_phy_init(struct chry_phy_device *phydev, struct chry_phy_config *config)
{
    /* LAN8742A does not require special initialization beyond standard PHY reset and config
     * which is handled by chry_phy_init in ec_netdev.c */
}

void lan8742_phy_get_status(struct chry_phy_device *phydev, struct chry_phy_status *status)
{
    uint16_t regval;

    /* Read Basic Status Register to check link status */
    regval = phydev->mdio_read(phydev, phydev->phy_addr, MII_BMSR);
    status->link = (regval & BMSR_LINKSTATUS) ? true : false;

    if (status->link) {
        /* Read PHY Special Control/Status Register for speed and duplex info */
        regval = phydev->mdio_read(phydev, phydev->phy_addr, LAN8742_PHYSCSR);
        
        /* Extract speed and duplex bits [4:2] */
        uint16_t speed_duplex = regval & LAN8742_PHYSCSR_HCDSPEEDMASK;
        
        switch (speed_duplex) {
            case LAN8742_PHYSCSR_10BT_HD:
                status->speed = 10;
                status->duplex = false;
                break;
            case LAN8742_PHYSCSR_10BT_FD:
                status->speed = 10;
                status->duplex = true;
                break;
            case LAN8742_PHYSCSR_100BTX_HD:
                status->speed = 100;
                status->duplex = false;
                break;
            case LAN8742_PHYSCSR_100BTX_FD:
                status->speed = 100;
                status->duplex = true;
                break;
            default:
                status->speed = 0;
                status->duplex = false;
                break;
        }
    }
}

const struct chry_phy_driver lan8742_driver = {
    .phy_id = 0x0007C130,           /* LAN8742A PHY ID base (matches 0x0007C13x with mask) */
    .phy_id_mask = 0xFFFFFFF0,      /* Mask for comparing PHY ID (matches revision 0x0-0xF) */
    .phy_name = "LAN8742A",
    .phy_desc = "MICROCHIP LAN8742A Ethernet PHY",
    .phy_init = lan8742_phy_init,
    .phy_get_status = lan8742_phy_get_status,
};
