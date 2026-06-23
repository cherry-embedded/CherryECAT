/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "chry_phy.h"

#define RTL8201_RMSR_P7 16 /* RMII Mode Setting Register */
#define RTL8201_PAGESEL 31 /* Page Select Register */

/*
 * RG_RMII_CLKDIR (RW)
 *
 * This Bit Sets the Type of TXC in RMII Mode.
 * 0: Output
 * 1: Input
 */
#define RTL8201_RMSR_P7_RG_RMII_CLKDIR_MASK   (0x1000U)
#define RTL8201_RMSR_P7_RG_RMII_CLKDIR_SHIFT  (12U)
#define RTL8201_RMSR_P7_RG_RMII_CLKDIR_SET(x) (((uint16_t)(x) << RTL8201_RMSR_P7_RG_RMII_CLKDIR_SHIFT) & RTL8201_RMSR_P7_RG_RMII_CLKDIR_MASK)
#define RTL8201_RMSR_P7_RG_RMII_CLKDIR_GET(x) (((uint16_t)(x)&RTL8201_RMSR_P7_RG_RMII_CLKDIR_MASK) >> RTL8201_RMSR_P7_RG_RMII_CLKDIR_SHIFT)

void rtl8201_phy_init(struct chry_phy_device *phydev, struct chry_phy_config *config)
{
    (void)phydev;
    (void)config;

    uint16_t regval;

    phydev->mdio_write(phydev, phydev->phy_addr, RTL8201_PAGESEL, 7);

    regval = phydev->mdio_read(phydev, phydev->phy_addr, RTL8201_RMSR_P7);
    regval &= ~RTL8201_RMSR_P7_RG_RMII_CLKDIR_MASK;
    regval |= RTL8201_RMSR_P7_RG_RMII_CLKDIR_SET(1);
    phydev->mdio_write(phydev, phydev->phy_addr, RTL8201_RMSR_P7, regval);
}

void rtl8201_phy_get_status(struct chry_phy_device *phydev, struct chry_phy_status *status)
{
    uint16_t regval;

    regval = phydev->mdio_read(phydev, phydev->phy_addr, MII_BMSR);
    status->link = regval & BMSR_LINKSTATUS;

    if (status->link) {
        regval = phydev->mdio_read(phydev, phydev->phy_addr, MII_BMCR);
        status->speed = regval & BMCR_SPEED100 ? 100 : 10;
        status->duplex = regval & BMCR_FULLDPLX;
    }
}

const struct chry_phy_driver rtl8201_driver = {
    .phy_id = 0x001CC810,
    .phy_id_mask = 0xFFFFFFF0,
    .phy_name = "RTL8201",
    .phy_desc = "REALTEK RTL8201 Ethernet PHY",
    .phy_init = rtl8201_phy_init,
    .phy_get_status = rtl8201_phy_get_status,
};