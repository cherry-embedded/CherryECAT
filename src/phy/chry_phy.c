/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_config.h"

#ifndef CONFIG_EC_PHY_CUSTOM
#include "phy/chry_phy.h"

#include "phy/chry_phy_dp83848.h"
#include "phy/chry_phy_dp83847.h"
#include "phy/chry_phy_jl1111.h"
#include "phy/chry_phy_ksz8081.h"
#include "phy/chry_phy_lan8720.h"
#include "phy/chry_phy_rtl8201.h"
#include "phy/chry_phy_rtl8211.h"
#include "phy/chry_phy_yt8522.h"

const struct chry_phy_driver *g_phy_driver_table[] = {
    &dp83848_driver,
    &dp83847_driver,
    &jl1111_driver,
    &ksz8081_driver,
    &lan8720_driver,
    &rtl8201_driver,
    &rtl8211_driver,
    &yt8522_driver,

};

int chry_phy_init(struct chry_phy_device *phydev, struct chry_phy_config *config)
{
    const struct chry_phy_driver *phydrv = NULL;
    uint16_t phy_id1;
    uint16_t phy_id2;
    uint16_t regval = 0;
    uint32_t phy_id;
    bool extended_status = false;
    uint16_t phy_addr;

    for (uint16_t i = 0; i < 32; i++) {
        phy_addr = i;
        phy_id1 = phydev->mdio_read(phydev, phy_addr, MII_PHYSID1);
        phy_id2 = phydev->mdio_read(phydev, phy_addr, MII_PHYSID2);

        phy_id = (phy_id1 << 16) | phy_id2;
        for (uint32_t i = 0; i < sizeof(g_phy_driver_table) / sizeof(g_phy_driver_table[0]); i++) {
            if (g_phy_driver_table[i]->phy_id == (phy_id & g_phy_driver_table[i]->phy_id_mask)) {
                phydrv = g_phy_driver_table[i];
                goto phydrv_found;
            }
        }
    }

    if (phydrv == NULL) {
        return -1;
    }

    /* PHY reset */
    phydev->mdio_write(phydev, phy_addr, MII_BMCR, BMCR_RESET);
    while (phydev->mdio_read(phydev, phy_addr, MII_BMCR) & BMCR_RESET) {
    }

phydrv_found:
    phydev->phy_port = config->phy_port;
    phydev->phy_id = phy_id;
    phydev->phy_addr = phy_addr;
    phydev->driver = phydrv;

    regval = phydev->mdio_read(phydev, phy_addr, MII_BMSR);

    phydev->support.support_pause = 1;

    if (regval & BMSR_100T4) {
        phydev->support.support_100base_t4 = 1;
    }
    if (regval & BMSR_100FULL) {
        phydev->support.support_100base_tx_full = 1;
    }
    if (regval & BMSR_100HALF) {
        phydev->support.support_1000base_tx_half = 1;
    }
    if (regval & BMSR_10FULL) {
        phydev->support.support_10base_tx_full = 1;
    }
    if (regval & BMSR_10HALF) {
        phydev->support.support_10base_tx_half = 1;
    }
    if (regval & BMSR_ANEGCAPABLE) {
        phydev->support.support_autoeng = 1;
    }

    if (regval & BMSR_ESTATEN) {
        regval = phydev->mdio_read(phydev, phy_addr, MII_GBESR);
        if (regval & GBESR_1000_TFULL) {
            phydev->support.support_1000base_tx_full = 1;
        }
        if (regval & GBESR_1000_THALF) {
            phydev->support.support_1000base_tx_half = 1;
        }
        extended_status = true;
    }

    regval = 0;
    if (config->loopback) {
        regval |= BMCR_LOOPBACK;
    }

    if (config->auto_negotiation) {
        regval |= BMCR_ANENABLE;
        regval |= BMCR_ANRESTART;
    } else {
        if (config->speed == 100) {
            regval |= BMCR_SPEED100;
        } else if (config->speed == 1000) {
            regval |= BMCR_SPEED100;
            regval |= BMCR_SPEED1000;
        } else {
            regval &= ~BMCR_SPEED100;
        }
    }
    phydev->mdio_write(phydev, phy_addr, MII_BMCR, regval);

    regval = phydev->mdio_read(phydev, phy_addr, MII_ANAR);
    regval &= ~(ANAR_SPEED_ALL | ANAR_SLCT | ANAR_PAUSE | ANAR_ASYM_PAUSE);
    regval |= phydev->support.support_100base_t4 ? ANAR_100T4 : 0;
    regval |= phydev->support.support_100base_tx_full ? ANAR_100FULL : 0;
    regval |= phydev->support.support_100base_tx_half ? ANAR_100HALF : 0;
    regval |= phydev->support.support_10base_tx_full ? ANAR_10FULL : 0;
    regval |= phydev->support.support_10base_tx_half ? ANAR_10HALF : 0;
    regval |= phydev->support.support_pause ? ANAR_PAUSE : 0;
    regval |= phydev->support.support_asym_pause ? ANAR_ASYM_PAUSE : 0;
    regval |= ANAR_CSMA;
    phydev->mdio_write(phydev, phy_addr, MII_ANAR, regval);

    if (extended_status) {
        regval = phydev->mdio_read(phydev, phy_addr, MII_GBCR);
        regval &= ~(GBCR_1000FULL | GBCR_1000HALF);
        regval |= phydev->support.support_1000base_tx_full ? GBCR_1000FULL : 0;
        regval |= phydev->support.support_1000base_tx_half ? GBCR_1000HALF : 0;
        phydev->mdio_write(phydev, phy_addr, MII_GBCR, regval);
    }

    if (phydev->driver && phydev->driver->phy_init) {
        phydev->driver->phy_init(phydev, config);
    }
    return 0;
}

void chry_phy_get_status(struct chry_phy_device *phydev, struct chry_phy_status *status)
{
    if (phydev->driver && phydev->driver->phy_get_status) {
        phydev->driver->phy_get_status(phydev, status);
    }
}
#endif