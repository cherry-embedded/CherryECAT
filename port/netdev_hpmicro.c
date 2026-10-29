/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "hpm_clock_drv.h"
#include "hpm_enet_drv.h"
#include "hpm_otp_drv.h"
#include "hpm_l1c_drv.h"
#include "board.h"
#include "ec_master.h"
#include "hpm_sdk_version.h"

#if defined(RGMII) && RGMII
#define HPM_ENET_RGMII 1
#endif

#if defined(RMII) && RMII
#define HPM_ENET_RMII 1
#endif

#if defined(HPM_ENET_RGMII) && defined(HPM_ENET_RMII)
#define BOARD_NETDEV_COUNT 2 // only support hpm6750
#else
#define BOARD_NETDEV_COUNT 1
#endif

#if !defined(HPM_ENET_RGMII) && !defined(HPM_ENET_RMII)
#error "Please select RGMII or RMII"
#endif

#define MAC_ADDR0 0x00
#define MAC_ADDR1 0x80
#define MAC_ADDR2 0xE1
#define MAC_ADDR3 0x00
#define MAC_ADDR4 0x00
#define MAC_ADDR5 0x00

#define ENET_TX_BUFF_COUNT CONFIG_EC_MAX_ENET_TXBUF_COUNT
#define ENET_RX_BUFF_COUNT CONFIG_EC_MAX_ENET_RXBUF_COUNT
#define ENET_RX_BUFF_SIZE  1536
#define ENET_TX_BUFF_SIZE  1536

ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ENET_SOC_DESC_ADDR_ALIGNMENT)
__RW enet_rx_desc_t dma_rx_desc_tab[BOARD_NETDEV_COUNT][ENET_RX_BUFF_COUNT]; /* Ethernet Rx DMA Descriptor */

ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ENET_SOC_DESC_ADDR_ALIGNMENT)
__RW enet_tx_desc_t dma_tx_desc_tab[BOARD_NETDEV_COUNT][ENET_TX_BUFF_COUNT]; /* Ethernet Tx DMA Descriptor */

ATTR_PLACE_AT_FAST_RAM_WITH_ALIGNMENT(ENET_SOC_BUFF_ADDR_ALIGNMENT)
__RW uint8_t rx_buff[BOARD_NETDEV_COUNT][ENET_RX_BUFF_COUNT][ENET_RX_BUFF_SIZE]; /* Ethernet Receive Buffer */

ATTR_PLACE_AT_FAST_RAM_WITH_ALIGNMENT(ENET_SOC_BUFF_ADDR_ALIGNMENT)
__RW uint8_t tx_buff[BOARD_NETDEV_COUNT][ENET_TX_BUFF_COUNT][ENET_TX_BUFF_SIZE]; /* Ethernet Transmit Buffer */

enet_desc_t desc[BOARD_NETDEV_COUNT];
uint8_t mac[BOARD_NETDEV_COUNT][ETH_ALEN];

ec_netdev_t g_netdev[BOARD_NETDEV_COUNT];

ATTR_WEAK void enet_get_mac_address(uint8_t *mac)
{
    bool invalid = true;

    uint32_t uuid[(ETH_ALEN + (ETH_ALEN - 1)) / sizeof(uint32_t)];

    for (int i = 0; i < ARRAY_SIZE(uuid); i++) {
        uuid[i] = otp_read_from_shadow(OTP_SOC_UUID_IDX + i);
        if (uuid[i] != 0xFFFFFFFFUL && uuid[i] != 0) {
            invalid = false;
        }
    }

    if (invalid == true) {
        ec_memcpy(mac, &uuid, ETH_ALEN);
    } else {
        mac[0] = MAC_ADDR0;
        mac[1] = MAC_ADDR1;
        mac[2] = MAC_ADDR2;
        mac[3] = MAC_ADDR3;
        mac[4] = MAC_ADDR4;
        mac[5] = MAC_ADDR5;
    }
}

hpm_stat_t enet_init(uint8_t idx)
{
    enet_int_config_t int_config = { .int_enable = 0, .int_mask = 0 };
    enet_mac_config_t enet_config;
    enet_tx_control_config_t enet_tx_control_config;
    enet_inf_type_t intf;
    ENET_Type *ptr;
#if defined(HPM_ENET_RGMII) && defined(HPM_ENET_RMII)
    ptr = (idx == 0) ? HPM_ENET0 : HPM_ENET1;
    intf = (idx == 0) ? enet_inf_rgmii : enet_inf_rmii;
#else
    ptr = HPM_ENET0;
#if defined(HPM_ENET_RGMII)
    intf = enet_inf_rgmii;
#elif defined(HPM_ENET_RMII)
    intf = enet_inf_rmii;
#endif
#endif

    /* Initialize td, rd and the corresponding buffers */
    memset((uint8_t *)dma_tx_desc_tab[idx], 0x00, sizeof(dma_tx_desc_tab[idx]));
    memset((uint8_t *)dma_rx_desc_tab[idx], 0x00, sizeof(dma_rx_desc_tab[idx]));
    memset((uint8_t *)rx_buff[idx], 0x00, sizeof(rx_buff[idx]));
    memset((uint8_t *)tx_buff[idx], 0x00, sizeof(tx_buff[idx]));

    desc[idx].tx_desc_list_head = (enet_tx_desc_t *)core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)dma_tx_desc_tab[idx]);
    desc[idx].rx_desc_list_head = (enet_rx_desc_t *)core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)dma_rx_desc_tab[idx]);

    desc[idx].tx_buff_cfg.buffer = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)tx_buff[idx]);
    desc[idx].tx_buff_cfg.count = ENET_TX_BUFF_COUNT;
    desc[idx].tx_buff_cfg.size = ENET_TX_BUFF_SIZE;

    desc[idx].rx_buff_cfg.buffer = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)rx_buff[idx]);
    desc[idx].rx_buff_cfg.count = ENET_RX_BUFF_COUNT;
    desc[idx].rx_buff_cfg.size = ENET_RX_BUFF_SIZE;

    /*Get a default control config for tx descriptor */
    enet_get_default_tx_control_config(ptr, &enet_tx_control_config);

    /* Set the control config for tx descriptor */
    ec_memcpy(&desc[idx].tx_control_config, &enet_tx_control_config, sizeof(enet_tx_control_config_t));

    /* Get MAC address */
    enet_get_mac_address(mac[idx]);

    /* Set MAC0 address */
    enet_config.mac_addr_high[0] = mac[idx][5] << 8 | mac[idx][4];
    enet_config.mac_addr_low[0] = mac[idx][3] << 24 | mac[idx][2] << 16 | mac[idx][1] << 8 | mac[idx][0];
    enet_config.valid_max_count = 1;

    /* Set DMA PBL */
    enet_config.dma_pbl = board_get_enet_dma_pbl(ptr);

    /* Set SARC */
    enet_config.sarc = enet_sarc_replace_mac0;

    /* Initialize enet controller */
    if (enet_controller_init(ptr, intf, &desc[idx], &enet_config, &int_config) != status_success) {
        return status_fail;
    }

    return status_success;
}

ec_netdev_t *ec_netdev_low_level_init(uint8_t netdev_index)
{
    ENET_Type *ptr;
#if defined(HPM_ENET_RGMII) && defined(HPM_ENET_RMII)
    ptr = (netdev_index == 0) ? HPM_ENET0 : HPM_ENET1;
#else
    ptr = HPM_ENET0;
#endif

    if (netdev_index >= BOARD_NETDEV_COUNT) {
        EC_LOG_ERR("Invalid netdev index: %d\n", netdev_index);
        return NULL;
    }

    /* Initialize GPIOs */
    board_init_enet_pins(ptr);

    /* Reset an enet PHY */
    board_reset_enet_phy(ptr);

#if defined(HPM_ENET_RGMII) && !defined(HPM_ENET_RMII)
    /* Set RGMII clock delay */
    board_init_enet_rgmii_clock_delay(ptr);
#elif !defined(HPM_ENET_RGMII) && defined(HPM_ENET_RMII)
    /* Set RMII reference clock */
    board_init_enet_rmii_reference_clock(ptr, BOARD_ENET_RMII_INT_REF_CLK);
    EC_LOG_DBG("Reference Clock: %s\n", BOARD_ENET_RMII_INT_REF_CLK ? "Internal Clock" : "External Clock");
#else
    if (ptr == HPM_ENET0) {
        /* Set RGMII clock delay */
        board_init_enet_rgmii_clock_delay(ptr);
    } else {
        /* Set RMII reference clock */
        board_init_enet_rmii_reference_clock(ptr, BOARD_ENET_RMII_INT_REF_CLK);
        EC_LOG_DBG("Reference Clock: %s\n", BOARD_ENET_RMII_INT_REF_CLK ? "Internal Clock" : "External Clock");
    }
#endif

    /* Initialize MAC and DMA */
    if (enet_init(netdev_index) == 0) {
    } else {
        EC_LOG_DBG("Enet initialization fails !!!\n");
        while (1) {
        }
    }

    ec_memcpy(g_netdev[netdev_index].mac_addr, mac[netdev_index], ETH_ALEN);

    for (uint32_t i = 0; i < ENET_TX_BUFF_COUNT; i++) {
        for (uint8_t j = 0; j < 6; j++) { // dst MAC
            EC_WRITE_U8(&tx_buff[netdev_index][i][j], 0xFF);
        }
        for (uint8_t j = 0; j < 6; j++) { // src MAC
            EC_WRITE_U8(&tx_buff[netdev_index][i][6 + j], mac[netdev_index][j]);
        }
        EC_WRITE_U16(&tx_buff[netdev_index][i][12], ec_htons(0x88a4));
    }

    return &g_netdev[netdev_index];
}

void ec_netdev_low_level_link_up(ec_netdev_t *netdev, struct chry_phy_status *status)
{
    ENET_Type *ptr;
#if defined(HPM_ENET_RGMII) && defined(HPM_ENET_RMII)
    ptr = (netdev->index == 0) ? HPM_ENET0 : HPM_ENET1;
#else
    ptr = HPM_ENET0;
#endif

    enet_line_speed_t line_speed = enet_line_speed_10mbps;

    switch (status->speed) {
        case 10:
            line_speed = enet_line_speed_10mbps;
            break;
        case 100:
            line_speed = enet_line_speed_100mbps;
            break;
        case 1000:
            line_speed = enet_line_speed_1000mbps;
            break;

        default:
            break;
    }
    if (status->link) {
        enet_set_line_speed(ptr, line_speed);
        enet_set_duplex_mode(ptr, status->duplex);
    } else {
    }
}

EC_FAST_CODE_SECTION uint8_t *ec_netdev_low_level_get_txbuf(ec_netdev_t *netdev)
{
    (void)netdev;

    __IO enet_tx_desc_t *dma_tx_desc;

    dma_tx_desc = desc[netdev->index].tx_desc_list_cur;

    EC_ASSERT_MSG(dma_tx_desc->tdes0_bm.own == 0, "No free tx buffer available\n");

    return (uint8_t *)sys_address_to_core_local_mem(BOARD_RUNNING_CORE, dma_tx_desc->tdes2_bm.buffer1);
}

EC_FAST_CODE_SECTION int ec_netdev_low_level_output(ec_netdev_t *netdev, uint32_t size)
{
    ENET_Type *ptr;
#if defined(HPM_ENET_RGMII) && defined(HPM_ENET_RMII)
    ptr = (netdev->index == 0) ? HPM_ENET0 : HPM_ENET1;
#else
    ptr = HPM_ENET0;
#endif

    __IO enet_tx_desc_t *dma_tx_desc;

    dma_tx_desc = desc[netdev->index].tx_desc_list_cur;
    if (dma_tx_desc->tdes0_bm.own != 0) {
        return -1;
    }

    /* Prepare transmit descriptors to give to DMA*/
    enet_prepare_transmission_descriptors(ptr, &desc[netdev->index].tx_desc_list_cur, size + 4, desc[netdev->index].tx_buff_cfg.size);

    return 0;
}

EC_FAST_CODE_SECTION int ec_netdev_low_level_get_rxbuf(ec_netdev_t *netdev, uint8_t **buffer, uint32_t *size)
{
    ENET_Type *ptr;
#if defined(HPM_ENET_RGMII) && defined(HPM_ENET_RMII)
    ptr = (netdev->index == 0) ? HPM_ENET0 : HPM_ENET1;
#else
    ptr = HPM_ENET0;
#endif

    enet_frame_t frame = { 0 };
    /* Check and get a received frame */
    if (enet_check_received_frame(&desc[netdev->index].rx_desc_list_cur, &desc[netdev->index].rx_frame_info) == 1) {
        frame = enet_get_received_frame(&desc[netdev->index].rx_desc_list_cur, &desc[netdev->index].rx_frame_info);
    }
    if (frame.length > 0) {
        *buffer = (uint8_t *)sys_address_to_core_local_mem(BOARD_RUNNING_CORE, (uint32_t)frame.buffer);
        *size = frame.length;
        return 0;
    } else {
        enet_rx_resume(ptr);
        return -1;
    }
}

EC_FAST_CODE_SECTION void ec_netdev_low_level_release_rxbuf(ec_netdev_t *netdev)
{
    ENET_Type *ptr;
#if defined(HPM_ENET_RGMII) && defined(HPM_ENET_RMII)
    ptr = (netdev->index == 0) ? HPM_ENET0 : HPM_ENET1;
#else
    ptr = HPM_ENET0;
#endif

    /* Release the current RX buffer back to DMA */
    if (desc[netdev->index].rx_frame_info.seg_count > 0) {
        enet_rx_desc_t *dma_rx_desc = desc[netdev->index].rx_frame_info.fs_rx_desc;
        for (uint32_t i = 0; i < desc[netdev->index].rx_frame_info.seg_count; i++) {
            dma_rx_desc->rdes0_bm.own = 1;
            dma_rx_desc = (enet_rx_desc_t *)(dma_rx_desc->rdes3_bm.next_desc);
        }
        desc[netdev->index].rx_frame_info.seg_count = 0;
    }
    enet_rx_resume(ptr);
}

void ec_mdio_low_level_write(struct chry_phy_device *phydev, uint16_t phy_addr, uint16_t regnum, uint16_t val)
{
    ENET_Type *ptr;
#if defined(HPM_ENET_RGMII) && defined(HPM_ENET_RMII)
    ptr = (phydev->phy_port == 0) ? HPM_ENET0 : HPM_ENET1;
#else
    ptr = HPM_ENET0;
#endif

    enet_write_phy(ptr, phy_addr, regnum, val);
}

uint16_t ec_mdio_low_level_read(struct chry_phy_device *phydev, uint16_t phy_addr, uint16_t regnum)
{
    ENET_Type *ptr;
#if defined(HPM_ENET_RGMII) && defined(HPM_ENET_RMII)
    ptr = (phydev->phy_port == 0) ? HPM_ENET0 : HPM_ENET1;
#else
    ptr = HPM_ENET0;
#endif

#if SDK_VERSION_NUMBER < 0x10C00
    return enet_read_phy(ptr, phy_addr, regnum);
#else
    uint16_t val = 0;
    enet_read_phy(ptr, phy_addr, regnum, &val);
    return val;
#endif
}

#include "hpm_gptmr_drv.h"

#define EC_HTIMER          BOARD_GPTMR
#define EC_HTIMER_CH       BOARD_GPTMR_CHANNEL
#define EC_HTIMER_IRQ      BOARD_GPTMR_IRQ
#define EC_HTIMER_CLK_NAME BOARD_GPTMR_CLK_NAME

static ec_htimer_cb g_ec_htimer_cb = NULL;
static void *g_ec_htimer_arg = NULL;
static double g_timer_tick_ns;

void ec_htimer_isr(void)
{
    if (gptmr_check_status(EC_HTIMER, GPTMR_CH_RLD_STAT_MASK(EC_HTIMER_CH))) {
        gptmr_clear_status(EC_HTIMER, GPTMR_CH_RLD_STAT_MASK(EC_HTIMER_CH));
        g_ec_htimer_cb(g_ec_htimer_arg);
    }
}
SDK_DECLARE_EXT_ISR_M(EC_HTIMER_IRQ, ec_htimer_isr);

void ec_htimer_start(uint32_t us, ec_htimer_cb cb, void *arg)
{
    uint32_t gptmr_freq;
    gptmr_channel_config_t config;

    g_ec_htimer_cb = cb;
    g_ec_htimer_arg = arg;

    gptmr_channel_get_default_config(EC_HTIMER, &config);

    clock_add_to_group(EC_HTIMER_CLK_NAME, 0);
    gptmr_freq = clock_get_frequency(EC_HTIMER_CLK_NAME);
    g_timer_tick_ns = 1000000000.0 / gptmr_freq;

    config.reload = (uint64_t)gptmr_freq * us / 1000000;
    gptmr_stop_counter(EC_HTIMER, EC_HTIMER_CH);
    gptmr_channel_config(EC_HTIMER, EC_HTIMER_CH, &config, false);
    gptmr_enable_irq(EC_HTIMER, GPTMR_CH_RLD_IRQ_MASK(EC_HTIMER_CH));
    intc_m_enable_irq_with_priority(EC_HTIMER_IRQ, 10);
    gptmr_channel_reset_count(EC_HTIMER, EC_HTIMER_CH);
    gptmr_start_counter(EC_HTIMER, EC_HTIMER_CH);
}

void ec_htimer_stop(void)
{
    gptmr_stop_counter(EC_HTIMER, EC_HTIMER_CH);
    gptmr_disable_irq(EC_HTIMER, GPTMR_CH_RLD_IRQ_MASK(EC_HTIMER_CH));
    intc_m_disable_irq(EC_HTIMER_IRQ);
}

EC_FAST_CODE_SECTION void ec_htimer_update(uint32_t ns)
{
    uint32_t ticks = ns / g_timer_tick_ns + 0.5;
    gptmr_channel_config_update_reload(EC_HTIMER, EC_HTIMER_CH, ticks);
}

#ifndef CONFIG_EC_TIMESTAMP_CUSTOM
uint32_t ec_get_cpu_frequency(void)
{
    return clock_get_frequency(clock_cpu0);
}
#endif
