/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "hpm_clock_drv.h"
#include "hpm_enet_drv.h"
#include "hpm_enet_phy_common.h"
#include "hpm_otp_drv.h"
#include "hpm_l1c_drv.h"
#include "board.h"
#include "ec_master.h"

#if defined(RGMII) && RGMII
#define ENET_INF_TYPE enet_inf_rgmii
#define ENET          BOARD_ENET_RGMII
#else
#define ENET_INF_TYPE enet_inf_rmii
#define ENET          BOARD_ENET_RMII
#endif

#define __ENABLE_ENET_RECEIVE_INTERRUPT 1

#define MAC_ADDR0 0x00
#define MAC_ADDR1 0x80
#define MAC_ADDR2 0xE1
#define MAC_ADDR3 0x00
#define MAC_ADDR4 0x00
#define MAC_ADDR5 0x00

#define ENET_TX_BUFF_COUNT CONFIG_EC_MAX_ENET_TXBUF_COUNT
#define ENET_RX_BUFF_COUNT CONFIG_EC_MAX_ENET_RXBUF_COUNT
#define ENET_RX_BUFF_SIZE  ENET_MAX_FRAME_SIZE
#define ENET_TX_BUFF_SIZE  ENET_MAX_FRAME_SIZE

ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ENET_SOC_DESC_ADDR_ALIGNMENT)
__RW enet_rx_desc_t dma_rx_desc_tab[ENET_RX_BUFF_COUNT]; /* Ethernet Rx DMA Descriptor */

ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ENET_SOC_DESC_ADDR_ALIGNMENT)
__RW enet_tx_desc_t dma_tx_desc_tab[ENET_TX_BUFF_COUNT]; /* Ethernet Tx DMA Descriptor */

ATTR_PLACE_AT_FAST_RAM_BSS_WITH_ALIGNMENT(ENET_SOC_BUFF_ADDR_ALIGNMENT)
__RW uint8_t rx_buff[ENET_RX_BUFF_COUNT][ENET_RX_BUFF_SIZE]; /* Ethernet Receive Buffer */

ATTR_PLACE_AT_FAST_RAM_BSS_WITH_ALIGNMENT(ENET_SOC_BUFF_ADDR_ALIGNMENT)
__RW uint8_t tx_buff[ENET_TX_BUFF_COUNT][ENET_TX_BUFF_SIZE]; /* Ethernet Transmit Buffer */

enet_desc_t desc;
uint8_t mac[ETH_ALEN];

ec_netdev_t g_netdev;

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

hpm_stat_t enet_init(ENET_Type *ptr)
{
    enet_int_config_t int_config = { .int_enable = 0, .int_mask = 0 };
    enet_mac_config_t enet_config;
    enet_tx_control_config_t enet_tx_control_config;

#ifdef CONFIG_EC_PHY_CUSTOM
#if defined(RGMII) && RGMII
#if defined(__USE_DP83867) && __USE_DP83867
    dp83867_config_t phy_config;
#else
    rtl8211_config_t phy_config;
#endif
#else
#if defined(__USE_DP83848) && __USE_DP83848
    dp83848_config_t phy_config;
#else
    rtl8201_config_t phy_config;
#endif
#endif
#endif

    /* Initialize td, rd and the corresponding buffers */
    memset((uint8_t *)dma_tx_desc_tab, 0x00, sizeof(dma_tx_desc_tab));
    memset((uint8_t *)dma_rx_desc_tab, 0x00, sizeof(dma_rx_desc_tab));
    memset((uint8_t *)rx_buff, 0x00, sizeof(rx_buff));
    memset((uint8_t *)tx_buff, 0x00, sizeof(tx_buff));

    desc.tx_desc_list_head = (enet_tx_desc_t *)core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)dma_tx_desc_tab);
    desc.rx_desc_list_head = (enet_rx_desc_t *)core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)dma_rx_desc_tab);

    desc.tx_buff_cfg.buffer = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)tx_buff);
    desc.tx_buff_cfg.count = ENET_TX_BUFF_COUNT;
    desc.tx_buff_cfg.size = ENET_TX_BUFF_SIZE;

    desc.rx_buff_cfg.buffer = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)rx_buff);
    desc.rx_buff_cfg.count = ENET_RX_BUFF_COUNT;
    desc.rx_buff_cfg.size = ENET_RX_BUFF_SIZE;

    /*Get a default control config for tx descriptor */
    enet_get_default_tx_control_config(ENET, &enet_tx_control_config);

    /* Set the control config for tx descriptor */
    ec_memcpy(&desc.tx_control_config, &enet_tx_control_config, sizeof(enet_tx_control_config_t));

    /* Get MAC address */
    enet_get_mac_address(mac);

    /* Set MAC0 address */
    enet_config.mac_addr_high[0] = mac[5] << 8 | mac[4];
    enet_config.mac_addr_low[0] = mac[3] << 24 | mac[2] << 16 | mac[1] << 8 | mac[0];
    enet_config.valid_max_count = 1;

    /* Set DMA PBL */
    enet_config.dma_pbl = board_get_enet_dma_pbl(ENET);

    /* Set SARC */
    enet_config.sarc = enet_sarc_replace_mac0;

#if defined(__ENABLE_ENET_RECEIVE_INTERRUPT) && __ENABLE_ENET_RECEIVE_INTERRUPT
    /* Enable Enet IRQ */
    board_enable_enet_irq(ENET);

    /* Get the default interrupt config */
    enet_get_default_interrupt_config(ENET, &int_config);
#endif

    /* Initialize enet controller */
    if (enet_controller_init(ptr, ENET_INF_TYPE, &desc, &enet_config, &int_config) != status_success) {
        return status_fail;
    }

#if defined(__ENABLE_ENET_RECEIVE_INTERRUPT) && __ENABLE_ENET_RECEIVE_INTERRUPT
    /* Disable LPI interrupt */
    enet_disable_lpi_interrupt(ENET);
#endif

#ifdef CONFIG_EC_PHY_CUSTOM
/* Initialize phy */
#if defined(RGMII) && RGMII
#if defined(__USE_DP83867) && __USE_DP83867
    dp83867_reset(ptr);
#if defined(__DISABLE_AUTO_NEGO) && __DISABLE_AUTO_NEGO
    dp83867_set_mdi_crossover_mode(ENET, enet_phy_mdi_crossover_manual_mdix);
#endif
    dp83867_basic_mode_default_config(ptr, &phy_config);
    if (dp83867_basic_mode_init(ptr, &phy_config) == true) {
#else
    rtl8211_reset(ptr);
    rtl8211_basic_mode_default_config(ptr, &phy_config);
    if (rtl8211_basic_mode_init(ptr, &phy_config) == true) {
#endif
#else
#if defined(__USE_DP83848) && __USE_DP83848
    dp83848_reset(ptr);
    dp83848_basic_mode_default_config(ptr, &phy_config);
    if (dp83848_basic_mode_init(ptr, &phy_config) == true) {
#else
    rtl8201_reset(ptr);
    rtl8201_basic_mode_default_config(ptr, &phy_config);
    if (rtl8201_basic_mode_init(ptr, &phy_config) == true) {
#endif
#endif
        EC_LOG_DBG("Enet phy init passed !\n");
    } else {
        EC_LOG_DBG("Enet phy init failed !\n");
        return status_fail;
    }
#endif
    return status_success;
}

ec_netdev_t *ec_netdev_low_level_init(uint8_t netdev_index)
{
    /* Initialize GPIOs */
    board_init_enet_pins(ENET);

    /* Reset an enet PHY */
    board_reset_enet_phy(ENET);
#if defined(RGMII) && RGMII
    /* Set RGMII clock delay */
    board_init_enet_rgmii_clock_delay(ENET);
#else
    /* Set RMII reference clock */
    board_init_enet_rmii_reference_clock(ENET, BOARD_ENET_RMII_INT_REF_CLK);
    EC_LOG_DBG("Reference Clock: %s\n", BOARD_ENET_RMII_INT_REF_CLK ? "Internal Clock" : "External Clock");
#endif

    /* Initialize MAC and DMA */
    if (enet_init(ENET) == 0) {
    } else {
        EC_LOG_DBG("Enet initialization fails !!!\n");
        while (1) {
        }
    }

    ec_memcpy(g_netdev.mac_addr, mac, ETH_ALEN);

    for (uint32_t i = 0; i < ENET_TX_BUFF_COUNT; i++) {
        for (uint8_t j = 0; j < 6; j++) { // dst MAC
            EC_WRITE_U8(&tx_buff[i][j], 0xFF);
        }
        for (uint8_t j = 0; j < 6; j++) { // src MAC
            EC_WRITE_U8(&tx_buff[i][6 + j], mac[j]);
        }
        EC_WRITE_U16(&tx_buff[i][12], ec_htons(0x88a4));
    }

    return &g_netdev;
}

#ifndef CONFIG_EC_PHY_CUSTOM
void ec_mdio_low_level_write(struct chry_phy_device *phydev, uint16_t phy_addr, uint16_t regnum, uint16_t val)
{
    //ec_netdev_t *netdev = (ec_netdev_t *)phydev->user_data;
    enet_write_phy(ENET, phy_addr, regnum, val);
}

uint16_t ec_mdio_low_level_read(struct chry_phy_device *phydev, uint16_t phy_addr, uint16_t regnum)
{
    //ec_netdev_t *netdev = (ec_netdev_t *)phydev->user_data;
    return enet_read_phy(ENET, phy_addr, regnum);
}

void ec_netdev_low_level_link_up(ec_netdev_t *netdev, struct chry_phy_status *status)
{
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
        enet_set_line_speed(ENET, line_speed);
        enet_set_duplex_mode(ENET, status->duplex);
    } else {
    }
}
#else
void ec_netdev_low_level_poll_link_state(ec_netdev_t *netdev)
{
    static enet_phy_status_t last_status;
    enet_phy_status_t status = { 0 };

    enet_line_speed_t line_speed[] = { enet_line_speed_10mbps, enet_line_speed_100mbps, enet_line_speed_1000mbps };

#if defined(RGMII) && RGMII
#if defined(__USE_DP83867) && __USE_DP83867
    dp83867_get_phy_status(ENET, &status);
#else
    rtl8211_get_phy_status(ENET, &status);
#endif
#else
#if defined(__USE_DP83848) && __USE_DP83848
    dp83848_get_phy_status(ENET, &status);
#else
    rtl8201_get_phy_status(ENET, &status);
#endif
#endif

    if (memcmp(&last_status, &status, sizeof(enet_phy_status_t)) != 0) {
        ec_memcpy(&last_status, &status, sizeof(enet_phy_status_t));
        if (status.enet_phy_link) {
            enet_set_line_speed(ENET, line_speed[status.enet_phy_speed]);
            enet_set_duplex_mode(ENET, status.enet_phy_duplex);
            netdev->link_state = true;
        } else {
            netdev->link_state = false;
        }
    }
}
#endif

EC_FAST_CODE_SECTION uint8_t *ec_netdev_low_level_get_txbuf(ec_netdev_t *netdev)
{
    __IO enet_tx_desc_t *dma_tx_desc;

    dma_tx_desc = desc.tx_desc_list_cur;

    EC_ASSERT_MSG(dma_tx_desc->tdes0_bm.own == 0, "No free tx buffer available\n");

    return (uint8_t *)sys_address_to_core_local_mem(BOARD_RUNNING_CORE, dma_tx_desc->tdes2_bm.buffer1);
}

EC_FAST_CODE_SECTION int ec_netdev_low_level_output(ec_netdev_t *netdev, uint32_t size)
{
    __IO enet_tx_desc_t *dma_tx_desc;

    dma_tx_desc = desc.tx_desc_list_cur;
    if (dma_tx_desc->tdes0_bm.own != 0) {
        return -1;
    }

    /* Prepare transmit descriptors to give to DMA*/
    enet_prepare_transmission_descriptors(ENET, &desc.tx_desc_list_cur, size + 4, desc.tx_buff_cfg.size);

    return 0;
}

EC_FAST_CODE_SECTION int ec_netdev_low_level_input(ec_netdev_t *netdev)
{
    uint32_t len;
    uint8_t *buffer;
    enet_frame_t frame = { 0, 0, 0 };
    enet_rx_desc_t *dma_rx_desc;
    uint32_t i = 0;
    int ret = 0;

    /* Check and get a received frame */
    if (enet_check_received_frame(&desc.rx_desc_list_cur, &desc.rx_frame_info) == 1) {
        frame = enet_get_received_frame(&desc.rx_desc_list_cur, &desc.rx_frame_info);
    }

    /* Obtain the size of the packet and put it into the "len" variable. */
    len = frame.length;
    buffer = (uint8_t *)sys_address_to_core_local_mem(BOARD_RUNNING_CORE, (uint32_t)frame.buffer);

    if (len > 0) {
        ec_netdev_receive(netdev, buffer, len);
        /* Release descriptors to DMA */
        dma_rx_desc = frame.rx_desc;

        /* Set Own bit in Rx descriptors: gives the buffers back to DMA */
        for (i = 0; i < desc.rx_frame_info.seg_count; i++) {
            dma_rx_desc->rdes0_bm.own = 1;
            dma_rx_desc = (enet_rx_desc_t *)(dma_rx_desc->rdes3_bm.next_desc);
        }

        /* Clear Segment_Count */
        desc.rx_frame_info.seg_count = 0;
    } else {
        ret = -1;
    }

    /* Resume Rx Process */
    enet_rx_resume(ENET);
    return ret;
}

#if defined(__ENABLE_ENET_RECEIVE_INTERRUPT) && __ENABLE_ENET_RECEIVE_INTERRUPT
void isr_enet(ENET_Type *ptr)
{
    uint32_t status;
    uint32_t rxgbfrmis;
    uint32_t intr_status;

    status = ptr->DMA_STATUS;
    rxgbfrmis = ptr->MMC_INTR_RX;
    intr_status = ptr->INTR_STATUS;

    if (ENET_DMA_STATUS_GLPII_GET(status)) {
        /* read LPI_CSR to clear interrupt status */
        ptr->LPI_CSR;
    }

    if (ENET_INTR_STATUS_RGSMIIIS_GET(intr_status)) {
        /* read XMII_CSR to clear interrupt status */
        ptr->XMII_CSR;
    }

    if (ENET_DMA_STATUS_RI_GET(status)) {
        ptr->DMA_STATUS |= ENET_DMA_STATUS_RI_MASK;
        while (ec_netdev_low_level_input(&g_netdev) == 0) {
        }
    }

    if (ENET_MMC_INTR_RX_RXCTRLFIS_GET(rxgbfrmis)) {
        ptr->RXFRAMECOUNT_GB;
    }
}

#ifdef HPM_ENET0_BASE
void isr_enet0(void)
{
    isr_enet(ENET);
}
SDK_DECLARE_EXT_ISR_M(IRQn_ENET0, isr_enet0)
#endif

#ifdef HPM_ENET1_BASE
void isr_enet1(void)
{
    isr_enet(ENET);
}
SDK_DECLARE_EXT_ISR_M(IRQn_ENET1, isr_enet1)
#endif

#endif

#include "hpm_gptmr_drv.h"

#define EC_HTIMER          BOARD_GPTMR
#define EC_HTIMER_CH       BOARD_GPTMR_CHANNEL
#define EC_HTIMER_IRQ      BOARD_GPTMR_IRQ
#define EC_HTIMER_CLK_NAME BOARD_GPTMR_CLK_NAME

ec_htimer_cb g_ec_htimer_cb = NULL;
void *g_ec_htimer_arg = NULL;

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

    config.reload = gptmr_freq / 1000000 * us;
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

uint32_t ec_get_cpu_frequency(void)
{
    return clock_get_frequency(clock_cpu0);
}
