/*
 * Copyright (c) 2025, sakumisu
 * Copyright (c) 2025, yans
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <hal_data.h>
#include "ec_master.h"

ec_netdev_t g_netdev;
#if !defined(SOC_SERIES_R7KA8P1) && !defined(SOC_SERIES_R9A07G0)
#error "Ethercat master only supports for R7KA8P1 and RZN2L"
#endif

#if !defined(CONFIG_EC_PHY_CUSTOM)
#error "CONFIG_EC_PHY_CUSTOM must be defined for Renesas MCUs"
#endif

#if defined(SOC_SERIES_R7KA8P1)
#define ETHER_BUFFER_PLACE_IN_SECTION BSP_PLACE_IN_SECTION(".ram_nocache")

#define ETHER_EVENT_INTERRUPT ETHER_EVENT_RX_COMPLETE

#define R_ETHER_Open          R_RMAC_Open
#define R_ETHER_Write         R_RMAC_Write
#define R_ETHER_Read          R_RMAC_Read
#define R_ETHER_LinkProcess   R_RMAC_LinkProcess
#define R_ETHER_BufferRelease R_RMAC_BufferRelease

void rmac_phy_target_rtl8211_initialize(rmac_phy_instance_ctrl_t *phydev)
{
#define RTL_8211F_PAGE_SELECT 0x1F
#define RTL_8211F_EEELCR_ADDR 0x11
#define RTL_8211F_LED_PAGE    0xD04
#define RTL_8211F_LCR_ADDR    0x10

    uint32_t val1, val2 = 0;

    /* switch to led page */
    R_RMAC_PHY_Write(phydev, RTL_8211F_PAGE_SELECT, RTL_8211F_LED_PAGE);

    /* set led1(green) Link 10/100/1000M, and set led2(yellow) Link 10/100/1000M+Active */
    R_RMAC_PHY_Read(phydev, RTL_8211F_LCR_ADDR, &val1);
    val1 |= (1 << 5);
    val1 |= (1 << 8);
    val1 &= (~(1 << 9));
    val1 |= (1 << 10);
    val1 |= (1 << 11);
    R_RMAC_PHY_Write(phydev, RTL_8211F_LCR_ADDR, val1);

    /* set led1(green) EEE LED function disabled so it can keep on when linked */
    R_RMAC_PHY_Read(phydev, RTL_8211F_EEELCR_ADDR, &val2);
    val2 &= (~(1 << 2));
    R_RMAC_PHY_Write(phydev, RTL_8211F_EEELCR_ADDR, val2);

    /* switch back to page0 */
    R_RMAC_PHY_Write(phydev, RTL_8211F_PAGE_SELECT, 0xa42);
}

bool rmac_phy_target_rtl8211_is_support_link_partner_ability(rmac_phy_instance_ctrl_t *p_instance_ctrl,
                                                             uint32_t line_speed_duplex)
{
    FSP_PARAMETER_NOT_USED(p_instance_ctrl);
    FSP_PARAMETER_NOT_USED(line_speed_duplex);

    /* This PHY-LSI supports half and full duplex mode. */
    return true;
}

/* Multi-PHY support structures */
typedef struct {
    rmac_phy_instance_ctrl_t *p_ctrl;
    uint8_t port_bit;
    const char *name;
} phy_port_info_t;

static const phy_port_info_t phy_ports[] = {
    { &g_rmac_phy0_ctrl, 0x01, "PHY0" },
    { &g_rmac_phy1_ctrl, 0x02, "PHY1" }
};

#define PHY_PORTS_COUNT (sizeof(phy_ports) / sizeof(phy_ports[0]))

#elif defined(SOC_SERIES_R9A07G0)

#if !defined(CONFIG_EC_TIMESTAMP_CUSTOM)
#error "CONFIG_EC_TIMESTAMP_CUSTOM must be defined for R9A07G0"
#endif

#define ETHER_BUFFER_PLACE_IN_SECTION BSP_PLACE_IN_SECTION(".noncache_buffer.eth")

#define status_ecsr           status_link
#define ETHER_EVENT_INTERRUPT ETHER_EVENT_SBD_INTERRUPT

#define R_ETHER_Open          R_GMAC_Open
#define R_ETHER_Write         R_GMAC_Write
#define R_ETHER_Read          R_GMAC_Read
#define R_ETHER_LinkProcess   R_GMAC_LinkProcess
#define R_ETHER_BufferRelease R_GMAC_BufferRelease

static int phy_rtl8211f_led_fixup(ether_phy_instance_ctrl_t *phydev)
{
#define RTL_8211F_PAGE_SELECT 0x1F
#define RTL_8211F_EEELCR_ADDR 0x11
#define RTL_8211F_LED_PAGE    0xD04
#define RTL_8211F_LCR_ADDR    0x10

    uint32_t val1, val2 = 0;

    /* switch to led page */
    R_ETHER_PHY_Write(phydev, RTL_8211F_PAGE_SELECT, RTL_8211F_LED_PAGE);

    /* set led1(green) Link 10/100/1000M, and set led2(yellow) Link 10/100/1000M+Active */
    R_ETHER_PHY_Read(phydev, RTL_8211F_LCR_ADDR, &val1);
    val1 |= (1 << 5);
    val1 |= (1 << 8);
    val1 &= (~(1 << 9));
    val1 |= (1 << 10);
    val1 |= (1 << 11);
    R_ETHER_PHY_Write(phydev, RTL_8211F_LCR_ADDR, val1);

    /* set led1(green) EEE LED function disabled so it can keep on when linked */
    R_ETHER_PHY_Read(phydev, RTL_8211F_EEELCR_ADDR, &val2);
    val2 &= (~(1 << 2));
    R_ETHER_PHY_Write(phydev, RTL_8211F_EEELCR_ADDR, val2);

    /* switch back to page0 */
    R_ETHER_PHY_Write(phydev, RTL_8211F_PAGE_SELECT, 0xa42);

    return 0;
}

void ether_phy_targets_initialize_rtl8211_rgmii(ether_phy_instance_ctrl_t *p_instance_ctrl)
{
    ec_osal_msleep(100);
    phy_rtl8211f_led_fixup(p_instance_ctrl);
}

#define PHY_PORTS_COUNT (3) ///< Count of port

#endif

__attribute__((__aligned__(32))) uint8_t tx_buffer[CONFIG_EC_MAX_ENET_TXBUF_COUNT][1536] ETHER_BUFFER_PLACE_IN_SECTION;

extern uint32_t SystemCoreClock;

static uint8_t g_link_change = 0; ///< Link change (bit0:port0, bit1:port1, bit2:port2)
static uint8_t g_link_status = 0; ///< Link status (bit0:port0, bit1:port1, bit2:port2)
static uint8_t previous_link_status = 0;

ec_netdev_t *ec_netdev_low_level_init(uint8_t netdev_index)
{
    fsp_err_t res;

    EC_ASSERT_MSG(g_ether0_cfg.zerocopy == ETHER_ZEROCOPY_ENABLE, "zerocopy must be enabled");
    EC_ASSERT_MSG(g_ether0_cfg.multicast == ETHER_MULTICAST_DISABLE, "multicast must be disabled");
    EC_ASSERT_MSG(g_ether0_cfg.promiscuous == ETHER_PROMISCUOUS_ENABLE, "promiscuous must be enabled");
    EC_ASSERT_MSG(g_ether0_cfg.num_tx_descriptors == CONFIG_EC_MAX_ENET_TXBUF_COUNT, "num_tx_descriptors must be the same as \
        CONFIG_EC_MAX_ENET_TXBUF_COUNT");
    EC_ASSERT_MSG(g_ether0_cfg.num_rx_descriptors == CONFIG_EC_MAX_ENET_RXBUF_COUNT, "num_rx_descriptors must be the same as \
        CONFIG_EC_MAX_ENET_RXBUF_COUNT");

    res = R_ETHER_Open(&g_ether0_ctrl, &g_ether0_cfg);
    if (res != FSP_SUCCESS)
        EC_LOG_ERR("R_ETHER_Open failed!, res = %d", res);

    ec_memcpy(g_netdev.mac_addr, g_ether0_cfg.p_mac_address, 6);

    for (uint32_t i = 0; i < g_ether0_cfg.num_tx_descriptors; i++) {
        for (uint8_t j = 0; j < 6; j++) { // dst MAC
            EC_WRITE_U8(&tx_buffer[i][j], 0xFF);
        }
        for (uint8_t j = 0; j < 6; j++) { // src MAC
            EC_WRITE_U8(&tx_buffer[i][6 + j], g_ether0_cfg.p_mac_address[j]);
        }
        EC_WRITE_U16(&tx_buffer[i][12], ec_htons(0x88a4));
    }

    res = R_GPT_Open(&g_timer0_ctrl, &g_timer0_cfg);
    if (res != FSP_SUCCESS) {
        EC_LOG_ERR("R_GPT_Open failed!, res = %d", res);
    }

    return &g_netdev;
}

#if defined(SOC_SERIES_R7KA8P1)
void ec_netdev_low_level_poll_link_state(ec_netdev_t *netdev)
{
    fsp_err_t res;
    uint8_t port;
    uint8_t port_bit;
    uint8_t status_change;
    uint32_t phy_data;
    uint8_t current_link_status = 0;
    uint8_t i;

    res = R_ETHER_LinkProcess(&g_ether0_ctrl);

    /* Check link status for all PHY ports */
    for (i = 0; i < PHY_PORTS_COUNT; i++) {
        res = R_RMAC_PHY_Read(phy_ports[i].p_ctrl, 0x1, &phy_data);
        if (res == FSP_SUCCESS) {
            if (phy_data & 0x04) /* PHY Basic Status Register Link Status bit */
            {
                current_link_status |= phy_ports[i].port_bit; /* Port link up */
            }

            status_change = previous_link_status ^ current_link_status;
            if (status_change & phy_ports[i].port_bit) {
                g_link_change |= phy_ports[i].port_bit;
            }
        } else {
            EC_LOG_ERR("%s PHY_Read failed!, res = %d", phy_ports[i].name, res);
        }
    }

    /* Update global link status */
    g_link_status = current_link_status;

    /* Process link changes for all ports */
    for (port = 0; port < PHY_PORTS_COUNT; port++) {
        port_bit = phy_ports[port].port_bit;

        if (g_link_change & port_bit) {
            /* Link status changed */
            g_link_change &= (uint8_t)(~port_bit); /* change bit clear */

            if (g_link_status & port_bit) {
                /* Changed to Link-up */
                netdev->link_state = true;
            } else {
                /* Changed to Link-down */
                netdev->link_state = false;
            }
        }
    }

    previous_link_status = g_link_status;
}
#elif defined(SOC_SERIES_R9A07G0)
void ec_netdev_low_level_poll_link_state(ec_netdev_t *netdev)
{
    fsp_err_t res;
    gmac_link_status_t port_status;
    uint8_t port = 0;
    uint8_t port_bit = 0;

    res = R_GMAC_LinkProcess(&g_ether0_ctrl);

    if (0 == g_ether0.p_cfg->p_callback) {
        for (port = 0; port < PHY_PORTS_COUNT; port++) {
            res = R_GMAC_GetLinkStatus(&g_ether0_ctrl, port, &port_status);
            if (FSP_SUCCESS != res) {
                /* An error has occurred */
                EC_LOG_ERR("R_GMAC_GetLinkStatus failed!, res = %d", res);
                break;
            }

            if (GMAC_LINK_STATUS_DOWN != port_status) {
                /* Set link up */
                g_link_status |= (uint8_t)(1U << port);
            }
        }
        if (FSP_SUCCESS == res) {
            /* Set changed link status */
            g_link_change = previous_link_status ^ g_link_status;
        }
    }

    previous_link_status = g_link_status;

    if (FSP_SUCCESS == res) {
        for (port = 0; port < PHY_PORTS_COUNT; port++) {
            port_bit = (uint8_t)(1U << port);

            if (g_link_change & port_bit) {
                /* Link status changed */
                g_link_change &= (uint8_t)(~port_bit); // change bit clear

                if (g_link_status & port_bit) {
                    /* Changed to Link-up */
                    netdev->link_state = true;
                } else {
                    /* Changed to Link-down */
                    netdev->link_state = false;
                }
            }
        }
    }
}
#endif

EC_FAST_CODE_SECTION uint8_t *ec_netdev_low_level_get_txbuf(ec_netdev_t *netdev)
{
    return (uint8_t *)tx_buffer[netdev->tx_frame_index];
}

EC_FAST_CODE_SECTION int ec_netdev_low_level_output(ec_netdev_t *netdev, uint32_t size)
{
    fsp_err_t res;

    res = R_ETHER_Write(&g_ether0_ctrl, tx_buffer[netdev->tx_frame_index], size);
    if (res != FSP_SUCCESS) {
        return -1;
    }

    netdev->tx_frame_index++;
    netdev->tx_frame_index %= g_ether0_cfg.num_tx_descriptors;

    return 0;
}

EC_FAST_CODE_SECTION int ec_netdev_low_level_input(ec_netdev_t *netdev)
{
    fsp_err_t res;
    uint8_t *buffer;
    uint32_t len = 0;

    res = R_ETHER_Read(&g_ether0_ctrl, (void *)&buffer, &len);
    if (res != FSP_SUCCESS) {
        return -1;
    }

    ec_netdev_receive(netdev, buffer, len);

    R_ETHER_BufferRelease(&g_ether0_ctrl);

    return 0;
}

static ec_htimer_cb g_ec_htimer_cb = NULL;
static void *g_ec_htimer_arg = NULL;
static uint32_t g_timer_reload_us_div = 0;

void timer0_esc_callback(timer_callback_args_t *p_args)
{
    rt_interrupt_enter();
    if (TIMER_EVENT_CYCLE_END == p_args->event) {
        if (g_ec_htimer_cb) {
            g_ec_htimer_cb(g_ec_htimer_arg);
        }
    }
    rt_interrupt_leave();
}

void ec_htimer_start(uint32_t us, ec_htimer_cb cb, void *arg)
{
    fsp_err_t fsp_err = FSP_SUCCESS;
    timer_info_t time_info;

    R_GPT_InfoGet(&g_timer0_ctrl, &time_info);

    g_timer_reload_us_div = time_info.clock_frequency / 1000000;

    g_ec_htimer_cb = cb;
    g_ec_htimer_arg = arg;

    fsp_err |= R_GPT_CounterSet(&g_timer0_ctrl, 0);
    fsp_err |= R_GPT_PeriodSet(&g_timer0_ctrl, us * g_timer_reload_us_div);
    fsp_err |= R_GPT_Start(&g_timer0_ctrl);

    if (fsp_err != FSP_SUCCESS) {
        EC_LOG_ERR("ec_htimer_start failed!, res = %d", fsp_err);
    }
}

void ec_htimer_stop(void)
{
    R_GPT_Stop(&g_timer0_ctrl);
}

EC_FAST_CODE_SECTION void ec_htimer_update(uint32_t us)
{
    R_GPT_PeriodSet(&g_timer0_ctrl, us * g_timer_reload_us_div);
}

void user_ether0_callback(ether_callback_args_t *p_args)
{
    rt_interrupt_enter();

    switch (p_args->event) {
        case ETHER_EVENT_LINK_ON:                          ///< Link up detection event/
            g_link_status |= (uint8_t)p_args->status_ecsr; ///< status up
            g_link_change |= (uint8_t)p_args->status_ecsr; ///< change bit set
            break;

        case ETHER_EVENT_LINK_OFF:                            ///< Link down detection event
            g_link_status &= (uint8_t)(~p_args->status_ecsr); ///< status down
            g_link_change |= (uint8_t)p_args->status_ecsr;    ///< change bit set
            break;

        case ETHER_EVENT_WAKEON_LAN: ///< Magic packet detection event
        /* If EDMAC FR (Frame Receive Event) or FDE (Receive Descriptor Empty Event)
        * interrupt occurs, send rx mailbox. */
        case ETHER_EVENT_INTERRUPT: ///< BSD Interrupt event
        {
            while (ec_netdev_low_level_input(&g_netdev) == 0) {
            }
            break;
        }

        default:
            break;
    }

    rt_interrupt_leave();
}

#ifndef CONFIG_EC_TIMESTAMP_CUSTOM
uint32_t ec_get_cpu_frequency(void)
{
    return SystemCoreClock;
}
#endif

#if defined(SOC_SERIES_R9A07G0)
volatile uint64_t mtu3_overflow_count = 0;

void g_mtu3_callback(timer_callback_args_t *p_args)
{
    rt_interrupt_enter();
    if (TIMER_EVENT_CYCLE_END == p_args->event) {
        mtu3_overflow_count++;
    }
    rt_interrupt_leave();
}

uint64_t gpt_get_count(void)
{
    mtu3_status_t status;
    uint64_t high;
    uint64_t low;

    do {
        high = mtu3_overflow_count;
        R_MTU3_StatusGet(&g_mtu3_ctrl, &status);
        low = status.counter;
    } while (high != mtu3_overflow_count);

    return (high << 16) | low;
}

void ec_timestamp_init(void)
{
    fsp_err_t fsp_err = FSP_SUCCESS;

    fsp_err = R_MTU3_Open(&g_mtu3_ctrl, &g_mtu3_cfg);
    fsp_err |= R_MTU3_Start(&g_mtu3_ctrl);

    if (fsp_err != FSP_SUCCESS) {
        EC_LOG_ERR("R_GPT_Open failed!, res = %d", fsp_err);
    }
}

EC_FAST_CODE_SECTION uint64_t ec_timestamp_get_time_ns(void)
{
    return (uint64_t)(gpt_get_count() * 5ULL);
}

EC_FAST_CODE_SECTION uint64_t ec_timestamp_get_time_us(void)
{
    return ec_timestamp_get_time_ns() / 1000ULL;
}
#endif