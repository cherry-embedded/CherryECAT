/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_EOE_H
#define EC_EOE_H

#ifdef CONFIG_EC_EOE
#include "netif/etharp.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/tcpip.h"
#include "lwip/ip_addr.h"

struct ec_eoe_ip_param {
    uint8_t hostname[EC_MAX_HOSTNAME_SIZE];
    const char *ipv4_addr_str;
    const char *ipv4_mask_str;
    const char *ipv4_gw_str;
    const char *dns_server_str;
    uint8_t mac_addr[6];
};

typedef struct ec_eoe {
    ec_master_t *master;
    uint16_t slave_index;
    ec_datagram_t tx_datagram;
    ec_datagram_t rx_datagram;
    uint8_t tx_buffer[1536];
    uint8_t rx_buffer[1536];
    uint32_t max_data_size;

    ec_osal_mq_t tx_pbuf_mq;
    uint8_t tx_frame_number;
    uint8_t tx_fragment_number;
    uint32_t tx_offset;
    uint32_t tx_total_size;
    struct pbuf *tx_pbuf;
    uint8_t rx_frame_number;
    uint8_t rx_fragment_number;
    uint32_t rx_offset;
    uint32_t rx_total_size;
    struct pbuf *rx_pbuf;

    struct ec_eoe_ip_param master_ip_param;
    struct ec_eoe_ip_param slave_ip_param;

    bool initialized;
    ec_osal_thread_t txrx_thread;
    ec_osal_mutex_t txrx_mutex;
    struct netif netif;
} ec_eoe_t;

int ec_eoe_start(ec_eoe_t *eoe,
                 ec_master_t *master,
                 uint16_t slave_index,
                 struct ec_eoe_ip_param *master_ip_param,
                 struct ec_eoe_ip_param *slave_ip_param);

#endif
#endif