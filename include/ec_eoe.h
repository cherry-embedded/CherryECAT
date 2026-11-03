/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_EOE_H
#define EC_EOE_H

#ifdef CONFIG_EC_EOE

#include "ec_osal.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"

typedef struct ec_master ec_master_t;
typedef struct ec_slave ec_slave_t;
typedef struct ec_mailbox ec_mailbox_t;
struct ec_eoe_ip_param;

typedef struct {
    uint32_t poll_empty, poll_errors;
    uint32_t rx_fragments, rx_frames, rx_bytes, rx_errors, rx_input_errors;
    uint32_t tx_fragments, tx_frames, tx_bytes, tx_errors, tx_wc_retries;
    int32_t tx_last_error;
} ec_eoe_stats_t;

typedef struct {
    ec_osal_thread_t thread;
    ec_osal_mq_t event_mq;
    ec_osal_sem_t event_done;
    int event_result;
    const struct ec_eoe_ip_param *pending_master_ip, *pending_slave_ip;
    uint8_t tx_frame_number;

    /* Receive frame reassembly; rx_pbuf->tot_len holds the allocated capacity. */
    uint8_t rx_frame_number, rx_fragment_number;
    uint16_t rx_offset;
    struct pbuf *rx_pbuf;
    uint64_t rx_deadline;

    /* Retained configuration for interface setup and slave recovery. */
    ip4_addr_t address, mask, gateway;
    uint8_t mac_addr[ETH_ALEN];
    uint8_t slave_ip_data[8 + ETH_ALEN + 16 + EC_MAX_HOSTNAME_SIZE];
    bool configured, initialized, netif_added, linked;
    bool link_down_pending, link_up_pending, link_down_fallback;
    ec_eoe_stats_t stats;
    struct netif netif;
} ec_eoe_t;

struct ec_eoe_ip_param {
    uint8_t hostname[EC_MAX_HOSTNAME_SIZE];
    const char *ipv4_addr_str;
    const char *ipv4_mask_str;
    const char *ipv4_gw_str;
    const char *dns_server_str;
    uint8_t mac_addr[6];
};

int ec_eoe_start(ec_master_t *master, uint16_t slave_index,
                 const struct ec_eoe_ip_param *master_ip_param,
                 const struct ec_eoe_ip_param *slave_ip_param);

void ec_eoe_receive(ec_slave_t *slave, const uint8_t *data, uint16_t size);
void ec_eoe_link_event(ec_slave_t *slave, bool up);
int ec_eoe_stop(ec_slave_t *slave);

#endif
#endif
