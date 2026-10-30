/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_EOE_H
#define EC_EOE_H

#ifdef CONFIG_EC_EOE

struct ec_eoe_ip_param {
    uint8_t hostname[EC_MAX_HOSTNAME_SIZE];
    const char *ipv4_addr_str;
    const char *ipv4_mask_str;
    const char *ipv4_gw_str;
    const char *dns_server_str;
    uint8_t mac_addr[6];
};

int ec_eoe_set_ip(ec_master_t *master, uint16_t slave_index,
                   const struct ec_eoe_ip_param *master_ip_param,
                   const struct ec_eoe_ip_param *slave_ip_param);
int ec_eoe_start(ec_master_t *master, uint16_t slave_index);

void ec_eoe_receive(ec_slave_t *slave, const uint8_t *data, uint16_t size);
bool ec_eoe_poll(ec_mailbox_t *mailbox);
void ec_eoe_clear(ec_slave_t *slave);

#endif
#endif
