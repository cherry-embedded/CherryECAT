/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_MAILBOX_H
#define EC_MAILBOX_H

uint8_t *ec_mailbox_fill_send(ec_master_t *master,
                              uint16_t slave_index,
                              ec_datagram_t *datagram,
                              uint8_t type,
                              uint16_t size);
int ec_mailbox_send(ec_master_t *master,
                    uint16_t slave_index,
                    ec_datagram_t *datagram);
int ec_mailbox_read_status(ec_master_t *master,
                           uint16_t slave_index,
                           ec_datagram_t *datagram,
                           uint32_t timeout_us);
int ec_mailbox_receive(ec_master_t *master,
                       uint16_t slave_index,
                       ec_datagram_t *datagram,
                       uint8_t *type,
                       uint32_t *size,
                       uint32_t timeout_us);

#endif