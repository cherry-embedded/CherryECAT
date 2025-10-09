/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_COE_H
#define EC_COE_H

int ec_coe_download(ec_master_t *master,
                    uint16_t slave_index,
                    ec_datagram_t *datagram,
                    uint16_t index,
                    uint8_t subindex,
                    const void *buf,
                    uint32_t size,
                    bool complete_access);

int ec_coe_upload(ec_master_t *master,
                  uint16_t slave_index,
                  ec_datagram_t *datagram,
                  uint16_t index,
                  uint8_t subindex,
                  const void *buf,
                  uint32_t maxsize,
                  uint32_t *size,
                  bool complete_access);
#endif