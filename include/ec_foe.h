/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_FOE_H
#define EC_FOE_H

int ec_foe_write(ec_master_t *master,
                 uint16_t slave_index,
                 ec_datagram_t *datagram,
                 const char *filename,
                 uint32_t password,
                 const void *buf,
                 uint32_t size);

int ec_foe_read(ec_master_t *master,
                uint16_t slave_index,
                ec_datagram_t *datagram,
                const char *filename,
                uint32_t password,
                void *buf,
                uint32_t maxsize,
                uint32_t *size);
#endif