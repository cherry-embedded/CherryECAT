/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_master.h"

static const char *type_strings[] = {
    "?",
    "APRD",
    "APWR",
    "APRW",
    "FPRD",
    "FPWR",
    "FPRW",
    "BRD",
    "BWR",
    "BRW",
    "LRD",
    "LWR",
    "LRW",
    "ARMW",
    "FRMW"
};

void ec_datagram_init(ec_datagram_t *datagram, size_t mem_size)
{
    uint8_t *data = NULL;

    data = ec_osal_malloc(mem_size);
    if (!data) {
        EC_LOG_ERR("Failed to allocate memory for datagram data\n");
        return;
    }

    ec_dlist_init(&datagram->queue);
    datagram->netdev_idx = EC_NETDEV_MAIN;
    datagram->type = EC_DATAGRAM_NONE;
    memset(datagram->address, 0x00, EC_ADDR_LEN);
    datagram->data = data;
    datagram->mem_size = mem_size;
    datagram->data_size = 0;
    datagram->index = 0x00;
    datagram->working_counter = 0x0000;
    datagram->state = EC_DATAGRAM_INIT;
    datagram->jiffies_sent = 0;
    datagram->jiffies_received = 0;
    memset(datagram->name, 0x00, EC_DATAGRAM_NAME_SIZE);

    datagram->waiter = 0;
    datagram->wait = ec_osal_sem_create(1, 0);
    if (!datagram->wait) {
        EC_LOG_ERR("Failed to create semaphore for datagram\n");
        ec_osal_free(data);
        datagram->data = NULL;
        return;
    }
}

void ec_datagram_init_static(ec_datagram_t *datagram, uint8_t *data, size_t mem_size)
{
    ec_dlist_init(&datagram->queue);
    datagram->netdev_idx = EC_NETDEV_MAIN;
    datagram->type = EC_DATAGRAM_NONE;
    datagram->static_alloc = true;
    memset(datagram->address, 0x00, EC_ADDR_LEN);
    datagram->data = data;
    datagram->mem_size = mem_size;
    datagram->data_size = 0;
    datagram->index = 0x00;
    datagram->working_counter = 0x0000;
    datagram->state = EC_DATAGRAM_INIT;
    datagram->jiffies_sent = 0;
    datagram->jiffies_received = 0;
    memset(datagram->name, 0x00, EC_DATAGRAM_NAME_SIZE);
}

void ec_datagram_clear(ec_datagram_t *datagram)
{
    ec_datagram_unqueue(datagram);

    if (datagram->data && !datagram->static_alloc) {
        ec_osal_free(datagram->data);
        datagram->data = NULL;

        if (datagram->wait) {
            ec_osal_sem_delete(datagram->wait);
            datagram->wait = NULL;
        }
    }
}

void ec_datagram_unqueue(ec_datagram_t *datagram)
{
    if (!ec_dlist_isempty(&datagram->queue)) {
        ec_dlist_del_init(&datagram->queue);
    }
}

EC_FAST_CODE_SECTION void ec_datagram_zero(ec_datagram_t *datagram)
{
    ec_memset(datagram->data, 0x00, datagram->data_size);
}

void ec_datagram_fill(ec_datagram_t *datagram,
                      ec_datagram_type_t type,
                      uint16_t adp,
                      uint16_t ado,
                      uint16_t size)
{
    datagram->index = 0;
    datagram->working_counter = 0;
    datagram->state = EC_DATAGRAM_INIT;

    datagram->type = type;
    EC_WRITE_U16(datagram->address, adp);
    EC_WRITE_U16(datagram->address + 2, ado);
    datagram->data_size = size;
    datagram->waiter = 0;
}

void ec_datagram_aprd(ec_datagram_t *datagram, uint16_t autoinc_address, uint16_t mem_address, size_t data_size)
{
    ec_datagram_fill(datagram, EC_DATAGRAM_APRD, autoinc_address, mem_address, data_size);
}

void ec_datagram_apwr(ec_datagram_t *datagram, uint16_t autoinc_address, uint16_t mem_address, size_t data_size)
{
    ec_datagram_fill(datagram, EC_DATAGRAM_APWR, autoinc_address, mem_address, data_size);
}

void ec_datagram_aprw(ec_datagram_t *datagram, uint16_t autoinc_address, uint16_t mem_address, size_t data_size)
{
    ec_datagram_fill(datagram, EC_DATAGRAM_APRW, autoinc_address, mem_address, data_size);
}

void ec_datagram_armw(ec_datagram_t *datagram, uint16_t autoinc_address, uint16_t mem_address, size_t data_size)
{
    ec_datagram_fill(datagram, EC_DATAGRAM_ARMW, autoinc_address, mem_address, data_size);
}

void ec_datagram_fprd(ec_datagram_t *datagram, uint16_t configured_address, uint16_t mem_address, size_t data_size)
{
    ec_datagram_fill(datagram, EC_DATAGRAM_FPRD, configured_address, mem_address, data_size);
}

void ec_datagram_fpwr(ec_datagram_t *datagram, uint16_t configured_address, uint16_t mem_address, size_t data_size)
{
    ec_datagram_fill(datagram, EC_DATAGRAM_FPWR, configured_address, mem_address, data_size);
}

void ec_datagram_fprw(ec_datagram_t *datagram, uint16_t configured_address, uint16_t mem_address, size_t data_size)
{
    ec_datagram_fill(datagram, EC_DATAGRAM_FPRW, configured_address, mem_address, data_size);
}

void ec_datagram_frmw(ec_datagram_t *datagram, uint16_t configured_address, uint16_t mem_address, size_t data_size)
{
    ec_datagram_fill(datagram, EC_DATAGRAM_FRMW, configured_address, mem_address, data_size);
}

void ec_datagram_brd(ec_datagram_t *datagram, uint16_t mem_address, size_t data_size)
{
    ec_datagram_fill(datagram, EC_DATAGRAM_BRD, 0x0000, mem_address, data_size);
}

void ec_datagram_bwr(ec_datagram_t *datagram, uint16_t mem_address, size_t data_size)
{
    ec_datagram_fill(datagram, EC_DATAGRAM_BWR, 0x0000, mem_address, data_size);
}

void ec_datagram_brw(ec_datagram_t *datagram, uint16_t mem_address, size_t data_size)
{
    ec_datagram_fill(datagram, EC_DATAGRAM_BRW, 0x0000, mem_address, data_size);
}

void ec_datagram_lrd(ec_datagram_t *datagram, uint32_t offset, size_t data_size)
{
    ec_datagram_fill(datagram, EC_DATAGRAM_LRD, offset & 0xffff, offset >> 16, data_size);
}

void ec_datagram_lwr(ec_datagram_t *datagram, uint32_t offset, size_t data_size)
{
    ec_datagram_fill(datagram, EC_DATAGRAM_LWR, offset & 0xffff, offset >> 16, data_size);
}

void ec_datagram_lrw(ec_datagram_t *datagram, uint32_t offset, size_t data_size)
{
    ec_datagram_fill(datagram, EC_DATAGRAM_LRW, offset & 0xffff, offset >> 16, data_size);
}

const char *ec_datagram_type_string(const ec_datagram_t *datagram)
{
    return type_strings[datagram->type];
}