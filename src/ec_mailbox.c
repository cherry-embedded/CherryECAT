/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_master.h"

#define EC_MBOX_READ_TIMEOUT (100 * 1000)

uint8_t *ec_mailbox_fill_send(ec_master_t *master,
                              uint16_t slave_index,
                              ec_datagram_t *datagram,
                              uint8_t type,
                              uint16_t size)
{
    ec_slave_t *slave;

    EC_ASSERT_MSG(slave_index < master->slave_count, "Invalid slave index");

    slave = &master->slaves[slave_index];

    EC_ASSERT_MSG(datagram->mem_size >= slave->configured_rx_mailbox_size, "Datagram size too small for RX mailbox");
    EC_ASSERT_MSG((EC_MBOX_HEADER_SIZE + size) <= slave->configured_rx_mailbox_size, "RX Mailbox size overflow");

    EC_WRITE_U16(datagram->data, size);                       // mailbox service data length
    EC_WRITE_U16(datagram->data + 2, slave->station_address); // station address
    EC_WRITE_U8(datagram->data + 4, 0x00);                    // channel & priority
    EC_WRITE_U8(datagram->data + 5, type);                    // protocol type

    return (datagram->data + EC_MBOX_HEADER_SIZE);
}

int ec_mailbox_send(ec_master_t *master,
                    uint16_t slave_index,
                    ec_datagram_t *datagram)
{
    ec_slave_t *slave;

    if (slave_index >= master->slave_count) {
        return -EC_ERR_INVAL;
    }

    slave = &master->slaves[slave_index];

    ec_datagram_fpwr(datagram, slave->station_address, slave->configured_rx_mailbox_offset, slave->configured_rx_mailbox_size);
    datagram->netdev_idx = slave->netdev_idx;
    return ec_master_queue_ext_datagram(slave->master, datagram, true, true);
}

int ec_mailbox_read_status(ec_master_t *master,
                           uint16_t slave_index,
                           ec_datagram_t *datagram,
                           uint32_t timeout_us)
{
    ec_slave_t *slave;
    uint64_t start_time;
    int ret;

    if (slave_index >= master->slave_count) {
        return -EC_ERR_INVAL;
    }

    slave = &master->slaves[slave_index];

    start_time = jiffies;

check_again:
    ec_datagram_fprd(datagram, slave->station_address, ESCREG_OF(ESCREG->SYNCM[EC_SM_INDEX_MBX_READ]), 8);
    ec_datagram_zero(datagram);
    datagram->netdev_idx = slave->netdev_idx;
    ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
    if (ret < 0) {
        return ret;
    }

    if (!(EC_READ_U8(datagram->data + 5) & ESC_SYNCM_STATUS_MBX_MODE_MASK)) {
        if ((jiffies - start_time) > timeout_us) {
            return -EC_ERR_MBOX_EMPTY;
        }
        goto check_again;
    }

    return 0;
}

int ec_mailbox_receive(ec_master_t *master,
                       uint16_t slave_index,
                       ec_datagram_t *datagram,
                       uint8_t *type,
                       uint32_t *size,
                       uint32_t timeout_us)
{
    ec_slave_t *slave;
    uint16_t code;
    uint32_t tmp_size;
    uint8_t tmp_type;
    int ret;

    if (slave_index >= master->slave_count) {
        return -EC_ERR_INVAL;
    }

    slave = &master->slaves[slave_index];

    ret = ec_mailbox_read_status(master, slave_index, datagram, timeout_us);
    if (ret < 0) {
        return ret;
    }

    EC_ASSERT_MSG(datagram->mem_size >= slave->configured_tx_mailbox_size, "Datagram size too small for TX mailbox");

    ec_datagram_fprd(datagram, slave->station_address, slave->configured_tx_mailbox_offset, slave->configured_tx_mailbox_size);
    ec_datagram_zero(datagram);
    datagram->netdev_idx = slave->netdev_idx;
    ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
    if (ret < 0) {
        return ret;
    }

    tmp_size = EC_READ_U16(datagram->data);
    tmp_type = EC_READ_U8(datagram->data + 5) & 0x0F;

    EC_ASSERT_MSG(tmp_size <= slave->configured_tx_mailbox_size, "TX Mailbox size overflow");

    if (tmp_type == 0x00) {
        code = EC_READ_U16(datagram->data + 8);

        EC_SLAVE_LOG_ERR("Slave %u mailbox errorcode: 0x%04x (%s)\n", slave->index, code, ec_mbox_error_string(code));
        return -EC_ERR_MBOX;
    }

    *type = tmp_type;
    *size = tmp_size;

    return 0;
}