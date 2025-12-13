/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_master.h"

#define SII_TIMEOUT_US (200 * 1000) // 200ms

static int ec_sii_assign_master(ec_slave_t *slave, ec_datagram_t *datagram)
{
    // assign SII to ECAT
    ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->EEPROM_CFG), 1);
    EC_WRITE_U8(datagram->data, 0x00);
    datagram->netdev_idx = slave->netdev_idx;
    return ec_master_queue_ext_datagram(slave->master, datagram, true, true);
}

static int esc_sii_assign_pdi(ec_slave_t *slave, ec_datagram_t *datagram)
{
    // assign SII to PDI
    ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->EEPROM_CFG), 1);
    EC_WRITE_U8(datagram->data, 0x01);
    datagram->netdev_idx = slave->netdev_idx;
    return ec_master_queue_ext_datagram(slave->master, datagram, true, true);
}

static int ec_sii_read_dword(ec_slave_t *slave, ec_datagram_t *datagram, uint16_t woffset, uint32_t *value)
{
    uint64_t start_time;
    int ret;

    ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->EEPROM_CTRL_STAT), 4);
    EC_WRITE_U8(datagram->data, 0x80);         // two address bytes
    EC_WRITE_U8(datagram->data + 1, 0x01);     // read command
    EC_WRITE_U16(datagram->data + 2, woffset); // word offset

    datagram->netdev_idx = slave->netdev_idx;
    ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
    if (ret < 0) {
        return ret;
    }

    start_time = jiffies;
sii_check:
    // read with 4 bytes
    ec_datagram_fprd(datagram, slave->station_address, ESCREG_OF(ESCREG->EEPROM_CTRL_STAT), 10);
    ec_datagram_zero(datagram);
    datagram->netdev_idx = slave->netdev_idx;
    ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
    if (ret < 0) {
        return ret;
    }

    if (EC_READ_U16(datagram->data) & ESC_EEPROM_CTRL_STAT_ERR_ACK_CMD_MASK) {
        return -EC_ERR_SII;
    }

    if (EC_READ_U16(datagram->data) & ESC_EEPROM_CTRL_STAT_BUSY_MASK) {
        if ((jiffies - start_time) > SII_TIMEOUT_US) {
            return -EC_ERR_TIMEOUT;
        }
        goto sii_check;
    }

    ec_memcpy(value, datagram->data + 6, 4);

    return 0;
}

static int ec_sii_write_word(ec_slave_t *slave, ec_datagram_t *datagram, uint16_t woffset, uint16_t value)
{
    uint64_t start_time;
    int ret;

    // write with 2 bytes
    ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->EEPROM_CTRL_STAT), 8);
    EC_WRITE_U8(datagram->data, 0x81);         // two address bytes + enable write access
    EC_WRITE_U8(datagram->data + 1, 0x02);     // write command
    EC_WRITE_U16(datagram->data + 2, woffset); // word offset
    EC_WRITE_U16(datagram->data + 4, 0x00);
    EC_WRITE_U16(datagram->data + 6, value);

    datagram->netdev_idx = slave->netdev_idx;
    ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
    if (ret < 0) {
        return ret;
    }

    start_time = jiffies;
sii_check:
    ec_datagram_fprd(datagram, slave->station_address, ESCREG_OF(ESCREG->EEPROM_CTRL_STAT), 2);
    ec_datagram_zero(datagram);
    datagram->netdev_idx = slave->netdev_idx;
    ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
    if (ret < 0) {
        return ret;
    }

    if (EC_READ_U16(datagram->data) & ESC_EEPROM_CTRL_STAT_ERR_ACK_CMD_SHIFT) {
        return -EC_ERR_SII;
    }

    if (EC_READ_U16(datagram->data) & ESC_EEPROM_CTRL_STAT_BUSY_MASK) {
        if ((jiffies - start_time) > SII_TIMEOUT_US) {
            return -EC_ERR_TIMEOUT;
        }
        goto sii_check;
    }

    if (EC_READ_U16(datagram->data) & ESC_EEPROM_CTRL_STAT_ERR_WEN_MASK) {
        return -EC_ERR_SII;
    }

    return 0;
}

int ec_sii_read(ec_master_t *master, uint16_t slave_index, ec_datagram_t *datagram, uint16_t woffset, uint32_t *buf, uint32_t len)
{
    ec_slave_t *slave;
    int ret;

    if (len % 4) {
        return -EC_ERR_INVAL;
    }

    if (slave_index >= master->slave_count) {
        return -EC_ERR_INVAL;
    }

    slave = &master->slaves[slave_index];

    ret = ec_sii_assign_master(slave, datagram);
    if (ret < 0) {
        return ret;
    }

    for (uint32_t i = 0; i < (len / 4); i++) {
        ret = ec_sii_read_dword(slave, datagram, woffset + i * 2, &buf[i]);
        if (ret < 0) {
            return ret;
        }
    }

    return esc_sii_assign_pdi(slave, datagram);
}

int ec_sii_write(ec_master_t *master, uint16_t slave_index, ec_datagram_t *datagram, uint16_t woffset, const uint16_t *buf, uint32_t len)
{
    ec_slave_t *slave;
    int ret;

    if (len % 2) {
        return -EC_ERR_INVAL;
    }

    if (slave_index >= master->slave_count) {
        return -EC_ERR_INVAL;
    }

    slave = &master->slaves[slave_index];

    ret = ec_sii_assign_master(slave, datagram);
    if (ret < 0) {
        return ret;
    }

    for (uint32_t i = 0; i < (len / 2); i++) {
        ret = ec_sii_write_word(slave, datagram, woffset + i, buf[i]);
        if (ret < 0) {
            return ret;
        }
    }

    return esc_sii_assign_pdi(slave, datagram);
}