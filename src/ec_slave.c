/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_master.h"

/** Maximum clock difference (in ns) before going to SAFEOP.
 *
 * Wait for DC time difference to drop under this absolute value before
 * requesting SAFEOP.
 */
#define EC_DC_MAX_SYNC_DIFF_NS 100

/** Maximum count to wait for clock discipline.
 */
#define EC_DC_SYNC_WAIT_COUNT (15000)

/** Time offset (in ns), that is added to cyclic start time.
 */
#define EC_DC_START_OFFSET 100000000ULL

static void ec_slave_init(ec_slave_t *slave,
                          uint32_t slave_index,
                          ec_master_t *master,
                          uint8_t netdev_idx,
                          uint16_t autoinc_address,
                          uint16_t station_address)
{
    slave->index = slave_index;
    slave->master = master;
    slave->netdev_idx = netdev_idx;
    slave->autoinc_address = autoinc_address;
    slave->station_address = station_address;

    slave->requested_state = EC_SLAVE_STATE_PREOP;
}

static void ec_slave_clear(ec_slave_t *slave)
{
    int i;

    if (slave->sii_image) {
        ec_osal_free(slave->sii_image);
        slave->sii_image = NULL;
        slave->sii_nwords = 0;
    }

    if (slave->sii.strings) {
        for (i = 0; i < slave->sii.string_count; i++)
            ec_osal_free(slave->sii.strings[i]);
        ec_osal_free(slave->sii.strings);
        slave->sii.strings = NULL;
    }

    if (slave->sm_info) {
        ec_osal_free(slave->sm_info);
        slave->sm_info = NULL;
    }
}

static int ec_slave_fetch_sii_strings(ec_slave_t *slave, const uint8_t *data, size_t data_size)
{
    int i, ret;
    uint32_t size;
    uint32_t offset;

    slave->sii.string_count = data[0];

    if (slave->sii.string_count) {
        slave->sii.strings = ec_osal_malloc(sizeof(char *) * slave->sii.string_count);
        if (!slave->sii.strings) {
            EC_SLAVE_LOG_ERR("Failed to allocate string array memory\n");
            ret = -EC_ERR_NOMEM;
            goto errorout1;
        }

        offset = 1;
        for (i = 0; i < slave->sii.string_count; i++) {
            size = data[offset];

            slave->sii.strings[i] = ec_osal_malloc(sizeof(char) * size + 1);
            if (!slave->sii.strings[i]) {
                EC_SLAVE_LOG_ERR("Failed to allocate string memory\n");
                ret = -EC_ERR_NOMEM;
                goto errorout2;
            }

            ec_memcpy(slave->sii.strings[i], data + offset + 1, size);
            slave->sii.strings[i][size] = 0x00;
            offset += 1 + size;
        }
    }

    return 0;

errorout2:
    for (i = 0; i < slave->sii.string_count; i++) {
        ec_osal_free(slave->sii.strings[i]);
    }
    ec_osal_free(slave->sii.strings);
    slave->sii.strings = NULL;
errorout1:
    slave->sii.string_count = 0;
    return ret;
}

/** Get timeout in ns.
 *
 * For defaults see ETG2000_S_R_V1i0i15 section 5.3.6.2.
 */
static uint64_t ec_slave_state_change_timeout_ns(ec_slave_state_t old_state, ec_slave_state_t requested_state)
{
    ec_slave_state_t from = old_state;
    ec_slave_state_t to = requested_state;

    if (from == EC_SLAVE_STATE_INIT &&
        (to == EC_SLAVE_STATE_PREOP || to == EC_SLAVE_STATE_BOOT)) {
        return (3000 * 1000 * 1000ULL); // PreopTimeout
    }
    if ((from == EC_SLAVE_STATE_PREOP && to == EC_SLAVE_STATE_SAFEOP) ||
        (from == EC_SLAVE_STATE_SAFEOP && to == EC_SLAVE_STATE_OP)) {
        return (10000 * 1000 * 1000ULL); // SafeopOpTimeout
    }
    if (to == EC_SLAVE_STATE_INIT ||
        ((from == EC_SLAVE_STATE_OP || from == EC_SLAVE_STATE_SAFEOP) && to == EC_SLAVE_STATE_PREOP)) {
        return (5000 * 1000 * 1000ULL); // BackToInitTimeout
    }
    if (from == EC_SLAVE_STATE_OP && to == EC_SLAVE_STATE_SAFEOP) {
        return (200 * 1000 * 1000ULL); // BackToSafeopTimeout
    }

    return (10000 * 1000 * 1000ULL); // default [ns]
}

static uint8_t ec_slave_get_previous_port(const ec_slave_t *slave, uint8_t port_index)
{
    static const uint8_t prev_table[EC_MAX_PORTS] = {
        2, 3, 1, 0
    };

    do {
        port_index = prev_table[port_index];
        if (slave->ports[port_index].next_slave) {
            return port_index;
        }
    } while (port_index);

    return 0;
}

static uint8_t ec_slave_get_next_port(const ec_slave_t *slave, uint8_t port_index)
{
    static const uint8_t next_table[EC_MAX_PORTS] = {
        3, 2, 0, 1
    };

    do {
        port_index = next_table[port_index];
        if (slave->ports[port_index].next_slave) {
            return port_index;
        }
    } while (port_index);

    return 0;
}

/** Calculates the sum of round-trip-times of connected ports 1-3.
 *
 * \return Round-trip-time in ns.
 */
static uint32_t ec_slave_calc_rtt_sum(const ec_slave_t *slave)
{
    uint32_t rtt_sum = 0, rtt;
    uint8_t port_index = ec_slave_get_next_port(slave, 0);

    while (port_index != 0) {
        uint8_t prev_index =
            ec_slave_get_previous_port(slave, port_index);

        rtt = slave->ports[port_index].receive_time -
              slave->ports[prev_index].receive_time;
        rtt_sum += rtt;
        port_index = ec_slave_get_next_port(slave, port_index);
    }

    return rtt_sum;
}

/** Finds the next slave supporting DC delay measurement.
 *
 * \return Next DC slave, or NULL.
 */
static ec_slave_t *ec_slave_find_next_dc_slave(ec_slave_t *slave)
{
    uint8_t port_index;
    ec_slave_t *dc_slave = NULL;

    if (slave->base_dc_supported) {
        dc_slave = slave;
    } else {
        port_index = ec_slave_get_next_port(slave, 0);

        while (port_index != 0) {
            ec_slave_t *next = slave->ports[port_index].next_slave;

            if (next) {
                dc_slave = ec_slave_find_next_dc_slave(next);

                if (dc_slave) {
                    break;
                }
            }
            port_index = ec_slave_get_next_port(slave, port_index);
        }
    }

    return dc_slave;
}

static void ec_slave_calc_port_delays(ec_slave_t *slave)
{
    uint8_t port_index;
    ec_slave_t *next_slave, *next_dc;
    uint32_t rtt, next_rtt_sum;

    if (!slave->base_dc_supported)
        return;

    port_index = ec_slave_get_next_port(slave, 0);

    while (port_index != 0) {
        next_slave = slave->ports[port_index].next_slave;
        next_dc = ec_slave_find_next_dc_slave(next_slave);

        if (next_dc) {
            uint8_t prev_port =
                ec_slave_get_previous_port(slave, port_index);

            rtt = slave->ports[port_index].receive_time -
                  slave->ports[prev_port].receive_time;
            next_rtt_sum = ec_slave_calc_rtt_sum(next_dc);

            slave->ports[port_index].delay_to_next_dc =
                (rtt - next_rtt_sum) / 2;
            next_dc->ports[0].delay_to_next_dc =
                (rtt - next_rtt_sum) / 2;
        }

        port_index = ec_slave_get_next_port(slave, port_index);
    }
}

static void ec_slave_calc_transmission_delays(ec_slave_t *slave, uint32_t *delay)
{
    unsigned int i;
    ec_slave_t *next_dc;

    slave->transmission_delay = *delay;

    i = ec_slave_get_next_port(slave, 0);

    while (i != 0) {
        ec_slave_port_t *port = &slave->ports[i];
        next_dc = ec_slave_find_next_dc_slave(port->next_slave);
        if (next_dc) {
            *delay = *delay + port->delay_to_next_dc;

            ec_slave_calc_transmission_delays(next_dc, delay);
        }

        i = ec_slave_get_next_port(slave, i);
    }

    *delay = *delay + slave->ports[0].delay_to_next_dc;
}

static int ec_slave_state_clear_ack_error(ec_slave_t *slave, ec_slave_state_t requested_state)
{
    ec_datagram_t *datagram;
    int ret;
    uint64_t start_time;
    uint32_t status_code;

    datagram = &slave->master->main_datagram;

    start_time = jiffies;

    ec_datagram_fprd(datagram, slave->station_address, ESCREG_OF(ESCREG->AL_STAT_CODE), 2);
    ec_datagram_zero(datagram);
    datagram->netdev_idx = slave->netdev_idx;
    ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
    if (ret < 0) {
        return ret;
    }

    status_code = EC_READ_U16(datagram->data);

    slave->alstatus_code = status_code;

    ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->AL_CTRL), 2);
    EC_WRITE_U16(datagram->data, slave->current_state);
    datagram->netdev_idx = slave->netdev_idx;
    ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
    if (ret < 0) {
        return ret;
    }

repeat_check:
    ec_datagram_fprd(datagram, slave->station_address, ESCREG_OF(ESCREG->AL_STAT), 2);
    ec_datagram_zero(datagram);
    datagram->netdev_idx = slave->netdev_idx;
    ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
    if (ret < 0) {
        return ret;
    }

    slave->current_state = EC_READ_U8(datagram->data);

    if (!(slave->current_state & EC_SLAVE_STATE_ACK_ERR)) {
        if (slave->current_state == slave->requested_state) {
            return 0;
        } else {
            EC_SLAVE_LOG_ERR("Slave %u acked state %s, alstatus code: 0x%04x (%s)\n",
                             slave->index,
                             ec_state_string(slave->current_state, 0),
                             status_code,
                             ec_alstatus_string(status_code));

            return -EC_ERR_ALERR;
        }
    } else {
        if ((jiffies - start_time) > ec_slave_state_change_timeout_ns(slave->current_state, requested_state)) {
            return -EC_ERR_TIMEOUT;
        }
        goto repeat_check;
    }
}

/* ec_slave_state_change - change slave state
 *
 * 1. write AL control register
 * 2. read AL status register
 * 3. if state is changed to correct state, return success
 * 4. if state is not changed and acknowledge bit is set, read AL status code
 *    and write AL control register to acknowledge error, then repeat 2
 * 5. if state is changed to other state, repeat step 2
*/
static int ec_slave_state_change(ec_slave_t *slave, ec_slave_state_t requested_state)
{
    ec_datagram_t *datagram;
    int ret;
    uint64_t start_time;
    ec_slave_state_t old_state;

    datagram = &slave->master->main_datagram;

    old_state = slave->current_state;
    start_time = jiffies;

    ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->AL_CTRL), 2);
    EC_WRITE_U16(datagram->data, requested_state);
    datagram->netdev_idx = slave->netdev_idx;
    ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
    if (ret < 0) {
        return ret;
    }

repeat_check:
    ec_datagram_fprd(datagram, slave->station_address, ESCREG_OF(ESCREG->AL_STAT), 2);
    ec_datagram_zero(datagram);
    datagram->netdev_idx = slave->netdev_idx;
    ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
    if (ret < 0) {
        return ret;
    }

    slave->current_state = EC_READ_U8(datagram->data);

    if (slave->current_state == requested_state) {
        EC_SLAVE_LOG_INFO("Slave %u State changed to %s\n", slave->index, ec_state_string(slave->current_state, 0));
        return 0;
    }

    if (slave->current_state != old_state) {
        if ((slave->current_state & 0x0F) == (old_state & 0x0F)) { // acknowledge bit enable
            return ec_slave_state_clear_ack_error(slave, requested_state);
        } else {
            old_state = slave->current_state;
            if ((jiffies - start_time) > ec_slave_state_change_timeout_ns(slave->current_state, requested_state)) {
                return -EC_ERR_TIMEOUT;
            }
            goto repeat_check;
        }
    }

    // still in old state
    if ((jiffies - start_time) > ec_slave_state_change_timeout_ns(slave->current_state, requested_state)) {
        return -EC_ERR_TIMEOUT;
    }
    goto repeat_check;
}

static inline void ec_slave_sm_config(ec_sm_info_t *sm, uint8_t *data)
{
    EC_WRITE_U16(data, sm->physical_start_address);
    EC_WRITE_U16(data + 2, sm->length);
    EC_WRITE_U8(data + 4, sm->control);
    EC_WRITE_U8(data + 5, 0x00); // status byte (read only)
    EC_WRITE_U16(data + 6, sm->enable);
}

static inline void ec_slave_fmmu_config(ec_sm_info_t *sm, uint8_t *data)
{
    EC_WRITE_U32(data, sm->fmmu.logical_start_address);
    EC_WRITE_U16(data + 4, sm->fmmu.data_size); // size of fmmu
    EC_WRITE_U8(data + 6, 0x00);                // logical start bit
    EC_WRITE_U8(data + 7, 0x07);                // logical end bit
    EC_WRITE_U16(data + 8, sm->physical_start_address);
    EC_WRITE_U8(data + 10, 0x00); // physical start bit
    EC_WRITE_U8(data + 11, sm->fmmu.dir == EC_DIR_INPUT ? 0x01 : 0x02);
    EC_WRITE_U16(data + 12, 0x0001); // enable
    EC_WRITE_U16(data + 14, 0x0000); // reserved
}

static int ec_slave_config(ec_slave_t *slave)
{
    ec_datagram_t *datagram;
    uint64_t start_time;
    uint8_t step = 0;
    bool coe_support;
    uint8_t pdo_sm_count;
    uint8_t pdo_sm_offset;
    int ret;

    datagram = &slave->master->main_datagram;

    coe_support = slave->sii.mailbox_protocols & EC_MBXPROT_COE ? true : false;
    pdo_sm_count = coe_support ? (slave->sm_count - 2) : slave->sm_count;
    pdo_sm_offset = coe_support ? 2 : 0;

    ret = ec_slave_state_change(slave, EC_SLAVE_STATE_INIT);
    if (ret < 0) {
        step = 1;
        goto errorout;
    }

    // clear FMMU configurations
    ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->FMMU[0]), EC_FMMU_PAGE_SIZE * slave->base_fmmu_count);
    ec_datagram_zero(datagram);
    datagram->netdev_idx = slave->netdev_idx;
    ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
    if (ret < 0) {
        step = 2;
        goto errorout;
    }

    // clear sync manager configurations
    ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->SYNCM[0]), EC_SYNC_PAGE_SIZE * slave->base_sync_count);
    ec_datagram_zero(datagram);
    datagram->netdev_idx = slave->netdev_idx;
    ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
    if (ret < 0) {
        step = 3;
        goto errorout;
    }

    // Clear the DC assignment
    ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->CYC_UNIT_CTRL), 2);
    ec_datagram_zero(datagram);
    datagram->netdev_idx = slave->netdev_idx;
    ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
    if (ret < 0) {
        step = 4;
        goto errorout;
    }

    // init state done
    if (slave->current_state == slave->requested_state) {
        ret = 0;
        goto errorout;
    }

    if (slave->requested_state == EC_SLAVE_STATE_BOOT) {
        if (!(slave->sii.mailbox_protocols & EC_MBXPROT_FOE)) {
            EC_SLAVE_LOG_ERR("Slave %u does not support BOOT mailbox protocol\n", slave->index);
            ret = -EC_ERR_NOSUPP;
            step = 5;
            goto errorout;
        }

        ec_sm_info_t sm_info[2];

        sm_info[0].physical_start_address = slave->sii.boot_rx_mailbox_offset;
        sm_info[0].control = 0x26;
        sm_info[0].length = slave->sii.boot_rx_mailbox_size;
        sm_info[0].enable = 0x01;

        sm_info[1].physical_start_address = slave->sii.boot_tx_mailbox_offset;
        sm_info[1].control = 0x22;
        sm_info[1].length = slave->sii.boot_tx_mailbox_size;
        sm_info[1].enable = 0x01;

        // Config mailbox sm
        ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->SYNCM[0]), EC_SYNC_PAGE_SIZE * 2);
        ec_datagram_zero(datagram);
        for (uint8_t i = 0; i < 2; i++) {
            ec_slave_sm_config(&sm_info[i], datagram->data + EC_SYNC_PAGE_SIZE * i);
        }
        datagram->netdev_idx = slave->netdev_idx;
        ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
        if (ret < 0) {
            step = 5;
            goto errorout;
        }

        slave->configured_rx_mailbox_offset = slave->sii.boot_rx_mailbox_offset;
        slave->configured_rx_mailbox_size = slave->sii.boot_rx_mailbox_size;
        slave->configured_tx_mailbox_offset = slave->sii.boot_tx_mailbox_offset;
        slave->configured_tx_mailbox_size = slave->sii.boot_tx_mailbox_size;

        ret = ec_slave_state_change(slave, EC_SLAVE_STATE_BOOT);
        if (ret < 0) {
            step = 6;
            goto errorout;
        }

        ret = 0;
        goto errorout;
    }

    if (coe_support) {
        // Config mailbox sm
        ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->SYNCM[0]), EC_SYNC_PAGE_SIZE * 2);
        ec_datagram_zero(datagram);
        for (uint8_t i = 0; i < 2; i++) {
            ec_slave_sm_config(&slave->sm_info[i], datagram->data + EC_SYNC_PAGE_SIZE * i);
        }
        datagram->netdev_idx = slave->netdev_idx;
        ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
        if (ret < 0) {
            step = 7;
            goto errorout;
        }

        slave->configured_rx_mailbox_offset = slave->sm_info[EC_SM_INDEX_MBX_WRITE].physical_start_address;
        slave->configured_rx_mailbox_size = slave->sm_info[EC_SM_INDEX_MBX_WRITE].length;
        slave->configured_tx_mailbox_offset = slave->sm_info[EC_SM_INDEX_MBX_READ].physical_start_address;
        slave->configured_tx_mailbox_size = slave->sm_info[EC_SM_INDEX_MBX_READ].length;
    }

    ret = ec_slave_state_change(slave, EC_SLAVE_STATE_PREOP);
    if (ret < 0) {
        step = 8;
        goto errorout;
    }

    // preop state done
    if (slave->current_state == slave->requested_state) {
        ret = 0;
        goto errorout;
    }

    if (slave->config && slave->sii.general.coe_details.enable_pdo_assign && coe_support) {
        uint32_t data;

        /* Config PDO assignments for 0x1c12, 0x1c13
         *
         * Clear existing assignments first
         * Reassign all entries
         * Set number of assigned entries
        */
        data = 0;
        ret = ec_coe_download(slave->master, slave->index, datagram, 0x1c12, 0x00, &data, (slave->sm_info[EC_SM_INDEX_PROCESS_DATA_OUTPUT].pdo_assign.count > 0xff) ? 2 : 1, false);
        if (ret < 0) {
            step = 9;
            goto errorout;
        }
        ret = ec_coe_download(slave->master, slave->index, datagram, 0x1c13, 0x00, &data, (slave->sm_info[EC_SM_INDEX_PROCESS_DATA_INPUT].pdo_assign.count > 0xff) ? 2 : 1, false);
        if (ret < 0) {
            step = 10;
            goto errorout;
        }
        for (uint32_t i = 0; i < slave->sm_info[EC_SM_INDEX_PROCESS_DATA_OUTPUT].pdo_assign.count; i++) {
            data = slave->sm_info[EC_SM_INDEX_PROCESS_DATA_OUTPUT].pdo_assign.entry[i];
            ret = ec_coe_download(slave->master, slave->index, datagram, 0x1c12, 0x01 + i, &data, 2, false);
            if (ret < 0) {
                step = 11;
                goto errorout;
            }
        }
        for (uint32_t i = 0; i < slave->sm_info[EC_SM_INDEX_PROCESS_DATA_INPUT].pdo_assign.count; i++) {
            data = slave->sm_info[EC_SM_INDEX_PROCESS_DATA_INPUT].pdo_assign.entry[i];
            ret = ec_coe_download(slave->master, slave->index, datagram, 0x1c13, 0x01 + i, &data, 2, false);
            if (ret < 0) {
                step = 12;
                goto errorout;
            }
        }
        data = slave->sm_info[EC_SM_INDEX_PROCESS_DATA_OUTPUT].pdo_assign.count;
        ret = ec_coe_download(slave->master, slave->index, datagram, 0x1c12, 0x00, &data, (data > 0xff) ? 2 : 1, false);
        if (ret < 0) {
            step = 13;
            goto errorout;
        }
        data = slave->sm_info[EC_SM_INDEX_PROCESS_DATA_INPUT].pdo_assign.count;
        ret = ec_coe_download(slave->master, slave->index, datagram, 0x1c13, 0x00, &data, (data > 0xff) ? 2 : 1, false);
        if (ret < 0) {
            step = 14;
            goto errorout;
        }

        /* Config PDO mappings
         *
         * Clear existing mappings first
         * Remap all entries
         * Set number of mapped entries
        */
        if (slave->sii.general.coe_details.enable_pdo_configuration) {
            for (uint32_t i = 0; i < slave->sm_info[EC_SM_INDEX_PROCESS_DATA_OUTPUT].pdo_assign.count; i++) {
                data = 0;
                ret = ec_coe_download(slave->master, slave->index, datagram, slave->sm_info[EC_SM_INDEX_PROCESS_DATA_OUTPUT].pdo_assign.entry[i], 0x00, &data, 1, false);
                if (ret < 0) {
                    step = 15;
                    goto errorout;
                }

                for (uint32_t j = 0; j < slave->sm_info[EC_SM_INDEX_PROCESS_DATA_OUTPUT].pdo_mapping[i].count; j++) {
                    data = slave->sm_info[EC_SM_INDEX_PROCESS_DATA_OUTPUT].pdo_mapping[i].entry[j];
                    ret = ec_coe_download(slave->master, slave->index, datagram, slave->sm_info[EC_SM_INDEX_PROCESS_DATA_OUTPUT].pdo_assign.entry[i], 0x01 + j, &data, 4, false);
                    if (ret < 0) {
                        step = 16;
                        goto errorout;
                    }
                }

                data = slave->sm_info[EC_SM_INDEX_PROCESS_DATA_OUTPUT].pdo_mapping[i].count;
                ret = ec_coe_download(slave->master, slave->index, datagram, slave->sm_info[EC_SM_INDEX_PROCESS_DATA_OUTPUT].pdo_assign.entry[i], 0x00, &data, 1, false);
                if (ret < 0) {
                    step = 17;
                    goto errorout;
                }
            }

            for (uint32_t i = 0; i < slave->sm_info[EC_SM_INDEX_PROCESS_DATA_INPUT].pdo_assign.count; i++) {
                data = 0;
                ret = ec_coe_download(slave->master, slave->index, datagram, slave->sm_info[EC_SM_INDEX_PROCESS_DATA_INPUT].pdo_assign.entry[i], 0x00, &data, 1, false);
                if (ret < 0) {
                    step = 18;
                    goto errorout;
                }

                for (uint32_t j = 0; j < slave->sm_info[EC_SM_INDEX_PROCESS_DATA_INPUT].pdo_mapping[i].count; j++) {
                    data = slave->sm_info[EC_SM_INDEX_PROCESS_DATA_INPUT].pdo_mapping[i].entry[j];
                    ret = ec_coe_download(slave->master, slave->index, datagram, slave->sm_info[EC_SM_INDEX_PROCESS_DATA_INPUT].pdo_assign.entry[i], 0x01 + j, &data, 4, false);
                    if (ret < 0) {
                        step = 19;
                        goto errorout;
                    }
                }

                data = slave->sm_info[EC_SM_INDEX_PROCESS_DATA_INPUT].pdo_mapping[i].count;
                ret = ec_coe_download(slave->master, slave->index, datagram, slave->sm_info[EC_SM_INDEX_PROCESS_DATA_INPUT].pdo_assign.entry[i], 0x00, &data, 1, false);
                if (ret < 0) {
                    step = 20;
                    goto errorout;
                }
            }
        }
    }

    if (slave->config) {
        // Config process data sm
        ec_datagram_fpwr(datagram, slave->station_address,
                         ESCREG_OF(ESCREG->SYNCM[pdo_sm_offset]), EC_SYNC_PAGE_SIZE * pdo_sm_count);
        ec_datagram_zero(datagram);
        for (uint8_t i = 0; i < pdo_sm_count; i++) {
            ec_slave_sm_config(&slave->sm_info[pdo_sm_offset + i], datagram->data + EC_SYNC_PAGE_SIZE * i);
        }
        datagram->netdev_idx = slave->netdev_idx;
        ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
        if (ret < 0) {
            step = 21;
            goto errorout;
        }

        ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->FMMU[0]), EC_FMMU_PAGE_SIZE * pdo_sm_count);
        ec_datagram_zero(datagram);
        for (uint8_t i = 0; i < pdo_sm_count; i++) {
            ec_slave_fmmu_config(&slave->sm_info[pdo_sm_offset + i], datagram->data + EC_FMMU_PAGE_SIZE * i);
        }
        datagram->netdev_idx = slave->netdev_idx;
        ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
        if (ret < 0) {
            step = 22;
            goto errorout;
        }
    }

    if (slave->config && slave->config->dc_assign_activate) {
        EC_ASSERT_MSG(slave->base_dc_supported, "Slave %u does not support DC", slave->index);

        // set DC system time offset and transmission delay
        ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->SYS_TIME_OFFSET), 12);
        EC_WRITE_U64(datagram->data, slave->system_time_offset);
        EC_WRITE_U32(datagram->data + 8, slave->transmission_delay);
        datagram->netdev_idx = slave->netdev_idx;
        ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
        if (ret < 0) {
            return ret;
        }

        // set DC cycle times
        ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->SYNC0_CYC_TIME), 8);
        EC_WRITE_U32(datagram->data, slave->config->dc_sync[0].cycle_time);
        EC_WRITE_U32(datagram->data + 4, slave->config->dc_sync[1].cycle_time);
        datagram->netdev_idx = slave->netdev_idx;
        ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
        if (ret < 0) {
            step = 23;
            goto errorout;
        }

        start_time = 0;
    read_check:
        ec_datagram_fprd(datagram, slave->station_address, ESCREG_OF(ESCREG->SYS_TIME_DIFF), 4);
        ec_datagram_zero(datagram);
        datagram->netdev_idx = slave->netdev_idx;
        ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
        if (ret < 0) {
            step = 24;
            goto errorout;
        }

        uint32_t time_diff = EC_READ_U32(datagram->data) & 0x7fffffff;
        if (time_diff > EC_DC_MAX_SYNC_DIFF_NS) {
            start_time++;
            if (start_time > EC_DC_SYNC_WAIT_COUNT) {
                step = 25;
                ret = -EC_ERR_TIMEOUT;
                goto errorout;
            }
            goto read_check;
        } else {
            EC_SLAVE_LOG_INFO("Slave %u DC time diff: %u ns\n", slave->index, time_diff);
        }

        uint64_t dc_start_time;
        uint32_t remainder = EC_DC_START_OFFSET / (slave->config->dc_sync[0].cycle_time + slave->config->dc_sync[1].cycle_time);

        dc_start_time = ec_timestamp_get_time_ns() + EC_DC_START_OFFSET +
                        slave->config->dc_sync[0].cycle_time + slave->config->dc_sync[1].cycle_time - remainder +
                        slave->config->dc_sync[0].shift_time;
        ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->START_TIME_CO), 8);

        EC_WRITE_U64(datagram->data, dc_start_time);
        datagram->netdev_idx = slave->netdev_idx;
        ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
        if (ret < 0) {
            step = 26;
            goto errorout;
        }

        ec_datagram_fpwr(datagram, slave->station_address, ESCREG_OF(ESCREG->CYC_UNIT_CTRL), 2);
        EC_WRITE_U16(datagram->data, slave->config->dc_assign_activate);
        datagram->netdev_idx = slave->netdev_idx;
        ret = ec_master_queue_ext_datagram(slave->master, datagram, true, true);
        if (ret < 0) {
            step = 27;
            goto errorout;
        }
    }

    ret = ec_slave_state_change(slave, EC_SLAVE_STATE_SAFEOP);
    if (ret < 0) {
        step = 28;
        goto errorout;
    }

    // safeop state done
    if (slave->current_state == slave->requested_state) {
        ret = 0;
        goto errorout;
    }

    ret = ec_slave_state_change(slave, EC_SLAVE_STATE_OP);
    if (ret < 0) {
        step = 29;
        goto errorout;
    }

errorout:
    if (ret < 0) {
        EC_SLAVE_LOG_ERR("Configure slave %u failed at step %u, errorcode: %d\n", slave->index, step, ret);
        return ret;
    } else {
        return 0;
    }
}

static void ec_master_clear_slaves(ec_master_t *master)
{
    ec_slave_t *slave;

    if (master->slaves) {
        for (uint32_t slave_index = 0; slave_index < master->slave_count; slave_index++) {
            slave = master->slaves + slave_index;
            ec_slave_clear(slave);
        }

        ec_osal_free(master->slaves);
        master->slaves = NULL;
    }

    master->slave_count = 0;
}

static int ec_master_calc_topology(ec_master_t *master, ec_slave_t *port0_slave, unsigned int *slave_position)
{
    ec_slave_t *slave = master->slaves + *slave_position;
    unsigned int port_index;
    int ret;

    static const unsigned int next_table[EC_MAX_PORTS] = {
        3, 2, 0, 1
    };

    slave->ports[0].next_slave = port0_slave;

    port_index = 3;
    while (port_index != 0) {
        if (!slave->ports[port_index].link.loop_closed) {
            *slave_position = *slave_position + 1;
            if (*slave_position < master->slave_count) {
                slave->ports[port_index].next_slave =
                    master->slaves + *slave_position;
                ret = ec_master_calc_topology(master,
                                              slave, slave_position);
                if (ret) {
                    return ret;
                }
            } else {
                return -1;
            }
        }

        port_index = next_table[port_index];
    }

    return 0;
}

static void ec_master_find_dc_ref_clock(ec_master_t *master)
{
    ec_slave_t *slave, *ref = NULL;

    // Use first slave with DC support as reference clock
    for (slave = master->slaves;
         slave < master->slaves + master->slave_count;
         slave++) {
        if (slave->base_dc_supported) {
            ref = slave;
            break;
        }
    }

    master->dc_ref_clock = ref;

    if (ref) {
        EC_LOG_INFO("Using slave %u as DC reference clock\n", ref->index);
    } else {
        EC_LOG_INFO("No DC reference clock found\n");
    }

    ec_datagram_fpwr(&master->dc_ref_sync_datagram,
                     ref ? ref->station_address : 0xffff, ESCREG_OF(ESCREG->SYS_TIME), ref ? (ref->base_dc_range == EC_DC_64 ? 8 : 4) : 4);
    ec_datagram_frmw(&master->dc_all_sync_datagram,
                     ref ? ref->station_address : 0xffff, ESCREG_OF(ESCREG->SYS_TIME), ref ? (ref->base_dc_range == EC_DC_64 ? 8 : 4) : 4);
}

static void ec_master_calc_transmission_delays(ec_master_t *master)
{
    ec_slave_t *slave;

    for (slave = master->slaves;
         slave < master->slaves + master->slave_count;
         slave++) {
        ec_slave_calc_port_delays(slave);
    }

    if (master->dc_ref_clock) {
        uint32_t delay = 0;
        ec_slave_calc_transmission_delays(master->dc_ref_clock, &delay);
    }
}

static void ec_master_calc_dc(ec_master_t *master)
{
    unsigned int slave_position = 0;

    // find DC reference clock
    ec_master_find_dc_ref_clock(master);

    // calculate bus topology
    EC_ASSERT_MSG(ec_master_calc_topology(master, NULL, &slave_position) == 0,
                  "Failed to calculate bus topology\n");

    // calculate transmission delay
    ec_master_calc_transmission_delays(master);
}

static void ec_master_scan_slaves_state(ec_master_t *master)
{
    ec_datagram_t *datagram;
    ec_slave_t *slave;
    uint8_t slave_state;
    int ret;

    datagram = &master->main_datagram;

    for (uint32_t slave_index = 0; slave_index < master->slave_count; slave_index++) {
        slave = master->slaves + slave_index;

        ec_datagram_fprd(datagram, slave->station_address, ESCREG_OF(ESCREG->AL_STAT), 2);
        ec_datagram_zero(datagram);
        datagram->netdev_idx = slave->netdev_idx;
        ret = ec_master_queue_ext_datagram(master, datagram, true, true);
        if (ret < 0) {
            continue;
        }

        slave_state = EC_READ_U8(datagram->data);

        if (slave->current_state != slave_state) {
            EC_SLAVE_LOG_WRN("Slave %u state changed to %s\n", slave->index, ec_state_string(slave_state, 0));
            slave->current_state = slave_state;
        }

        if (slave->current_state & EC_SLAVE_STATE_ACK_ERR) { // acknowledge bit enable
            ret = ec_slave_state_clear_ack_error(slave, slave->requested_state);
            if (ret < 0) {
                continue;
            }
        } else {
            slave->alstatus_code = 0;
        }

        if (((slave->requested_state != slave->current_state) && (slave->alstatus_code == 0)) || slave->force_update) {
            ec_slave_config(slave);
            slave->force_update = false;
        }
    }
}

void ec_slaves_scanning(ec_master_t *master)
{
    ec_datagram_t *datagram;
    ec_slave_t *slave;
    uint8_t netdev_idx;
    bool rescan_required = false;
    uint64_t scan_jiffies;
    uint64_t ref_time[CONFIG_EC_MAX_NETDEVS] = { 0 };
    int ret;

    datagram = &master->main_datagram;

    for (netdev_idx = EC_NETDEV_MAIN; netdev_idx < CONFIG_EC_MAX_NETDEVS; netdev_idx++) {
        if (!master->link_state[netdev_idx] && master->netdev[netdev_idx]->link_state) {
            EC_LOG_INFO("Detect link up on %s\n",
                        master->netdev[netdev_idx]->name);
        }

        if (master->link_state[netdev_idx] && !master->netdev[netdev_idx]->link_state) {
            EC_LOG_INFO("Detect link down on %s\n",
                        master->netdev[netdev_idx]->name);

            ec_master_stop(master);

            ec_osal_mutex_take(master->scan_lock);
            ec_master_clear_slaves(master);

            for (uint8_t i = EC_NETDEV_MAIN; i < CONFIG_EC_MAX_NETDEVS; i++) {
                master->slaves_state[i] = 0x00;
                master->slaves_working_counter[i] = 0;
            }
            master->scan_done = false;
            ec_osal_mutex_give(master->scan_lock);
        }
        master->link_state[netdev_idx] = master->netdev[netdev_idx]->link_state;
    }

    for (netdev_idx = EC_NETDEV_MAIN; netdev_idx < CONFIG_EC_MAX_NETDEVS; netdev_idx++) {
        ec_datagram_brd(datagram, ESCREG_OF(ESCREG->AL_STAT), 2);
        ec_datagram_zero(datagram);
        datagram->netdev_idx = netdev_idx;
        ret = ec_master_queue_ext_datagram(master, datagram, true, true);
        if (ret < 0) {
            return;
        }

        if (datagram->working_counter != master->slaves_working_counter[netdev_idx]) {
            rescan_required = 1;
            master->slaves_working_counter[netdev_idx] = datagram->working_counter;
            EC_LOG_INFO("%u slaves responding on %s device\n",
                        master->slaves_working_counter[netdev_idx],
                        master->netdev[netdev_idx]->name);
        }

        if (master->slaves_working_counter[netdev_idx] > 0) {
            uint8_t states = EC_READ_U8(datagram->data);
            if (states != master->slaves_state[netdev_idx]) {
                // slave states changed
                master->slaves_state[netdev_idx] = states;
                EC_LOG_INFO("Slaves state on %s device: %s\n",
                            master->netdev[netdev_idx]->name, ec_state_string(states, 1));
            }
        } else {
            master->slaves_state[netdev_idx] = 0;
        }
    }

    if (rescan_required || master->rescan_request) {
        uint32_t count = 0, slave_index, autoinc_address;
        uint8_t step = 0;

        master->rescan_request = false;
        rescan_required = 0;

        ec_master_stop(master);

        ec_osal_mutex_take(master->scan_lock);

        master->scan_done = false;
        EC_LOG_INFO("Rescanning bus...\n");

        ec_master_clear_slaves(master);

        scan_jiffies = jiffies;

        for (uint8_t i = EC_NETDEV_MAIN; i < CONFIG_EC_MAX_NETDEVS; i++) {
            count += master->slaves_working_counter[i];
        }

        if (!count) {
            step = 1;
            goto mutex_unlock;
        }

        master->slaves = ec_osal_malloc(sizeof(ec_slave_t) * count);
        if (!master->slaves) {
            step = 2;
            goto mutex_unlock;
        }

        master->slave_count = count;
        memset(master->slaves, 0, sizeof(ec_slave_t) * count);

        slave_index = 0;
        for (uint8_t netdev_idx = EC_NETDEV_MAIN; netdev_idx < CONFIG_EC_MAX_NETDEVS; netdev_idx++) {
            autoinc_address = 0;
            for (uint32_t j = 0; j < master->slaves_working_counter[netdev_idx]; j++) {
                slave = master->slaves + slave_index;

                ec_slave_init(slave, slave_index, master, netdev_idx, (int16_t)autoinc_address * (-1), slave_index + 1001);

                slave_index++;
                autoinc_address++;
            }
        }

        for (uint8_t netdev_idx = EC_NETDEV_MAIN; netdev_idx < CONFIG_EC_MAX_NETDEVS; netdev_idx++) {
            if (master->slaves_working_counter[netdev_idx] == 0) {
                continue;
            }
            // Clear station address
            ec_datagram_bwr(datagram, ESCREG_OF(ESCREG->STATION_ADDR), 2);
            ec_datagram_zero(datagram);
            datagram->netdev_idx = netdev_idx;
            ret = ec_master_queue_ext_datagram(master, datagram, true, true);
            if (ret < 0) {
                step = 3;
                goto mutex_unlock;
            }

            // Clear receive time for dc measure delays
            ec_datagram_bwr(datagram, ESCREG_OF(ESCREG->RCV_TIME[0]), 4);
            ec_datagram_zero(datagram);
            datagram->netdev_idx = netdev_idx;
            ret = ec_master_queue_ext_datagram(master, datagram, true, true);
            if (ret < 0) {
                step = 4;
                goto mutex_unlock;
            }

            ref_time[netdev_idx] = datagram->jiffies_sent;
        }

        for (uint32_t slave_index = 0; slave_index < master->slave_count; slave_index++) {
            slave = master->slaves + slave_index;

            EC_SLAVE_LOG_INFO("Scanning slave %u on %s\n", slave->index, master->netdev[slave->netdev_idx]->name);

            // Set station address
            ec_datagram_apwr(datagram, slave->autoinc_address, ESCREG_OF(ESCREG->STATION_ADDR), 2);
            EC_WRITE_U16(datagram->data, slave->station_address);
            datagram->netdev_idx = slave->netdev_idx;
            ret = ec_master_queue_ext_datagram(master, datagram, true, true);
            if (ret < 0) {
                step = 5;
                goto mutex_unlock;
            }

            // Read AL state
            ec_datagram_fprd(datagram, slave->station_address, ESCREG_OF(ESCREG->AL_STAT), 2);
            ec_datagram_zero(datagram);
            datagram->netdev_idx = slave->netdev_idx;
            ret = ec_master_queue_ext_datagram(master, datagram, true, true);
            if (ret < 0) {
                step = 6;
                goto mutex_unlock;
            }

            // Read base information
            ec_datagram_fprd(datagram, slave->station_address, ESCREG_OF(ESCREG->TYPE), 12);
            ec_datagram_zero(datagram);
            datagram->netdev_idx = slave->netdev_idx;
            ret = ec_master_queue_ext_datagram(master, datagram, true, true);
            if (ret < 0) {
                step = 7;
                goto mutex_unlock;
            }

            slave->base_type = EC_READ_U8(datagram->data);
            slave->base_revision = EC_READ_U8(datagram->data + 1);
            slave->base_build = EC_READ_U16(datagram->data + 2);

            slave->base_fmmu_count = EC_READ_U8(datagram->data + 4);
            EC_ASSERT_MSG(slave->base_fmmu_count <= EC_MAX_FMMUS,
                          "Slave %u FMMU count %u is overflow",
                          slave->index, slave->base_fmmu_count);

            slave->base_sync_count = EC_READ_U8(datagram->data + 5);
            EC_ASSERT_MSG(slave->base_sync_count <= EC_MAX_SYNC_MANAGERS,
                          "Slave %u sync managers count %u is overflow",
                          slave->index, slave->base_fmmu_count);

            uint8_t data = EC_READ_U8(datagram->data + 7);
            for (uint8_t i = 0; i < EC_MAX_PORTS; i++) {
                slave->ports[i].desc = (data >> (2 * i)) & 0x03;
            }

            data = EC_READ_U8(datagram->data + 8);
            slave->base_fmmu_bit_operation = data & 0x01;
            slave->base_dc_supported = (data >> 2) & 0x01;
            slave->base_dc_range = ((data >> 3) & 0x01) ? EC_DC_64 : EC_DC_32;

            if (slave->base_dc_supported) {
                // Read DC capabilities
                ec_datagram_fprd(datagram, slave->station_address, ESCREG_OF(ESCREG->SYS_TIME),
                                 slave->base_dc_range == EC_DC_64 ? 8 : 4);
                ec_datagram_zero(datagram);
                datagram->netdev_idx = slave->netdev_idx;
                ret = ec_master_queue_ext_datagram(master, datagram, true, true);
                if (ret < 0) {
                    step = 8;
                    goto mutex_unlock;
                }

                // Read DC port receive times
                ec_datagram_fprd(datagram, slave->station_address, ESCREG_OF(ESCREG->RCV_TIME[0]), 16);
                ec_datagram_zero(datagram);
                datagram->netdev_idx = slave->netdev_idx;
                ret = ec_master_queue_ext_datagram(master, datagram, true, true);
                if (ret < 0) {
                    step = 9;
                    goto mutex_unlock;
                }

                for (uint8_t i = 0; i < EC_MAX_PORTS; i++) {
                    slave->ports[i].receive_time = EC_READ_U32(datagram->data + 4 * i);
                }

                ec_datagram_fprd(datagram, slave->station_address, ESCREG_OF(ESCREG->RCVT_ECAT_PU), 8);
                ec_datagram_zero(datagram);
                datagram->netdev_idx = slave->netdev_idx;
                ret = ec_master_queue_ext_datagram(master, datagram, true, true);
                if (ret < 0) {
                    step = 9;
                    goto mutex_unlock;
                }

                slave->system_time_offset = ref_time[netdev_idx] - EC_READ_U64(datagram->data);
            } else {
            }

            // Read data link status
            ec_datagram_fprd(datagram, slave->station_address, ESCREG_OF(ESCREG->ESC_DL_STAT), 2);
            ec_datagram_zero(datagram);
            datagram->netdev_idx = slave->netdev_idx;
            ret = ec_master_queue_ext_datagram(master, datagram, true, true);
            if (ret < 0) {
                step = 10;
                goto mutex_unlock;
            }

            uint16_t dl_status = EC_READ_U16(datagram->data);
            for (uint8_t i = 0; i < EC_MAX_PORTS; i++) {
                slave->ports[i].link.link_up =
                    dl_status & (1 << (4 + i)) ? 1 : 0;
                slave->ports[i].link.loop_closed =
                    dl_status & (1 << (8 + i * 2)) ? 1 : 0;
                slave->ports[i].link.signal_detected =
                    dl_status & (1 << (9 + i * 2)) ? 1 : 0;
            }

            uint16_t sii_offset = EC_FIRST_SII_CATEGORY_OFFSET;
            uint16_t cat_type, cat_size;
            uint32_t sii_data;
            uint16_t *cat_data;

            // Read SII category headers to determine full SII size
            do {
                ret = ec_sii_read(master, slave_index, datagram, sii_offset, &sii_data, 4);
                if (ret < 0) {
                    step = 11;
                    goto mutex_unlock;
                }

                cat_type = sii_data & 0xFFFF;
                cat_size = (sii_data >> 16) & 0xFFFF;

                sii_offset += 2 + cat_size;
                EC_SLAVE_LOG_DBG("Found category type 0x%04x with size 0x%04x, next offset 0x%04x\n",
                                 cat_type, cat_size * 2, sii_offset);
            } while (cat_type != 0xFFFF && (sii_offset < EC_MAX_SII_SIZE));

            slave->sii_nwords = EC_ALIGN_UP(sii_offset + 1, 2);

            slave->sii_image = ec_osal_malloc(slave->sii_nwords * 2);
            if (!slave->sii_image) {
                step = 12;
                goto mutex_unlock;
            }
            memset(slave->sii_image, 0, slave->sii_nwords * 2);

            // Read full SII and parse it
            ret = ec_sii_read(master, slave_index, datagram, 0x0000, (uint32_t *)slave->sii_image, slave->sii_nwords * 2);
            if (ret < 0) {
                step = 13;
                goto mutex_unlock;
            }

            slave->sii.aliasaddr =
                EC_READ_U16(slave->sii_image + 0x0004);
            slave->alias_address = slave->sii.aliasaddr;
            slave->sii.vendor_id =
                EC_READ_U32(slave->sii_image + 0x0008);
            slave->sii.product_code =
                EC_READ_U32(slave->sii_image + 0x000A);
            slave->sii.revision_number =
                EC_READ_U32(slave->sii_image + 0x000C);
            slave->sii.serial_number =
                EC_READ_U32(slave->sii_image + 0x000E);
            slave->sii.boot_rx_mailbox_offset =
                EC_READ_U16(slave->sii_image + 0x0014);
            slave->sii.boot_rx_mailbox_size =
                EC_READ_U16(slave->sii_image + 0x0015);
            slave->sii.boot_tx_mailbox_offset =
                EC_READ_U16(slave->sii_image + 0x0016);
            slave->sii.boot_tx_mailbox_size =
                EC_READ_U16(slave->sii_image + 0x0017);
            slave->sii.std_rx_mailbox_offset =
                EC_READ_U16(slave->sii_image + 0x0018);
            slave->sii.std_rx_mailbox_size =
                EC_READ_U16(slave->sii_image + 0x0019);
            slave->sii.std_tx_mailbox_offset =
                EC_READ_U16(slave->sii_image + 0x001A);
            slave->sii.std_tx_mailbox_size =
                EC_READ_U16(slave->sii_image + 0x001B);
            slave->sii.mailbox_protocols =
                EC_READ_U16(slave->sii_image + 0x001C);

            EC_SLAVE_LOG_INFO("Slave %u mbxprot support: %s\n", slave->index, ec_mbox_protocol_string(slave->sii.mailbox_protocols));

            cat_data = slave->sii_image + EC_FIRST_SII_CATEGORY_OFFSET;

            while (EC_READ_U16(cat_data) != 0xFFFF) {
                cat_type = EC_READ_U16(cat_data);     // category type
                cat_size = EC_READ_U16(cat_data + 1); // category size
                cat_data += 2;

                EC_SLAVE_LOG_DBG("Parsing category type 0x%04x with size 0x%04x\n",
                                 cat_type, cat_size * 2);

                switch (cat_type) {
                    case EC_SII_TYPE_STRINGS:
                        ret = ec_slave_fetch_sii_strings(slave, (uint8_t *)cat_data, cat_size * 2);
                        if (ret < 0) {
                            step = 14;
                            goto mutex_unlock;
                        }
                        break;
                    case EC_SII_TYPE_GENERAL:
                        slave->sii.has_general = true;
                        ec_memcpy(&slave->sii.general, cat_data, sizeof(ec_sii_general_t));
                        break;
                    case EC_SII_TYPE_FMMU:
                        break;
                    case EC_SII_TYPE_SM:
                        slave->sm_count = (cat_size * 2) / sizeof(ec_sii_sm_t);

                        slave->sm_info = ec_osal_malloc(slave->sm_count * sizeof(ec_sm_info_t));
                        if (!slave->sm_info) {
                            step = 15;
                            goto mutex_unlock;
                        }
                        memset(slave->sm_info, 0, slave->sm_count * sizeof(ec_sm_info_t));

                        for (uint8_t i = 0; i < slave->sm_count; i++) {
                            ec_sii_sm_t *sm = (ec_sii_sm_t *)((uint8_t *)cat_data + i * sizeof(ec_sii_sm_t));

                            slave->sm_info[i].physical_start_address = sm->physical_start_address;
                            slave->sm_info[i].length = sm->length;
                            slave->sm_info[i].control = sm->control;
                            slave->sm_info[i].enable = sm->active;
                        }
                        break;
                    case EC_SII_TYPE_TXPDO:
                        break;
                    case EC_SII_TYPE_RXPDO:
                        break;
                    case EC_SII_TYPE_DC:
                        break;
                    default:
                        EC_SLAVE_LOG_WRN("Unknown SII category type 0x%04x\n", cat_type);
                        break;
                }

                cat_data += cat_size;
            }

            EC_SLAVE_LOG_INFO("Slave %u parse eeprom success\n", slave->index);

            ret = ec_slave_config(slave);
            if (ret < 0) {
                step = 16;
                goto mutex_unlock;
            }
        }

        EC_LOG_INFO("Bus scanning completed in %u ms\n", (unsigned int)((jiffies - scan_jiffies) / 1000000));
        master->scan_done = true;

        ec_master_calc_dc(master);

    mutex_unlock:
        ec_osal_mutex_give(master->scan_lock);
        if (step != 0) {
            EC_LOG_ERR("Bus scanning failed at step %u, errorcode: %d", step, ret);
        }
    }

    if (master->slave_count && master->scan_done) {
        ec_master_scan_slaves_state(master);
    }
}

char *ec_slave_get_sii_string(const ec_slave_t *slave, uint32_t index)
{
    if (!index--)
        return NULL;

    if (index >= slave->sii.string_count) {
        EC_SLAVE_LOG_ERR("Slave %u string %u not found\n", slave->index, index);
        return NULL;
    }

    return slave->sii.strings[index];
}