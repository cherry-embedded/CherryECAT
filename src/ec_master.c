/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_master.h"

#define EC_DATAGRAM_TIMEOUT_NS (50 * 1000 * 1000ULL) // 50ms

void ec_master_period_process(void *arg);

EC_FAST_CODE_SECTION void ec_master_queue_datagram(ec_master_t *master, ec_datagram_t *datagram)
{
    ec_datagram_t *queued_datagram;

    ec_dlist_for_each_entry(queued_datagram, &master->datagram_queue, queue)
    {
        if (queued_datagram == datagram) {
            datagram->state = EC_DATAGRAM_QUEUED;
            return;
        }
    }

    ec_dlist_add_tail(&master->datagram_queue, &datagram->queue);
    datagram->state = EC_DATAGRAM_QUEUED;
}

EC_FAST_CODE_SECTION void ec_master_unqueue_datagram(ec_master_t *master, ec_datagram_t *datagram)
{
    ec_dlist_del_init(&datagram->queue);

    if (datagram->waiter) {
        datagram->waiter = false;
        ec_osal_sem_give(datagram->wait);
    }
}

EC_FAST_CODE_SECTION void ec_master_send_datagrams(ec_master_t *master, uint8_t netdev_idx)
{
    ec_datagram_t *datagram, *next;
    size_t datagram_size;
    uint8_t *frame_data, *cur_data = NULL;
    void *follows_word;
    uint64_t jiffies_sent;
    unsigned int datagram_count, more_datagrams_waiting;
    ec_dlist_t sent_datagrams;

    datagram_count = 0;
    ec_dlist_init(&sent_datagrams);

    do {
        frame_data = NULL;
        follows_word = NULL;
        more_datagrams_waiting = 0;

        // fill current frame with datagrams
        ec_dlist_for_each_entry(datagram, &master->datagram_queue, queue)
        {
            if (datagram->state != EC_DATAGRAM_QUEUED ||
                datagram->netdev_idx != netdev_idx) {
                continue;
            }

            if (!frame_data) {
                // fetch pointer to transmit socket buffer
                frame_data =
                    ec_netdev_get_txbuf(master->netdev[netdev_idx]);
                cur_data = frame_data + EC_FRAME_HEADER_SIZE;
            }

            // does the current datagram fit in the frame?
            datagram_size = EC_DATAGRAM_HEADER_SIZE + datagram->data_size + EC_DATAGRAM_WC_SIZE;
            if (cur_data - frame_data + datagram_size > ETH_DATA_LEN) {
                more_datagrams_waiting = 1;
                break;
            }

            ec_dlist_add_tail(&sent_datagrams, &datagram->sent);
            datagram->index = master->datagram_index++;

            EC_LOG_DBG("Adding datagram 0x%02X\n", datagram->index);

            // set "datagram following" flag in previous datagram
            if (follows_word) {
                EC_WRITE_U16(follows_word,
                             EC_READ_U16(follows_word) | 0x8000);
            }

            // EtherCAT datagram header
            EC_WRITE_U8(cur_data, datagram->type);
            EC_WRITE_U8(cur_data + 1, datagram->index);
            ec_memcpy(cur_data + 2, datagram->address, EC_ADDR_LEN);
            EC_WRITE_U16(cur_data + 6, datagram->data_size & 0x7FF);
            EC_WRITE_U16(cur_data + 8, 0x0000); // IRQ
            follows_word = cur_data + 6;
            cur_data += EC_DATAGRAM_HEADER_SIZE;

            // EtherCAT datagram data
            ec_memcpy(cur_data, datagram->data, datagram->data_size);
            cur_data += datagram->data_size;

            // EtherCAT datagram footer
            EC_WRITE_U16(cur_data, 0x0000); // reset working counter
            cur_data += EC_DATAGRAM_WC_SIZE;
        }

        if (ec_dlist_isempty(&sent_datagrams)) {
            EC_LOG_DBG("nothing to send.\n");
            break;
        }

        // EtherCAT frame header
        EC_WRITE_U16(frame_data, ((cur_data - frame_data - EC_FRAME_HEADER_SIZE) & 0x7FF) | 0x1000);

        // pad frame
        while (cur_data - frame_data < ETH_ZLEN - ETH_HLEN)
            EC_WRITE_U8(cur_data++, 0x00);

        EC_LOG_DBG("frame size: %u\n", cur_data - frame_data);

        // send frame
        if (ec_netdev_send(master->netdev[netdev_idx], cur_data - frame_data) < 0) {
            EC_LOG_ERR("ec_netdev_send() failed.\n");
        }

        jiffies_sent = jiffies;

        // set datagram states and sending timestamps
        ec_dlist_for_each_entry_safe(datagram, next, &sent_datagrams, sent)
        {
            datagram->state = EC_DATAGRAM_SENT;
            datagram->jiffies_sent = jiffies_sent;
            ec_dlist_del_init(&datagram->sent); // empty list of sent datagrams

            datagram_count++;
        }
    } while (more_datagrams_waiting);
}

EC_FAST_CODE_SECTION void ec_master_receive_datagrams(ec_master_t *master,
                                                      uint8_t netdev_idx,
                                                      const uint8_t *frame_data,
                                                      size_t size)
{
    size_t frame_size, data_size;
    uint8_t datagram_type, datagram_index;
    uint32_t cmd_follows, matched;
    const uint8_t *cur_data;
    uint64_t jiffies_received;
    uint32_t datagram_count;
    ec_datagram_t *datagram;

    if (size < EC_FRAME_HEADER_SIZE) {
        EC_LOG_ERR("Corrupted frame received on %s (size %u < %u byte)\n",
                   master->netdev[netdev_idx]->name, size, EC_FRAME_HEADER_SIZE);
        master->stats.corrupted++;
        return;
    }

    jiffies_received = jiffies;

    cur_data = frame_data;

    // check length of entire frame
    frame_size = EC_READ_U16(cur_data) & 0x07FF;
    cur_data += EC_FRAME_HEADER_SIZE;

    if (frame_size > size) {
        EC_LOG_ERR("Corrupted frame received on %s (invalid frame size %u > received size %u)\n",
                   master->netdev[netdev_idx]->name, frame_size, size);
        master->stats.corrupted++;
        return;
    }

    datagram_count = 0;
    cmd_follows = 1;

    while (cmd_follows) {
        // process datagram header
        datagram_type = EC_READ_U8(cur_data);
        datagram_index = EC_READ_U8(cur_data + 1);
        data_size = EC_READ_U16(cur_data + 6) & 0x07FF;
        cmd_follows = EC_READ_U16(cur_data + 6) & 0x8000;
        cur_data += EC_DATAGRAM_HEADER_SIZE;

        if ((cur_data - frame_data + data_size + EC_DATAGRAM_WC_SIZE) > size) {
            EC_LOG_ERR("Corrupted frame received on %s (invalid data size %u)\n",
                       master->netdev[netdev_idx]->name, data_size);
            master->stats.corrupted++;
            return;
        }

        // search for matching datagram in the queue
        matched = 0;
        ec_dlist_for_each_entry(datagram, &master->datagram_queue, queue)
        {
            if ((datagram->index == datagram_index) &&
                (datagram->state == EC_DATAGRAM_SENT) &&
                (datagram->type == datagram_type) &&
                (datagram->data_size == data_size)) {
                matched = 1;
                break;
            }
        }

        // no matching datagram was found
        if (!matched) {
            EC_LOG_DBG("No matching datagram found for index 0x%02X, type 0x%02X, size %u on %s\n",
                       datagram_index, datagram_type, data_size,
                       master->netdev[netdev_idx]->name);
            master->stats.unmatched++;

            cur_data += data_size + EC_DATAGRAM_WC_SIZE;
            continue;
        }

        if (datagram->type != EC_DATAGRAM_APWR &&
            datagram->type != EC_DATAGRAM_FPWR &&
            datagram->type != EC_DATAGRAM_BWR &&
            datagram->type != EC_DATAGRAM_LWR) {
            ec_memcpy(datagram->data, cur_data, data_size);
        }
        cur_data += data_size;

        // set the datagram's working counter
        datagram->working_counter = EC_READ_U16(cur_data);
        cur_data += EC_DATAGRAM_WC_SIZE;

        // dequeue the received datagram
        datagram->state = EC_DATAGRAM_RECEIVED;
        datagram->jiffies_received = jiffies_received;
        ec_master_unqueue_datagram(master, datagram);

        datagram_count++;
    }
}

EC_FAST_CODE_SECTION void ec_master_send(ec_master_t *master)
{
    ec_datagram_t *datagram, *n;
    uint8_t netdev_idx;

    // update netdev statistics
    for (netdev_idx = EC_NETDEV_MAIN; netdev_idx < CONFIG_EC_MAX_NETDEVS;
         netdev_idx++) {
        ec_netdev_update_stats(master->netdev[netdev_idx]);
    }

    // dequeue all datagrams that timed out
    ec_dlist_for_each_entry_safe(datagram, n, &master->datagram_queue, queue)
    {
        if (datagram->state != EC_DATAGRAM_SENT)
            continue;

        if ((jiffies - datagram->jiffies_sent) > EC_DATAGRAM_TIMEOUT_NS) {
            datagram->state = EC_DATAGRAM_TIMED_OUT;
            ec_master_unqueue_datagram(master, datagram);
            master->stats.timeouts++;
        }
    }

    for (netdev_idx = EC_NETDEV_MAIN; netdev_idx < CONFIG_EC_MAX_NETDEVS; netdev_idx++) {
        if (!master->netdev[netdev_idx]->link_state) {
            // link is down, no datagram can be sent
            ec_dlist_for_each_entry_safe(datagram, n, &master->datagram_queue, queue)
            {
                if (datagram->netdev_idx == netdev_idx) {
                    datagram->state = EC_DATAGRAM_ERROR;
                    ec_master_unqueue_datagram(master, datagram);
                }
            }

            if (!master->netdev[netdev_idx]) {
                continue;
            }

            // clear frame statistics
            memset(&master->netdev[netdev_idx]->stats, 0, sizeof(ec_netdev_stats_t));
            continue;
        }

        // send frames
        ec_master_send_datagrams(master, netdev_idx);
    }
}

EC_FAST_CODE_SECTION void ec_master_receive(ec_master_t *master,
                                            uint8_t netdev_idx,
                                            const uint8_t *frame_data,
                                            size_t size)
{
    ec_slave_t *slave;
    uint64_t start_time;
    uint32_t exec_ns;

    start_time = ec_timestamp_get_time_ns();

    ec_master_receive_datagrams(master, netdev_idx, frame_data, size);

    if (master->phase != EC_OPERATION) {
        return;
    }

    master->actual_working_counter = 0;
#ifndef CONFIG_EC_PDO_MULTI_DOMAIN
    if (master->pdo_datagram.state == EC_DATAGRAM_RECEIVED) {
        for (uint32_t i = 0; i < master->slave_count; i++) {
            slave = &master->slaves[i];

            if (slave->config && slave->config->pdo_callback) {
                slave->config->pdo_callback(slave,
                                            (uint8_t *)&master->pdo_buffer[EC_NETDEV_MAIN][slave->logical_start_address],
                                            (uint8_t *)&master->pdo_buffer[EC_NETDEV_MAIN][slave->logical_start_address + slave->odata_size]);
            }
        }
        master->actual_working_counter = master->pdo_datagram.working_counter;
    }
#else
    for (uint32_t i = 0; i < master->slave_count; i++) {
        slave = &master->slaves[i];

        if (slave->pdo_datagram.state == EC_DATAGRAM_RECEIVED) {
            if (slave->config && slave->config->pdo_callback) {
                slave->config->pdo_callback(slave,
                                            (uint8_t *)&master->pdo_buffer[EC_NETDEV_MAIN][slave->logical_start_address],
                                            (uint8_t *)&master->pdo_buffer[EC_NETDEV_MAIN][slave->logical_start_address + slave->odata_size]);
            }
            master->actual_working_counter += slave->pdo_datagram.working_counter;
            slave->actual_working_counter = slave->pdo_datagram.working_counter;
        }
    }
#endif
    exec_ns = ec_timestamp_get_time_ns() - start_time;
    if (master->perf_enable) {
        master->min_recv_exec_ns = MIN(exec_ns, master->min_recv_exec_ns);
        master->max_recv_exec_ns = MAX(exec_ns, master->max_recv_exec_ns);
        master->total_recv_exec_ns += exec_ns;
        master->recv_exec_count++;
    }
}

static void ec_netdev_linkpoll_timer(void *argument)
{
    ec_master_t *master = (ec_master_t *)argument;
    uint8_t netdev_idx;

    for (netdev_idx = EC_NETDEV_MAIN; netdev_idx < CONFIG_EC_MAX_NETDEVS; netdev_idx++) {
        ec_netdev_poll_link_state(master->netdev[netdev_idx]);
    }
}

static int ec_master_enter_idle(ec_master_t *master)
{
    master->phase = EC_IDLE;
    master->nonperiod_suspend = false;

    ec_osal_thread_resume(master->nonperiod_thread);

    return 0;
}

static void ec_master_exit_idle(ec_master_t *master)
{
    master->nonperiod_suspend = false;
    ec_osal_thread_suspend(master->nonperiod_thread);
}

static void ec_master_nonperiod_thread(void *argument)
{
    ec_master_t *master = (ec_master_t *)argument;
    uintptr_t flags;

    while (1) {
        ec_osal_sem_take(master->nonperiod_sem, CONFIG_EC_NONPERIOD_INTERVAL_MS);
        flags = ec_osal_enter_critical_section();
        ec_master_send(master);
        ec_osal_leave_critical_section(flags);

        if (master->nonperiod_suspend) {
            ec_master_exit_idle(master);
        }
    }
}

static void ec_master_scan_thread(void *argument)
{
    ec_master_t *master = (ec_master_t *)argument;

    while (1) {
        ec_slaves_scanning(master);
        ec_osal_msleep(CONFIG_EC_SCAN_INTERVAL_MS);
    }
}

int ec_master_init(ec_master_t *master, uint8_t master_index)
{
    uint8_t netdev_idx;

    memset(master, 0, sizeof(ec_master_t));
    master->index = master_index;
    master->datagram_index = 1; // start with index 1

    ec_dlist_init(&master->datagram_queue);

    ec_timestamp_init();

    for (netdev_idx = EC_NETDEV_MAIN; netdev_idx < CONFIG_EC_MAX_NETDEVS; netdev_idx++) {
        master->netdev[netdev_idx] = ec_netdev_init(netdev_idx);
        if (!master->netdev[netdev_idx]) {
            return -1;
        }
        master->netdev[netdev_idx]->master = master;
    }

    ec_datagram_init(&master->main_datagram, EC_MAX_DATA_SIZE);
    ec_datagram_init(&master->dc_ref_sync_datagram, 8);
    ec_datagram_init(&master->dc_all_sync_datagram, 8);

    master->scan_lock = ec_osal_mutex_create();
    if (!master->scan_lock) {
        return -1;
    }

    master->nonperiod_sem = ec_osal_sem_create(CONFIG_EC_NONPERIOD_WAITERS, 0);
    if (!master->nonperiod_sem) {
        return -1;
    }

    master->nonperiod_thread = ec_osal_thread_create("ec_nonperiod", CONFIG_EC_NONPERIOD_STACKSIZE, CONFIG_EC_NONPERIOD_PRIO, ec_master_nonperiod_thread, master);
    if (!master->nonperiod_thread) {
        return -1;
    }

    master->scan_thread = ec_osal_thread_create("ec_scan", CONFIG_EC_SCAN_STACKSIZE, CONFIG_EC_SCAN_PRIO, ec_master_scan_thread, master);
    if (!master->scan_thread) {
        return -1;
    }

    master->linkdetect_timer = ec_osal_timer_create("ec_linkdetect", 1000, ec_netdev_linkpoll_timer, master, true);
    if (!master->linkdetect_timer) {
        return -1;
    }

    ec_osal_timer_start(master->linkdetect_timer);

    ec_master_enter_idle(master);

    return 0;
}

void ec_master_deinit(ec_master_t *master)
{
}

int ec_master_start(ec_master_t *master)
{
    ec_slave_t *slave;
    uint32_t bitlen;
    uint8_t sm_idx;

    EC_ASSERT_MSG(master->cycle_time >= (40 * 1000), "Cycle time %u ns is too small. Minimum is 40000 ns.\n", master->cycle_time);
    EC_ASSERT_MSG(master->cycle_time >= master->shift_time, "Shift time %u ns is larger than cycle time %u ns.\n", master->shift_time, master->cycle_time);

    if (master->started) {
        return 0;
    }

    while (!master->scan_done) {
        ec_osal_msleep(10);
    }

    ec_osal_mutex_take(master->scan_lock);

    master->actual_working_counter = 0;
    master->expected_working_counter = 0;
    master->actual_pdo_size = 0;
    master->phase = EC_OPERATION;
    master->nonperiod_suspend = true;
    master->interval = 0;
    master->dc_sync_integral = 0;

    // wait for non-periodic thread to suspend
    while (master->nonperiod_suspend) {
        ec_osal_msleep(10);
    }

    for (uint32_t slave_idx = 0; slave_idx < master->slave_count; slave_idx++) {
        slave = &master->slaves[slave_idx];

        EC_ASSERT_MSG(slave->config != NULL, "Slave %u has no configuration\n", slave_idx);

        slave->logical_start_address = master->actual_pdo_size;
        slave->odata_size = 0;
        slave->idata_size = 0;
        for (uint8_t i = 0; i < slave->config->sync_count; i++) {
            bitlen = 0;

            sm_idx = slave->config->sync[i].index;
            EC_ASSERT_MSG(sm_idx < slave->sm_count, "Slave %u: Invalid sync manager index %u\n",
                          slave_idx, sm_idx);

            slave->sm_info[sm_idx].pdo_assign.count = slave->config->sync[i].n_pdos;

            EC_ASSERT_MSG(slave->sm_info[sm_idx].pdo_assign.count <= CONFIG_EC_PER_SM_MAX_PDOS,
                          "Slave %u: Too many PDOs %u for SM %u\n",
                          slave_idx, slave->sm_info[sm_idx].pdo_assign.count, sm_idx);

            for (uint32_t j = 0; j < slave->config->sync[i].n_pdos; j++) {
                slave->sm_info[sm_idx].pdo_assign.entry[j] = slave->config->sync[i].pdos[j].index;

                slave->sm_info[sm_idx].pdo_mapping[j].count = slave->config->sync[i].pdos[j].n_entries;

                EC_ASSERT_MSG(slave->sm_info[sm_idx].pdo_mapping[j].count <= CONFIG_EC_PER_PDO_MAX_PDO_ENTRIES,
                              "Slave %u: Too many entries %u for PDO 0x%04X\n",
                              slave_idx, slave->sm_info[sm_idx].pdo_mapping[j].count,
                              slave->config->sync[i].pdos[j].index);

                for (uint32_t k = 0; k < slave->config->sync[i].pdos[j].n_entries; k++) {
                    uint32_t entry = (slave->config->sync[i].pdos[j].entries[k].index << 16) |
                                     (slave->config->sync[i].pdos[j].entries[k].subindex & 0xFF) << 8 |
                                     (slave->config->sync[i].pdos[j].entries[k].bit_length & 0xFF);
                    slave->sm_info[sm_idx].pdo_mapping[j].entry[k] = entry;

                    bitlen += slave->config->sync[i].pdos[j].entries[k].bit_length;
                }
            }

            // update SM
            slave->sm_info[sm_idx].length = (bitlen + 7) / 8;
            slave->sm_info[sm_idx].enable = true;

            // update FMMU
            slave->sm_info[sm_idx].fmmu.data_size = (bitlen + 7) / 8;
            slave->sm_info[sm_idx].fmmu.logical_start_address = master->actual_pdo_size;
            slave->sm_info[sm_idx].fmmu.dir = slave->config->sync[i].dir;
            slave->sm_info[sm_idx].fmmu_enable = true;
            master->actual_pdo_size += (bitlen + 7) / 8;

            if (slave->config->sync[i].dir == EC_DIR_INPUT) {
                slave->idata_size += (bitlen + 7) / 8;
            }
            if (slave->config->sync[i].dir == EC_DIR_OUTPUT) {
                slave->odata_size += (bitlen + 7) / 8;
            }
        }
#ifdef CONFIG_EC_PDO_MULTI_DOMAIN
        ec_datagram_init_static(&slave->pdo_datagram,
                                &master->pdo_buffer[EC_NETDEV_MAIN][slave->logical_start_address],
                                slave->odata_size + slave->idata_size);
        ec_datagram_lrw(&slave->pdo_datagram, slave->logical_start_address, slave->odata_size + slave->idata_size);
        ec_datagram_zero(&slave->pdo_datagram);
#endif
        slave->expected_working_counter = 3;
        master->expected_working_counter += slave->expected_working_counter;

        EC_SLAVE_LOG_INFO("Slave %u: Logical address 0x%08x, obyte %u, ibyte %u, expected working counter %u\n",
                          slave->index,
                          slave->logical_start_address, slave->odata_size, slave->idata_size,
                          slave->expected_working_counter);
    }
#ifndef CONFIG_EC_PDO_MULTI_DOMAIN
    ec_datagram_init_static(&master->pdo_datagram,
                            &master->pdo_buffer[EC_NETDEV_MAIN][0],
                            master->actual_pdo_size);
    ec_datagram_lrw(&master->pdo_datagram, 0, master->actual_pdo_size);
    ec_datagram_zero(&master->pdo_datagram);
#endif
    ec_htimer_start(master->cycle_time / 1000, ec_master_period_process, master);

    for (uint32_t i = 0; i < master->slave_count; i++) {
        master->slaves[i].requested_state = EC_SLAVE_STATE_OP;
        master->slaves[i].alstatus_code = 0;
        master->slaves[i].force_update = true;
    }
    master->started = true;

    ec_osal_mutex_give(master->scan_lock);
    return 0;
}

int ec_master_stop(ec_master_t *master)
{
    uint8_t netdev_idx;

    if (!master->started) {
        return 0;
    }

    ec_osal_mutex_take(master->scan_lock);
    master->started = false;

    for (uint32_t i = 0; i < master->slave_count; i++) {
        master->slaves[i].requested_state = EC_SLAVE_STATE_PREOP;
        master->slaves[i].alstatus_code = 0;
        master->slaves[i].force_update = true;
    }

    while (1) {
        for (netdev_idx = EC_NETDEV_MAIN; netdev_idx < CONFIG_EC_MAX_NETDEVS; netdev_idx++) {
            if (master->netdev[netdev_idx]->link_state == 0) {
                goto out;
            }

            if ((master->slaves_state[netdev_idx] & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_PREOP) {
                goto out;
            }
        }
        ec_osal_msleep(10);
    }

out:
    ec_htimer_stop();
#ifndef CONFIG_EC_PDO_MULTI_DOMAIN
    ec_datagram_clear(&master->pdo_datagram);
#else
    for (uint32_t i = 0; i < master->slave_count; i++) {
        ec_datagram_clear(&master->slaves[i].pdo_datagram);
    }
#endif

    ec_master_enter_idle(master);

    ec_osal_mutex_give(master->scan_lock);

    return 0;
}

int ec_master_queue_ext_datagram(ec_master_t *master, ec_datagram_t *datagram, bool wakep_poll, bool waiter)
{
    uintptr_t flags;
    int ret;

    flags = ec_osal_enter_critical_section();
    datagram->waiter = waiter;
    ec_master_queue_datagram(master, datagram);

    if (wakep_poll && master->nonperiod_sem) {
        ec_osal_sem_give(master->nonperiod_sem);
    }
    ec_osal_leave_critical_section(flags);

    if (waiter) {
        ret = ec_osal_sem_take(datagram->wait, EC_OSAL_WAITING_FOREVER);
        if (ret < 0) {
            return ret;
        }

        if (datagram->state == EC_DATAGRAM_RECEIVED) {
            if (datagram->working_counter == 0) {
                return -EC_ERR_WC;
            }
            return 0;
        } else if (datagram->state == EC_DATAGRAM_TIMED_OUT) {
            return -EC_ERR_TIMEOUT;
        } else if (datagram->state == EC_DATAGRAM_ERROR) {
            return -EC_ERR_IO;
        } else {
            return -EC_ERR_UNKNOWN;
        }
    }

    return 0;
}

uint8_t *ec_master_get_slave_domain(ec_master_t *master, uint32_t slave_index)
{
    ec_slave_t *slave;

    if (slave_index >= master->slave_count) {
        return NULL;
    }

    slave = &master->slaves[slave_index];
    if ((slave->odata_size + slave->idata_size) == 0) {
        return NULL;
    }

    return &master->pdo_buffer[EC_NETDEV_MAIN][slave->logical_start_address];
}

uint8_t *ec_master_get_slave_domain_output(ec_master_t *master, uint32_t slave_index)
{
    ec_slave_t *slave;

    if (slave_index >= master->slave_count) {
        return NULL;
    }

    slave = &master->slaves[slave_index];
    if (slave->odata_size == 0) {
        return NULL;
    }

    return &master->pdo_buffer[EC_NETDEV_MAIN][slave->logical_start_address];
}

uint8_t *ec_master_get_slave_domain_input(ec_master_t *master, uint32_t slave_index)
{
    ec_slave_t *slave;

    if (slave_index >= master->slave_count) {
        return NULL;
    }

    slave = &master->slaves[slave_index];
    if (slave->idata_size == 0) {
        return NULL;
    }

    return &master->pdo_buffer[EC_NETDEV_MAIN][slave->logical_start_address + slave->odata_size];
}

uint32_t ec_master_get_slave_domain_size(ec_master_t *master, uint32_t slave_index)
{
    ec_slave_t *slave;

    if (slave_index >= master->slave_count) {
        return 0;
    }

    slave = &master->slaves[slave_index];

    return (slave->odata_size + slave->idata_size);
}

uint32_t ec_master_get_slave_domain_osize(ec_master_t *master, uint32_t slave_index)
{
    ec_slave_t *slave;

    if (slave_index >= master->slave_count) {
        return 0;
    }

    slave = &master->slaves[slave_index];

    return slave->odata_size;
}

uint32_t ec_master_get_slave_domain_isize(ec_master_t *master, uint32_t slave_index)
{
    ec_slave_t *slave;

    if (slave_index >= master->slave_count) {
        return 0;
    }

    slave = &master->slaves[slave_index];

    return slave->idata_size;
}

EC_FAST_CODE_SECTION void ec_master_dc_sync_with_pi(ec_master_t *master, uint64_t dc_ref_time, int32_t *offsettime)
{
    int64_t delta;

    delta = (dc_ref_time - master->shift_time) % master->cycle_time;
    if (delta > (master->cycle_time / 2)) {
        delta = delta - master->cycle_time;
    }
    if (delta > 0) {
        master->dc_sync_integral++;
    }
    if (delta < 0) {
        master->dc_sync_integral--;
    }
    *offsettime = -(delta / 100) - (master->dc_sync_integral / 20); // Kp = 0.1f, Ki = 0.05f
}

EC_FAST_CODE_SECTION void ec_master_period_process(void *arg)
{
    ec_master_t *master = (ec_master_t *)arg;
    uint64_t dc_ref_systime = 0;
#ifdef CONFIG_EC_PDO_MULTI_DOMAIN
    ec_slave_t *slave;
#endif
    int32_t offsettime = 0;
    uint64_t start_time;
    uint32_t period_ns;
    uint32_t exec_ns;

    if (master->phase != EC_OPERATION) {
        return;
    }

    start_time = ec_timestamp_get_time_ns();

    if (master->dc_ref_clock) {
        if (master->dc_sync_with_dc_ref_enable) {
            if (master->dc_all_sync_datagram.state == EC_DATAGRAM_RECEIVED) {
                dc_ref_systime = EC_READ_U64(master->dc_all_sync_datagram.data) - master->dc_ref_clock->transmission_delay;

                ec_master_dc_sync_with_pi(master, dc_ref_systime, &offsettime);

                ec_htimer_update(master->cycle_time / 1000 + offsettime / 1000);
            }
        } else {
            EC_WRITE_U32(master->dc_ref_sync_datagram.data, ec_timestamp_get_time_ns() & 0xffffffff);
            if (master->dc_ref_clock->base_dc_range == EC_DC_64) {
                EC_WRITE_U32(master->dc_ref_sync_datagram.data + 4, (uint32_t)(ec_timestamp_get_time_ns() >> 32));
            }
            ec_master_queue_datagram(master, &master->dc_ref_sync_datagram);
        }
    }

    if (master->dc_ref_clock) {
        ec_datagram_zero(&master->dc_all_sync_datagram);
        ec_master_queue_datagram(master, &master->dc_all_sync_datagram);
    }

#ifndef CONFIG_EC_PDO_MULTI_DOMAIN
    ec_master_queue_datagram(master, &master->pdo_datagram);
#else
    for (uint32_t i = 0; i < master->slave_count; i++) {
        slave = &master->slaves[i];
        ec_master_queue_datagram(master, &slave->pdo_datagram);
    }
#endif
    ec_master_send(master);

    period_ns = start_time - master->last_start_time;
    exec_ns = ec_timestamp_get_time_ns() - start_time;
    master->last_start_time = start_time;

    if (master->perf_enable) {
        master->min_period_ns = MIN(period_ns, master->min_period_ns);
        master->max_period_ns = MAX(period_ns, master->max_period_ns);
        master->total_period_ns += period_ns;
        master->period_count++;

        master->min_send_exec_ns = MIN(exec_ns, master->min_send_exec_ns);
        master->max_send_exec_ns = MAX(exec_ns, master->max_send_exec_ns);
        master->total_send_exec_ns += exec_ns;
        master->send_exec_count++;

        master->min_offset_ns = MIN(offsettime, master->min_offset_ns);
        master->max_offset_ns = MAX(offsettime, master->max_offset_ns);
    }

    master->interval++;
}
