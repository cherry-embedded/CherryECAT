/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_MASTER_H
#define EC_MASTER_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "ec_config.h"
#include "ec_util.h"
#include "ec_list.h"
#include "ec_errno.h"
#include "ec_log.h"
#include "esc_register.h"
#include "ec_def.h"
#include "ec_osal.h"
#include "ec_port.h"
#include "ec_timestamp.h"
#include "ec_version.h"
#include "ec_datagram.h"
#include "ec_common.h"
#include "ec_sii.h"
#include "ec_slave.h"
#include "ec_mailbox.h"
#include "ec_coe.h"
#include "ec_foe.h"
#include "ec_eoe.h"
#include "ec_cmd.h"

typedef struct {
    unsigned int timeouts;
    unsigned int corrupted;
    unsigned int unmatched;
} ec_master_stats_t;

typedef enum {
    EC_UNKNOWN,  /**< Unknown phase. No Ethernet device is attached. */
    EC_IDLE,     /**< Idle phase. An Ethernet device is attached, but the master is not in use, yet. */
    EC_OPERATION /**< Operation phase. The master was requested by a realtime application. */
} ec_master_phase_t;

typedef struct ec_master {
    uint8_t index;
    ec_netdev_t *netdev[CONFIG_EC_MAX_NETDEVS];
    bool link_state[CONFIG_EC_MAX_NETDEVS];
    uint32_t slaves_working_counter[CONFIG_EC_MAX_NETDEVS];
    ec_slave_state_t slaves_state[CONFIG_EC_MAX_NETDEVS];

    bool started;        /**< Master is started. */
    bool scan_done;      /**< Slave scan is done. */
    bool rescan_request; /**< Rescan requested. */
    ec_master_stats_t stats;
    ec_master_phase_t phase;

    ec_datagram_t main_datagram; /**< Main datagram for slave scan & state change & config & sii */
    ec_datagram_t pdo_datagram;  /**< pdo datagram */

    ec_dlist_t datagram_queue; /**< Queue of pending datagrams*/
    uint8_t datagram_index;

    ec_slave_t *dc_ref_clock;           /**< DC reference clock slave. */
    ec_datagram_t dc_ref_sync_datagram; /**< Datagram used for synchronizing the reference clock to the master clock. */
    ec_datagram_t dc_all_sync_datagram; /**< Datagram used for synchronizing all slaves to the dc ref clock. */

    bool dc_sync_with_dc_ref_enable; /**< true: Sync the reference clock by dc ref clock, false: by master */
    uint32_t cycle_time;             /**< Cycle time [ns]. */
    int32_t shift_time;              /**< Shift time [ns]. */
    int64_t dc_sync_integral;        /**< DC integral value. */

    uint64_t interval;

    ec_slave_t *slaves;
    uint32_t slave_count;

    bool perf_enable;
    uint64_t last_start_time;
    uint32_t min_period_ns;
    uint32_t max_period_ns;
    uint64_t total_period_ns;
    uint64_t period_count;
    uint32_t min_send_exec_ns;
    uint32_t max_send_exec_ns;
    uint64_t total_send_exec_ns;
    uint64_t send_exec_count;
    uint32_t min_recv_exec_ns;
    uint32_t max_recv_exec_ns;
    uint64_t total_recv_exec_ns;
    uint64_t recv_exec_count;
    int32_t min_offset_ns;
    int32_t max_offset_ns;

    ec_osal_mutex_t scan_lock;
    ec_osal_thread_t scan_thread;
    ec_osal_thread_t nonperiod_thread;
    ec_osal_sem_t nonperiod_sem;
    struct ec_osal_timer *linkdetect_timer;
    bool nonperiod_suspend;

    uint8_t pdo_buffer[CONFIG_EC_MAX_NETDEVS][CONFIG_EC_MAX_PDO_BUFSIZE];
    uint32_t actual_pdo_size;          /**< Actual PDO size for current setting. */
    uint32_t expected_working_counter; /**< Expected working counter for PDO datagrams. */
    uint32_t actual_working_counter;   /**< Actual working counter for PDO datagrams. */
} ec_master_t;

int ec_master_init(ec_master_t *master, uint8_t master_index);
void ec_master_deinit(ec_master_t *master);
int ec_master_start(ec_master_t *master);
int ec_master_stop(ec_master_t *master);
int ec_master_queue_ext_datagram(ec_master_t *master, ec_datagram_t *datagram, bool wakep_poll, bool waiter);
uint8_t *ec_master_get_slave_domain(ec_master_t *master, uint32_t slave_index);
uint8_t *ec_master_get_slave_domain_output(ec_master_t *master, uint32_t slave_index);
uint8_t *ec_master_get_slave_domain_input(ec_master_t *master, uint32_t slave_index);
uint32_t ec_master_get_slave_domain_size(ec_master_t *master, uint32_t slave_index);
uint32_t ec_master_get_slave_domain_osize(ec_master_t *master, uint32_t slave_index);
uint32_t ec_master_get_slave_domain_isize(ec_master_t *master, uint32_t slave_index);

int ec_master_find_slave_sync_info(uint32_t vendor_id,
                                   uint32_t product_code,
                                   uint32_t revision_number,
                                   uint8_t cia402_mode,
                                   ec_sync_info_t **syncs,
                                   uint8_t *sync_count);
#endif