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
#include "ec_perf.h"
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

/** Netdev statistics.
 */
typedef struct {
    uint64_t tx_count;                     /**< Number of frames sent. */
    uint64_t last_tx_count;                /**< Number of frames sent of last statistics cycle. */
    uint64_t rx_count;                     /**< Number of frames received. */
    uint64_t last_rx_count;                /**< Number of frames received of last statistics cycle. */
    uint64_t tx_bytes;                     /**< Number of bytes sent. */
    uint64_t last_tx_bytes;                /**< Number of bytes sent of last statistics cycle. */
    uint64_t rx_bytes;                     /**< Number of bytes received. */
    uint64_t last_rx_bytes;                /**< Number of bytes received of last statistics cycle. */
    uint64_t last_loss;                    /**< Tx/Rx difference of last statistics cycle. */
    int32_t tx_frame_rates[EC_RATE_COUNT]; /**< Transmit rates in frames/s for different statistics cycle periods.*/
    int32_t rx_frame_rates[EC_RATE_COUNT]; /**< Receive rates in frames/s for different statistics cycle periods.*/
    int32_t tx_byte_rates[EC_RATE_COUNT];  /**< Transmit rates in byte/s for different statistics cycle periods. */
    int32_t rx_byte_rates[EC_RATE_COUNT];  /**< Receive rates in byte/s for different statistics cycle periods. */
    int32_t loss_rates[EC_RATE_COUNT];     /**< Frame loss rates for different statistics cycle periods. */
    uint64_t last_jiffies;                 /**< Jiffies of last statistic cycle. */
} ec_netdev_stats_t;

/** Cyclic statistics.
 */
typedef struct {
    unsigned int timeouts;        /**< datagram timeouts */
    unsigned int corrupted;       /**< corrupted frames */
    unsigned int unmatched;       /**< unmatched datagrams (received, but not queued any longer) */
    unsigned long output_jiffies; /**< time of last output */
} ec_stats_t;

typedef enum {
    EC_ORPHANED, /**< Orphaned phase. The master has no Ethernet device attached. */
    EC_IDLE,     /**< Idle phase. An Ethernet device is attached, but the master is not in use, yet. */
    EC_OPERATION /**< Operation phase. The master was requested by a realtime application. */
} ec_master_phase_t;

typedef struct {
    ec_dlist_t queue;
    ec_datagram_t datagrams[CONFIG_EC_MAX_NETDEVS];
#if CONFIG_EC_MAX_NETDEVS > 1
    uint8_t *send_buffer;
#endif
    uint32_t expected_working_counter;
    ec_slave_t *slave;
} ec_pdo_datagram_t;

typedef struct ec_master {
    uint8_t index;
    ec_netdev_t *netdev[CONFIG_EC_MAX_NETDEVS];
    bool link_state[CONFIG_EC_MAX_NETDEVS];
    uint32_t slaves_responding[CONFIG_EC_MAX_NETDEVS];
    ec_slave_state_t slaves_state[CONFIG_EC_MAX_NETDEVS];
    ec_netdev_stats_t netdev_stats;
    ec_stats_t stats;
    ec_master_phase_t phase;
    bool active;         /**< Master is started. */
    bool scan_done;      /**< Slave scan is done. */
    bool rescan_request; /**< Rescan requested. */

    ec_datagram_t main_datagram; /**< Main datagram for slave scan & state change & config & sii */

    ec_dlist_t datagram_queue;     /**< Queue of pending datagrams*/
    ec_dlist_t pdo_datagram_queue; /**< Queue of pdo datagrams*/
    uint8_t datagram_index;

    ec_slave_t *dc_ref_clock;                /**< DC reference clock slave. */
    ec_datagram_t dc_ref_sync_datagram;      /**< Datagram used for synchronizing the reference clock to the master clock. */
    ec_datagram_t dc_all_sync_datagram;      /**< Datagram used for synchronizing all slaves to the dc ref clock. */
    ec_datagram_t systime_diff_mon_datagram; /**< Datagram used for reading the system time difference between master and reference clock. */

    uint32_t min_systime_diff;
    uint32_t max_systime_diff;
    uint32_t curr_systime_diff;
    uint32_t systime_diff_count;
    uint64_t total_systime_diff;
    bool systime_diff_enable;

    uint64_t interval;

    ec_slave_t *slaves;
    uint32_t slave_count;

#ifdef CONFIG_EC_PERF_ENABLE
    ec_perf_t perf;
#endif

    ec_osal_mutex_t scan_lock;
    ec_osal_thread_t scan_thread;
    ec_osal_thread_t nonperiod_thread;
    ec_osal_sem_t nonperiod_sem;
    struct ec_osal_timer *linkdetect_timer;
    bool nonperiod_suspend;

    uint8_t pdo_buffer[CONFIG_EC_MAX_NETDEVS][CONFIG_EC_MAX_PDO_BUFSIZE];
    uint32_t actual_pdo_size;
    uint32_t expected_working_counter;
    uint32_t actual_working_counter;
} ec_master_t;

int ec_master_init(ec_master_t *master, uint8_t master_index);
void ec_master_deinit(ec_master_t *master);
int ec_master_start(ec_master_t *master, uint32_t period_us);
int ec_master_stop(ec_master_t *master);
int ec_master_queue_ext_datagram(ec_master_t *master, ec_datagram_t *datagram, bool wakep_poll, bool waiter);
uint8_t *ec_master_get_slave_domain(ec_master_t *master, uint32_t slave_index);
uint8_t *ec_master_get_slave_domain_output(ec_master_t *master, uint32_t slave_index);
uint8_t *ec_master_get_slave_domain_input(ec_master_t *master, uint32_t slave_index);
uint32_t ec_master_get_slave_domain_size(ec_master_t *master, uint32_t slave_index);
uint32_t ec_master_get_slave_domain_osize(ec_master_t *master, uint32_t slave_index);
uint32_t ec_master_get_slave_domain_isize(ec_master_t *master, uint32_t slave_index);

#endif