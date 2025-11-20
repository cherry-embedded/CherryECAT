/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_SLAVE_H
#define EC_SLAVE_H

typedef struct ec_master ec_master_t;
typedef struct ec_slave ec_slave_t;

typedef void (*ec_pdo_callback_t)(ec_slave_t *slave, uint8_t *output, uint8_t *input);

typedef struct
{
    ec_direction_t dir;
    uint32_t logical_start_address;
    uint32_t data_size;
} ec_fmmu_info_t;

typedef struct {
    uint16_t physical_start_address;
    uint16_t length;
    uint8_t control;
    uint8_t enable;
    ec_pdo_assign_t pdo_assign;
    ec_pdo_mapping_t pdo_mapping[CONFIG_EC_PER_SM_MAX_PDOS];
    ec_fmmu_info_t fmmu;
    bool fmmu_enable;
} ec_sm_info_t;

typedef struct {
    ec_sync_info_t *sync;                           /**< Sync manager configuration. */
    uint8_t sync_count;                             /**< Number of sync managers. */
    ec_pdo_callback_t pdo_callback;                 /**< PDO process data callback. */
    uint16_t dc_assign_activate;                    /**< dc assign control */
    ec_sync_signal_t dc_sync[EC_SYNC_SIGNAL_COUNT]; /**< DC sync signals. */
} ec_slave_config_t;

/** EtherCAT slave port descriptor.
 */
typedef enum {
    EC_PORT_NOT_IMPLEMENTED, /**< Port is not implemented. */
    EC_PORT_NOT_CONFIGURED,  /**< Port is not configured. */
    EC_PORT_EBUS,            /**< Port is an E-Bus. */
    EC_PORT_MII              /**< Port is a MII. */
} ec_slave_port_desc_t;

/** EtherCAT slave port information.
 */
typedef struct {
    uint8_t link_up;         /**< Link detected. */
    uint8_t loop_closed;     /**< Loop closed. */
    uint8_t signal_detected; /**< Detected signal on RX port. */
} ec_slave_port_link_t;

typedef struct {
    ec_slave_port_desc_t desc; /**< Port descriptors. */
    ec_slave_port_link_t link; /**< Port link status. */
    ec_slave_t *next_slave;    /**< Connected slaves. */
    uint32_t receive_time;     /**< Port receive times for delay measurement. */
    uint32_t delay_to_next_dc; /**< Delay to next slave with DC support behind this port [ns]. */
} ec_slave_port_t;

typedef struct ec_slave {
    uint32_t index;               /**< Index of the slave in the master slave array. */
    ec_master_t *master;          /**< Master owning the slave. */
    ec_netdev_index_t netdev_idx; /**< Index of device the slave responds on. */

    uint16_t autoinc_address; /**< Auto-increment address. */
    uint16_t station_address; /**< Configured station address. */
    uint16_t effective_alias; /**< Effective alias address. */

    ec_slave_port_t ports[EC_MAX_PORTS]; /**< Port information. */

    ec_slave_state_t requested_state; /**< Requested application state. */
    ec_slave_state_t current_state;   /**< Current application state. */
    uint32_t alstatus_code;           /**< Error code in AL Status register. */
    bool force_update;                /**< Force update of the slave. */

    uint16_t configured_rx_mailbox_offset; /**< Configured receive mailbox offset. */
    uint16_t configured_rx_mailbox_size;   /**< Configured receive mailbox size.*/
    uint16_t configured_tx_mailbox_offset; /**< Configured send mailbox offset. */
    uint16_t configured_tx_mailbox_size;   /**< Configured send mailbox size. */

    uint8_t base_type;                 /**< Slave type. */
    uint8_t base_revision;             /**< Revision. */
    uint16_t base_build;               /**< Build number. */
    uint8_t base_fmmu_count;           /**< Number of supported FMMUs. */
    uint8_t base_sync_count;           /**< Number of supported sync managers. */
    uint8_t base_fmmu_bit_operation;   /**< FMMU bit operation is supported. */
    uint8_t base_dc_supported;         /**< Distributed clocks are supported. */
    ec_slave_dc_range_t base_dc_range; /**< DC range. */
    uint8_t has_dc_system_time;        /**< The slave supports the DC system time register. Otherwise it can only be used for delay measurement. */
    uint32_t transmission_delay;       /**< DC system time transmission delay (offset from reference clock). */

    uint32_t logical_start_address;
    uint32_t odata_size;
    uint32_t idata_size;
    uint32_t expected_working_counter;
    uint32_t actual_working_counter;

    uint16_t *sii_image; /**< Complete SII image. */
    size_t sii_nwords;   /**< Size of the SII contents in words. */

    ec_sii_t sii; /**< Extracted SII data. */

    ec_sm_info_t *sm_info;
    uint8_t sm_count; /**< Number of sync managers. */

    ec_slave_config_t *config; /**< Slave custom configuration. */
} ec_slave_t;

void ec_slaves_scanning(ec_master_t *master);
char *ec_slave_get_sii_string(const ec_slave_t *slave, uint32_t index);

#endif