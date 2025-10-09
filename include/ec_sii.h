/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_SII_H
#define EC_SII_H

typedef struct ec_slave ec_slave_t;

typedef struct ec_sii {
    // Non-category data
    uint16_t aliasaddr;              /**< Configured station alias. */
    uint32_t vendor_id;              /**< Vendor ID. */
    uint32_t product_code;           /**< Vendor-specific product code. */
    uint32_t revision_number;        /**< Revision number. */
    uint32_t serial_number;          /**< Serial number. */
    uint16_t boot_rx_mailbox_offset; /**< Bootstrap receive mailbox address. */
    uint16_t boot_rx_mailbox_size;   /**< Bootstrap receive mailbox size. */
    uint16_t boot_tx_mailbox_offset; /**< Bootstrap transmit mailbox address. */
    uint16_t boot_tx_mailbox_size;   /**< Bootstrap transmit mailbox size. */
    uint16_t std_rx_mailbox_offset;  /**< Standard receive mailbox address. */
    uint16_t std_rx_mailbox_size;    /**< Standard receive mailbox size. */
    uint16_t std_tx_mailbox_offset;  /**< Standard transmit mailbox address. */
    uint16_t std_tx_mailbox_size;    /**< Standard transmit mailbox size. */
    uint16_t mailbox_protocols;      /**< Supported mailbox protocols. */

    // General
    ec_sii_general_t general;
    bool has_general;

    // Strings
    char **strings;        /**< Strings in SII categories. */
    uint32_t string_count; /**< Number of SII strings. */
} ec_sii_t;

int ec_sii_read(ec_master_t *master, uint16_t slave_index, ec_datagram_t *datagram, uint16_t woffset, uint32_t *buf, uint32_t len);
int ec_sii_write(ec_master_t *master, uint16_t slave_index, ec_datagram_t *datagram, uint16_t woffset, const uint16_t *buf, uint32_t len);

#endif