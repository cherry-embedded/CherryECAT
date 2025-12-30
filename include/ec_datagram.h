/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_DATAGRAM_H
#define EC_DATAGRAM_H

/** EtherCAT datagram type.
 */
typedef enum {
    EC_DATAGRAM_NONE = 0x00, /**< Dummy. */
    EC_DATAGRAM_APRD = 0x01, /**< Auto Increment Physical Read. */
    EC_DATAGRAM_APWR = 0x02, /**< Auto Increment Physical Write. */
    EC_DATAGRAM_APRW = 0x03, /**< Auto Increment Physical ReadWrite. */
    EC_DATAGRAM_FPRD = 0x04, /**< Configured Address Physical Read. */
    EC_DATAGRAM_FPWR = 0x05, /**< Configured Address Physical Write. */
    EC_DATAGRAM_FPRW = 0x06, /**< Configured Address Physical ReadWrite. */
    EC_DATAGRAM_BRD = 0x07,  /**< Broadcast Read. */
    EC_DATAGRAM_BWR = 0x08,  /**< Broadcast Write. */
    EC_DATAGRAM_BRW = 0x09,  /**< Broadcast ReadWrite. */
    EC_DATAGRAM_LRD = 0x0A,  /**< Logical Read. */
    EC_DATAGRAM_LWR = 0x0B,  /**< Logical Write. */
    EC_DATAGRAM_LRW = 0x0C,  /**< Logical ReadWrite. */
    EC_DATAGRAM_ARMW = 0x0D, /**< Auto Increment Physical Read Multiple
                               Write.  */
    EC_DATAGRAM_FRMW = 0x0E, /**< Configured Address Physical Read Multiple
                               Write. */
} ec_datagram_type_t;

/** EtherCAT datagram state.
 */
typedef enum {
    EC_DATAGRAM_INIT,      /**< Initial state of a new datagram. */
    EC_DATAGRAM_QUEUED,    /**< Queued for sending. */
    EC_DATAGRAM_SENT,      /**< Sent (still in the queue). */
    EC_DATAGRAM_RECEIVED,  /**< Received (dequeued). */
    EC_DATAGRAM_TIMED_OUT, /**< Timed out (dequeued). */
    EC_DATAGRAM_ERROR      /**< Error while sending/receiving (dequeued). */
} ec_datagram_state_t;

/** EtherCAT datagram.
 */
typedef struct {
    ec_dlist_t queue;
    ec_dlist_t ext_queue;
    ec_dlist_t sent;
    uint8_t netdev_idx;               /**< Netdev via which the datagram shall be / was sent. */
    ec_datagram_type_t type;          /**< Datagram type (APRD, BWR, etc.). */
    bool static_alloc;                /**< True, if \a data is statically allocated. */
    uint8_t address[EC_ADDR_LEN];     /**< Recipient address. */
    uint8_t *data;                    /**< Datagram payload. */
    size_t mem_size;                  /**< Datagram \a data memory size. */
    size_t data_size;                 /**< Size of the data in \a data. */
    uint8_t index;                    /**< Index (set by master). */
    uint16_t working_counter;         /**< Working counter. */
    ec_datagram_state_t state;        /**< State. */
    uint32_t lrw_read_offset;         /**< Read Offset in LRW datagram. */
    uint32_t lrw_read_size;           /**< Read Size in LRW datagram. */
    uint64_t jiffies_sent;            /**< Jiffies, when the datagram was sent. */
    uint64_t jiffies_received;        /**< Jiffies, when the datagram was received. */
    char name[EC_DATAGRAM_NAME_SIZE]; /**< Description of the datagram. */
    bool waiter;                      /**< True, if someone is waiting for the datagram. */
    ec_osal_sem_t wait;               /**< Semaphore for waiting. */
} ec_datagram_t;

void ec_datagram_init(ec_datagram_t *datagram, size_t mem_size);
void ec_datagram_init_static(ec_datagram_t *datagram, uint8_t *data, size_t mem_size);
void ec_datagram_clear(ec_datagram_t *datagram);
void ec_datagram_unqueue(ec_datagram_t *datagram);
void ec_datagram_zero(ec_datagram_t *datagram);
void ec_datagram_fill(ec_datagram_t *datagram,
                      ec_datagram_type_t type,
                      uint16_t adp,
                      uint16_t ado,
                      uint16_t size);
void ec_datagram_aprd(ec_datagram_t *datagram, uint16_t autoinc_address, uint16_t mem_address, size_t data_size);
void ec_datagram_apwr(ec_datagram_t *datagram, uint16_t autoinc_address, uint16_t mem_address, size_t data_size);
void ec_datagram_aprw(ec_datagram_t *datagram, uint16_t autoinc_address, uint16_t mem_address, size_t data_size);
void ec_datagram_armw(ec_datagram_t *datagram, uint16_t autoinc_address, uint16_t mem_address, size_t data_size);
void ec_datagram_fprd(ec_datagram_t *datagram, uint16_t configured_address, uint16_t mem_address, size_t data_size);
void ec_datagram_fpwr(ec_datagram_t *datagram, uint16_t configured_address, uint16_t mem_address, size_t data_size);
void ec_datagram_fprw(ec_datagram_t *datagram, uint16_t configured_address, uint16_t mem_address, size_t data_size);
void ec_datagram_frmw(ec_datagram_t *datagram, uint16_t configured_address, uint16_t mem_address, size_t data_size);
void ec_datagram_brd(ec_datagram_t *datagram, uint16_t mem_address, size_t data_size);
void ec_datagram_bwr(ec_datagram_t *datagram, uint16_t mem_address, size_t data_size);
void ec_datagram_brw(ec_datagram_t *datagram, uint16_t mem_address, size_t data_size);
void ec_datagram_lrd(ec_datagram_t *datagram, uint32_t offset, size_t data_size);
void ec_datagram_lwr(ec_datagram_t *datagram, uint32_t offset, size_t data_size);
void ec_datagram_lrw(ec_datagram_t *datagram, uint32_t offset, size_t data_size);
const char *ec_datagram_type_string(const ec_datagram_t *datagram);

#endif