/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_DEF_H
#define EC_DEF_H

/*
 *	IEEE 802.3 Ethernet magic constants.  The frame sizes omit the preamble
 *	and FCS/CRC (frame check sequence).
 */
#define ETH_ALEN      6    /* Octets in one ethernet addr	 */
#define ETH_TLEN      2    /* Octets in ethernet type field */
#define ETH_HLEN      14   /* Total octets in header.	 */
#define ETH_ZLEN      60   /* Min. octets in frame sans FCS */
#define ETH_DATA_LEN  1500 /* Max. octets in payload	 */
#define ETH_FRAME_LEN 1514 /* Max. octets in frame sans FCS */
#define ETH_FCS_LEN   4    /* Octets in the FCS		 */

#define ETH_MIN_MTU 68      /* Min IPv4 MTU per RFC791	*/
#define ETH_MAX_MTU 0xFFFFU /* 65535, same as IP_MAX_MTU	*/

/** Datagram timeout in microseconds. */
#define EC_IO_TIMEOUT 500

/** Time to send a byte in nanoseconds.
 *
 * t_ns = 1 / (100 MBit/s / 8 bit/byte) = 80 ns/byte
 */
#define EC_BYTE_TRANSMISSION_TIME_NS 80

/** Minimum size of a buffer used with ec_state_string(). */
#define EC_STATE_STRING_SIZE 32

/** Maximum SII size in words, to avoid infinite reading. */
#define EC_MAX_SII_SIZE 4096

/** Number of statistic rate intervals to maintain. */
#define EC_RATE_COUNT 3

/*****************************************************************************
 * EtherCAT protocol
 ****************************************************************************/

/** Size of an EtherCAT frame header. */
#define EC_FRAME_HEADER_SIZE 2

/** Size of an EtherCAT datagram header. */
#define EC_DATAGRAM_HEADER_SIZE 10

/** Size of an EtherCAT datagram workcounter. */
#define EC_DATAGRAM_WC_SIZE 2

/** Size of the EtherCAT address field. */
#define EC_ADDR_LEN 4

/** Resulting maximum data size of a single datagram in a frame. */
#define EC_MAX_DATA_SIZE (ETH_DATA_LEN - EC_FRAME_HEADER_SIZE - EC_DATAGRAM_HEADER_SIZE - EC_DATAGRAM_WC_SIZE)

/** Mailbox header size.  */
#define EC_MBOX_HEADER_SIZE 6

/** Word offset of first SII category. */
#define EC_FIRST_SII_CATEGORY_OFFSET 0x40

/** Size of a sync manager configuration page. */
#define EC_SYNC_PAGE_SIZE 8

/** Maximum number of FMMUs per slave. */
#define EC_MAX_FMMUS 16

/** Size of an FMMU configuration page. */
#define EC_FMMU_PAGE_SIZE 16

/** Number of DC sync signals. */
#define EC_SYNC_SIGNAL_COUNT 2

/** Size of the datagram decription string.
 *
 * This is also used as the maximum lenth of EoE device names.
 **/
#define EC_DATAGRAM_NAME_SIZE 20

/** Maximum hostname size.
 *
 * Used inside the EoE set IP parameter request.
 */
#define EC_MAX_HOSTNAME_SIZE 32

/** Maximum number of sync managers per slave.
 */
#define EC_MAX_SYNC_MANAGERS 16

/** Maximum string length.
 *
 * Used in ec_slave_info_t.
 */
#define EC_MAX_STRING_LENGTH 64

/** Maximum number of slave ports. */
#define EC_MAX_PORTS 4

/** Slave state mask.
 *
 * Apply this mask to a slave state byte to get the slave state without
 * the error flag.
 */
#define EC_SLAVE_STATE_MASK 0x0F

/** State of an EtherCAT slave.
 */
typedef enum {
    EC_SLAVE_STATE_UNKNOWN = 0x00,
    /**< unknown state */
    EC_SLAVE_STATE_INIT = 0x01,
    /**< INIT state (no mailbox communication, no IO) */
    EC_SLAVE_STATE_PREOP = 0x02,
    /**< PREOP state (mailbox communication, no IO) */
    EC_SLAVE_STATE_BOOT = 0x03,
    /**< Bootstrap state (mailbox communication, firmware update) */
    EC_SLAVE_STATE_SAFEOP = 0x04,
    /**< SAFEOP (mailbox communication and input update) */
    EC_SLAVE_STATE_OP = 0x08,
    /**< OP (mailbox communication and input/output update) */
    EC_SLAVE_STATE_ACK_ERR = 0x10
    /**< Acknowledge/Error bit (no actual state) */
} ec_slave_state_t;

/** Slave information interface CANopen over EtherCAT details flags.
 */
typedef struct {
    uint8_t enable_sdo                 : 1; /**< Enable SDO access. */
    uint8_t enable_sdo_info            : 1; /**< SDO information service available. */
    uint8_t enable_pdo_assign          : 1; /**< PDO mapping configurable. */
    uint8_t enable_pdo_configuration   : 1; /**< PDO configuration possible. */
    uint8_t enable_upload_at_startup   : 1; /**< ?. */
    uint8_t enable_sdo_complete_access : 1; /**< Complete access possible. */
    uint8_t                            : 2; /**< Reserved bits. */
} ec_sii_coe_details_t;

/** Slave information interface general flags.
 */
typedef struct {
    uint8_t enable_safeop  : 1; /**< ?. */
    uint8_t enable_not_lrw : 1; /**< Slave does not support LRW. */
    uint8_t                : 6; /**< Reserved bits. */
} ec_sii_general_flags_t;

/** EtherCAT slave distributed clocks range.
 */
typedef enum {
    EC_DC_32, /**< 32 bit. */
    EC_DC_64  /*< 64 bit for system time, system time offset and
               port 0 receive time. */
} ec_slave_dc_range_t;

/** EtherCAT slave sync signal configuration.
 */
typedef struct {
    uint32_t cycle_time; /**< Cycle time [ns]. */
    int32_t shift_time;  /**< Shift time [ns]. */
} ec_sync_signal_t;

/** Master netdev.
 */
typedef enum {
    EC_NETDEV_MAIN,  /**< Main netdev. */
    EC_NETDEV_BACKUP /**< Backup netdev */
} ec_netdev_index_t;

typedef struct ec_alstatus {
    uint16_t alstatus;
    uint16_t unused;
    uint16_t alstatuscode;
} ec_alstatus_t;

/* AL Status Codes */
#define EC_ALSTATUSCODE_NOERROR                    0x0000 /**< No error*/
#define EC_ALSTATUSCODE_UNSPECIFIEDERROR           0x0001 /**< Unspecified error*/
#define EC_ALSTATUSCODE_NOMEMORY                   0x0002 /**< No Memory*/
#define EC_ALSTATUSCODE_INVALID_REVISION           0x0004 /**< Output/Input mapping is not valid for this hardware or software revision (0x1018:03)*/
#define EC_ALSTATUSCODE_FW_SII_NOT_MATCH           0x0006 /**< Firmware and EEPROM do not match. Slave needs BOOT-INIT transition*/
#define EC_ALSTATUSCODE_FW_UPDATE_FAILED           0x0007 /**< Firmware update not successful. Old firmware still running*/
#define EC_ALSTATUSCODE_INVALIDALCONTROL           0x0011 /**< Invalid requested state change*/
#define EC_ALSTATUSCODE_UNKNOWNALCONTROL           0x0012 /**< Unknown requested state*/
#define EC_ALSTATUSCODE_BOOTNOTSUPP                0x0013 /**< Bootstrap not supported*/
#define EC_ALSTATUSCODE_NOVALIDFIRMWARE            0x0014 /**< No valid firmware*/
#define EC_ALSTATUSCODE_INVALIDMBXCFGINBOOT        0x0015 /**< Invalid mailbox configuration (BOOT state)*/
#define EC_ALSTATUSCODE_INVALIDMBXCFGINPREOP       0x0016 /**< Invalid mailbox configuration (PreOP state)*/
#define EC_ALSTATUSCODE_INVALIDSMCFG               0x0017 /**< Invalid sync manager configuration*/
#define EC_ALSTATUSCODE_NOVALIDINPUTS              0x0018 /**< No valid inputs available*/
#define EC_ALSTATUSCODE_NOVALIDOUTPUTS             0x0019 /**< No valid outputs*/
#define EC_ALSTATUSCODE_SYNCERROR                  0x001A /**< Synchronization error*/
#define EC_ALSTATUSCODE_SMWATCHDOG                 0x001B /**< Sync manager watchdog*/
#define EC_ALSTATUSCODE_SYNCTYPESNOTCOMPATIBLE     0x001C /**< Invalid Sync Manager Types*/
#define EC_ALSTATUSCODE_INVALIDSMOUTCFG            0x001D /**< Invalid Output Configuration*/
#define EC_ALSTATUSCODE_INVALIDSMINCFG             0x001E /**< Invalid Input Configuration*/
#define EC_ALSTATUSCODE_INVALIDWDCFG               0x001F /**< Invalid Watchdog Configuration*/
#define EC_ALSTATUSCODE_WAITFORCOLDSTART           0x0020 /**< Slave needs cold start*/
#define EC_ALSTATUSCODE_WAITFORINIT                0x0021 /**< Slave needs INIT*/
#define EC_ALSTATUSCODE_WAITFORPREOP               0x0022 /**< Slave needs PREOP*/
#define EC_ALSTATUSCODE_WAITFORSAFEOP              0x0023 /**< Slave needs SAFEOP*/
#define EC_ALSTATUSCODE_INVALIDINPUTMAPPING        0x0024 /**< Invalid Input Mapping*/
#define EC_ALSTATUSCODE_INVALIDOUTPUTMAPPING       0x0025 /**< Invalid Output Mapping*/
#define EC_ALSTATUSCODE_INCONSISTENTSETTINGS       0x0026 /**< Inconsistent Settings*/
#define EC_ALSTATUSCODE_FREERUNNOTSUPPORTED        0x0027 /**< FreeRun not supported*/
#define EC_ALSTATUSCODE_SYNCHRONNOTSUPPORTED       0x0028 /**< SyncMode not supported*/
#define EC_ALSTATUSCODE_FREERUNNEEDS3BUFFERMODE    0x0029 /**< FreeRun needs 3Buffer Mode*/
#define EC_ALSTATUSCODE_BACKGROUNDWATCHDOG         0x002A /**< Background Watchdog*/
#define EC_ALSTATUSCODE_NOVALIDINPUTSANDOUTPUTS    0x002B /**< No Valid Inputs and Outputs*/
#define EC_ALSTATUSCODE_FATALSYNCERROR             0x002C /**< Fatal Sync Error*/
#define EC_ALSTATUSCODE_NOSYNCERROR                0x002D /**< No Sync Error*/
#define EC_ALSTATUSCODE_CYCLETIMETOOSMALL          0x002E /**< EtherCAT cycle time smaller Minimum Cycle Time supported by slave*/
#define EC_ALSTATUSCODE_DCINVALIDSYNCCFG           0x0030 /**< Invalid DC SYNCH Configuration*/
#define EC_ALSTATUSCODE_DCINVALIDLATCHCFG          0x0031 /**< Invalid DC Latch Configuration*/
#define EC_ALSTATUSCODE_DCPLLSYNCERROR             0x0032 /**< PLL Error*/
#define EC_ALSTATUSCODE_DCSYNCIOERROR              0x0033 /**< DC Sync IO Error*/
#define EC_ALSTATUSCODE_DCSYNCMISSEDERROR          0x0034 /**< DC Sync Timeout Error*/
#define EC_ALSTATUSCODE_DCINVALIDSYNCCYCLETIME     0x0035 /**< DC Invalid Sync Cycle Time*/
#define EC_ALSTATUSCODE_DCSYNC0CYCLETIME           0x0036 /**< DC Sync0 Cycle Time*/
#define EC_ALSTATUSCODE_DCSYNC1CYCLETIME           0x0037 /**< DC Sync1 Cycle Time*/
#define EC_ALSTATUSCODE_MBX_AOE                    0x0041 /**< MBX_AOE*/
#define EC_ALSTATUSCODE_MBX_EOE                    0x0042 /**< MBX_EOE*/
#define EC_ALSTATUSCODE_MBX_COE                    0x0043 /**< MBX_COE*/
#define EC_ALSTATUSCODE_MBX_FOE                    0x0044 /**< MBX_FOE*/
#define EC_ALSTATUSCODE_MBX_SOE                    0x0045 /**< MBX_SOE*/
#define EC_ALSTATUSCODE_MBX_VOE                    0x004F /**< MBX_VOE*/
#define EC_ALSTATUSCODE_EE_NOACCESS                0x0050 /**< EEPROM no access*/
#define EC_ALSTATUSCODE_EE_ERROR                   0x0051 /**< EEPROM Error*/
#define EC_ALSTATUSCODE_EXT_HARDWARE_NOT_READY     0x0052 /**< External hardware not ready. This AL Status Code should be used if the EtherCAT-Slave refused the state transition due to an external connection to another device or signal is missing*/
#define EC_ALSTATUSCODE_DEVICE_IDENT_VALUE_UPDATED 0x0061 /**< In legacy identification mode (dip switch mapped to register 0x12) this error is returned if the EEPROM ID value does not match to dipswitch value*/
#define EC_ALSTATUSCODE_MODULE_ID_LIST_NOT_MATCH   0x0070 /**< Detected Module Ident List (0xF030) and Configured Module Ident List (0xF050) does not match*/
#define EC_ALSTATUSCODE_SUPPLY_VOLTAGE_TOO_LOW     0x0080 /**< The slave supply voltage is too low*/
#define EC_ALSTATUSCODE_SUPPLY_VOLTAGE_TOO_HIGH    0x0081 /**< The slave supply voltage is too high*/
#define EC_ALSTATUSCODE_TEMPERATURE_TOO_LOW        0x0082 /**< The slave temperature is too low*/
#define EC_ALSTATUSCODE_TEMPERATURE_TOO_HIGH       0x0083 /**< The slave temperature is too high*/

#define EC_SII_ADDRESS_MANUF           (0x0008)
#define EC_SII_ADDRESS_PRODUCTCODE     (0x000a)
#define EC_SII_ADDRESS_REVISION        (0x000c)
#define EC_SII_ADDRESS_SN              (0x000E)
#define EC_SII_ADDRESS_BOOTRXMBX       (0x0014)
#define EC_SII_ADDRESS_BOOTTXMBX       (0x0016)
#define EC_SII_ADDRESS_MBXSIZE         (0x0019)
#define EC_SII_ADDRESS_TXMBXADR        (0x001a)
#define EC_SII_ADDRESS_RXMBXADR        (0x0018)
#define EC_SII_ADDRESS_MBXPROTO        (0x001c)
#define EC_SII_ADDRESS_ADDITIONAL_INFO (0x0040)

#define EC_SII_TYPE_NOP       0x0000
#define EC_SII_TYPE_STRINGS   0x000A
#define EC_SII_TYPE_DATATYPES 0x0014
#define EC_SII_TYPE_GENERAL   0x001E
#define EC_SII_TYPE_FMMU      0x0028
#define EC_SII_TYPE_SM        0x0029
#define EC_SII_TYPE_FMMUX     0x002A
#define EC_SII_TYPE_SYNCUNIT  0x002B
#define EC_SII_TYPE_TXPDO     0x0032
#define EC_SII_TYPE_RXPDO     0x0033
#define EC_SII_TYPE_DC        0x003C
#define EC_SII_TYPE_END       0xFFFF

#define EC_SII_FMMU_NONE      0x0000
#define EC_SII_FMMU_READ      0x0001
#define EC_SII_FMMU_WRITE     0x0002
#define EC_SII_FMMU_SM_STATUS 0x0003

#define EC_SII_SM_UNKNOWN             0x0000
#define EC_SII_SM_MBX_OUT             0x0001
#define EC_SII_SM_MBX_IN              0x0002
#define EC_SII_SM_PROCESS_DATA_OUTPUT 0x0003
#define EC_SII_SM_PROCESS_DATA_INPUT  0x0004

#define EC_SM_INDEX_MBX_WRITE           0x0000
#define EC_SM_INDEX_MBX_READ            0x0001
#define EC_SM_INDEX_PROCESS_DATA_OUTPUT 0x0002
#define EC_SM_INDEX_PROCESS_DATA_INPUT  0x0003

typedef struct __PACKED ec_sii_base {
    uint16_t pdi_control;
    uint16_t pdi_config;
    uint16_t sync_impulselen;
    uint16_t pdi_config2;
    uint16_t aliasaddr; /**< Configured station alias. */
    uint8_t reserved[4];
    uint16_t checksum;
    uint32_t vendor_id;       /**< Vendor ID. */
    uint32_t product_code;    /**< Vendor-specific product code. */
    uint32_t revision_number; /**< Revision number. */
    uint32_t serial_number;   /**< Serial number. */
    uint8_t reserved2[8];
    uint16_t boot_rx_mailbox_offset; /**< Bootstrap receive mailbox address. */
    uint16_t boot_rx_mailbox_size;   /**< Bootstrap receive mailbox size. */
    uint16_t boot_tx_mailbox_offset; /**< Bootstrap transmit mailbox address. */
    uint16_t boot_tx_mailbox_size;   /**< Bootstrap transmit mailbox size. */
    uint16_t std_rx_mailbox_offset;  /**< Standard receive mailbox address. */
    uint16_t std_rx_mailbox_size;    /**< Standard receive mailbox size. */
    uint16_t std_tx_mailbox_offset;  /**< Standard transmit mailbox address. */
    uint16_t std_tx_mailbox_size;    /**< Standard transmit mailbox size. */
    uint16_t mailbox_protocols;      /**< Supported mailbox protocols. */
    uint8_t reserved3[66];
    uint16_t size;
    uint16_t version;
} ec_sii_base_t;

typedef struct __PACKED ec_sii_general {
    uint8_t groupidx;
    uint8_t imgidx;
    uint8_t orderidx;
    uint8_t nameidx;
    uint8_t reserved1;
    ec_sii_coe_details_t coe_details;
    uint8_t foe_details;
    uint8_t eoe_details;
    uint8_t soe_channels;
    uint8_t ds402_channels;
    uint8_t sysmanclass;
    ec_sii_general_flags_t flags;
    int16_t current_on_ebus;
    uint8_t reserved2;
    uint8_t reserved3;
    uint16_t phy_port;
    uint16_t phy_memaddress;
    uint8_t pad[12];
} ec_sii_general_t;

typedef struct __PACKED ec_sii_sm {
    uint16_t physical_start_address;
    uint16_t length;
    uint8_t control;
    uint8_t status;
    uint8_t active;
    uint8_t type;
} ec_sii_sm_t;

typedef struct __PACKED ec_sii_pdo_entry {
    uint16_t index;
    uint8_t subindex;
    uint8_t nameidx;
    uint8_t data_type;
    uint8_t bitlen;
    uint16_t flags;
} ec_sii_pdo_entry_t;

typedef struct __PACKED ec_sii_pdo_mapping {
    uint16_t index; /* txpdo: 1a00~1bff, rxpdo: 1600~17ff */
    uint8_t nentry;
    uint8_t sm_idx;
    uint8_t synchronization;
    uint8_t nameidx;
    uint16_t flags;
    ec_sii_pdo_entry_t entry[];
} ec_sii_pdo_mapping_t;

typedef struct __PACKED ec_sm_reg {
    uint16_t physical_start_address;
    uint16_t length;
    uint8_t control;
    uint8_t status;
    uint8_t active;
    uint8_t pdi_control;
} ec_sm_reg_t;

typedef struct __PACKED ec_fmmu_reg {
    uint32_t logical_start_address;
    uint16_t length;
    uint8_t logical_start_bit;
    uint8_t logical_stop_bit;
    uint16_t physical_start_address;
    uint8_t physical_start_bit;
    uint8_t type;
    uint8_t active;
    uint8_t reserved[3];
} ec_fmmu_reg_t;

/**
 * \brief SmAssignObjects SyncManager Assignment Objects
 * SyncManager 2 : 0x1C12<br>
 * SyncManager 3 : 0x1C13<br>
 */
typedef struct __PACKED ec_pdo_assign_t {
    uint16_t count; /**< PDO mapping count. */
    uint16_t entry[CONFIG_EC_PER_SM_MAX_PDOS];
} ec_pdo_assign_t;

typedef struct __PACKED ec_pdo_mapping_t {
    uint16_t count; /**< PDO entry count. */
    uint32_t entry[CONFIG_EC_PER_PDO_MAX_PDO_ENTRIES];
} ec_pdo_mapping_t;

typedef struct __PACKED ec_mailbox_header {
    uint16_t length;
    uint16_t address;
    uint8_t channel  : 6;
    uint8_t priority : 2;
    uint8_t type     : 4;
    uint8_t counter  : 3;
    uint8_t reserved : 1;
} ec_mailbox_header_t;

/** Size of the mailbox header.
 */
#define EC_MBOX_HEADER_SIZE 6

#define EC_MBXPROT_AOE 0x0001
#define EC_MBXPROT_EOE 0x0002
#define EC_MBXPROT_COE 0x0004
#define EC_MBXPROT_FOE 0x0008
#define EC_MBXPROT_SOE 0x0010
#define EC_MBXPROT_VOE 0x0020

/** Mailbox types.
 *
 * These are used in the 'Type' field of the mailbox header.
 */
enum {
    EC_MBOX_TYPE_EOE = 0x02,
    EC_MBOX_TYPE_COE = 0x03,
    EC_MBOX_TYPE_FOE = 0x04,
    EC_MBOX_TYPE_SOE = 0x05,
    EC_MBOX_TYPE_VOE = 0x0f,
};

#define EC_MBXERR_SYNTAX              0x01 /**< \brief Mailbox error "syntax"*/
#define EC_MBXERR_UNSUPPORTEDPROTOCOL 0x02 /**< \brief Mailbox error "unsupported protocol"*/
#define EC_MBXERR_INVALIDCHANNEL      0x03 /**< \brief Mailbox error "invalid channel"*/
#define EC_MBXERR_SERVICENOTSUPPORTED 0x04 /**< \brief Mailbox error "service not supported"*/
#define EC_MBXERR_INVALIDHEADER       0x05 /**< \brief Mailbox error "invalid header"*/
#define EC_MBXERR_SIZETOOSHORT        0x06 /**< \brief Mailbox error "Size too short"*/
#define EC_MBXERR_NOMOREMEMORY        0x07 /**< \brief Mailbox error "No memory"*/
#define EC_MBXERR_INVALIDSIZE         0x08 /**< \brief Mailbox error "Invalid size"*/
#define EC_MBXERR_SERVICEINWORK       0x09 /**< \brief Mailbox error "Service in work"*/

typedef struct __PACKED ec_coe_header {
    uint16_t number   : 9;
    uint16_t reserved : 3;
    uint16_t service  : 4;
} ec_coe_header_t;

typedef struct __PACKED ec_sdo_header_common {
    uint8_t size_indicator  : 1;
    uint8_t transfertype    : 1; // expedited transfer
    uint8_t data_set_size   : 2;
    uint8_t complete_access : 1;
    uint8_t command         : 3;
} ec_sdo_header_common_t;

typedef struct __PACKED ec_sdo_header_segment {
    uint8_t more_follows : 1;
    uint8_t segdata_size : 3;
    uint8_t toggle       : 1;
    uint8_t command      : 3;
} ec_sdo_header_segment_t;

typedef struct __PACKED ec_sdo_header {
    union {
        uint8_t byte;
        ec_sdo_header_common_t common;
        ec_sdo_header_segment_t segment;
    };
    uint16_t index;
    uint8_t subindex;
} ec_sdo_header_t;

#define EC_COE_SERVICE_EMERGENCY            0x01
#define EC_COE_SERVICE_SDO_REQUEST          0x02
#define EC_COE_SERVICE_SDO_RESPONSE         0x03
#define EC_COE_SERVICE_TXPDO                0x04
#define EC_COE_SERVICE_RXPDO                0x05
#define EC_COE_SERVICE_TXPDO_REMOTE_REQUSET 0x06
#define EC_COE_SERVICE_RXPDO_REMOTE_REQUEST 0x07
#define EC_COE_SERVICE_SDOINFO              0x08

#define EC_COE_REQUEST_SEGMENT_DOWNLOAD 0x00
#define EC_COE_REQUEST_DOWNLOAD         0x01
#define EC_COE_REQUEST_UPLOAD           0x02
#define EC_COE_REQUEST_SEGMENT_UPLOAD   0x03
#define EC_COE_REQUEST_ABORT            0x04

#define EC_COE_RESPONSE_SEGMENT_UPLOAD   0x00
#define EC_COE_RESPONSE_SEGMENT_DOWNLOAD 0x01
#define EC_COE_RESPONSE_UPLOAD           0x02
#define EC_COE_RESPONSE_DOWNLOAD         0x03

typedef struct __PACKED {
    uint16_t opcode;
    union {
        uint32_t password;
        uint32_t packet_number;
        uint32_t error_code;
    };
} ec_foe_header_t;

#define EC_FOE_OPCODE_READ  0x0001
#define EC_FOE_OPCODE_WRITE 0x0002
#define EC_FOE_OPCODE_DATA  0x0003
#define EC_FOE_OPCODE_ACK   0x0004
#define EC_FOE_OPCODE_ERROR 0x0005
#define EC_FOE_OPCODE_BUSY  0x0006

#define EC_FOE_ERRCODE_NOTDEFINED       0x8000 /**< \brief Not defined*/
#define EC_FOE_ERRCODE_NOTFOUND         0x8001 /**< \brief The file requested by an FoE upload service could not be found on the server*/
#define EC_FOE_ERRCODE_ACCESS           0x8002 /**< \brief Read or write access to this file not allowed (e.g. due to local control).*/
#define EC_FOE_ERRCODE_DISKFULL         0x8003 /**< \brief Disk to store file is full or memory allocation exceeded*/
#define EC_FOE_ERRCODE_ILLEGAL          0x8004 /**< \brief Illegal FoE operation, e.g. service identifier invalid*/
#define EC_FOE_ERRCODE_PACKENO          0x8005 /**< \brief FoE packet number invalid*/
#define EC_FOE_ERRCODE_EXISTS           0x8006 /**< \brief The file which is requested to be downloaded does already exist*/
#define EC_FOE_ERRCODE_NOUSER           0x8007 /**< \brief No User*/
#define EC_FOE_ERRCODE_BOOTSTRAPONLY    0x8008 /**< \brief FoE only supported in Bootstrap*/
#define EC_FOE_ERRCODE_NOTINBOOTSTRAP   0x8009 /**< \brief This file may not be accessed in BOOTSTRAP state*/
#define EC_FOE_ERRCODE_NORIGHTS         0x800A /**< \brief Password invalid*/
#define EC_FOE_ERRCODE_PROGERROR        0x800B /**< \brief Generic programming error. Should only be returned if  error reason cannot be distinguished*/
#define EC_FOE_ERRCODE_INVALID_CHECKSUM 0x800C /**< \brief checksum included in the file is invalid*/
#define EC_FOE_ERRCODE_INVALID_FIRMWARE 0x800D /**< \brief The hardware does not support the downloaded firmware*/
#define EC_FOE_ERRCODE_NO_FILE          0x800F /**< \brief Do not use (identical with 0x8001)*/
#define EC_FOE_ERRCODE_NO_FILE_HEADER   0x8010 /**< \brief Missing file header of error in file header*/
#define EC_FOE_ERRCODE_FLASH_ERROR      0x8011 /**< \brief Flash cannot be accessed*/

#define EC_EOE_TYPE_FRAGMENT                0x00
#define EC_EOE_TYPE_TIMESTAMP               0x01
#define EC_EOE_TYPE_SET_IP_REQUEST          0x02
#define EC_EOE_TYPE_SET_IP_RESPONSE         0x03
#define EC_EOE_TYPE_MACFILTER_REQUEST       0x04
#define EC_EOE_TYPE_MACFILTER_RESPONSE      0x05
#define EC_EOE_TYPE_GET_IP_REQUEST          0x06
#define EC_EOE_TYPE_GET_IP_RESPONSE         0x07
#define EC_EOE_TYPE_GET_MAC_FILTER_REQUEST  0x08
#define EC_EOE_TYPE_GET_MAC_FILTER_RESPONSE 0x09

#define EC_EOE_RESULT_NOERROR                  0x0000 /**< \brief No Error*/
#define EC_EOE_RESULT_UNSPECIFIED_ERROR        0x0001 /**< \brief Unspecified error*/
#define EC_EOE_RESULT_UNSUPPORTED_TYPE         0x0002 /**< \brief unsupported type*/
#define EC_EOE_RESULT_NO_IP_SUPPORT            0x0201 /**< \brief No IP supported*/
#define EC_EOE_RESULT_NO_DHCP_SUPPORT          0x0202 /**< \brief No DHCP supported*/
#define EC_EOE_RESULT_NO_MACFILTERMASK_SUPPORT 0x0401 /**< \brief No mac filter supported*/

typedef enum {
    EC_DIR_OUTPUT, /**< Values written by the master. */
    EC_DIR_INPUT,  /**< Values read by the master. */
} ec_direction_t;

typedef enum {
    EC_WD_DEFAULT, /**< Use the default setting of the sync manager. */
    EC_WD_ENABLE,  /**< Enable the watchdog. */
    EC_WD_DISABLE, /**< Disable the watchdog. */
} ec_watchdog_mode_t;

typedef struct {
    uint16_t index;     /**< PDO entry index. */
    uint8_t subindex;   /**< PDO entry subindex. */
    uint8_t bit_length; /**< Size of the PDO entry in bit. */
} ec_pdo_entry_info_t;

typedef struct {
    uint16_t index; /**< PDO index. */
    uint32_t n_entries;
    ec_pdo_entry_info_t const *entries;
} ec_pdo_info_t;

typedef struct {
    uint8_t index; /**< Sync manager index. */
    ec_direction_t dir;
    uint32_t n_pdos;
    ec_pdo_info_t const *pdos;
    ec_watchdog_mode_t watchdog_mode;
} ec_sync_info_t;

#endif