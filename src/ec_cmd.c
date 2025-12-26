/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_master.h"

#ifdef CONFIG_EC_CMD_ENABLE

typedef struct {
    uint32_t slave_count;
    uint8_t phase;
    uint8_t active;
    struct ec_ioctl_device {
        uint8_t mac_addr[6];
        uint8_t attached;
        uint8_t link_state;
        uint64_t tx_count;
        uint64_t rx_count;
        uint64_t tx_bytes;
        uint64_t rx_bytes;
        uint64_t tx_errors;
        int32_t tx_frame_rates[EC_RATE_COUNT];
        int32_t rx_frame_rates[EC_RATE_COUNT];
        int32_t tx_byte_rates[EC_RATE_COUNT];
        int32_t rx_byte_rates[EC_RATE_COUNT];
    } netdevs[CONFIG_EC_MAX_NETDEVS];
    uint32_t num_netdevs;
    uint64_t tx_count;
    uint64_t rx_count;
    uint64_t tx_bytes;
    uint64_t rx_bytes;
    int32_t tx_frame_rates[EC_RATE_COUNT];
    int32_t rx_frame_rates[EC_RATE_COUNT];
    int32_t tx_byte_rates[EC_RATE_COUNT];
    int32_t rx_byte_rates[EC_RATE_COUNT];
    int32_t loss_rates[EC_RATE_COUNT];
    uint64_t app_time;
    uint64_t dc_ref_time;
    uint16_t ref_clock;
} ec_cmd_master_info_t;

typedef struct {
    uint32_t netdev_idx;
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t revision_number;
    uint32_t serial_number;
    uint16_t alias;
    uint16_t boot_rx_mailbox_offset;
    uint16_t boot_rx_mailbox_size;
    uint16_t boot_tx_mailbox_offset;
    uint16_t boot_tx_mailbox_size;
    uint16_t std_rx_mailbox_offset;
    uint16_t std_rx_mailbox_size;
    uint16_t std_tx_mailbox_offset;
    uint16_t std_tx_mailbox_size;
    uint16_t mailbox_protocols;
    bool has_general;
    ec_sii_coe_details_t coe_details;
    ec_sii_general_flags_t general_flags;
    int16_t current_on_ebus;
    struct {
        ec_slave_port_desc_t desc;
        ec_slave_port_link_t link;
        uint32_t receive_time;
        uint16_t next_slave;
        uint32_t delay_to_next_dc;
    } ports[EC_MAX_PORTS];
    uint8_t base_fmmu_bit_operation;
    uint8_t base_dc_supported;
    ec_slave_dc_range_t base_dc_range;
    uint8_t has_dc_system_time;
    uint32_t transmission_delay;
    uint8_t current_state;
    uint8_t error_flag;
    uint8_t sync_count;
    uint16_t sdo_count;
    uint32_t sii_nwords;
    char *group;
    char *image;
    char *order;
    char *name;
} ec_cmd_slave_info_t;

static ec_master_t *global_cmd_master = NULL;

#ifdef CONFIG_EC_EOE
static ec_eoe_t *global_cmd_eoe = NULL;
#endif

void ec_master_cmd_init(ec_master_t *master)
{
    global_cmd_master = master;
}

#ifdef CONFIG_EC_EOE
void ec_master_cmd_eoe_init(ec_eoe_t *master)
{
    global_cmd_eoe = master;
}
#endif

static void ec_master_cmd_show_help(void)
{
    EC_LOG_RAW("CherryECAT " CHERRYECAT_VERSION_STR " Command Line Tool\n\n");
    EC_LOG_RAW("Usage: ethercat <command> [options]\n");
    EC_LOG_RAW("Commands:\n");
    EC_LOG_RAW("  master                                         Show master information\n");
    EC_LOG_RAW("  rescan                                         Request a slaves rescan\n");
    EC_LOG_RAW("  slaves                                         Show slaves overview\n");
    EC_LOG_RAW("  slaves -v                                      Show detailed information for all slaves\n");
    EC_LOG_RAW("  slaves -p <idx>                                Show information for slave <idx>\n");
    EC_LOG_RAW("  slaves -p <idx> -v                             Show detailed information for slave <idx>\n");
    EC_LOG_RAW("  pdos                                           Show PDOs for all slaves\n");
    EC_LOG_RAW("  pdos -p <idx>                                  Show PDOs for slave <idx>\n");
    EC_LOG_RAW("  states <state>                                 Request state for all slaves (hex)\n");
    EC_LOG_RAW("  states -p <idx> <state>                        Request state for slave <idx> (hex)\n");
    EC_LOG_RAW("  coe_read -p [idx] [index] [subindex]           Read SDO via CoE\n");
    EC_LOG_RAW("  coe_write -p [idx] [index] [subindex] [data]   Write SDO via CoE\n");
#ifdef CONFIG_EC_FOE
    EC_LOG_RAW("  foe_write -p [idx] [filename] [pwd] [hexdata]  Read hexarray via FoE\n");
    EC_LOG_RAW("  foe_read -p [idx] [filename] [pwd]             Write hexarray via FoE\n");
#endif
#ifdef CONFIG_EC_EOE
    EC_LOG_RAW("  eoe_start -p [slave_idx]                       Start EoE on slave <idx>\n");
#endif
    EC_LOG_RAW("  pdo_read                                       Read process data\n");
    EC_LOG_RAW("  pdo_read -p [idx]                              Read slave <idx> process data\n");
    EC_LOG_RAW("  pdo_write [offset] [hex low...high]            Write hexarray with offset to pdo\n");
    EC_LOG_RAW("  pdo_write -p [idx] [offset] [hex low...high]   Write slave <idx> hexarray with offset to pdo\n");
    EC_LOG_RAW("  sii_read -p [idx]                              Read SII\n");
    EC_LOG_RAW("  sii_write -p [idx]                             Write SII\n");
    EC_LOG_RAW("  wc                                             Show master working counter\n");
    EC_LOG_RAW("  perf -s                                        Start performance test\n");
    EC_LOG_RAW("  perf -d                                        Stop performance test\n");
    EC_LOG_RAW("  perf -v                                        Show performance statistics\n");
    EC_LOG_RAW("  timediff -s                                    Enable system time diff monitor\n");
    EC_LOG_RAW("  timediff -d                                    Disable system time diff monitor\n");
    EC_LOG_RAW("  timediff -v                                    Show system time diff statistics\n");
    EC_LOG_RAW("  help                                           Show this help\n\n");
}

static const char *ec_slave_state_string(uint8_t state)
{
    switch (state) {
        case 0x01:
            return "INIT";
        case 0x02:
            return "PREOP";
        case 0x03:
            return "BOOT";
        case 0x04:
            return "SAFEOP";
        case 0x08:
            return "OP";
        default:
            return "UNKNOWN";
    }
}

static const char *ec_port_desc_string(uint8_t desc)
{
    switch (desc) {
        case 0:
            return "N/A";
        case 1:
            return "N/C";
        case 2:
            return "EBUS";
        case 3:
            return "MII";
        default:
            return "???";
    }
}

static void ec_master_get_master_info(ec_master_t *master, ec_cmd_master_info_t *info)
{
    unsigned int dev_idx;
    int j;

    info->slave_count = master->slave_count;
    info->phase = (uint8_t)master->phase;

    for (dev_idx = EC_NETDEV_MAIN; dev_idx < CONFIG_EC_MAX_NETDEVS; dev_idx++) {
        ec_netdev_t *device = master->netdev[dev_idx];

        ec_memcpy(info->netdevs[dev_idx].mac_addr, device->mac_addr, ETH_ALEN);

        info->netdevs[dev_idx].attached = master->netdev[dev_idx] ? 1 : 0;
        info->netdevs[dev_idx].link_state = device->link_state ? 1 : 0;
        info->netdevs[dev_idx].tx_count = device->tx_count;
        info->netdevs[dev_idx].rx_count = device->rx_count;
        info->netdevs[dev_idx].tx_bytes = device->tx_bytes;
        info->netdevs[dev_idx].rx_bytes = device->rx_bytes;
        info->netdevs[dev_idx].tx_errors = device->tx_errors;
        for (j = 0; j < EC_RATE_COUNT; j++) {
            info->netdevs[dev_idx].tx_frame_rates[j] =
                device->tx_frame_rates[j];
            info->netdevs[dev_idx].rx_frame_rates[j] =
                device->rx_frame_rates[j];
            info->netdevs[dev_idx].tx_byte_rates[j] =
                device->tx_byte_rates[j];
            info->netdevs[dev_idx].rx_byte_rates[j] =
                device->rx_byte_rates[j];
        }
    }
    info->num_netdevs = CONFIG_EC_MAX_NETDEVS;

    info->tx_count = master->netdev_stats.tx_count;
    info->rx_count = master->netdev_stats.rx_count;
    info->tx_bytes = master->netdev_stats.tx_bytes;
    info->rx_bytes = master->netdev_stats.rx_bytes;
    for (j = 0; j < EC_RATE_COUNT; j++) {
        info->tx_frame_rates[j] =
            master->netdev_stats.tx_frame_rates[j];
        info->rx_frame_rates[j] =
            master->netdev_stats.rx_frame_rates[j];
        info->tx_byte_rates[j] =
            master->netdev_stats.tx_byte_rates[j];
        info->rx_byte_rates[j] =
            master->netdev_stats.rx_byte_rates[j];
        info->loss_rates[j] =
            master->netdev_stats.loss_rates[j];
    }

    // info->app_time = master->app_time;
    // info->dc_ref_time = master->dc_ref_time;
    // info->ref_clock = master->dc_ref_clock ? master->dc_ref_clock->autoinc_address : 0xffff;
}

static void ec_master_get_slave_info(ec_slave_t *slave, ec_cmd_slave_info_t *info)
{
    int i;

    info->netdev_idx = slave->netdev_idx;
    info->vendor_id = slave->sii.vendor_id;
    info->product_code = slave->sii.product_code;
    info->revision_number = slave->sii.revision_number;
    info->serial_number = slave->sii.serial_number;
    info->alias = slave->effective_alias;
    info->boot_rx_mailbox_offset = slave->sii.boot_rx_mailbox_offset;
    info->boot_rx_mailbox_size = slave->sii.boot_rx_mailbox_size;
    info->boot_tx_mailbox_offset = slave->sii.boot_tx_mailbox_offset;
    info->boot_tx_mailbox_size = slave->sii.boot_tx_mailbox_size;
    info->std_rx_mailbox_offset = slave->sii.std_rx_mailbox_offset;
    info->std_rx_mailbox_size = slave->sii.std_rx_mailbox_size;
    info->std_tx_mailbox_offset = slave->sii.std_tx_mailbox_offset;
    info->std_tx_mailbox_size = slave->sii.std_tx_mailbox_size;
    info->mailbox_protocols = slave->sii.mailbox_protocols;
    info->has_general = slave->sii.has_general;
    info->coe_details = slave->sii.general.coe_details;
    info->general_flags = slave->sii.general.flags;
    info->current_on_ebus = slave->sii.general.current_on_ebus;
    for (i = 0; i < EC_MAX_PORTS; i++) {
        info->ports[i].desc = slave->ports[i].desc;
        info->ports[i].link.link_up = slave->ports[i].link.link_up;
        info->ports[i].link.loop_closed = slave->ports[i].link.loop_closed;
        info->ports[i].link.signal_detected =
            slave->ports[i].link.signal_detected;
        info->ports[i].receive_time = slave->ports[i].receive_time;
        if (slave->ports[i].next_slave) {
            info->ports[i].next_slave =
                slave->ports[i].next_slave->autoinc_address;
        } else {
            info->ports[i].next_slave = 0xffff;
        }
        info->ports[i].delay_to_next_dc = slave->ports[i].delay_to_next_dc;
    }
    info->base_fmmu_bit_operation = slave->base_fmmu_bit_operation;
    info->base_dc_supported = slave->base_dc_supported;
    info->base_dc_range = slave->base_dc_range;
    info->has_dc_system_time = slave->has_dc_system_time;
    info->transmission_delay = slave->transmission_delay;
    info->current_state = slave->current_state;

    info->group = ec_slave_get_sii_string(slave, slave->sii.general.groupidx);
    info->image = ec_slave_get_sii_string(slave, slave->sii.general.imgidx);
    info->order = ec_slave_get_sii_string(slave, slave->sii.general.orderidx);
    info->name = ec_slave_get_sii_string(slave, slave->sii.general.nameidx);
}

void ec_master_cmd_master(ec_master_t *master)
{
    unsigned int dev_idx, j;
    uint64_t lost;
    double perc;
    int colwidth = 8;
    ec_cmd_master_info_t data;

    ec_osal_mutex_take(master->scan_lock);
    ec_master_get_master_info(master, &data);
    ec_osal_mutex_give(master->scan_lock);

    EC_LOG_RAW("Master%d\n", master->index);
    EC_LOG_RAW("  Phase: ");
    switch (data.phase) {
        case 0:
            EC_LOG_RAW("Waiting for device(s)...");
            break;
        case 1:
            EC_LOG_RAW("Idle");
            break;
        case 2:
            EC_LOG_RAW("Operation");
            break;
        default:
            EC_LOG_RAW("???");
    }

    EC_LOG_RAW("\n");
    EC_LOG_RAW("  Slaves: %u\n", data.slave_count);
    EC_LOG_RAW("  Ethernet net devices:\n");

    for (dev_idx = EC_NETDEV_MAIN; dev_idx < CONFIG_EC_MAX_NETDEVS; dev_idx++) {
        EC_LOG_RAW("    %s: ", dev_idx == EC_NETDEV_MAIN ? "Main" : "Backup");
        EC_LOG_RAW("%02x:%02x:%02x:%02x:%02x:%02x\n",
                   data.netdevs[dev_idx].mac_addr[0],
                   data.netdevs[dev_idx].mac_addr[1],
                   data.netdevs[dev_idx].mac_addr[2],
                   data.netdevs[dev_idx].mac_addr[3],
                   data.netdevs[dev_idx].mac_addr[4],
                   data.netdevs[dev_idx].mac_addr[5]);
        EC_LOG_RAW("      Link: %s\n", data.netdevs[dev_idx].link_state ? "UP" : "DOWN");
        EC_LOG_RAW("      Tx frames:   %llu\n", data.netdevs[dev_idx].tx_count);
        EC_LOG_RAW("      Tx bytes:    %llu\n", data.netdevs[dev_idx].tx_bytes);
        EC_LOG_RAW("      Rx frames:   %llu\n", data.netdevs[dev_idx].rx_count);
        EC_LOG_RAW("      Rx bytes:    %llu\n", data.netdevs[dev_idx].rx_bytes);
        EC_LOG_RAW("      Tx errors:   %llu\n", data.netdevs[dev_idx].tx_errors);

        EC_LOG_RAW("      Tx frame rate [1/s]: ");
        for (j = 0; j < EC_RATE_COUNT; j++) {
            EC_LOG_RAW("%*.*f", colwidth, 0, data.netdevs[dev_idx].tx_frame_rates[j] / 1000.0);
            if (j < EC_RATE_COUNT - 1)
                EC_LOG_RAW(" ");
        }
        EC_LOG_RAW("\n      Tx rate [KByte/s]:   ");
        for (j = 0; j < EC_RATE_COUNT; j++) {
            EC_LOG_RAW("%*.*f", colwidth, 1, data.netdevs[dev_idx].tx_byte_rates[j] / 1024.0);
            if (j < EC_RATE_COUNT - 1)
                EC_LOG_RAW(" ");
        }
        EC_LOG_RAW("\n      Rx frame rate [1/s]: ");
        for (j = 0; j < EC_RATE_COUNT; j++) {
            EC_LOG_RAW("%*.*f", colwidth, 0, data.netdevs[dev_idx].rx_frame_rates[j] / 1000.0);
            if (j < EC_RATE_COUNT - 1)
                EC_LOG_RAW(" ");
        }
        EC_LOG_RAW("\n      Rx rate [KByte/s]:   ");
        for (j = 0; j < EC_RATE_COUNT; j++) {
            EC_LOG_RAW("%*.*f", colwidth, 1, data.netdevs[dev_idx].rx_byte_rates[j] / 1024.0);
            if (j < EC_RATE_COUNT - 1)
                EC_LOG_RAW(" ");
        }
        EC_LOG_RAW("\n");
    }

    lost = data.tx_count - data.rx_count;
    if (lost == 1)
        lost = 0;
    EC_LOG_RAW("    Common:\n");
    EC_LOG_RAW("      Tx frames:   %llu\n", data.tx_count);
    EC_LOG_RAW("      Tx bytes:    %llu\n", data.tx_bytes);
    EC_LOG_RAW("      Rx frames:   %llu\n", data.rx_count);
    EC_LOG_RAW("      Rx bytes:    %llu\n", data.rx_bytes);
    EC_LOG_RAW("      Lost frames: %llu\n", lost);

    EC_LOG_RAW("      Tx frame rate [1/s]: ");
    for (j = 0; j < EC_RATE_COUNT; j++) {
        EC_LOG_RAW("%*.*f", colwidth, 0, data.tx_frame_rates[j] / 1000.0);
        if (j < EC_RATE_COUNT - 1)
            EC_LOG_RAW(" ");
    }
    EC_LOG_RAW("\n      Tx rate [KByte/s]:   ");
    for (j = 0; j < EC_RATE_COUNT; j++) {
        EC_LOG_RAW("%*.*f", colwidth, 1, data.tx_byte_rates[j] / 1024.0);
        if (j < EC_RATE_COUNT - 1)
            EC_LOG_RAW(" ");
    }
    EC_LOG_RAW("\n      Rx frame rate [1/s]: ");
    for (j = 0; j < EC_RATE_COUNT; j++) {
        EC_LOG_RAW("%*.*f", colwidth, 0, data.rx_frame_rates[j] / 1000.0);
        if (j < EC_RATE_COUNT - 1)
            EC_LOG_RAW(" ");
    }
    EC_LOG_RAW("\n      Rx rate [KByte/s]:   ");
    for (j = 0; j < EC_RATE_COUNT; j++) {
        EC_LOG_RAW("%*.*f", colwidth, 1, data.rx_byte_rates[j] / 1024.0);
        if (j < EC_RATE_COUNT - 1)
            EC_LOG_RAW(" ");
    }
    EC_LOG_RAW("\n      Loss rate [1/s]:     ");
    for (j = 0; j < EC_RATE_COUNT; j++) {
        EC_LOG_RAW("%*.*f", colwidth, 0, data.loss_rates[j] / 1000.0);
        if (j < EC_RATE_COUNT - 1)
            EC_LOG_RAW(" ");
    }
    EC_LOG_RAW("\n      Frame loss [%%]:      ");
    for (j = 0; j < EC_RATE_COUNT; j++) {
        perc = 0.0;
        if (data.tx_frame_rates[j])
            perc = 100.0 * data.loss_rates[j] / data.tx_frame_rates[j];
        EC_LOG_RAW("%*.*f", colwidth, 1, perc);
        if (j < EC_RATE_COUNT - 1)
            EC_LOG_RAW(" ");
    }
    EC_LOG_RAW("\n");
}

static void ec_cmd_show_slave_detail(ec_master_t *master, uint32_t slave_idx)
{
    unsigned int port_idx;
    ec_slave_t *slave;
    ec_cmd_slave_info_t data;

    if (slave_idx >= master->slave_count) {
        EC_LOG_RAW("No slaves found\n");
        return;
    }

    slave = &master->slaves[slave_idx];

    ec_osal_mutex_take(master->scan_lock);
    ec_master_get_slave_info(slave, &data);
    ec_osal_mutex_give(master->scan_lock);

    EC_LOG_RAW("=== Master %d, Slave %d ===\n", master->index, slave_idx);

    if (data.alias != 0) {
        EC_LOG_RAW("Alias: 0x%04x\n", data.alias);
    }

    EC_LOG_RAW("Device: %s\n", master->netdev[slave->netdev_idx]->name);
    EC_LOG_RAW("State: %s\n", ec_slave_state_string(data.current_state));

    EC_LOG_RAW("Identity:\n");
    EC_LOG_RAW("  Vendor Id:       0x%08x\n", data.vendor_id);
    EC_LOG_RAW("  Product code:    0x%08x\n", data.product_code);
    EC_LOG_RAW("  Revision number: 0x%08x\n", data.revision_number);
    EC_LOG_RAW("  Serial number:   0x%08x\n", data.serial_number);

    EC_LOG_RAW("DL information:\n");
    EC_LOG_RAW("  FMMU bit operation: %s\n", (data.base_fmmu_bit_operation ? "yes" : "no"));
    EC_LOG_RAW("  Distributed clocks: ");

    if (data.base_dc_supported) {
        if (data.has_dc_system_time) {
            EC_LOG_RAW("yes, ");
            if (data.base_dc_range) {
                EC_LOG_RAW("64 bit\n");
            } else {
                EC_LOG_RAW("32 bit\n");
            }
            EC_LOG_RAW("  DC system time transmission delay: %d ns\n",
                       data.transmission_delay);
        } else {
            EC_LOG_RAW("yes, delay measurement only\n");
        }
    } else {
        EC_LOG_RAW("no\n");
    }

    EC_LOG_RAW("Port  Type  Link  Loop    Signal  NextSlave");
    if (data.base_dc_supported) {
        EC_LOG_RAW("  RxTime [ns]  Diff [ns]   NextDc [ns]");
    }
    EC_LOG_RAW("\n");

    for (port_idx = 0; port_idx < EC_MAX_PORTS; port_idx++) {
        EC_LOG_RAW("   %d  %-4s  %-4s  %-6s  %-6s  ",
                   port_idx,
                   ec_port_desc_string(data.ports[port_idx].desc),
                   (data.ports[port_idx].link.link_up ? "up" : "down"),
                   (data.ports[port_idx].link.loop_closed ? "closed" : "open"),
                   (data.ports[port_idx].link.signal_detected ? "yes" : "no"));

        if (data.ports[port_idx].next_slave != 0xffff) {
            EC_LOG_RAW("%-9d", data.ports[port_idx].next_slave);
        } else {
            EC_LOG_RAW("%-9s", "-");
        }

        if (data.base_dc_supported) {
            if (!data.ports[port_idx].link.loop_closed) {
                EC_LOG_RAW("  %11u  %10d  %10d",
                           data.ports[port_idx].receive_time,
                           data.ports[port_idx].receive_time - data.ports[0].receive_time,
                           data.ports[port_idx].delay_to_next_dc);
            } else {
                EC_LOG_RAW("  %11s  %10s  %10s", "-", "-", "-");
            }
        }

        EC_LOG_RAW("\n");
    }

    if (data.mailbox_protocols) {
        EC_LOG_RAW("Mailboxes:\n");
        EC_LOG_RAW("  Bootstrap RX: 0x%04x/%d, TX: 0x%04x/%d\n",
                   data.boot_rx_mailbox_offset,
                   data.boot_rx_mailbox_size,
                   data.boot_tx_mailbox_offset,
                   data.boot_tx_mailbox_size);
        EC_LOG_RAW("  Standard  RX: 0x%04x/%d, TX: 0x%04x/%d\n",
                   data.std_rx_mailbox_offset,
                   data.std_rx_mailbox_size,
                   data.std_tx_mailbox_offset,
                   data.std_tx_mailbox_size);

        EC_LOG_RAW("  Supported protocols: %s\n", ec_mbox_protocol_string(data.mailbox_protocols));
    }

    if (data.has_general) {
        EC_LOG_RAW("General:\n");
        EC_LOG_RAW("  Group: %s\n", data.group ? data.group : "");
        EC_LOG_RAW("  Image name: %s\n", data.image ? data.image : "");
        EC_LOG_RAW("  Order number: %s\n", data.order ? data.order : "");
        EC_LOG_RAW("  Device name: %s\n", data.name ? data.name : "");

        if (data.mailbox_protocols & EC_MBXPROT_COE) {
            EC_LOG_RAW("  CoE details:\n");
            EC_LOG_RAW("    Enable SDO: %s\n",
                       (data.coe_details.enable_sdo ? "yes" : "no"));
            EC_LOG_RAW("    Enable SDO Info: %s\n",
                       (data.coe_details.enable_sdo_info ? "yes" : "no"));
            EC_LOG_RAW("    Enable PDO Assign: %s\n",
                       (data.coe_details.enable_pdo_assign ? "yes" : "no"));
            EC_LOG_RAW("    Enable PDO Configuration: %s\n",
                       (data.coe_details.enable_pdo_configuration ? "yes" : "no"));
            EC_LOG_RAW("    Enable Upload at startup: %s\n",
                       (data.coe_details.enable_upload_at_startup ? "yes" : "no"));
            EC_LOG_RAW("    Enable SDO complete access: %s\n",
                       (data.coe_details.enable_sdo_complete_access ? "yes" : "no"));
        }

        EC_LOG_RAW("  Flags:\n");
        EC_LOG_RAW("    Enable SafeOp: %s\n",
                   (data.general_flags.enable_safeop ? "yes" : "no"));
        EC_LOG_RAW("    Enable notLRW: %s\n",
                   (data.general_flags.enable_not_lrw ? "yes" : "no"));
        EC_LOG_RAW("  Current consumption: %d mA\n", data.current_on_ebus);
    }

    EC_LOG_RAW("\n");
}

static void ec_cmd_show_slave_simple(ec_master_t *master, uint32_t slave_idx)
{
    ec_slave_t *slave;
    ec_cmd_slave_info_t data;

    if (slave_idx >= master->slave_count) {
        EC_LOG_RAW("No slaves found\n");
        return;
    }

    slave = &master->slaves[slave_idx];

    ec_osal_mutex_take(master->scan_lock);
    ec_master_get_slave_info(slave, &data);
    ec_osal_mutex_give(master->scan_lock);

    EC_LOG_RAW("%-3u  %u:%04x   %-13s %s\n",
               master->index,
               slave_idx,
               slave->autoinc_address,
               ec_state_string(slave->current_state, 0),
               ec_alstatus_string(slave->alstatus_code));
}

static void ec_cmd_show_slave_pdos(ec_master_t *master, uint32_t slave_idx)
{
    if (slave_idx >= master->slave_count) {
        EC_LOG_RAW("No slaves found\n");
        return;
    }

    EC_LOG_RAW("=== Master %d, Slave %d ===\n", master->index, slave_idx);

    for (uint8_t i = 0; i < master->slaves[slave_idx].sm_count; i++) {
        EC_LOG_RAW("SM%u: physaddr 0x%04x, length 0x%04x, control 0x%02x, enable %d\r\n",
                   i,
                   master->slaves[slave_idx].sm_info[i].physical_start_address,
                   master->slaves[slave_idx].sm_info[i].length,
                   master->slaves[slave_idx].sm_info[i].control,
                   master->slaves[slave_idx].sm_info[i].enable);

        if (master->slaves[slave_idx].sm_info[i].fmmu_enable) {
            // if sm is process data output, print rxpdo info
            if (master->slaves[slave_idx].sm_info[i].control & (1 << 2)) {
                for (uint16_t j = 0; j < master->slaves[slave_idx].sm_info[i].pdo_assign.count; j++) {
                    EC_LOG_RAW("\tRxPDO 0x%04x\r\n", master->slaves[slave_idx].sm_info[i].pdo_assign.entry[j]);

                    for (uint16_t k = 0; k < master->slaves[slave_idx].sm_info[i].pdo_mapping[j].count; k++) {
                        EC_LOG_RAW("\t\tPDO entry 0x%04x:0x%02x, bitlen 0x%02x\r\n",
                                   master->slaves[slave_idx].sm_info[i].pdo_mapping[j].entry[k] >> 16 & 0xffff,
                                   master->slaves[slave_idx].sm_info[i].pdo_mapping[j].entry[k] >> 8 & 0xff,
                                   master->slaves[slave_idx].sm_info[i].pdo_mapping[j].entry[k] >> 0 & 0xff);
                    }
                }
            } else {
                for (uint16_t j = 0; j < master->slaves[slave_idx].sm_info[i].pdo_assign.count; j++) {
                    EC_LOG_RAW("\tTxPDO 0x%04x\r\n", master->slaves[slave_idx].sm_info[i].pdo_assign.entry[j]);

                    for (uint16_t k = 0; k < master->slaves[slave_idx].sm_info[i].pdo_mapping[j].count; k++) {
                        EC_LOG_RAW("\t\tPDO entry 0x%04x:0x%02x, bitlen 0x%02x\r\n",
                                   master->slaves[slave_idx].sm_info[i].pdo_mapping[j].entry[k] >> 16 & 0xffff,
                                   master->slaves[slave_idx].sm_info[i].pdo_mapping[j].entry[k] >> 8 & 0xff,
                                   master->slaves[slave_idx].sm_info[i].pdo_mapping[j].entry[k] >> 0 & 0xff);
                    }
                }
            }
        }
    }
}

static void ec_cmd_slave_state_request(ec_master_t *master, uint32_t slave_idx, ec_slave_state_t state)
{
    if (slave_idx >= master->slave_count) {
        EC_LOG_RAW("No slaves found\n");
        return;
    }

    ec_slave_t *slave = &master->slaves[slave_idx];
    slave->requested_state = state;
}

static int hex_char_to_num(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    } else if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static int parse_hex_string(const char *hex_str, uint8_t *output, uint32_t max_len)
{
    const char *ptr = hex_str;
    uint32_t byte_count = 0;

    if (strlen(hex_str) >= 2 && hex_str[0] == '0' &&
        (hex_str[1] == 'x' || hex_str[1] == 'X')) {
        ptr = hex_str + 2;
    }

    uint32_t str_len = strlen(ptr);
    if (str_len % 2) {
        EC_LOG_RAW("Hex string length must be even\n");
        return -1;
    }

    while (*ptr && *(ptr + 1) && byte_count < max_len) {
        int high = hex_char_to_num(*ptr);
        int low = hex_char_to_num(*(ptr + 1));

        if (high < 0 || low < 0) {
            EC_LOG_RAW("Invalid hex character: %c%c\n", *ptr, *(ptr + 1));
            return -1;
        }

        output[byte_count++] = (uint8_t)((high << 4) | low);
        ptr += 2;
    }

    if (*ptr) {
        if (byte_count >= max_len) {
            EC_LOG_RAW("Hex string too long, maximum %u bytes\n", max_len);
            return -1;
        }
        EC_LOG_RAW("Incomplete hex pair at end of string\n");
        return -1;
    }

    return byte_count;
}

int ethercat(int argc, const char **argv)
{
    int ret;

    if (global_cmd_master == NULL) {
        EC_LOG_RAW("No master configured\n");
        return -1;
    }

    if (argc < 2) {
        ec_master_cmd_show_help();
        return -1;
    }

    if (strcmp(argv[1], "help") == 0) {
        ec_master_cmd_show_help();
        return 0;
    } else if (strcmp(argv[1], "master") == 0) {
        ec_master_cmd_master(global_cmd_master);
        return 0;
    } else if (strcmp(argv[1], "rescan") == 0) {
        global_cmd_master->rescan_request = true;
        return 0;
    } else if (strcmp(argv[1], "slaves") == 0) {
        // ethercat slaves
        if (argc == 2) {
            for (uint32_t i = 0; i < global_cmd_master->slave_count; i++) {
                ec_cmd_show_slave_simple(global_cmd_master, i);
            }
            return 0;
        } else if (argc >= 4 && strcmp(argv[2], "-p") == 0) {
            // ethercat slaves -p [x] [-v]
            int show_detail = 0;
            if (argc == 5) {
                if (argc == 5 && strcmp(argv[4], "-v") == 0) {
                    show_detail = 1;
                }
            }

            if (show_detail) {
                ec_cmd_show_slave_detail(global_cmd_master, atoi(argv[3]));
            } else {
                ec_cmd_show_slave_simple(global_cmd_master, atoi(argv[3]));
            }

            return 0;
        } else if (argc == 3 && strcmp(argv[2], "-v") == 0) {
            // ethercat slaves -v
            for (uint32_t i = 0; i < global_cmd_master->slave_count; i++) {
                ec_cmd_show_slave_detail(global_cmd_master, i);
            }

            return 0;
        } else {
        }
    } else if (strcmp(argv[1], "pdos") == 0) {
        // ethercat pdos
        if (argc == 2) {
            for (uint32_t i = 0; i < global_cmd_master->slave_count; i++) {
                ec_cmd_show_slave_pdos(global_cmd_master, i);
            }
            return 0;
        } else if (argc == 4 && strcmp(argv[2], "-p") == 0) {
            // ethercat pdos -p [x]
            ec_cmd_show_slave_pdos(global_cmd_master, atoi(argv[3]));
            return 0;
        } else {
        }
    } else if (argc >= 3 && strcmp(argv[1], "states") == 0) {
        // ethercat states
        if (argc == 3) {
            // ethercat states [state]
            for (uint32_t i = 0; i < global_cmd_master->slave_count; i++) {
                ec_cmd_slave_state_request(global_cmd_master, i, strtoul(argv[4], NULL, 16));
            }
            return 0;
        } else if (argc == 5 && strcmp(argv[2], "-p") == 0) {
            // ethercat states -p [x] [state]
            ec_cmd_slave_state_request(global_cmd_master, atoi(argv[3]), strtoul(argv[4], NULL, 16));
            return 0;
        } else {
        }
    } else if (argc >= 2 && strcmp(argv[1], "wc") == 0) {
        // ethercat wc
        if (argc == 2) {
            // ethercat wc
            EC_LOG_RAW("Master %d working counter(actual/expect): %u/%u\n",
                       global_cmd_master->index,
                       global_cmd_master->actual_working_counter,
                       global_cmd_master->expected_working_counter);

            for (uint32_t i = 0; i < global_cmd_master->slave_count; i++) {
                EC_LOG_RAW("%-3u  %u:%04x         (actual/expect): %u/%u\n",
                           global_cmd_master->index,
                           i,
                           global_cmd_master->slaves[i].autoinc_address,
                           global_cmd_master->slaves[i].actual_working_counter,
                           global_cmd_master->slaves[i].expected_working_counter);
            }
            return 0;
        } else {
        }
    } else if (argc >= 5 && strcmp(argv[1], "coe_read") == 0) {
        // ethercat coe_read -p [slave_idx] [index] [subindex]
        static ec_datagram_t datagram;
        static uint8_t output_buffer[512];
        uint32_t actual_size;

        uint32_t slave_idx = atoi(argv[3]);

        ec_datagram_init(&datagram, 1024);
        ret = ec_coe_upload(global_cmd_master,
                            slave_idx,
                            &datagram,
                            strtoul(argv[4], NULL, 16),
                            argc >= 6 ? strtoul(argv[5], NULL, 16) : 0x00,
                            output_buffer,
                            sizeof(output_buffer),
                            &actual_size,
                            argc >= 6 ? false : true);
        if (ret < 0) {
            EC_LOG_RAW("Slave %u coe_read failed: %d\n", slave_idx, ret);
        } else {
            ec_hexdump(output_buffer, actual_size);
        }
        ec_datagram_clear(&datagram);

        return 0;
    } else if (argc >= 7 && strcmp(argv[1], "coe_write") == 0) {
        // ethercat coe_write -p [slave_idx] [index] [subindex] [u32data]
        static ec_datagram_t datagram;
        uint32_t u32data;
        uint32_t size;

        uint32_t slave_idx = atoi(argv[3]);
        u32data = strtoul(argv[6], NULL, 16);

        if (u32data < 0xff)
            size = 1;
        else if (u32data < 0xffff)
            size = 2;
        else
            size = 4;

        ec_datagram_init(&datagram, 1024);
        ret = ec_coe_download(global_cmd_master,
                              slave_idx,
                              &datagram,
                              strtoul(argv[4], NULL, 16),
                              strtoul(argv[5], NULL, 16),
                              &u32data,
                              size,
                              false);
        if (ret < 0) {
            EC_LOG_RAW("Slave %u coe_write failed: %d\n", slave_idx, ret);
        } else {
            EC_LOG_RAW("Slave %u coe write success\n", slave_idx);
        }
        ec_datagram_clear(&datagram);
        return 0;
    } else if (argc >= 4 && strcmp(argv[1], "sii_read") == 0) {
        // ethercat sii_read -p [slave_idx]
        uint32_t slave_idx = atoi(argv[3]);
        if (slave_idx >= global_cmd_master->slave_count) {
            EC_LOG_RAW("No slaves found\n");
            return -1;
        }

        ec_hexdump(global_cmd_master->slaves[slave_idx].sii_image,
                   global_cmd_master->slaves[slave_idx].sii_nwords * 2);

        return 0;
    } else if (argc >= 5 && strcmp(argv[1], "sii_write") == 0) {
        // ethercat sii_write -p [slave_idx]
        static ec_datagram_t datagram;
        extern unsigned char cherryecat_eepromdata[2048];

        ec_datagram_init(&datagram, 1024);

        uint32_t slave_idx = atoi(argv[3]);

        ec_osal_mutex_take(global_cmd_master->scan_lock);
        ret = ec_sii_write(global_cmd_master, slave_idx, &datagram, 0x0000, (const uint16_t *)cherryecat_eepromdata, sizeof(cherryecat_eepromdata));
        ec_osal_mutex_give(global_cmd_master->scan_lock);

        if (ret < 0) {
            EC_LOG_RAW("Slave %u sii_write failed: %d\n", slave_idx, ret);
        } else {
            EC_LOG_RAW("Slave %u sii write success\n", slave_idx);
        }

        ec_datagram_clear(&datagram);
        return 0;
    } else if (argc >= 2 && strcmp(argv[1], "pdo_read") == 0) {
        // ethercat pdo_read
        if (argc == 2) {
            for (uint32_t slave_idx = 0; slave_idx < global_cmd_master->slave_count; slave_idx++) {
                uint8_t *buffer = ec_master_get_slave_domain_input(global_cmd_master, slave_idx);
                uint32_t data_size = ec_master_get_slave_domain_isize(global_cmd_master, slave_idx);
                EC_LOG_RAW("\r");
                for (uint32_t i = 0; i < data_size; i++) {
                    EC_LOG_RAW("%02x ", buffer[i]);
                }
                EC_LOG_RAW("\n");
            }
            return 0;
        } else if (argc == 4 && strcmp(argv[2], "-p") == 0) {
            // ethercat pdo_read -p [slave_idx]
            uint32_t slave_idx = atoi(argv[3]);
            if (slave_idx >= global_cmd_master->slave_count) {
                EC_LOG_RAW("No slaves found\n");
                return -1;
            }
            uint8_t *buffer = ec_master_get_slave_domain_input(global_cmd_master, slave_idx);
            uint32_t data_size = ec_master_get_slave_domain_isize(global_cmd_master, slave_idx);

            for (uint32_t count = 0; count < 10; count++) {
                EC_LOG_RAW("\r");
                for (uint32_t i = 0; i < data_size; i++) {
                    EC_LOG_RAW("%02x ", buffer[i]);
                }
                fflush(stdout);
                if (count < 9) {
                    ec_osal_msleep(1000);
                }
            }
            EC_LOG_RAW("\n");
            return 0;
        } else {
        }
    } else if (argc >= 4 && strcmp(argv[1], "pdo_write") == 0) {
        // ethercat pdo_write -p [slave_idx] [offset] [hexdata]
        uint8_t hexdata[256];
        uint32_t offset;
        int size;

        if (argc >= 6 && strcmp(argv[2], "-p") == 0) {
            uint32_t slave_idx = atoi(argv[3]);
            if (slave_idx >= global_cmd_master->slave_count) {
                EC_LOG_RAW("No slaves found\n");
                return -1;
            }

            offset = strtoul(argv[4], NULL, 16);

            size = parse_hex_string(argv[5], hexdata, sizeof(hexdata));
            if (size < 0) {
                return -1;
            }

            uint8_t *buffer = ec_master_get_slave_domain_output(global_cmd_master, slave_idx);
            if (buffer) {
                EC_LOG_RAW("Slave %u pdo write offset 0x%04x, size %u\n",
                           slave_idx, offset, size);
                ec_memcpy(&buffer[offset], hexdata, size);
            }
            return 0;
        } else {
            offset = strtoul(argv[2], NULL, 16);

            size = parse_hex_string(argv[3], hexdata, sizeof(hexdata));
            if (size < 0) {
                return -1;
            }

            EC_LOG_RAW("Slaves pdo write offset 0x%04x, size %u\n",
                       offset, size);

            for (uint32_t slave_idx = 0; slave_idx < global_cmd_master->slave_count; slave_idx++) {
                uint8_t *buffer = ec_master_get_slave_domain_output(global_cmd_master, slave_idx);
                if (buffer) {
                    ec_memcpy(&buffer[offset], hexdata, size);
                }
            }
            return 0;
        }
    }
#ifdef CONFIG_EC_FOE
    else if (argc >= 7 && strcmp(argv[1], "foe_write") == 0) {
        // ethercat foe_write -p [slave_idx] [filename] [password] [hexdata]
        uint8_t hexdata[256];
        uint32_t size;
        uint32_t slave_idx = atoi(argv[3]);
        const char *filename = argv[4];
        uint32_t password = strtoul(argv[5], NULL, 16);

        size = parse_hex_string(argv[6], hexdata, sizeof(hexdata));
        if (size < 0) {
            return -1;
        }
        static ec_datagram_t datagram;

        ec_datagram_init(&datagram, 1024);

        EC_LOG_RAW("Slave %u foe write file %s, password: 0x%08x, size %u\n", slave_idx, filename, password, size);

        ec_osal_mutex_take(global_cmd_master->scan_lock);
        ret = ec_foe_write(global_cmd_master, slave_idx, &datagram, filename, password, hexdata, size);
        ec_osal_mutex_give(global_cmd_master->scan_lock);

        if (ret < 0) {
            EC_LOG_RAW("Slave %u foe_write failed: %d\n", slave_idx, ret);
        } else {
            EC_LOG_RAW("Slave %u foe write success\n", slave_idx);
        }

        ec_datagram_clear(&datagram);
        return 0;
    } else if (argc >= 6 && strcmp(argv[1], "foe_read") == 0) {
        // ethercat foe_read -p [slave_idx] [filename] [password]
        uint8_t hexdata[256];
        uint32_t size;
        uint32_t slave_idx = atoi(argv[3]);
        const char *filename = argv[4];
        uint32_t password = strtoul(argv[5], NULL, 16);

        EC_LOG_RAW("Slave %u foe read file %s, password: 0x%08x\n", slave_idx, filename, password);

        static ec_datagram_t datagram;

        ec_datagram_init(&datagram, 1024);

        ec_osal_mutex_take(global_cmd_master->scan_lock);
        ret = ec_foe_read(global_cmd_master, slave_idx, &datagram, filename, password, hexdata, sizeof(hexdata), &size);
        ec_osal_mutex_give(global_cmd_master->scan_lock);

        if (ret < 0) {
            EC_LOG_RAW("Slave %u foe_read failed: %d\n", slave_idx, ret);
        } else {
            ec_hexdump(hexdata, size);
        }

        ec_datagram_clear(&datagram);
        return 0;
    }
#endif
#ifdef CONFIG_EC_EOE
    else if (argc >= 4 && strcmp(argv[1], "eoe_start") == 0) {
        // ethercat eoe_start -p [slave_idx]
        struct ec_eoe_ip_param slave_ip_param = { 0 };
        struct ec_eoe_ip_param master_ip_param = { 0 };

        slave_ip_param.ipv4_addr_str = "192.168.100.10";
        slave_ip_param.ipv4_mask_str = "255.255.255.0";
        slave_ip_param.ipv4_gw_str = "192.168.100.1";
        slave_ip_param.dns_server_str = "192.168.2.1";

        for (uint8_t i = 0; i < 6; i++) {
            slave_ip_param.mac_addr[i] = i + 0xf0;
        }

        master_ip_param.ipv4_addr_str = "192.168.100.8";
        master_ip_param.ipv4_mask_str = "255.255.255.0";
        master_ip_param.ipv4_gw_str = "192.168.100.1";

        for (uint8_t i = 0; i < 6; i++) {
            master_ip_param.mac_addr[i] = i;
        }
        ret = ec_eoe_start(global_cmd_eoe, global_cmd_master, atoi(argv[3]), &master_ip_param, &slave_ip_param);
        if (ret < 0) {
            EC_LOG_RAW("eoe start failed: %d\n", ret);
        } else {
            EC_LOG_RAW("eoe start success\n");
        }
        return 0;
    }
#endif
    else if (argc >= 3 && strcmp(argv[1], "timediff") == 0) {
        if (strcmp(argv[2], "-s") == 0) {
            uintptr_t flags;

            flags = ec_osal_enter_critical_section();
            global_cmd_master->systime_diff_enable = true;
            global_cmd_master->curr_systime_diff = 0;
            global_cmd_master->min_systime_diff = 0xffffffff;
            global_cmd_master->max_systime_diff = 0;
            global_cmd_master->systime_diff_count = 0;
            global_cmd_master->total_systime_diff = 0;
            ec_osal_leave_critical_section(flags);

        } else if (strcmp(argv[2], "-d") == 0) {
            uintptr_t flags;

            flags = ec_osal_enter_critical_section();
            global_cmd_master->systime_diff_enable = false;
            ec_osal_leave_critical_section(flags);
        } else if (strcmp(argv[2], "-v") == 0) {
            for (uint32_t i = 0; i < 10; i++) {
                EC_LOG_RAW("System Time Diff curr = %u, min = %u, max = %u, avg = %u ns\n",
                           global_cmd_master->curr_systime_diff,
                           global_cmd_master->min_systime_diff,
                           global_cmd_master->max_systime_diff,
                           (unsigned int)(global_cmd_master->total_systime_diff / global_cmd_master->systime_diff_count));
                ec_osal_msleep(1000);
            }
        }
        return 0;
    } else if (strcmp(argv[1], "perf") == 0) {
        if (strcmp(argv[2], "-s") == 0) {
            uintptr_t flags;

            flags = ec_osal_enter_critical_section();
            global_cmd_master->perf_enable = true;
            global_cmd_master->min_period_ns = 0xffffffff;
            global_cmd_master->max_period_ns = 0;
            global_cmd_master->total_period_ns = 0;
            global_cmd_master->period_count = 0;

            global_cmd_master->min_exec_ns = 0xffffffff;
            global_cmd_master->max_exec_ns = 0;
            global_cmd_master->total_exec_ns = 0;
            global_cmd_master->exec_count = 0;

            global_cmd_master->min_offset_ns = 0xffffffff;
            global_cmd_master->max_offset_ns = 0;
            ec_osal_leave_critical_section(flags);
            return 0;
        } else if (strcmp(argv[2], "-d") == 0) {
            uintptr_t flags;

            flags = ec_osal_enter_critical_section();
            global_cmd_master->perf_enable = false;
            ec_osal_leave_critical_section(flags);
            return 0;
        } else if (strcmp(argv[2], "-v") == 0) {
            for (uint32_t i = 0; i < 10; i++) {
                EC_LOG_RAW("Period min = %10u, max = %10u, avg = %10u ns\n",
                           global_cmd_master->min_period_ns,
                           global_cmd_master->max_period_ns,
                           (unsigned int)(global_cmd_master->total_period_ns / global_cmd_master->period_count));
                EC_LOG_RAW("Exec   min = %10u, max = %10u, avg = %10u ns\n",
                           global_cmd_master->min_exec_ns,
                           global_cmd_master->max_exec_ns,
                           (unsigned int)(global_cmd_master->total_exec_ns / global_cmd_master->exec_count));

                EC_LOG_RAW("Offset min = %10d, max = %10d ns\n",
                           global_cmd_master->min_offset_ns,
                           global_cmd_master->max_offset_ns);

                ec_osal_msleep(1000);
            }
            return 0;
        }
    } else {
    }

    EC_LOG_RAW("Invalid command: %s\n", argv[1]);
    ec_master_cmd_show_help();
    return -1;
}

#ifdef FINSH_USING_MSH
#include <finsh.h>
MSH_CMD_EXPORT(ethercat, cherryecat command line tool);
#endif

#endif