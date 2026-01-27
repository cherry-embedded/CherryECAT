/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_master.h"

#ifdef CONFIG_EC_CMD_ENABLE

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

__WEAK unsigned char cherryecat_eepromdata[2048]; // EEPROM data buffer, please generate by esi_parse.py

__WEAK void ec_pdo_callback(ec_slave_t *slave, uint8_t *output, uint8_t *input)
{
}

static void ec_master_cmd_show_help(void)
{
    EC_LOG_RAW("CherryECAT " CHERRYECAT_VERSION_STR " Command Line Tool\n\n");
    EC_LOG_RAW("Usage: ethercat <command> [options]\n");
    EC_LOG_RAW("Commands:\n");
    EC_LOG_RAW("  start <cyclic_time_us> <cia402_mode>           Start master pdo timer\n");
    EC_LOG_RAW("  stop                                           Stop master pdo timer\n");
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
    EC_LOG_RAW("  help                                           Show this help\n\n");
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

void ec_master_cmd_master(ec_master_t *master)
{
    unsigned int dev_idx, j;
    uint64_t lost;
    int colwidth = 8;
    ec_netdev_stats_t netdev_stats;
    uintptr_t flags;

    EC_LOG_RAW("Master%d\n", master->index);
    EC_LOG_RAW("  Phase: ");
    switch (master->phase) {
        case 0:
            EC_LOG_RAW("Waiting for device attached...\n");
            break;
        case 1:
            EC_LOG_RAW("Idle\n");
            break;
        case 2:
            EC_LOG_RAW("Operation\n");
            break;
        default:
            EC_LOG_RAW("Unknown\n");
            break;
    }

    EC_LOG_RAW("  Slaves: %u\n", master->slave_count);
    EC_LOG_RAW("  Ethernet net devices:\n");

    for (dev_idx = EC_NETDEV_MAIN; dev_idx < CONFIG_EC_MAX_NETDEVS; dev_idx++) {
        EC_LOG_RAW("    %s: ", dev_idx == EC_NETDEV_MAIN ? "Main" : "Backup");
        EC_LOG_RAW("%02x:%02x:%02x:%02x:%02x:%02x\n",
                   master->netdev[dev_idx]->mac_addr[0],
                   master->netdev[dev_idx]->mac_addr[1],
                   master->netdev[dev_idx]->mac_addr[2],
                   master->netdev[dev_idx]->mac_addr[3],
                   master->netdev[dev_idx]->mac_addr[4],
                   master->netdev[dev_idx]->mac_addr[5]);
        EC_LOG_RAW("      Link: %s\n", master->netdev[dev_idx]->link_state ? "UP" : "DOWN");

        flags = ec_osal_enter_critical_section();
        memcpy(&netdev_stats, &master->netdev[dev_idx]->stats, sizeof(ec_netdev_stats_t));
        ec_osal_leave_critical_section(flags);

        EC_LOG_RAW("      Tx frames:   %llu\n", netdev_stats.tx_count);
        EC_LOG_RAW("      Tx bytes:    %llu\n", netdev_stats.tx_bytes);
        EC_LOG_RAW("      Rx frames:   %llu\n", netdev_stats.rx_count);
        EC_LOG_RAW("      Rx bytes:    %llu\n", netdev_stats.rx_bytes);
        EC_LOG_RAW("      Tx errors:   %llu\n", netdev_stats.tx_errors);

        lost = netdev_stats.loss_count;
        if (lost == 1)
            lost = 0;
        EC_LOG_RAW("      Lost frames: %llu\n", lost);

        EC_LOG_RAW("      Tx frame rate [1/s]: ");
        for (j = 0; j < EC_RATE_COUNT; j++) {
            EC_LOG_RAW("%*.*f", colwidth, 0, netdev_stats.tx_frame_rates[j] / 1000.0);
            if (j < EC_RATE_COUNT - 1)
                EC_LOG_RAW(" ");
        }
        EC_LOG_RAW("\n      Tx rate [KByte/s]:   ");
        for (j = 0; j < EC_RATE_COUNT; j++) {
            EC_LOG_RAW("%*.*f", colwidth, 1, netdev_stats.tx_byte_rates[j] / 1024.0);
            if (j < EC_RATE_COUNT - 1)
                EC_LOG_RAW(" ");
        }
        EC_LOG_RAW("\n      Rx frame rate [1/s]: ");
        for (j = 0; j < EC_RATE_COUNT; j++) {
            EC_LOG_RAW("%*.*f", colwidth, 0, netdev_stats.rx_frame_rates[j] / 1000.0);
            if (j < EC_RATE_COUNT - 1)
                EC_LOG_RAW(" ");
        }
        EC_LOG_RAW("\n      Rx rate [KByte/s]:   ");
        for (j = 0; j < EC_RATE_COUNT; j++) {
            EC_LOG_RAW("%*.*f", colwidth, 1, netdev_stats.rx_byte_rates[j] / 1024.0);
            if (j < EC_RATE_COUNT - 1)
                EC_LOG_RAW(" ");
        }

        EC_LOG_RAW("\n      Loss rate [1/s]:     ");
        for (j = 0; j < EC_RATE_COUNT; j++) {
            EC_LOG_RAW("%*.*f", colwidth, 0, netdev_stats.loss_rates[j] / 1000.0);
            if (j < EC_RATE_COUNT - 1)
                EC_LOG_RAW(" ");
        }

        EC_LOG_RAW("\n");
    }
}

static void ec_cmd_show_slave_detail(ec_master_t *master, uint32_t slave_idx)
{
    unsigned int port_idx;
    ec_slave_t *slave;
    ec_slave_t slave_data;

    if (slave_idx >= master->slave_count) {
        EC_LOG_RAW("No slaves found\n");
        return;
    }

    slave = &master->slaves[slave_idx];

    ec_osal_mutex_take(master->scan_lock);
    memcpy(&slave_data, slave, sizeof(ec_slave_t));
    ec_osal_mutex_give(master->scan_lock);

    EC_LOG_RAW("=== Master %d, Slave %d ===\n", master->index, slave_idx);

    if (slave_data.alias_address != 0) {
        EC_LOG_RAW("Alias: 0x%04x\n", slave_data.alias_address);
    }

    EC_LOG_RAW("Device: %s\n", master->netdev[slave->netdev_idx]->name);
    EC_LOG_RAW("State: %s\n", ec_state_string(slave_data.current_state, 0));

    EC_LOG_RAW("Identity:\n");
    EC_LOG_RAW("  Vendor Id:       0x%08x\n", slave_data.sii.vendor_id);
    EC_LOG_RAW("  Product code:    0x%08x\n", slave_data.sii.product_code);
    EC_LOG_RAW("  Revision number: 0x%08x\n", slave_data.sii.revision_number);
    EC_LOG_RAW("  Serial number:   0x%08x\n", slave_data.sii.serial_number);

    EC_LOG_RAW("DL information:\n");
    EC_LOG_RAW("  FMMU bit operation: %s\n", (slave_data.base_fmmu_bit_operation ? "yes" : "no"));
    EC_LOG_RAW("  Distributed clocks: ");

    if (slave_data.base_dc_supported) {
        EC_LOG_RAW("yes, ");
        if (slave_data.base_dc_range) {
            EC_LOG_RAW("64 bit\n");
        } else {
            EC_LOG_RAW("32 bit\n");
        }
        EC_LOG_RAW("  DC system time offset: %lld ns\n",
                   slave_data.system_time_offset);
        EC_LOG_RAW("  DC system time transmission delay: %d ns\n",
                   slave_data.transmission_delay);

    } else {
        EC_LOG_RAW("no\n");
    }

    EC_LOG_RAW("Port  Type  Link  Loop    Signal  NextSlave");
    if (slave_data.base_dc_supported) {
        EC_LOG_RAW("  RxTime [ns]  Diff [ns]   NextDc [ns]");
    }
    EC_LOG_RAW("\n");

    for (port_idx = 0; port_idx < EC_MAX_PORTS; port_idx++) {
        EC_LOG_RAW("   %d  %-4s  %-4s  %-6s  %-6s  ",
                   port_idx,
                   ec_port_desc_string(slave_data.ports[port_idx].desc),
                   (slave_data.ports[port_idx].link.link_up ? "up" : "down"),
                   (slave_data.ports[port_idx].link.loop_closed ? "closed" : "open"),
                   (slave_data.ports[port_idx].link.signal_detected ? "yes" : "no"));

        if (slave_data.ports[port_idx].next_slave) {
            EC_LOG_RAW("%-9d", slave_data.ports[port_idx].next_slave->autoinc_address);
        } else {
            EC_LOG_RAW("%-9s", "-");
        }

        if (slave_data.base_dc_supported) {
            if (!slave_data.ports[port_idx].link.loop_closed) {
                EC_LOG_RAW("  %11u  %10d  %10d",
                           slave_data.ports[port_idx].receive_time,
                           slave_data.ports[port_idx].receive_time - slave_data.ports[0].receive_time,
                           slave_data.ports[port_idx].delay_to_next_dc);
            } else {
                EC_LOG_RAW("  %11s  %10s  %10s", "-", "-", "-");
            }
        }

        EC_LOG_RAW("\n");
    }

    if (slave_data.sii.mailbox_protocols) {
        EC_LOG_RAW("Mailboxes:\n");
        EC_LOG_RAW("  Bootstrap RX: 0x%04x/%d, TX: 0x%04x/%d\n",
                   slave_data.sii.boot_rx_mailbox_offset,
                   slave_data.sii.boot_rx_mailbox_size,
                   slave_data.sii.boot_tx_mailbox_offset,
                   slave_data.sii.boot_tx_mailbox_size);
        EC_LOG_RAW("  Standard  RX: 0x%04x/%d, TX: 0x%04x/%d\n",
                   slave_data.sii.std_rx_mailbox_offset,
                   slave_data.sii.std_rx_mailbox_size,
                   slave_data.sii.std_tx_mailbox_offset,
                   slave_data.sii.std_tx_mailbox_size);

        EC_LOG_RAW("  Supported protocols: %s\n", ec_mbox_protocol_string(slave_data.sii.mailbox_protocols));
    }

    if (slave_data.sii.has_general) {
        EC_LOG_RAW("General:\n");
        EC_LOG_RAW("  Group: %s\n", slave->sii.general.groupidx ? ec_slave_get_sii_string(slave, slave->sii.general.groupidx) : "");
        EC_LOG_RAW("  Image name: %s\n", slave->sii.general.imgidx ? ec_slave_get_sii_string(slave, slave->sii.general.imgidx) : "");
        EC_LOG_RAW("  Order number: %s\n", slave->sii.general.orderidx ? ec_slave_get_sii_string(slave, slave->sii.general.orderidx) : "");
        EC_LOG_RAW("  Device name: %s\n", slave->sii.general.nameidx ? ec_slave_get_sii_string(slave, slave->sii.general.nameidx) : "");

        if (slave_data.sii.mailbox_protocols & EC_MBXPROT_COE) {
            EC_LOG_RAW("  CoE details:\n");
            EC_LOG_RAW("    Enable SDO: %s\n",
                       (slave_data.sii.general.coe_details.enable_sdo ? "yes" : "no"));
            EC_LOG_RAW("    Enable SDO Info: %s\n",
                       (slave_data.sii.general.coe_details.enable_sdo_info ? "yes" : "no"));
            EC_LOG_RAW("    Enable PDO Assign: %s\n",
                       (slave_data.sii.general.coe_details.enable_pdo_assign ? "yes" : "no"));
            EC_LOG_RAW("    Enable PDO Configuration: %s\n",
                       (slave_data.sii.general.coe_details.enable_pdo_configuration ? "yes" : "no"));
            EC_LOG_RAW("    Enable Upload at startup: %s\n",
                       (slave_data.sii.general.coe_details.enable_upload_at_startup ? "yes" : "no"));
            EC_LOG_RAW("    Enable SDO complete access: %s\n",
                       (slave_data.sii.general.coe_details.enable_sdo_complete_access ? "yes" : "no"));
        }

        EC_LOG_RAW("  Flags:\n");
        EC_LOG_RAW("    Enable SafeOp: %s\n",
                   (slave_data.sii.general.flags.enable_safeop ? "yes" : "no"));
        EC_LOG_RAW("    Enable notLRW: %s\n",
                   (slave_data.sii.general.flags.enable_not_lrw ? "yes" : "no"));
        EC_LOG_RAW("  Current consumption: %d mA\n", slave_data.sii.general.current_on_ebus);
    }

    EC_LOG_RAW("\n");
}

static void ec_cmd_show_slave_simple(ec_master_t *master, uint32_t slave_idx)
{
    ec_slave_t *slave;

    if (slave_idx >= master->slave_count) {
        EC_LOG_RAW("No slaves found\n");
        return;
    }

    slave = &master->slaves[slave_idx];

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
    } else if (strcmp(argv[1], "start") == 0) {
        static ec_slave_config_t slave_config[32];
        uint8_t motor_mode;

        if (argc == 4) {
            motor_mode = atoi(argv[3]);
        } else {
            motor_mode = 0;
        }

        for (uint32_t i = 0; i < global_cmd_master->slave_count; i++) {
            ret = ec_master_find_slave_sync_info(global_cmd_master->slaves[i].sii.vendor_id,
                                                 global_cmd_master->slaves[i].sii.product_code,
                                                 global_cmd_master->slaves[i].sii.revision_number,
                                                 motor_mode,
                                                 &slave_config[i].sync,
                                                 &slave_config[i].sync_count);
            if (ret != 0) {
                EC_LOG_ERR("Failed to find slave sync info: vendor_id=0x%08x, product_code=0x%08x\n",
                           global_cmd_master->slaves[i].sii.vendor_id,
                           global_cmd_master->slaves[i].sii.product_code);
                return -1;
            }

            slave_config[i].dc_assign_activate = 0x300;

            slave_config[i].dc_sync[0].cycle_time = atoi(argv[2]) * 1000;
            slave_config[i].dc_sync[0].shift_time = 1000000;
            slave_config[i].dc_sync[1].cycle_time = 0;
            slave_config[i].dc_sync[1].shift_time = 0;
            slave_config[i].pdo_callback = ec_pdo_callback;

            global_cmd_master->slaves[i].config = &slave_config[i];
        }

        global_cmd_master->cycle_time = atoi(argv[2]) * 1000;       // cycle time in ns
        global_cmd_master->shift_time = atoi(argv[2]) * 1000 * 0.2; // 20% shift time in ns
        global_cmd_master->dc_sync_with_dc_ref_enable = true;       // enable DC sync with dc reference clock
        ec_master_start(global_cmd_master);
        return 0;
    } else if (strcmp(argv[1], "stop") == 0) {
        ec_master_stop(global_cmd_master);
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
#ifdef CONFIG_EC_PDO_MULTI_DOMAIN
            for (uint32_t i = 0; i < global_cmd_master->slave_count; i++) {
                EC_LOG_RAW("%-3u  %u:%04x         (actual/expect): %u/%u\n",
                           global_cmd_master->index,
                           i,
                           global_cmd_master->slaves[i].autoinc_address,
                           global_cmd_master->slaves[i].actual_working_counter,
                           global_cmd_master->slaves[i].expected_working_counter);
            }
#endif
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
    else if (strcmp(argv[1], "perf") == 0) {
        if (strcmp(argv[2], "-s") == 0) {
            uintptr_t flags;

            flags = ec_osal_enter_critical_section();
            global_cmd_master->perf_enable = true;
            global_cmd_master->min_period_ns = 0xffffffff;
            global_cmd_master->max_period_ns = 0;
            global_cmd_master->total_period_ns = 0;
            global_cmd_master->period_count = 0;

            global_cmd_master->min_send_exec_ns = 0xffffffff;
            global_cmd_master->max_send_exec_ns = 0;
            global_cmd_master->total_send_exec_ns = 0;
            global_cmd_master->send_exec_count = 0;
            global_cmd_master->min_recv_exec_ns = 0xffffffff;
            global_cmd_master->max_recv_exec_ns = 0;
            global_cmd_master->total_recv_exec_ns = 0;
            global_cmd_master->recv_exec_count = 0;

            global_cmd_master->min_offset_ns = INT32_MAX;
            global_cmd_master->max_offset_ns = INT32_MIN;
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
                EC_LOG_RAW("Period    min = %10u, max = %10u, avg = %10u ns\n",
                           global_cmd_master->min_period_ns,
                           global_cmd_master->max_period_ns,
                           (unsigned int)(global_cmd_master->total_period_ns / global_cmd_master->period_count));
                EC_LOG_RAW("Send exec min = %10u, max = %10u, avg = %10u ns\n",
                           global_cmd_master->min_send_exec_ns,
                           global_cmd_master->max_send_exec_ns,
                           (unsigned int)(global_cmd_master->total_send_exec_ns / global_cmd_master->send_exec_count));
                EC_LOG_RAW("Recv exec min = %10u, max = %10u, avg = %10u ns\n",
                           global_cmd_master->min_recv_exec_ns,
                           global_cmd_master->max_recv_exec_ns,
                           (unsigned int)(global_cmd_master->total_recv_exec_ns / global_cmd_master->recv_exec_count));

                EC_LOG_RAW("Offset    min = %10d, max = %10d ns\n",
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
#endif
