/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_master.h"

// HPM IO, VID 0x0048504D, PID 0x00000001
static ec_pdo_entry_info_t hpm_dio_output_pdo_entries[] = {
    { 0x6000, 0x00, 0x20 },
};

static ec_pdo_entry_info_t hpm_dio_input_pdo_entries[] = {
    { 0x7010, 0x00, 0x20 },
};

static ec_pdo_info_t hpm_dio_pdos[] = {
    { 0x1600, 1, hpm_dio_output_pdo_entries },
    { 0x1a00, 1, hpm_dio_input_pdo_entries },

};

static ec_sync_info_t hpm_dio_syncs[] = {
    { 2, EC_DIR_OUTPUT, 1, &hpm_dio_pdos[0], EC_WD_DISABLE },
    { 3, EC_DIR_INPUT, 1, &hpm_dio_pdos[1], EC_WD_DISABLE },
};

// HPM CIA402 CSV, VID 0x0048504D, PID 0x00000003/0x00000004
static ec_pdo_entry_info_t hpm_cia402_csv_output_pdo_entries[] = {
    { 0x6040, 0x00, 0x10 }, // Control Word
    { 0x60ff, 0x00, 0x20 }, // Target velocity
    { 0x0000, 0x00, 0x10 }, // Padding
};

static ec_pdo_entry_info_t hpm_cia402_csv_input_pdo_entries[] = {
    { 0x6041, 0x00, 0x10 }, // Status Word
    { 0x6064, 0x00, 0x20 }, // Actual Position
    { 0x0000, 0x00, 0x10 }, // Padding
};

static ec_pdo_info_t hpm_cia402_csv_pdos[] = {
    { 0x1602, 3, hpm_cia402_csv_output_pdo_entries },
    { 0x1a02, 3, hpm_cia402_csv_input_pdo_entries },

};

static ec_sync_info_t hpm_cia402_csv_syncs[] = {
    { 2, EC_DIR_OUTPUT, 1, &hpm_cia402_csv_pdos[0] },
    { 3, EC_DIR_INPUT, 1, &hpm_cia402_csv_pdos[1] },
};

// HPM CIA402 CSP, VID 0x0048504D, PID 0x00000003/0x00000004
static ec_pdo_entry_info_t hpm_cia402_csp_output_pdo_entries[] = {
    { 0x6040, 0x00, 0x10 }, // Control Word
    { 0x607A, 0x00, 0x20 }, // Target position
    { 0x0000, 0x00, 0x10 }, // Padding
};

static ec_pdo_entry_info_t hpm_cia402_csp_input_pdo_entries[] = {
    { 0x6041, 0x00, 0x10 }, // Status Word
    { 0x6064, 0x00, 0x20 }, // Actual Position
    { 0x0000, 0x00, 0x10 }, // Padding
};

static ec_pdo_info_t hpm_cia402_csp_pdos[] = {
    { 0x1601, 3, hpm_cia402_csp_output_pdo_entries },
    { 0x1a01, 3, hpm_cia402_csp_input_pdo_entries },

};

static ec_sync_info_t hpm_cia402_csp_syncs[] = {
    { 2, EC_DIR_OUTPUT, 1, &hpm_cia402_csp_pdos[0], EC_WD_DISABLE },
    { 3, EC_DIR_INPUT, 1, &hpm_cia402_csp_pdos[1], EC_WD_DISABLE },
};

// EM32DX-E4-V30, VID 0x00004321, PID 0x01100073, REV 0x18050210
static ec_pdo_entry_info_t em32dx_4e_v30_output_pdo_entries[] = {
    { 0x6000, 0x01, 0x10 },
};

static ec_pdo_entry_info_t em32dx_4e_v30_input_pdo_entries[] = {
    { 0x7000, 0x01, 0x10 },
};

static ec_pdo_info_t em32dx_4e_v30_pdos[] = {
    { 0x1600, 1, em32dx_4e_v30_output_pdo_entries },
    { 0x1a00, 1, em32dx_4e_v30_input_pdo_entries },

};

static ec_sync_info_t em32dx_4e_v30_syncs[] = {
    { 2, EC_DIR_OUTPUT, 1, &em32dx_4e_v30_pdos[0], EC_WD_DISABLE },
    { 3, EC_DIR_INPUT, 1, &em32dx_4e_v30_pdos[1], EC_WD_DISABLE },
};

// SG-ELC IO, VID 0xEE000002, PID 0x00010204, REV 0x00000001
static ec_pdo_entry_info_t sg_elc_output_pdo_entries[] = {
    { 0x3101, 0x01, 0x08 },
    { 0x3101, 0x02, 0x08 },
};

static ec_pdo_entry_info_t sg_elc_input_pdo_entries[] = {
    { 0x3001, 0x01, 0x08 },
    { 0x3001, 0x02, 0x08 },
};

static ec_pdo_info_t sg_elc_pdos[] = {
    { 0x1600, 1, &sg_elc_output_pdo_entries[0] },
    { 0x1601, 1, &sg_elc_output_pdo_entries[1] },
    { 0x1a00, 1, &sg_elc_input_pdo_entries[0] },
    { 0x1a01, 1, &sg_elc_input_pdo_entries[1] },
};

static ec_sync_info_t sg_elc_syncs[] = {
    { 0, EC_DIR_OUTPUT, 1, &sg_elc_pdos[0], EC_WD_DISABLE },
    { 1, EC_DIR_OUTPUT, 1, &sg_elc_pdos[1], EC_WD_DISABLE },
    { 2, EC_DIR_INPUT, 2, &sg_elc_pdos[2], EC_WD_DISABLE },
};

// Inovance SV660N CSP, VID 0x00100000, PID 0x000C010D, REV 0x00010000
static ec_pdo_entry_info_t sv660n_csp_output_pdo_entries[] = {
    { 0x6060, 0x00, 8 },  // 6060h(mode)
    { 0x6040, 0x00, 16 }, // 6040h(control)
    { 0x607A, 0x00, 32 }, // 607Ah(dest position)
    { 0x60FF, 0x00, 32 }, // 60FFh(dest speed)
    { 0x6071, 0x00, 16 }, // 6071h(dest torque)
};

static ec_pdo_entry_info_t sv660n_csp_input_pdo_entries[] = {
    { 0x603F, 0x00, 16 }, // 603Fh(error)
    { 0x6041, 0x00, 16 }, // 6041h(status)
    { 0x6064, 0x00, 32 }, // 6064h(current postion)
    { 0x606C, 0x00, 32 }, // 606Ch(current speed)
    { 0x6077, 0x00, 16 }, // 6077h(current torque)
};

static ec_pdo_info_t sv660n_csp_pdos[] = {
    { 0x1600, 5, sv660n_csp_output_pdo_entries },
    { 0x1a00, 5, sv660n_csp_input_pdo_entries },
};

static ec_sync_info_t sv660n_csp_syncs[] = {
    { 2, EC_DIR_OUTPUT, 1, &sv660n_csp_pdos[0], EC_WD_DISABLE },
    { 3, EC_DIR_INPUT, 1, &sv660n_csp_pdos[1], EC_WD_DISABLE },
};

// Inovance SV660N CSV
static ec_pdo_entry_info_t sv660n_csv_output_pdo_entries[] = {
    { 0x6040, 0x00, 16 }, // 0x6040 控制字
    { 0x60FF, 0x00, 32 }, // 0x60FF 目标速度
    { 0x6060, 0x00, 8 },  // 0x6060 模式选择 (CSV=9)
};

static ec_pdo_entry_info_t sv660n_csv_input_pdo_entries[] = {
    { 0x6041, 0x00, 16 }, // 0x6041 状态字
    { 0x6064, 0x00, 32 }, // 0x6064 实际位置
    { 0x606C, 0x00, 32 }, // 0x606C 实际速度
    { 0x6061, 0x00, 8 },  // 0x6061 运行模式显示
};

static ec_pdo_info_t sv660n_csv_pdos[] = {
    { 0x1600, 3, sv660n_csv_output_pdo_entries },
    { 0x1a00, 4, sv660n_csv_input_pdo_entries },
};

static ec_sync_info_t sv660n_csv_syncs[] = {
    { 2, EC_DIR_OUTPUT, 1, &sv660n_csv_pdos[0], EC_WD_DISABLE },
    { 3, EC_DIR_INPUT, 1, &sv660n_csv_pdos[1], EC_WD_DISABLE },
};

// Inovance SV660N CST
static ec_pdo_entry_info_t sv660n_cst_output_pdo_entries[] = {
    { 0x6040, 0x00, 16 }, // 0x6040 控制字
    { 0x6071, 0x00, 16 }, // 0x6071 目标转矩
    { 0x60B2, 0x00, 16 }, // 0x60B2 转矩偏置
    { 0x6060, 0x00, 8 },  // 0x6060 模式选择 (CST=10)
    { 0x6072, 0x00, 16 }, // 0x6072 最大允许转矩 (百分比/额定)
    { 0x607F, 0x00, 32 }, // 0x607F 最大转速 (rpm)
    { 0x60E0, 0x00, 32 }, // 0x60E0 正向转矩限制
    { 0x60E1, 0x00, 32 }, // 0x60E1 反向转矩限制
};

static ec_pdo_entry_info_t sv660n_cst_input_pdo_entries[] = {
    { 0x6041, 0x00, 16 }, // 0x6041 状态字
    { 0x6064, 0x00, 32 }, // 0x6064 实际位置
    { 0x606C, 0x00, 32 }, // 0x606C 实际速度
    { 0x6077, 0x00, 16 }, // 0x6077 实际转矩
    { 0x6061, 0x00, 8 },  // 0x6061 运行模式显示
};

static ec_pdo_info_t sv660n_cst_pdos[] = {
    { 0x1600, 8, sv660n_cst_output_pdo_entries },
    { 0x1a00, 5, sv660n_cst_input_pdo_entries },
};

static ec_sync_info_t sv660n_cst_syncs[] = {
    { 2, EC_DIR_OUTPUT, 1, &sv660n_cst_pdos[0], EC_WD_DISABLE },
    { 3, EC_DIR_INPUT, 1, &sv660n_cst_pdos[1], EC_WD_DISABLE },
};

int ec_master_find_slave_sync_info(uint32_t vendor_id,
                                   uint32_t product_code,
                                   uint32_t revision_number,
                                   uint8_t cia402_mode,
                                   ec_sync_info_t **syncs,
                                   uint8_t *sync_count)
{
    switch (vendor_id) {
        case 0x0048504D: // HPMicro
            switch (product_code) {
                case 0x00000001:
                case 0x00000005:
                    *syncs = hpm_dio_syncs;
                    *sync_count = sizeof(hpm_dio_syncs) / sizeof(ec_sync_info_t);
                    return 0;
                case 0x00000003:
                case 0x00000004:
                    switch (cia402_mode) {
                        case EC_CIA402_OPERATION_MODE_CSV:
                            *syncs = hpm_cia402_csv_syncs;
                            *sync_count = sizeof(hpm_cia402_csv_syncs) / sizeof(ec_sync_info_t);
                            return 0;
                        case EC_CIA402_OPERATION_MODE_CSP:
                            *syncs = hpm_cia402_csp_syncs;
                            *sync_count = sizeof(hpm_cia402_csp_syncs) / sizeof(ec_sync_info_t);
                            return 0;

                        default:
                            break;
                    }
                    break;

                default:
                    break;
            }
            break;
        case 0xEE000002: // Tianjin Sange Elc Co.,Ltd
            switch (product_code) {
                case 0x00010204:
                    *syncs = sg_elc_syncs;
                    *sync_count = sizeof(sg_elc_syncs) / sizeof(ec_sync_info_t);
                    return 0;

                default:
                    break;
            }
            break;
        case 0x00004321: // Leadshine Technology Co., Ltd
            switch (product_code) {
                case 0x01100073:
                    *syncs = em32dx_4e_v30_syncs;
                    *sync_count = sizeof(em32dx_4e_v30_syncs) / sizeof(ec_sync_info_t);
                    return 0;

                default:
                    break;
            }
            break;
        case 0x00100000: // Inovance
            switch (product_code) {
                case 0x000C010D:
                    switch (cia402_mode) {
                        case EC_CIA402_OPERATION_MODE_CSV:
                            *syncs = sv660n_csv_syncs;
                            *sync_count = sizeof(sv660n_csv_syncs) / sizeof(ec_sync_info_t);
                            return 0;
                        case EC_CIA402_OPERATION_MODE_CSP:
                            *syncs = sv660n_csp_syncs;
                            *sync_count = sizeof(sv660n_csp_syncs) / sizeof(ec_sync_info_t);
                            return 0;
                        case EC_CIA402_OPERATION_MODE_CST:
                            *syncs = sv660n_cst_syncs;
                            *sync_count = sizeof(sv660n_cst_syncs) / sizeof(ec_sync_info_t);
                            return 0;

                        default:
                            break;
                    }
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }
    return -1;
}