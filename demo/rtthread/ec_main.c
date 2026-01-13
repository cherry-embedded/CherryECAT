/*
 * Copyright (c) 2026, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <rtthread.h>
#include "ec_master.h"

ec_master_t g_ec_master;

// weak api used in ec_cmd.c
unsigned char cherryecat_eepromdata[2048]; // EEPROM data buffer, please generate by esi_parse.py

// weak api used in ec_cmd.c
void ec_pdo_callback(ec_slave_t *slave, uint8_t *output, uint8_t *input)
{
}

int ec_master_app_init(void)
{
    ec_master_cmd_init(&g_ec_master);
    ec_master_init(&g_ec_master, 0);

    return 0;
}
INIT_APP_EXPORT(ec_master_app_init);

#ifdef FINSH_USING_MSH
#include <finsh.h>
MSH_CMD_EXPORT(ethercat, cherryecat command line tool);
#endif
