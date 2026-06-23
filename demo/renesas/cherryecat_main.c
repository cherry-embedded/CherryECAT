/*
 * Copyright (c) 2026, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "ec_master.h"

ec_master_t g_ec_master;

// weak api used in ec_cmd.c
unsigned char cherryecat_eepromdata[2048]; // EEPROM data buffer, please generate by esi_parse.py

// weak api used in ec_cmd.c
void ec_pdo_callback(ec_slave_t *slave, uint8_t *output, uint8_t *input)
{
}

static void cherryecat_main(void *pvParameters)
{
    ec_master_cmd_init(&g_ec_master);
    ec_master_init(&g_ec_master, 0);
    vTaskDelete(NULL);
}

void cherryecat_thread_create(void)
{
    xTaskCreate(cherryecat_main, "ecat_main", 1024U, NULL, 10, NULL);
}
