/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/*  HPM example includes. */
#include <stdio.h>
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_uart_drv.h"
#include "shell.h"
#include "hpm_gptmr_drv.h"
#include "cia402_def.h"
#include "ec_master.h"
#ifdef CONFIG_EC_EOE
#include "lwip/tcpip.h"
#endif

SDK_DECLARE_EXT_ISR_M(BOARD_CONSOLE_UART_IRQ, shell_uart_isr)

#define task_start_PRIORITY (configMAX_PRIORITIES - 2U)

ATTR_PLACE_AT_FAST_RAM_BSS ec_master_t g_ec_master;

#ifdef CONFIG_EC_EOE
ATTR_PLACE_AT_FAST_RAM_BSS ec_eoe_t g_ec_eoe;
#endif

static void task_start(void *param);

int main(void)
{
    board_init();

    if (pdPASS != xTaskCreate(task_start, "task_start", 1024U, NULL, task_start_PRIORITY, NULL)) {
        printf("Task start creation failed!\r\n");
        while (1) {
        };
    }

#ifdef CONFIG_EC_EOE
    /* Initialize the LwIP stack */
    tcpip_init(NULL, NULL);
#endif
    vTaskStartScheduler();
    printf("Unexpected scheduler exit!\r\n");
    while (1) {
    };

    return 0;
}

static void task_start(void *param)
{
    (void)param;

    printf("Try to initialize the uart\r\n"
           "  if you are using the console uart as the shell uart\r\n"
           "  failure to initialize may result in no log\r\n");

    uart_config_t shell_uart_config = { 0 };
    uart_default_config(BOARD_CONSOLE_UART_BASE, &shell_uart_config);
    shell_uart_config.src_freq_in_hz = clock_get_frequency(BOARD_CONSOLE_UART_CLK_NAME);
    shell_uart_config.baudrate = 115200;

    if (status_success != uart_init(BOARD_CONSOLE_UART_BASE, &shell_uart_config)) {
        /* uart failed to be initialized */
        printf("Failed to initialize uart\r\n");
        while (1) {
        };
    }

    printf("Initialize shell uart successfully\r\n");

    /* default password is : 12345678 */
    /* shell_init() must be called in-task */
    if (0 != shell_init(BOARD_CONSOLE_UART_BASE, false)) {
        /* shell failed to be initialized */
        printf("Failed to initialize shell\r\n");
        while (1) {
        };
    }

    printf("Initialize shell successfully\r\n");

    /* irq must be enabled after shell_init() */
    uart_enable_irq(BOARD_CONSOLE_UART_BASE, uart_intr_rx_data_avail_or_timeout);
    intc_m_enable_irq_with_priority(BOARD_CONSOLE_UART_IRQ, 1);

    printf("Enable shell uart interrupt\r\n");

    ec_master_cmd_init(&g_ec_master);
#ifdef CONFIG_EC_EOE
    ec_master_cmd_eoe_init(&g_ec_eoe);
#endif
    ec_master_init(&g_ec_master, 0);

    printf("Exit start task\r\n");

    vTaskDelete(NULL);
}

CSH_CMD_EXPORT(ethercat, );

unsigned char cherryecat_eepromdata[2048]; // EEPROM data buffer, please generate by esi_parse.py

void ec_pdo_callback(ec_slave_t *slave, uint8_t *output, uint8_t *input)
{
}

#ifdef CONFIG_EC_EOE
#include "tcp_client.h"
int tcp_client(int argc, const char **argv)
{
    tcp_client_init(&g_ec_eoe.netif);
    return 0;
}
CSH_CMD_EXPORT(tcp_client, );
#endif