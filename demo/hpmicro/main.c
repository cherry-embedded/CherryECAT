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
#include <stdlib.h>
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

// weak api used in ec_cmd.c
unsigned char cherryecat_eepromdata[2048]; // EEPROM data buffer, please generate by esi_parse.py

#define HPM_IO_TOGGLE_NS 500000000ULL

#define TEST_HPM_MOTOR_MAX_SLAVES 32U

typedef struct {
    volatile bool enable;
    bool fault_reset_sent;
    volatile bool position_set;
    volatile int32_t speed;
    volatile int32_t position;
} hpm_motor_config_t;

static hpm_motor_config_t hpm_motors[TEST_HPM_MOTOR_MAX_SLAVES];

static uint8_t hpm_motor_mode(const ec_slave_t *slave)
{
    if ((slave->sii.product_code != 0x00000003U && slave->sii.product_code != 0x00000004U) ||
        slave->odata_size != sizeof(TCiA402PDO1601) ||
        slave->idata_size != sizeof(TCiA402PDO1A01) || !slave->config) {
        return NO_MODE;
    }

    for (uint8_t i = 0; i < slave->config->sync_count; i++) {
        const ec_sync_info_t *sync = &slave->config->sync[i];
        if (sync->dir != EC_DIR_OUTPUT || sync->n_pdos != 1) {
            continue;
        }
        if (sync->pdos[0].index == 0x1601) {
            return CYCLIC_SYNC_POSITION_MODE;
        }
        if (sync->pdos[0].index == 0x1602) {
            return CYCLIC_SYNC_VELOCITY_MODE;
        }
    }
    return NO_MODE;
}

static void hpm_motor_pdo(ec_slave_t *slave, uint8_t *output, uint8_t *input, uint8_t mode)
{
    hpm_motor_config_t *motor = &hpm_motors[slave->index];
    TCiA402PDO1A01 feedback;

    ec_memcpy(&feedback, input, sizeof(feedback));
    uint16_t state = EC_READ_U16(input);
    state &= STATUSWORD_STATE_MASK;
    int32_t actual_position = feedback.ObjPositionActualValue;
    uint16_t control = CONTROLWORD_COMMAND_DISABLEVOLTAGE;

    bool enable = motor->enable;
    if (!enable) {
        control = CONTROLWORD_COMMAND_DISABLEVOLTAGE;
        motor->fault_reset_sent = false;
    } else {
        switch (state) {
            case STATUSWORD_STATE_SWITCHEDONDISABLED:
                control = CONTROLWORD_COMMAND_SHUTDOWN;
                break;
            case STATUSWORD_STATE_READYTOSWITCHON:
                control = CONTROLWORD_COMMAND_SWITCHON;
                break;
            case STATUSWORD_STATE_SWITCHEDON:
            case STATUSWORD_STATE_OPERATIONENABLED:
            case STATUSWORD_STATE_QUICKSTOPACTIVE:
                control = CONTROLWORD_COMMAND_ENABLEOPERATION;
                break;
            case STATUSWORD_STATE_FAULT:
                if (!motor->fault_reset_sent) {
                    control = CONTROLWORD_COMMAND_FAULTRESET;
                    motor->fault_reset_sent = true;
                }
                break;
            default:
                break;
        }
        if (state != STATUSWORD_STATE_FAULT) {
            motor->fault_reset_sent = false;
        }
    }

    /* Packed PDO fields may be unaligned in the process data buffer. */
    if (mode == CYCLIC_SYNC_VELOCITY_MODE) {
        TCiA402PDO1602 command = {
            .ObjControlWord = control,
            .ObjTargetVelocity = enable ? motor->speed : 0,
            .Padding16Bit = 0,
        };
        ec_memcpy(output, &command, sizeof(command));
    } else {
        TCiA402PDO1601 command = {
            .ObjControlWord = control,
            .ObjTargetPosition = enable && motor->position_set ? motor->position : actual_position,
            .Padding16Bit = 0,
        };
        ec_memcpy(output, &command, sizeof(command));
    }
}

// weak api used in ec_cmd.c
void ec_pdo_callback(ec_slave_t *slave, uint8_t *output, uint8_t *input)
{
    if (slave->sii.vendor_id != 0x0048504DU || slave->current_state != EC_SLAVE_STATE_OP) {
        return;
    }

    switch (slave->sii.product_code) {
        case 0x00000001U:
        case 0x00000005U: {
            uint32_t value = 0;

            value = EC_READ_U32(input);
            EC_WRITE_U32(output, value);
            break;
        }
        case 0x00000003U:
        case 0x00000004U: {
            uint8_t mode = hpm_motor_mode(slave);
            if (mode != NO_MODE) {
                hpm_motor_pdo(slave, output, input, mode);
            }
        } break;
    }
}

int motor_control(int argc, const char **argv)
{
    if (argc != 4) {
        printf("Usage: motor_control -p <idx> <0|1>\n");
        return -1;
    }

    int index = atoi(argv[2]);
    if (index < 0 || index >= TEST_HPM_MOTOR_MAX_SLAVES) {
        printf("Invalid slave index, expected 0-%u\n", TEST_HPM_MOTOR_MAX_SLAVES - 1U);
        return -1;
    }

    hpm_motors[index].enable = atoi(argv[3]);
    return 0;
}
CSH_CMD_EXPORT(motor_control, motor_control - p<idx><0 | 1>);

int speed_control(int argc, const char **argv)
{
    if (argc != 4) {
        printf("Usage: speed_control -p <idx> <velocity>\n");
        return -1;
    }

    int index = atoi(argv[2]);
    if (index < 0 || index >= TEST_HPM_MOTOR_MAX_SLAVES) {
        printf("Invalid slave index, expected 0-%u\n", TEST_HPM_MOTOR_MAX_SLAVES - 1U);
        return -1;
    }

    hpm_motors[index].speed = atoi(argv[3]);
    return 0;
}
CSH_CMD_EXPORT(speed_control, speed_control - p<idx><velocity>);

int position_control(int argc, const char **argv)
{
    if (argc != 4) {
        printf("Usage: position_control -p <idx> <position>\n");
        return -1;
    }

    int index = atoi(argv[2]);
    if (index < 0 || index >= TEST_HPM_MOTOR_MAX_SLAVES) {
        printf("Invalid slave index, expected 0-%u\n", TEST_HPM_MOTOR_MAX_SLAVES - 1U);
        return -1;
    }

    hpm_motors[index].position = atoi(argv[3]);
    hpm_motors[index].position_set = true;
    return 0;
}
CSH_CMD_EXPORT(position_control, position_control - p<idx><position>);

#ifdef CONFIG_EC_EOE
#include "tcp_client.h"
int tcp_client(int argc, const char **argv)
{
    tcp_client_init(&g_ec_eoe.netif);
    return 0;
}
CSH_CMD_EXPORT(tcp_client, );
#endif