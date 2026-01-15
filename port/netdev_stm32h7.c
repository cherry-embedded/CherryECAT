/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "stm32h7xx_hal.h"
#include "ec_master.h"

#ifndef CONFIG_EC_PHY_RESET_PORT
#error "Please define CONFIG_EC_PHY_RESET_PORT in ec_config.h"
#endif

#ifndef CONFIG_EC_PHY_RESET_PIN
#error "Please define CONFIG_EC_PHY_RESET_PIN in ec_config.h"
#endif

#if USE_HAL_TIM_REGISTER_CALLBACKS == 0
#error "Please set USE_HAL_TIM_REGISTER_CALLBACKS to 1 in stm32h7xx_hal_conf.h"
#endif

#define ETH_RX_BUFFER_SIZE 1536U
#define ETH_TX_BUFFER_SIZE 1536U

/* Global Ethernet handle*/
ETH_HandleTypeDef EthHandle;
ETH_TxPacketConfig TxConfig;

// clang-format off
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000080
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */
__attribute__((section(".RxDescripSection"))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((section(".TxDescripSection"))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

__attribute__((section(".TRx_PoolSection"))) __attribute__((aligned(32))) uint8_t rx_buffer[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE]; /* Ethernet Receive Buffer */
__attribute__((section(".TRx_PoolSection"))) __attribute__((aligned(32))) uint8_t tx_buffer[ETH_TX_DESC_CNT][ETH_TX_BUFFER_SIZE]; /* Ethernet Transmit Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */
__attribute__((section(".RxDescripSection"))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((section(".TxDescripSection"))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

__attribute__((section(".TRx_PoolSection"))) __attribute__((aligned(32))) uint8_t rx_buffer[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE]; /* Ethernet Receive Buffer */
__attribute__((section(".TRx_PoolSection"))) __attribute__((aligned(32))) uint8_t tx_buffer[ETH_TX_DESC_CNT][ETH_TX_BUFFER_SIZE]; /* Ethernet Transmit Buffer */

#endif
// clang-format on

static uint32_t g_devinuse = 0;

static uint8_t *ec_master_enet_buffer_alloc(void)
{
    uint8_t devno;

    for (devno = 0; devno < ETH_RX_DESC_CNT; devno++) {
        if ((g_devinuse & (1U << devno)) == 0) {
            g_devinuse |= (1U << devno);
            return rx_buffer[devno];
        }
    }
    return NULL;
}
 
static void ec_master_enet_buffer_free(uint8_t *buffer)
{
    uint8_t devno;

    devno = (buffer - &rx_buffer[0][0]) / ETH_RX_BUFFER_SIZE;
    g_devinuse &= ~(1U << devno);
}

ec_netdev_t g_netdev;

ec_netdev_t *ec_netdev_low_level_init(uint8_t netdev_index)
{
    static uint8_t MACAddr[6];

    /* Enable D2 domain SRAM1 Clock (0x30000000 AXI)*/
    __HAL_RCC_D2SRAM1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    switch (CONFIG_EC_PHY_RESET_PORT) {
        case 0:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            break;
        case 1:
            __HAL_RCC_GPIOB_CLK_ENABLE();
            break;
        case 2:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            break;
        case 3:
            __HAL_RCC_GPIOD_CLK_ENABLE();
            break;
        case 4:
            __HAL_RCC_GPIOE_CLK_ENABLE();
            break;
        case 5:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            break;
        case 6:
            __HAL_RCC_GPIOG_CLK_ENABLE();
            break;
        case 7:
            __HAL_RCC_GPIOH_CLK_ENABLE();
            break;

        default:
            EC_ASSERT_MSG(0, "Invalid CONFIG_EC_PHY_RESET_PORT\n");
            break;
    }

    GPIO_InitStruct.Pin = (1 << CONFIG_EC_PHY_RESET_PIN);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init((GPIO_TypeDef *)(GPIOA_BASE + CONFIG_EC_PHY_RESET_PORT * 0x400UL), &GPIO_InitStruct);

    // phy reset
    HAL_GPIO_WritePin((GPIO_TypeDef *)(GPIOA_BASE + CONFIG_EC_PHY_RESET_PORT * 0x400UL), (1 << CONFIG_EC_PHY_RESET_PIN), GPIO_PIN_RESET);
    ec_osal_msleep(10);
    HAL_GPIO_WritePin((GPIO_TypeDef *)(GPIOA_BASE + CONFIG_EC_PHY_RESET_PORT * 0x400UL), (1 << CONFIG_EC_PHY_RESET_PIN), GPIO_PIN_SET);

    EthHandle.Instance = ETH;
    MACAddr[0] = 0x00;
    MACAddr[1] = 0x80;
    MACAddr[2] = 0xE1;
    MACAddr[3] = 0x00;
    MACAddr[4] = 0x00;
    MACAddr[5] = 0x00;
    EthHandle.Init.MACAddr = &MACAddr[0];
    EthHandle.Init.MediaInterface = HAL_ETH_RMII_MODE;
    EthHandle.Init.TxDesc = DMATxDscrTab;
    EthHandle.Init.RxDesc = DMARxDscrTab;
    EthHandle.Init.RxBuffLen = ETH_RX_BUFFER_SIZE;

    if (HAL_ETH_Init(&EthHandle) != HAL_OK) {
        EC_LOG_ERR("HAL_ETH_Init failed\n");
        while (1) {
        }
    }
    HAL_ETH_SetMDIOClockRange(&EthHandle);

    memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
    TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
    TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

    ec_memcpy(g_netdev.mac_addr, MACAddr, 6);

    for (uint32_t i = 0; i < ETH_TX_DESC_CNT; i++) {
        for (uint8_t j = 0; j < 6; j++) { // dst MAC
            EC_WRITE_U8(&tx_buffer[i][j], 0xFF);
        }
        for (uint8_t j = 0; j < 6; j++) { // src MAC
            EC_WRITE_U8(&tx_buffer[i][6 + j], MACAddr[j]);
        }
        EC_WRITE_U16(&tx_buffer[i][12], ec_htons(0x88a4));
    }

    return &g_netdev;
}

void ec_mdio_low_level_write(struct chry_phy_device *phydev, uint16_t phy_addr, uint16_t regnum, uint16_t val)
{
    //ec_netdev_t *netdev = (ec_netdev_t *)phydev->user_data;
    HAL_ETH_WritePHYRegister(&EthHandle, phy_addr, regnum, val);
}

uint16_t ec_mdio_low_level_read(struct chry_phy_device *phydev, uint16_t phy_addr, uint16_t regnum)
{
    //ec_netdev_t *netdev = (ec_netdev_t *)phydev->user_data;
    uint32_t pRegVal = 0;

    HAL_ETH_ReadPHYRegister(&EthHandle, phy_addr, regnum, &pRegVal);

    return pRegVal;
}

void ec_netdev_low_level_link_up(ec_netdev_t *netdev, struct chry_phy_status *status)
{
    ETH_MACConfigTypeDef MACConf = { 0 };

    if (status->link) {
        /* Get MAC Config MAC */
        HAL_ETH_GetMACConfig(&EthHandle, &MACConf);
        MACConf.DuplexMode = ETH_FULLDUPLEX_MODE;
        MACConf.Speed = ETH_SPEED_100M;
        HAL_ETH_SetMACConfig(&EthHandle, &MACConf);
        HAL_ETH_Start_IT(&EthHandle);
    } else {
        HAL_ETH_Stop_IT(&EthHandle);
    }
}

EC_FAST_CODE_SECTION uint8_t *ec_netdev_low_level_get_txbuf(ec_netdev_t *netdev)
{
    return (uint8_t *)tx_buffer[netdev->tx_frame_index];
}

EC_FAST_CODE_SECTION int ec_netdev_low_level_output(ec_netdev_t *netdev, uint32_t size)
{
    ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT] = { 0 };
    HAL_StatusTypeDef status;

    memset(Txbuffer, 0, ETH_TX_DESC_CNT * sizeof(ETH_BufferTypeDef));

    Txbuffer[0].buffer = tx_buffer[netdev->tx_frame_index];
    Txbuffer[0].len = size;
    Txbuffer[0].next = NULL;

    TxConfig.Length = size;
    TxConfig.TxBuffer = Txbuffer;
    TxConfig.pData = NULL;

    SCB_CleanDCache_by_Addr((uint32_t *)tx_buffer[netdev->tx_frame_index], size);
    status = HAL_ETH_Transmit(&EthHandle, &TxConfig, 20);
    if (status != HAL_OK) {
        return -1;
    }
    netdev->tx_frame_index++;
    netdev->tx_frame_index %= ETH_TX_DESC_CNT;

    return 0;
}

static ec_htimer_cb g_ec_htimer_cb = NULL;
static void *g_ec_htimer_arg = NULL;

static TIM_HandleTypeDef ECTimHandle;

void HAL_TIM7_EC_Callback(TIM_HandleTypeDef *htim)
{
    if (g_ec_htimer_cb) {
        g_ec_htimer_cb(g_ec_htimer_arg);
    }
}

void ec_htimer_start(uint32_t us, ec_htimer_cb cb, void *arg)
{
    RCC_ClkInitTypeDef clkconfig;
    uint32_t uwTimclock, uwAPB1Prescaler;
    uint32_t uwPrescalerValue;
    uint32_t pFLatency;
    HAL_StatusTypeDef status;

    g_ec_htimer_cb = cb;
    g_ec_htimer_arg = arg;

    /* Enable TIM7 clock */
    __HAL_RCC_TIM7_CLK_ENABLE();

    /* Get clock configuration */
    HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

    /* Get APB1 prescaler */
    uwAPB1Prescaler = clkconfig.APB1CLKDivider;

    /* Compute TIM7 clock */
    if (uwAPB1Prescaler == RCC_HCLK_DIV1) {
        uwTimclock = HAL_RCC_GetPCLK1Freq();
    } else {
        uwTimclock = 2UL * HAL_RCC_GetPCLK1Freq();
    }

    /* Compute the prescaler value to have TIM7 counter clock equal to 1MHz */
    uwPrescalerValue = (uint32_t)((uwTimclock / 1000000U) - 1U);

    /* Initialize TIM7 */
    ECTimHandle.Instance = TIM7;
    ECTimHandle.Init.Period = us - 1U;
    ECTimHandle.Init.Prescaler = uwPrescalerValue;
    ECTimHandle.Init.ClockDivision = 0U;
    ECTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    ECTimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    status = HAL_TIM_Base_Init(&ECTimHandle);
    if (status == HAL_OK) {
        __HAL_TIM_CLEAR_FLAG(&ECTimHandle, TIM_FLAG_UPDATE);
        HAL_TIM_RegisterCallback(&ECTimHandle, HAL_TIM_PERIOD_ELAPSED_CB_ID, HAL_TIM7_EC_Callback);
        /* Start the TIM time Base generation in interrupt mode */
        status = HAL_TIM_Base_Start_IT(&ECTimHandle);
        if (status == HAL_OK) {
            /* Enable the TIM7 global Interrupt */
            HAL_NVIC_EnableIRQ(TIM7_IRQn);

            /* Enable the TIM7 global Interrupt */
            HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
        }
    }
}

void ec_htimer_stop(void)
{
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
    __HAL_TIM_CLEAR_FLAG(&ECTimHandle, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Stop_IT(&ECTimHandle);
}

EC_FAST_CODE_SECTION void ec_htimer_update(uint32_t us)
{
    TIM7->ARR = us - 1U;
}

#ifndef CONFIG_EC_TIMESTAMP_CUSTOM
extern uint32_t SystemCoreClock;
uint32_t ec_get_cpu_frequency(void)
{
    return SystemCoreClock;
}
#endif

void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
    uint8_t *p;

    p = ec_master_enet_buffer_alloc();
    if (p) {
        *buff = p;
    } else {
        *buff = NULL;
    }
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length)
{
    /* Invalidate data cache because Rx DMA's writing to physical memory makes it stale. */

    *pStart = buff;
    *pEnd = buff + Length;
    SCB_InvalidateDCache_by_Addr((uint32_t *)buff, Length);

    ec_netdev_receive(&g_netdev, buff, Length);
    ec_master_enet_buffer_free(buff);
}

void HAL_ETH_TxFreeCallback(uint32_t *buff)
{
}

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
    uint8_t *buffer = NULL;

    HAL_ETH_ReadData(&EthHandle, (void **)&buffer);
}

/**
  * @brief ETH MSP Initialization
  * This function configures the hardware resources used in this example
  * @param heth: ETH handle pointer
  * @retval None
  */
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    if (heth->Instance == ETH) {
        /* USER CODE BEGIN ETH_MspInit 0 */

        /* USER CODE END ETH_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_ETH1MAC_CLK_ENABLE();
        __HAL_RCC_ETH1TX_CLK_ENABLE();
        __HAL_RCC_ETH1RX_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**ETH GPIO Configuration
        PC1     ------> ETH_MDC
        PA1     ------> ETH_REF_CLK
        PA2     ------> ETH_MDIO
        PA7     ------> ETH_CRS_DV
        PC4     ------> ETH_RXD0
        PC5     ------> ETH_RXD1
        PB11     ------> ETH_TX_EN
        PB12     ------> ETH_TXD0
        PB13     ------> ETH_TXD1
        */
        GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* ETH interrupt Init */
        HAL_NVIC_SetPriority(ETH_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ETH_IRQn);
        /* USER CODE BEGIN ETH_MspInit 1 */

        /* USER CODE END ETH_MspInit 1 */
    }
}

/**
  * @brief ETH MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param heth: ETH handle pointer
  * @retval None
  */
void HAL_ETH_MspDeInit(ETH_HandleTypeDef *heth)
{
    if (heth->Instance == ETH) {
        /* USER CODE BEGIN ETH_MspDeInit 0 */

        /* USER CODE END ETH_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_ETH1MAC_CLK_DISABLE();
        __HAL_RCC_ETH1TX_CLK_DISABLE();
        __HAL_RCC_ETH1RX_CLK_DISABLE();

        /**ETH GPIO Configuration
        PC1     ------> ETH_MDC
        PA1     ------> ETH_REF_CLK
        PA2     ------> ETH_MDIO
        PA7     ------> ETH_CRS_DV
        PC4     ------> ETH_RXD0
        PC5     ------> ETH_RXD1
        PB11     ------> ETH_TX_EN
        PB12     ------> ETH_TXD0
        PB13     ------> ETH_TXD1
        */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5);

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7);

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13);

        /* ETH interrupt DeInit */
        HAL_NVIC_DisableIRQ(ETH_IRQn);
        /* USER CODE BEGIN ETH_MspDeInit 1 */

        /* USER CODE END ETH_MspDeInit 1 */
    }
}

void ETH_IRQHandler(void)
{
    HAL_ETH_IRQHandler(&EthHandle);
}

void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&ECTimHandle);
}
