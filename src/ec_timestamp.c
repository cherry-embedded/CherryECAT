/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_master.h"

#ifndef CONFIG_EC_TIMESTAMP_CUSTOM
#if defined(__riscv) || defined(__ICCRISCV__)

#define READ_CSR(csr_num) ({ uint32_t v; __asm volatile("csrr %0, %1" : "=r"(v) : "i"(csr_num)); v; })

#define CSR_MCYCLE  (0xB00)
#define CSR_MCYCLEH (0xB80)

static inline uint64_t riscv_csr_get_core_mcycle(void)
{
    uint64_t result;
    uint32_t resultl_first = READ_CSR(CSR_MCYCLE);
    uint32_t resulth = READ_CSR(CSR_MCYCLEH);
    uint32_t resultl_second = READ_CSR(CSR_MCYCLE);
    if (resultl_first < resultl_second) {
        result = ((uint64_t)resulth << 32) | resultl_first; /* if MCYCLE didn't roll over, return the value directly */
    } else {
        resulth = READ_CSR(CSR_MCYCLEH);
        result = ((uint64_t)resulth << 32) | resultl_second; /* if MCYCLE rolled over, need to get the MCYCLEH again */
    }
    return result;
}

static uint32_t g_clock_time_div;

void ec_timestamp_init(void)
{
    g_clock_time_div = ec_get_cpu_frequency() / 1000000;

    uint64_t start_cycle = ec_timestamp_get_time_us();
    ec_osal_msleep(10);

    EC_ASSERT_MSG((ec_timestamp_get_time_us() - start_cycle) >= 9000, "Timestamp timer not running\n");
}

EC_FAST_CODE_SECTION uint64_t ec_timestamp_get_time_ns(void)
{
    return (riscv_csr_get_core_mcycle() * 1000) / g_clock_time_div;
}

EC_FAST_CODE_SECTION uint64_t ec_timestamp_get_time_us(void)
{
    return riscv_csr_get_core_mcycle() / g_clock_time_div;
}
#elif defined(__arm__) || defined(__ICCARM__) || defined(__ARMCC_VERSION)

#define DWT_CR     (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)
#define DEM_CR     (*(volatile uint32_t *)0xE000EDFC)

#define DEM_CR_TRCENA    (1 << 24)
#define DWT_CR_CYCCNTENA (1 << 0)

static volatile uint32_t g_dwt_high = 0;
static volatile uint32_t g_dwt_last_low = 0;
static uint32_t g_clock_time_div;

static inline uint64_t arm_dwt_get_cycle_count(void)
{
    uint32_t current_low = DWT_CYCCNT;

    if (current_low < g_dwt_last_low) {
        g_dwt_high++;
    }
    g_dwt_last_low = current_low;

    return ((uint64_t)g_dwt_high << 32) | current_low;
}

void ec_timestamp_init(void)
{
    g_clock_time_div = ec_get_cpu_frequency() / 1000000;

    g_dwt_high = 0;
    g_dwt_last_low = 0;

    DEM_CR |= (uint32_t)DEM_CR_TRCENA;
    DWT_CYCCNT = (uint32_t)0u;
    g_dwt_last_low = 0;

    DWT_CR |= (uint32_t)DWT_CR_CYCCNTENA;

    uint64_t start_cycle = ec_timestamp_get_time_us();
    ec_osal_msleep(10);

    EC_ASSERT_MSG((ec_timestamp_get_time_us() - start_cycle) >= 9000, "Timestamp timer not running\n");
}

EC_FAST_CODE_SECTION uint64_t ec_timestamp_get_time_ns(void)
{
    return (arm_dwt_get_cycle_count() * 1000) / g_clock_time_div;
}

EC_FAST_CODE_SECTION uint64_t ec_timestamp_get_time_us(void)
{
    return arm_dwt_get_cycle_count() / g_clock_time_div;
}

#else
#error "Unsupported architecture"
#endif

#endif