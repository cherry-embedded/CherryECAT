/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_TIMESTAMP_H
#define EC_TIMESTAMP_H

void ec_timestamp_init(void);
uint64_t ec_timestamp_get_time_ns(void);

#define jiffies ec_timestamp_get_time_ns()

#endif