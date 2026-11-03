/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_CMD_H
#define EC_CMD_H

typedef struct ec_master ec_master_t;

void ec_master_cmd_init(ec_master_t *master);
int ethercat(int argc, const char **argv);

#endif
