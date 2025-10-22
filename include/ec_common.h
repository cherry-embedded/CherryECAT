/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_COMMON_H
#define EC_COMMON_H

void *ec_memcpy(void *s1, const void *s2, size_t n);
void ec_memset(void *s, int c, size_t n);
const char *ec_state_string(uint8_t states, uint8_t multi);
const char *ec_mbox_protocol_string(uint8_t prot);
const char *ec_alstatus_string(uint16_t errorcode);
const char *ec_mbox_error_string(uint16_t errorcode);
const char *ec_sdo_abort_string(uint32_t errorcode);
const char *ec_foe_errorcode_string(uint16_t errorcode);
const char *ec_eoe_errorcode_string(uint16_t errorcode);

#endif