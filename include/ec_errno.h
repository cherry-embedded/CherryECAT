/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EC_ERRNO_H
#define EC_ERRNO_H

#define EC_ERR_OK          0  /**< No error */
#define EC_ERR_NOMEM       1  /**< Out of memory */
#define EC_ERR_INVAL       2  /**< Invalid argument */
#define EC_ERR_TIMEOUT     3  /**< Timeout */
#define EC_ERR_IO          4  /**< I/O error */
#define EC_ERR_WC          5  /**< working counter error */
#define EC_ERR_ALERR       6  /**< AL status error */
#define EC_ERR_SII         7  /**< SII error */
#define EC_ERR_MBOX        8  /**< mailbox error */
#define EC_ERR_MBOX_EMPTY  9  /**< mailbox is empty */
#define EC_ERR_COE_TYPE    10 /**< COE type error */
#define EC_ERR_COE_SIZE    11 /**< COE size error */
#define EC_ERR_COE_REQUEST 12 /**< COE request & index & subindex error */
#define EC_ERR_COE_TOGGLE  13 /**< COE toggle error */
#define EC_ERR_COE_ABORT   14 /**< COE abort error */
#define EC_ERR_FOE_TYPE    15 /**< FOE type error */
#define EC_ERR_FOE_SIZE    16 /**< FOE size error */
#define EC_ERR_FOE_OPCODE  17 /**< FOE opcode error */
#define EC_ERR_FOE_PACKNO  18 /**< FOE packet number error */
#define EC_ERR_EOE_TYPE    19 /**< EOE type error */
#define EC_ERR_EOE_SIZE    20 /**< EOE size error */
#define EC_ERR_EOE_FRAME   21 /**< EOE frametype error */

#define EC_ERR_UNKNOWN 255

#endif