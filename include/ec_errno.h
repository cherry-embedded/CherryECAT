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
#define EC_ERR_NOSUPP      4  /**< Not supported */
#define EC_ERR_IO          5  /**< I/O error */
#define EC_ERR_WC          6  /**< working counter error */
#define EC_ERR_ALERR       7  /**< AL status error */
#define EC_ERR_SII         8  /**< SII error */
#define EC_ERR_MBOX        9  /**< mailbox error */
#define EC_ERR_MBOX_EMPTY  10  /**< mailbox is empty */
#define EC_ERR_COE_TYPE    11 /**< COE type error */
#define EC_ERR_COE_SIZE    12 /**< COE size error */
#define EC_ERR_COE_REQUEST 13 /**< COE request & index & subindex error */
#define EC_ERR_COE_TOGGLE  14 /**< COE toggle error */
#define EC_ERR_COE_ABORT   15 /**< COE abort error */
#define EC_ERR_FOE_TYPE    16 /**< FOE type error */
#define EC_ERR_FOE_SIZE    17 /**< FOE size error */
#define EC_ERR_FOE_OPCODE  18 /**< FOE opcode error */
#define EC_ERR_FOE_PACKNO  19 /**< FOE packet number error */
#define EC_ERR_EOE_TYPE    20 /**< EOE type error */
#define EC_ERR_EOE_SIZE    21 /**< EOE size error */
#define EC_ERR_EOE_FRAME   22 /**< EOE frametype error */

#define EC_ERR_UNKNOWN 255

#endif