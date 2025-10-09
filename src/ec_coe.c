/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_master.h"

typedef struct __PACKED {
    ec_coe_header_t coe_header;
    ec_sdo_header_common_t sdo_header;
    uint16_t index;
    uint8_t subindex;
    uint8_t data[4];
} ec_coe_download_common_header_t;

typedef struct __PACKED {
    ec_coe_header_t coe_header;
    ec_sdo_header_segment_t sdo_header;
} ec_coe_download_segment_header_t;

typedef ec_coe_download_common_header_t ec_coe_upload_common_header_t;
typedef ec_coe_download_segment_header_t ec_coe_upload_segment_header_t;

/** CoE download request header size.
 */
#define EC_COE_DOWN_REQ_HEADER_SIZE (sizeof(ec_coe_download_common_header_t))

/** CoE upload request header size.
 */
#define EC_COE_UP_REQ_HEADER_SIZE (sizeof(ec_coe_upload_common_header_t))

/** CoE download segment request header size.
 */
#define EC_COE_DOWN_SEG_REQ_HEADER_SIZE (sizeof(ec_coe_download_segment_header_t))

/** CoE upload segment request header size.
 */
#define EC_COE_UP_SEG_REQ_HEADER_SIZE (sizeof(ec_coe_upload_segment_header_t))

/** Minimum size of download segment.
 */
#define EC_COE_DOWN_SEG_MIN_DATA_SIZE 7

#define EC_COE_TIMEOUT_US (1000 * 1000) /* 1s */

static int ec_coe_download_expedited(ec_master_t *master,
                                     uint16_t slave_index,
                                     ec_datagram_t *datagram,
                                     uint16_t index,
                                     uint8_t subindex,
                                     const void *buf,
                                     uint32_t size,
                                     bool complete_access)
{
    uint8_t *data;
    uint8_t mbox_proto;
    uint32_t recv_size;
    ec_coe_download_common_header_t *download_common;
    int ret;

    data = ec_mailbox_fill_send(master, slave_index, datagram, EC_MBOX_TYPE_COE, EC_COE_DOWN_REQ_HEADER_SIZE);

    download_common = (ec_coe_download_common_header_t *)data;
    download_common->coe_header.number = 0;
    download_common->coe_header.reserved = 0;
    download_common->coe_header.service = EC_COE_SERVICE_SDO_REQUEST;
    download_common->sdo_header.size_indicator = 1;
    download_common->sdo_header.transfertype = 1; // expedited
    download_common->sdo_header.data_set_size = 4 - size;
    download_common->sdo_header.complete_access = complete_access ? 1 : 0;
    download_common->sdo_header.command = EC_COE_REQUEST_DOWNLOAD;

    download_common->index = index;
    download_common->subindex = complete_access ? 0x00 : subindex;

    ec_memcpy(download_common->data, buf, size);
    memset(download_common->data + size, 0x00, 4 - size);
    ret = ec_mailbox_send(master, slave_index, datagram);
    if (ret < 0) {
        return ret;
    }

    ret = ec_mailbox_receive(master, slave_index, datagram, &mbox_proto, &recv_size, EC_COE_TIMEOUT_US);
    if (ret < 0) {
        return ret;
    }

    if (mbox_proto != EC_MBOX_TYPE_COE) {
        return -EC_ERR_COE_TYPE;
    }

    if (recv_size < 6) {
        return -EC_ERR_COE_SIZE;
    }

    data = datagram->data + EC_MBOX_HEADER_SIZE;

    if (EC_READ_U16(data) >> 12 == EC_COE_SERVICE_SDO_REQUEST &&
        EC_READ_U8(data + 2) >> 5 == EC_COE_REQUEST_ABORT) {
        EC_SLAVE_LOG_ERR("Slave %u SDO abort code: 0x%08x (%s)\n", slave_index, EC_READ_U32(data + 6), ec_sdo_abort_string(EC_READ_U32(data + 6)));
        return -EC_ERR_COE_ABORT;
    }

    if (EC_READ_U16(data) >> 12 != EC_COE_SERVICE_SDO_RESPONSE ||
        EC_READ_U8(data + 2) >> 5 != EC_COE_RESPONSE_DOWNLOAD ||
        EC_READ_U16(data + 3) != index ||
        EC_READ_U8(data + 5) != subindex) {
        return -EC_ERR_COE_REQUEST;
    }

    return 0;
}

static int ec_coe_download_common(ec_master_t *master,
                                  uint16_t slave_index,
                                  ec_datagram_t *datagram,
                                  uint16_t index,
                                  uint8_t subindex,
                                  const void *buf,
                                  uint32_t size,
                                  bool complete_access)
{
    uint8_t *data;
    uint8_t mbox_proto;
    uint32_t recv_size;
    ec_coe_download_common_header_t *download_common;
    int ret;

    data = ec_mailbox_fill_send(master, slave_index, datagram, EC_MBOX_TYPE_COE, size + EC_COE_DOWN_REQ_HEADER_SIZE);

    download_common = (ec_coe_download_common_header_t *)data;
    download_common->coe_header.number = 0;
    download_common->coe_header.reserved = 0;
    download_common->coe_header.service = EC_COE_SERVICE_SDO_REQUEST;
    download_common->sdo_header.size_indicator = 1;
    download_common->sdo_header.transfertype = 0; // normal
    download_common->sdo_header.data_set_size = 0;
    download_common->sdo_header.complete_access = complete_access ? 1 : 0;
    download_common->sdo_header.command = EC_COE_REQUEST_DOWNLOAD;

    download_common->index = index;
    download_common->subindex = complete_access ? 0x00 : subindex;

    ec_memcpy(download_common->data, &size, 4);
    ec_memcpy(data + EC_COE_DOWN_REQ_HEADER_SIZE, buf, size);
    ret = ec_mailbox_send(master, slave_index, datagram);
    if (ret < 0) {
        return ret;
    }

    ret = ec_mailbox_receive(master, slave_index, datagram, &mbox_proto, &recv_size, EC_COE_TIMEOUT_US);
    if (ret < 0) {
        return ret;
    }

    if (mbox_proto != EC_MBOX_TYPE_COE) {
        return -EC_ERR_COE_TYPE;
    }

    if (recv_size < 6) {
        return -EC_ERR_COE_SIZE;
    }

    data = datagram->data + EC_MBOX_HEADER_SIZE;

    if (EC_READ_U16(data) >> 12 == EC_COE_SERVICE_SDO_REQUEST &&
        EC_READ_U8(data + 2) >> 5 == EC_COE_REQUEST_ABORT) {
        EC_SLAVE_LOG_ERR("Slave %u SDO abort code: 0x%08x (%s)\n", slave_index, EC_READ_U32(data + 6), ec_sdo_abort_string(EC_READ_U32(data + 6)));
        return -EC_ERR_COE_ABORT;
    }

    if (EC_READ_U16(data) >> 12 != EC_COE_SERVICE_SDO_RESPONSE ||
        EC_READ_U8(data + 2) >> 5 != EC_COE_RESPONSE_DOWNLOAD ||
        EC_READ_U16(data + 3) != index ||
        EC_READ_U8(data + 5) != subindex) {
        return -EC_ERR_COE_REQUEST;
    }

    return 0;
}

static int ec_coe_download_segment(ec_master_t *master,
                                   uint16_t slave_index,
                                   ec_datagram_t *datagram,
                                   const void *seg_data,
                                   uint16_t size,
                                   bool toggle,
                                   bool last)
{
    uint8_t *data;
    uint8_t mbox_proto;
    uint32_t data_size, recv_size;
    uint32_t seg_size;
    ec_coe_download_segment_header_t *download_seg;
    int ret;

    if (size > EC_COE_DOWN_SEG_MIN_DATA_SIZE) {
        seg_size = 0;
        data_size = size;
    } else {
        seg_size = EC_COE_DOWN_SEG_MIN_DATA_SIZE - size;
        data_size = EC_COE_DOWN_SEG_MIN_DATA_SIZE;
    }

    data = ec_mailbox_fill_send(master, slave_index, datagram, EC_MBOX_TYPE_COE, data_size + EC_COE_DOWN_SEG_REQ_HEADER_SIZE);

    download_seg = (ec_coe_download_segment_header_t *)data;
    download_seg->coe_header.number = 0;
    download_seg->coe_header.reserved = 0;
    download_seg->coe_header.service = EC_COE_SERVICE_SDO_REQUEST;

    download_seg->sdo_header.more_follows = last ? 1 : 0;
    download_seg->sdo_header.toggle = toggle ? 1 : 0;
    download_seg->sdo_header.command = EC_COE_REQUEST_SEGMENT_DOWNLOAD;
    download_seg->sdo_header.segdata_size = seg_size;

    ec_memcpy(data + EC_COE_DOWN_SEG_REQ_HEADER_SIZE, seg_data, size);
    if (size < EC_COE_DOWN_SEG_MIN_DATA_SIZE) {
        memset(data + EC_COE_DOWN_SEG_REQ_HEADER_SIZE + size, 0x00, EC_COE_DOWN_SEG_MIN_DATA_SIZE - size);
    }

    ret = ec_mailbox_send(master, slave_index, datagram);
    if (ret < 0) {
        return ret;
    }

    ret = ec_mailbox_receive(master, slave_index, datagram, &mbox_proto, &recv_size, EC_COE_TIMEOUT_US);
    if (ret < 0) {
        return ret;
    }

    if (mbox_proto != EC_MBOX_TYPE_COE) {
        return -EC_ERR_COE_TYPE;
    }

    if (recv_size < 6) {
        return -EC_ERR_COE_SIZE;
    }

    data = datagram->data + EC_MBOX_HEADER_SIZE;

    if (EC_READ_U16(data) >> 12 == EC_COE_SERVICE_SDO_REQUEST &&
        EC_READ_U8(data + 2) >> 5 == EC_COE_REQUEST_ABORT) {
        EC_SLAVE_LOG_ERR("Slave %u SDO abort code: 0x%08x (%s)\n", slave_index, EC_READ_U32(data + 6), ec_sdo_abort_string(EC_READ_U32(data + 6)));
        return -EC_ERR_COE_ABORT;
    }

    if (EC_READ_U16(data) >> 12 != EC_COE_SERVICE_SDO_RESPONSE ||
        EC_READ_U8(data + 2) >> 5 != EC_COE_RESPONSE_SEGMENT_DOWNLOAD) {
        return -EC_ERR_COE_REQUEST;
    }

    if (((EC_READ_U8(data + 2) >> 4) & 0x01) != toggle) {
        return -EC_ERR_COE_TOGGLE;
    }

    return 0;
}

int ec_coe_download(ec_master_t *master,
                    uint16_t slave_index,
                    ec_datagram_t *datagram,
                    uint16_t index,
                    uint8_t subindex,
                    const void *buf,
                    uint32_t size,
                    bool complete_access)
{
    ec_slave_t *slave;
    uint8_t *ptr;
    uint32_t seg_size, max_data_size;
    bool toggle;
    bool last;
    int ret;

    if (slave_index >= master->slave_count) {
        return -EC_ERR_INVAL;
    }

    slave = &master->slaves[slave_index];

    ptr = (uint8_t *)buf;

    if (size <= 4) {
        return ec_coe_download_expedited(master, slave_index, datagram, index, subindex, ptr, size, complete_access);
    } else {
        max_data_size = slave->configured_rx_mailbox_size - EC_MBOX_HEADER_SIZE - EC_COE_DOWN_REQ_HEADER_SIZE;
        if (size <= max_data_size) {
            return ec_coe_download_common(master, slave_index, datagram, index, subindex, ptr, size, complete_access);
        } else {
            ret = ec_coe_download_common(master, slave_index, datagram, index, subindex, ptr, max_data_size, complete_access);
            if (ret < 0) {
                return ret;
            }

            size -= max_data_size;
            ptr += max_data_size;
            toggle = false;
            last = false;
            max_data_size += 7;

            while (1) {
                seg_size = MIN(size, max_data_size);
                if (size <= max_data_size) {
                    last = true;
                }
                ret = ec_coe_download_segment(master, slave_index, datagram, ptr, seg_size, toggle, last);
                if (ret < 0) {
                    return ret;
                }
                ptr += seg_size;
                size -= seg_size;
                toggle ^= 1;
                if (size == 0) {
                    return 0;
                }
            }
        }
    }
}

int ec_coe_upload(ec_master_t *master,
                  uint16_t slave_index,
                  ec_datagram_t *datagram,
                  uint16_t index,
                  uint8_t subindex,
                  const void *buf,
                  uint32_t maxsize,
                  uint32_t *size,
                  bool complete_access)
{
    uint8_t *data;
    uint8_t *ptr;
    uint8_t mbox_proto;
    uint32_t recv_size;
    uint16_t rec_index;
    uint8_t rec_subindex;
    uint32_t data_size, total_size, offset;
    bool expedited, size_specified;
    ec_coe_upload_common_header_t *upload_common;
    ec_coe_upload_segment_header_t *upload_seg;
    bool toggle;
    bool last;
    int ret;

    ptr = (uint8_t *)buf;

    data = ec_mailbox_fill_send(master, slave_index, datagram, EC_MBOX_TYPE_COE, EC_COE_UP_REQ_HEADER_SIZE);

    upload_common = (ec_coe_upload_common_header_t *)data;
    upload_common->coe_header.number = 0;
    upload_common->coe_header.reserved = 0;
    upload_common->coe_header.service = EC_COE_SERVICE_SDO_REQUEST;
    upload_common->sdo_header.size_indicator = 0;
    upload_common->sdo_header.transfertype = 0;
    upload_common->sdo_header.data_set_size = 0;
    upload_common->sdo_header.complete_access = complete_access ? 1 : 0;
    upload_common->sdo_header.command = EC_COE_REQUEST_UPLOAD;

    upload_common->index = index;
    upload_common->subindex = complete_access ? 0x00 : subindex;

    memset(upload_common->data, 0x00, 4);
    ret = ec_mailbox_send(master, slave_index, datagram);
    if (ret < 0) {
        return ret;
    }

    ret = ec_mailbox_receive(master, slave_index, datagram, &mbox_proto, &recv_size, EC_COE_TIMEOUT_US);
    if (ret < 0) {
        return ret;
    }

    if (mbox_proto != EC_MBOX_TYPE_COE) {
        return -EC_ERR_COE_TYPE;
    }

    if (recv_size < 6) {
        return -EC_ERR_COE_SIZE;
    }

    data = datagram->data + EC_MBOX_HEADER_SIZE;

    if (EC_READ_U16(data) >> 12 == EC_COE_SERVICE_SDO_REQUEST &&
        EC_READ_U8(data + 2) >> 5 == EC_COE_REQUEST_ABORT) {
        EC_SLAVE_LOG_ERR("Slave %u SDO abort code: 0x%08x (%s)\n", slave_index, EC_READ_U32(data + 6), ec_sdo_abort_string(EC_READ_U32(data + 6)));
        return -EC_ERR_COE_ABORT;
    }

    if (EC_READ_U16(data) >> 12 != EC_COE_SERVICE_SDO_RESPONSE ||
        EC_READ_U8(data + 2) >> 5 != EC_COE_RESPONSE_UPLOAD) {
        return -EC_ERR_COE_REQUEST;
    }

    rec_index = EC_READ_U16(data + 3);
    rec_subindex = EC_READ_U8(data + 5);

    if (rec_index != index || rec_subindex != (complete_access ? 0x00 : subindex)) {
        return -EC_ERR_COE_REQUEST;
    }

    expedited = EC_READ_U8(data + 2) & 0x02;

    if (expedited) {
        size_specified = EC_READ_U8(data + 2) & 0x01;
        if (size_specified) {
            total_size = 4 - ((EC_READ_U8(data + 2) & 0x0C) >> 2);
        } else {
            total_size = 4;
        }

        if (recv_size < (total_size + 6)) {
            return -EC_ERR_COE_SIZE;
        }

        ec_memcpy(ptr, data + 6, total_size);

        if (maxsize < total_size) {
            return -EC_ERR_COE_SIZE;
        }

        if (size) {
            *size = total_size;
        }
        return 0;
    } else { // normal or segment
        if (recv_size < EC_COE_UP_REQ_HEADER_SIZE) {
            return -EC_ERR_COE_SIZE;
        }

        data_size = recv_size - EC_COE_UP_REQ_HEADER_SIZE;
        total_size = EC_READ_U32(data + 6);
        offset = 0;

        if (maxsize < total_size) {
            return -EC_ERR_COE_SIZE;
        }

        ec_memcpy(ptr, data + EC_COE_UP_REQ_HEADER_SIZE, data_size);

        ptr += data_size;
        offset += data_size;

        toggle = false;

        if (offset < total_size) {
            while (1) {
                data = ec_mailbox_fill_send(master, slave_index, datagram, EC_MBOX_TYPE_COE, EC_COE_UP_REQ_HEADER_SIZE);

                upload_seg = (ec_coe_upload_segment_header_t *)data;
                upload_seg->coe_header.number = 0;
                upload_seg->coe_header.reserved = 0;
                upload_seg->coe_header.service = EC_COE_SERVICE_SDO_REQUEST;
                upload_seg->sdo_header.more_follows = 0;
                upload_seg->sdo_header.toggle = toggle ? 1 : 0;
                upload_seg->sdo_header.command = EC_COE_REQUEST_SEGMENT_UPLOAD;
                upload_seg->sdo_header.segdata_size = 0;
                memset(data + EC_COE_DOWN_SEG_REQ_HEADER_SIZE, 0x00, 7);
                ret = ec_mailbox_send(master, slave_index, datagram);
                if (ret < 0) {
                    return ret;
                }

                ret = ec_mailbox_receive(master, slave_index, datagram, &mbox_proto, &recv_size, EC_COE_TIMEOUT_US);
                if (ret < 0) {
                    return ret;
                }

                if (mbox_proto != EC_MBOX_TYPE_COE) {
                    return -EC_ERR_COE_TYPE;
                }

                if (recv_size < EC_COE_UP_REQ_HEADER_SIZE) {
                    return -EC_ERR_COE_SIZE;
                }

                data = datagram->data + EC_MBOX_HEADER_SIZE;

                if (EC_READ_U16(data) >> 12 == EC_COE_SERVICE_SDO_REQUEST &&
                    EC_READ_U8(data + 2) >> 5 == EC_COE_REQUEST_ABORT) {
                    EC_SLAVE_LOG_ERR("Slave %u SDO abort code: 0x%08x (%s)\n", slave_index, EC_READ_U32(data + 6), ec_sdo_abort_string(EC_READ_U32(data + 6)));
                    return -EC_ERR_COE_ABORT;
                }

                if (EC_READ_U16(data) >> 12 != EC_COE_SERVICE_SDO_RESPONSE ||
                    EC_READ_U8(data + 2) >> 5 != EC_COE_RESPONSE_SEGMENT_UPLOAD) {
                    return -EC_ERR_COE_REQUEST;
                }

                data_size = recv_size - EC_COE_UP_SEG_REQ_HEADER_SIZE;

                if (recv_size == EC_COE_UP_REQ_HEADER_SIZE) {
                    uint8_t seg_size = (EC_READ_U8(data + 2) & 0xE) >> 1;
                    data_size -= seg_size;
                }

                if ((offset + data_size) > total_size) {
                    return -EC_ERR_COE_SIZE;
                }

                ec_memcpy(ptr, data + EC_COE_UP_SEG_REQ_HEADER_SIZE, data_size);
                ptr += data_size;
                offset += data_size;
                toggle ^= 1;

                last = EC_READ_U8(data + 2) & 0x01;

                if (last) {
                    if (offset != total_size) {
                        return -EC_ERR_COE_SIZE;
                    }

                    if (size) {
                        *size = total_size;
                    }
                    return 0;
                }
            }
        } else {
            if (size) {
                *size = total_size;
            }
            return 0;
        }
    }
}