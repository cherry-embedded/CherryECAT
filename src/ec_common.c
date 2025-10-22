/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ec_master.h"

#define ALIGN_UP_DWORD(x) ((uint32_t)(uintptr_t)(x) & (sizeof(uint32_t) - 1))

static inline void dword2array(char *addr, uint32_t w)
{
    addr[0] = w;
    addr[1] = w >> 8;
    addr[2] = w >> 16;
    addr[3] = w >> 24;
}

EC_FAST_CODE_SECTION void *ec_memcpy(void *s1, const void *s2, size_t n)
{
    char *b1 = (char *)s1;
    const char *b2 = (const char *)s2;
    uint32_t *w1;
    const uint32_t *w2;

    if (ALIGN_UP_DWORD(b1) == ALIGN_UP_DWORD(b2)) {
        while (ALIGN_UP_DWORD(b1) != 0 && n > 0) {
            *b1++ = *b2++;
            --n;
        }

        w1 = (uint32_t *)b1;
        w2 = (const uint32_t *)b2;

        while (n >= 4 * sizeof(uint32_t)) {
            *w1++ = *w2++;
            *w1++ = *w2++;
            *w1++ = *w2++;
            *w1++ = *w2++;
            n -= 4 * sizeof(uint32_t);
        }

        while (n >= sizeof(uint32_t)) {
            *w1++ = *w2++;
            n -= sizeof(uint32_t);
        }

        b1 = (char *)w1;
        b2 = (const char *)w2;

        while (n--) {
            *b1++ = *b2++;
        }
    } else {
        while (n > 0 && ALIGN_UP_DWORD(b2) != 0) {
            *b1++ = *b2++;
            --n;
        }

        w2 = (const uint32_t *)b2;

        while (n >= 4 * sizeof(uint32_t)) {
            dword2array(b1, *w2++);
            b1 += sizeof(uint32_t);
            dword2array(b1, *w2++);
            b1 += sizeof(uint32_t);
            dword2array(b1, *w2++);
            b1 += sizeof(uint32_t);
            dword2array(b1, *w2++);
            b1 += sizeof(uint32_t);
            n -= 4 * sizeof(uint32_t);
        }

        while (n >= sizeof(uint32_t)) {
            dword2array(b1, *w2++);
            b1 += sizeof(uint32_t);
            n -= sizeof(uint32_t);
        }

        b2 = (const char *)w2;

        while (n--) {
            *b1++ = *b2++;
        }
    }
    return s1;
}

EC_FAST_CODE_SECTION void ec_memset(void *s, int c, size_t n)
{
    char *b = (char *)s;
    uint32_t *w;

    while (ALIGN_UP_DWORD(b) != 0 && n > 0) {
        *b++ = (char)c;
        --n;
    }

    w = (uint32_t *)b;
    c = (c & 0xff) | ((c & 0xff) << 8) | ((c & 0xff) << 16) | ((c & 0xff) << 24);

    while (n >= 4 * sizeof(uint32_t)) {
        *w++ = c;
        *w++ = c;
        *w++ = c;
        *w++ = c;
        n -= 4 * sizeof(uint32_t);
    }

    while (n >= sizeof(uint32_t)) {
        *w++ = c;
        n -= sizeof(uint32_t);
    }

    b = (char *)w;

    while (n--) {
        *b++ = (char)c;
    }
}

typedef struct {
    uint32_t code;
    const char *message;
} ec_code_msg_t;

const char *ec_state_string(uint8_t states, uint8_t multi)
{
    uint32_t off = 0;
    unsigned int first = 1;
    static char buffer[64];

    memset(buffer, 0, sizeof(buffer));

    if (!states) {
        off += sprintf(buffer + off, "(unknown)");
        return buffer;
    }

    if (multi) { // multiple slaves
        if (states & EC_SLAVE_STATE_INIT) {
            off += sprintf(buffer + off, "INIT");
            first = 0;
        }
        if (states & EC_SLAVE_STATE_PREOP) {
            if (!first)
                off += sprintf(buffer + off, ", ");
            off += sprintf(buffer + off, "PREOP");
            first = 0;
        }
        if (states & EC_SLAVE_STATE_SAFEOP) {
            if (!first)
                off += sprintf(buffer + off, ", ");
            off += sprintf(buffer + off, "SAFEOP");
            first = 0;
        }
        if (states & EC_SLAVE_STATE_OP) {
            if (!first)
                off += sprintf(buffer + off, ", ");
            off += sprintf(buffer + off, "OP");
        }
    } else { // single slave
        if ((states & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_INIT) {
            off += sprintf(buffer + off, "INIT");
        } else if ((states & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_PREOP) {
            off += sprintf(buffer + off, "PREOP");
        } else if ((states & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_BOOT) {
            off += sprintf(buffer + off, "BOOT");
        } else if ((states & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_SAFEOP) {
            off += sprintf(buffer + off, "SAFEOP");
        } else if ((states & EC_SLAVE_STATE_MASK) == EC_SLAVE_STATE_OP) {
            off += sprintf(buffer + off, "OP");
        } else {
            off += sprintf(buffer + off, "(invalid)");
        }
        first = 0;
    }

    if (states & EC_SLAVE_STATE_ACK_ERR) {
        if (!first)
            off += sprintf(buffer + off, " + ");
        off += sprintf(buffer + off, "ERROR");
    }

    return buffer;
}

const ec_code_msg_t ec_mbox_protocol_messages[] = {
    { EC_MBXPROT_AOE, "AOE" },
    { EC_MBXPROT_EOE, "EOE" },
    { EC_MBXPROT_COE, "COE" },
    { EC_MBXPROT_FOE, "FOE" },
    { EC_MBXPROT_SOE, "SOE" },
    { EC_MBXPROT_VOE, "VOE" },
    { 0, "" }
};

const char *ec_mbox_protocol_string(uint8_t prot)
{
    static char buffer[64];

    memset(buffer, 0, sizeof(buffer));

    int first = 1;
    for (uint32_t i = 0; ec_mbox_protocol_messages[i].code != 0; i++) {
        if (prot & ec_mbox_protocol_messages[i].code) {
            if (!first) {
                strcat(buffer, "|");
            }
            strcat(buffer, ec_mbox_protocol_messages[i].message);
            first = 0;
        }
    }
    if (first) {
        return "";
    }
    return buffer;
}

const ec_code_msg_t al_status_messages[] = {
    { 0x0000, "No error" },
    { 0x0001, "Unspecified error" },
    { 0x0002, "No Memory" },
    { 0x0011, "Invalid requested state change" },
    { 0x0012, "Unknown requested state" },
    { 0x0013, "Bootstrap not supported" },
    { 0x0014, "No valid firmware" },
    { 0x0015, "Invalid mailbox configuration (BOOT state)" },
    { 0x0016, "Invalid mailbox configuration (PreOP state)" },
    { 0x0017, "Invalid sync manager configuration" },
    { 0x0018, "No valid inputs available" },
    { 0x0019, "No valid outputs" },
    { 0x001A, "Synchronization error" },
    { 0x001B, "Sync manager watchdog" },
    { 0x001C, "Invalid sync manager types" },
    { 0x001D, "Invalid output configuration" },
    { 0x001E, "Invalid input configuration" },
    { 0x001F, "Invalid watchdog configuration" },
    { 0x0020, "Slave needs cold start" },
    { 0x0021, "Slave needs INIT" },
    { 0x0022, "Slave needs PREOP" },
    { 0x0023, "Slave needs SAFEOP" },
    { 0x0024, "Invalid Input Mapping" },
    { 0x0025, "Invalid Output Mapping" },
    { 0x0026, "Inconsistent Settings" },
    { 0x0027, "Freerun not supported" },
    { 0x0028, "Synchronization not supported" },
    { 0x0029, "Freerun needs 3 Buffer Mode" },
    { 0x002A, "Background Watchdog" },
    { 0x002B, "No Valid Inputs and Outputs" },
    { 0x002C, "Fatal Sync Error" },
    { 0x002D, "No Sync Error" },
    { 0x0030, "Invalid DC SYNCH configuration" },
    { 0x0031, "Invalid DC latch configuration" },
    { 0x0032, "PLL error" },
    { 0x0033, "DC Sync IO Error" },
    { 0x0034, "DC Sync Timeout Error" },
    { 0x0035, "DC Invalid Sync Cycle Time" },
    { 0x0036, "DC Sync0 Cycle Time" },
    { 0x0037, "DC Sync1 Cycle Time" },
    { 0x0041, "MBX_AOE" },
    { 0x0042, "MBX_EOE" },
    { 0x0043, "MBX_COE" },
    { 0x0044, "MBX_FOE" },
    { 0x0045, "MBX_SOE" },
    { 0x004F, "MBX_VOE" },
    { 0x0050, "EEPROM No Access" },
    { 0x0051, "EEPROM Error" },
    { 0x0060, "Slave Restarted Locally" },
    { 0xffff }
};

const char *ec_alstatus_string(uint16_t errorcode)
{
    for (uint32_t i = 0; al_status_messages[i].code != 0xffff; i++) {
        if (al_status_messages[i].code == errorcode) {
            return al_status_messages[i].message;
        }
    }
    return "Unknown errorcode";
}

const ec_code_msg_t mbox_error_messages[] = {
    { 0x00000001, "MBXERR_SYNTAX" },
    { 0x00000002, "MBXERR_UNSUPPORTEDPROTOCOL" },
    { 0x00000003, "MBXERR_INVAILDCHANNEL" },
    { 0x00000004, "MBXERR_SERVICENOTSUPPORTED" },
    { 0x00000005, "MBXERR_INVALIDHEADER" },
    { 0x00000006, "MBXERR_SIZETOOSHORT" },
    { 0x00000007, "MBXERR_NOMOREMEMORY" },
    { 0x00000008, "MBXERR_INVALIDSIZE" },
    { 0xffffffff }
};

const char *ec_mbox_error_string(uint16_t errorcode)
{
    for (uint32_t i = 0; mbox_error_messages[i].code != 0xffffffff; i++) {
        if (mbox_error_messages[i].code == errorcode) {
            return mbox_error_messages[i].message;
        }
    }
    return "Unknown errorcode";
}

const ec_code_msg_t sdo_abort_messages[] = {
    { 0x05030000, "Toggle bit not changed" },
    { 0x05040000, "SDO protocol timeout" },
    { 0x05040001, "Client/Server command specifier not valid or unknown" },
    { 0x05040005, "Out of memory" },
    { 0x06010000, "Unsupported access to an object" },
    { 0x06010001, "Attempt to read a write-only object" },
    { 0x06010002, "Attempt to write a read-only object" },
    { 0x06020000, "This object does not exist in the object directory" },
    { 0x06040041, "The object cannot be mapped into the PDO" },
    { 0x06040042, "The number and length of the objects to be mapped would"
                  " exceed the PDO length" },
    { 0x06040043, "General parameter incompatibility reason" },
    { 0x06040047, "Gerneral internal incompatibility in device" },
    { 0x06060000, "Access failure due to a hardware error" },
    { 0x06070010, "Data type does not match, length of service parameter does"
                  " not match" },
    { 0x06070012, "Data type does not match, length of service parameter too"
                  " high" },
    { 0x06070013, "Data type does not match, length of service parameter too"
                  " low" },
    { 0x06090011, "Subindex does not exist" },
    { 0x06090030, "Value range of parameter exceeded" },
    { 0x06090031, "Value of parameter written too high" },
    { 0x06090032, "Value of parameter written too low" },
    { 0x06090036, "Maximum value is less than minimum value" },
    { 0x08000000, "General error" },
    { 0x08000020, "Data cannot be transferred or stored to the application" },
    { 0x08000021, "Data cannot be transferred or stored to the application"
                  " because of local control" },
    { 0x08000022, "Data cannot be transferred or stored to the application"
                  " because of the present device state" },
    { 0x08000023, "Object dictionary dynamic generation fails or no object"
                  " dictionary is present" },
    { 0xffffffff }
};

const char *ec_sdo_abort_string(uint32_t errorcode)
{
    for (uint32_t i = 0; sdo_abort_messages[i].code != 0xffffffff; i++) {
        if (sdo_abort_messages[i].code == errorcode) {
            return sdo_abort_messages[i].message;
        }
    }
    return "Unknown errorcode";
}

const ec_code_msg_t foe_errcode_messages[] = {
    { EC_FOE_ERRCODE_NOTDEFINED, "Not defined" },
    { EC_FOE_ERRCODE_NOTFOUND, "The file requested by an FoE upload service could not be found on the server" },
    { EC_FOE_ERRCODE_ACCESS, "Read or write access to this file not allowed (e.g. due to local control)" },
    { EC_FOE_ERRCODE_DISKFULL, "Disk to store file is full or memory allocation exceeded" },
    { EC_FOE_ERRCODE_ILLEGAL, "Illegal FoE operation, e.g. service identifier invalid" },
    { EC_FOE_ERRCODE_PACKENO, "FoE packet number invalid" },
    { EC_FOE_ERRCODE_EXISTS, "The file which is requested to be downloaded does already exist" },
    { EC_FOE_ERRCODE_NOUSER, "No User" },
    { EC_FOE_ERRCODE_BOOTSTRAPONLY, "FoE only supported in Bootstrap" },
    { EC_FOE_ERRCODE_NOTINBOOTSTRAP, "This file may not be accessed in BOOTSTRAP state" },
    { EC_FOE_ERRCODE_NORIGHTS, "Password invalid" },
    { EC_FOE_ERRCODE_PROGERROR, "Generic programming error. Should only be returned if  error reason cannot be distinguished" },
    { EC_FOE_ERRCODE_INVALID_CHECKSUM, "checksum included in the file is invalid" },
    { EC_FOE_ERRCODE_INVALID_FIRMWARE, "The hardware does not support the downloaded firmware" },
    { EC_FOE_ERRCODE_NO_FILE, "Do not use (identical with 0x8001)" },
    { EC_FOE_ERRCODE_NO_FILE_HEADER, "Missing file header of error in file header" },
    { EC_FOE_ERRCODE_FLASH_ERROR, "Flash cannot be accessed" },
    { 0xffffffff }

};

const char *ec_foe_errorcode_string(uint16_t errorcode)
{
    for (uint32_t i = 0; foe_errcode_messages[i].code != 0xffffffff; i++) {
        if (foe_errcode_messages[i].code == errorcode) {
            return foe_errcode_messages[i].message;
        }
    }
    return "Unknown errorcode";
}

const ec_code_msg_t eoe_errcode_messages[] = {
    { EC_EOE_RESULT_NOERROR, "No Error" },
    { EC_EOE_RESULT_UNSPECIFIED_ERROR, "Unspecified error" },
    { EC_EOE_RESULT_UNSUPPORTED_TYPE, "Unsupported type" },
    { EC_EOE_RESULT_NO_IP_SUPPORT, "No IP support" },
    { EC_EOE_RESULT_NO_DHCP_SUPPORT, "No DHCP support" },
    { EC_EOE_RESULT_NO_MACFILTERMASK_SUPPORT, "No MAC filter/mask support" },

    { 0xffffffff }
};

const char *ec_eoe_errorcode_string(uint16_t errorcode)
{
    for (uint32_t i = 0; eoe_errcode_messages[i].code != 0xffffffff; i++) {
        if (eoe_errcode_messages[i].code == errorcode) {
            return eoe_errcode_messages[i].message;
        }
    }
    return "Unknown errorcode";
}