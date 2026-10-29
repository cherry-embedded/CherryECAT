#ifndef CIA402_DEF_H
#define CIA402_DEF_H

#include "ec_util.h"

/**
STRUCT_PACKED_START: Is defined before the typedef struct construct to pack the generic structures if necessary */
#ifndef STRUCT_PACKED_START
#define STRUCT_PACKED_START                       __PACKED_STRUCT
#endif

/**
STRUCT_PACKED_END: Is defined after the typedef struct {} construct to pack the generic structures if necessary */
#ifndef STRUCT_PACKED_END
#define STRUCT_PACKED_END
#endif

/*---------------------------------------------
-    ControlWord Commands (IEC61800_184e)
-----------------------------------------------*/
#define CONTROLWORD_COMMAND_SHUTDOWN                 0x0006 /**< \brief Shutdown command*/
#define CONTROLWORD_COMMAND_SWITCHON                 0x0007 /**< \brief Switch on command*/
#define CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION 0x000F /**< \brief Switch on & Enable command*/
#define CONTROLWORD_COMMAND_DISABLEVOLTAGE           0x0000 /**< \brief Disable voltage command*/
#define CONTROLWORD_COMMAND_QUICKSTOP                0x0002 /**< \brief Quickstop command*/
#define CONTROLWORD_COMMAND_DISABLEOPERATION         0x0007 /**< \brief Disable operation command*/
#define CONTROLWORD_COMMAND_ENABLEOPERATION          0x000F /**< \brief Enable operation command*/
#define CONTROLWORD_COMMAND_FAULTRESET               0x0080 /**< \brief Fault reset command*/

/*---------------------------------------------
-    StatusWord Masks and Flags
-----------------------------------------------*/
#define STATUSWORD_STATE_MASK                        0x006F /**< \brief State mask*/
#define STATUSWORD_VOLTAGE_ENABLED                   0x0010 /**< \brief Indicate high voltage enabled*/
#define STATUSWORD_WARNING                           0x0080 /**< \brief Warning active*/
#define STATUSWORD_MANUFACTORSPECIFIC                0x0100 /**< \brief Manufacturer specific*/
#define STATUSWORD_INTERNAL_LIMIT                    0x0800 /**< \brief Internal limit*/
#define STATUSWORD_REMOTE                            0x0200 /**< \brief Set if the control word is processed*/
#define STATUSWORD_TARGET_REACHED                    0x0400 /**< \brief Target reached*/
#define STATUSWORD_INTERNALLIMITACTIVE               0x0800 /**< \brief Internal limit active*/
#define STATUSWORD_DRIVE_FOLLOWS_COMMAND             0x1000 /**< \brief Drive follows command (used in cyclic synchronous modes)*/

/*---------------------------------------------
-    StatusWord
-----------------------------------------------*/
#define STATUSWORD_STATE_NOTREADYTOSWITCHON          0x0000 /**< \brief Not ready to switch on*/
#define STATUSWORD_STATE_SWITCHEDONDISABLED          0x0040 /**< \brief Switched on but disabled*/
#define STATUSWORD_STATE_READYTOSWITCHON             0x0021 /**< \brief Ready to switch on*/
#define STATUSWORD_STATE_SWITCHEDON                  0x0023 /**< \brief Switched on*/
#define STATUSWORD_STATE_OPERATIONENABLED            0x0027 /**< \brief Operation enabled*/
#define STATUSWORD_STATE_QUICKSTOPACTIVE             0x0007 /**< \brief Quickstop active*/
#define STATUSWORD_STATE_FAULTREACTIONACTIVE         0x000F /**< \brief Fault reaction active*/
#define STATUSWORD_STATE_FAULT                       0x0008 /**< \brief Fault state*/

/*---------------------------------------------
-    CiA402 Modes of Operation (object 0x6060) (IEC61800_184e)
-----------------------------------------------*/
// -128 to -1 Manufacturer-specific operation modes
#define NO_MODE                                      0 /**< \brief No mode*/
#define PROFILE_POSITION_MODE                        1 /**< \brief Position Profile mode*/
#define VELOCITY_MODE                                2 /**< \brief Velocity mode*/
#define PROFILE_VELOCITY_MOCE                        3 /**< \brief Velocity Profile mode*/
#define PROFILE_TORQUE_MODE                          4 /**< \brief Torque Profile mode*/
//5 reserved
#define HOMING_MODE                                  6  /**< \brief Homing mode*/
#define INTERPOLATION_POSITION_MODE                  7  /**< \brief Interpolation Position mode*/
#define CYCLIC_SYNC_POSITION_MODE                    8  /**< \brief Cyclic Synchronous Position mode*/
#define CYCLIC_SYNC_VELOCITY_MODE                    9  /**< \brief Cyclic Synchronous Velocity mode*/
#define CYCLIC_SYNC_TORQUE_MODE                      10 /**< \brief Cyclic Synchronous Torque mode*/
//+11 to +127 reserved

/**
 * \addtogroup PDO Process Data Objects
 * @{
 */
/** \brief Data structure to handle the process data transmitted via 0x1A00 (csp/csv TxPDO)*/
typedef STRUCT_PACKED_START
{
    uint16_t ObjStatusWord; /**< \brief Status word (0x6041)*/
    int32_t ObjPositionActualValue; /**< \brief Actual position (0x6064)*/
    int32_t ObjVelocityActualValue; /**< \brief Actual velocity (0x606C)*/
    int16_t ObjModesOfOperationDisplay; /**< \brief Current mode of operation (0x6061)*/
}STRUCT_PACKED_END
TCiA402PDO1A00;


/** \brief Data structure to handle the process data transmitted via 0x1A01 (csp TxPDO)*/
typedef STRUCT_PACKED_START
{
    uint16_t ObjStatusWord; /**< \brief Status word (0x6041)*/
    int32_t ObjPositionActualValue; /**< \brief Actual position (0x6064)*/
    uint16_t Padding16Bit; /**< \brief 16bit padding*/
}STRUCT_PACKED_END
TCiA402PDO1A01;


/** \brief Data structure to handle the process data transmitted via 0x1A02 (csv TxPDO)*/
typedef STRUCT_PACKED_START
{
    uint16_t ObjStatusWord; /**< \brief Status word (0x6041)*/
    int32_t ObjPositionActualValue; /**< \brief Actual position (0x6064)*/
    uint16_t Padding16Bit; /**< \brief 16bit padding*/
}STRUCT_PACKED_END
TCiA402PDO1A02;


/** \brief Data structure to handle the process data transmitted via 0x1600 (csp/csv RxPDO)*/
typedef STRUCT_PACKED_START
{
    uint16_t ObjControlWord; /**< \brief Control word (0x6040)*/
    int32_t ObjTargetPosition; /**< \brief Target position (0x607A)*/
    int32_t ObjTargetVelocity; /**< \brief Target velocity (0x60FF)*/
    int16_t ObjModesOfOperation; /**< \brief Mode of operation (0x6060)*/
}STRUCT_PACKED_END
TCiA402PDO1600;


/** \brief Data structure to handle the process data transmitted via 0x1601 (csp RxPDO)*/
typedef STRUCT_PACKED_START
{
    uint16_t ObjControlWord; /**< \brief Control word (0x6040)*/
    int32_t ObjTargetPosition; /**< \brief Target position (0x607A)*/
    uint16_t Padding16Bit; /**< \brief 16bit padding*/
}STRUCT_PACKED_END
TCiA402PDO1601;


/** \brief Data structure to handle the process data transmitted via 0x1602 (csv RxPDO)*/
typedef STRUCT_PACKED_START
{
    uint16_t ObjControlWord; /**< \brief Control word (0x6040)*/
    int32_t ObjTargetVelocity; /**< \brief Target velocity (0x60FF)*/
    uint16_t Padding16Bit; /**< \brief 16bit padding*/
}STRUCT_PACKED_END
TCiA402PDO1602;
/** @}*/

#endif