/**
 * @file    can_protocol.h
 * @brief   CAN protocol stack header
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO-TP (ISO 15765-2) transport protocol
 * @note    UDS (ISO 14229) diagnostic services
 *
 * @copyright Copyright (c) 2026
 */

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include "bsp_can.h"
#include "rul.h"

/*============================================================================*/
/* CONSTANTS                                                                  */
/*============================================================================*/

/** @brief CAN message IDs - TX (BCU -> Host) */
#define CAN_ID_BCU_STATUS           (0x100U)  /**< System status */
#define CAN_ID_BCU_CURRENT_1        (0x101U)  /**< LEM currents 0-3 */
#define CAN_ID_BCU_CURRENT_2        (0x102U)  /**< LEM currents 4-7 */
#define CAN_ID_BCU_CURRENT_3        (0x103U)  /**< LEM currents 8-9 */
#define CAN_ID_BCU_VOLTAGE          (0x104U)  /**< Power rail voltages */
#define CAN_ID_BCU_TEMPERATURE      (0x105U)  /**< Board temperature */
#define CAN_ID_BCU_OUTPUTS          (0x106U)  /**< Output states (bitmap) */
#define CAN_ID_BCU_INPUTS           (0x107U)  /**< Digital input states (bitmap) */
#define CAN_ID_BCU_FAULTS           (0x108U)  /**< Active fault count and codes */
#define CAN_ID_BCU_BTT_DIAG_1       (0x110U)  /**< BTT6200 IC0-1 diagnostics */
#define CAN_ID_BCU_BTT_DIAG_2       (0x111U)  /**< BTT6200 IC2-3 diagnostics */
#define CAN_ID_BCU_BTT_DIAG_3       (0x112U)  /**< BTT6200 IC4 diagnostics */
#define CAN_ID_BCU_BTT_CURRENT_1    (0x113U)  /**< BTT6200 IC0-1 sense currents */
#define CAN_ID_BCU_BTT_CURRENT_2    (0x114U)  /**< BTT6200 IC2-3 sense currents */
#define CAN_ID_BCU_BTT_CURRENT_3    (0x115U)  /**< BTT6200 IC4 sense current */
#define CAN_ID_BCU_RUL_STATUS       (0x120U)  /**< RUL prediction status */
#define CAN_ID_BCU_TIMING           (0x121U)  /**< Scheduler timing info */

/** @brief CAN message IDs - RX (Host -> BCU) Control Commands */
#define CAN_ID_CMD_OUTPUT_SET       (0x200U)  /**< Set single output ON/OFF */
#define CAN_ID_CMD_OUTPUT_ALL       (0x201U)  /**< Set all outputs (bitmap) */
#define CAN_ID_CMD_OUTPUT_PWM       (0x202U)  /**< Set output PWM duty cycle */
#define CAN_ID_CMD_SYSTEM           (0x210U)  /**< System commands (reset, safe state) */
#define CAN_ID_CMD_CONFIG           (0x211U)  /**< Configuration commands */

/** @brief UDS diagnostic request/response IDs */
#define CAN_ID_UDS_REQUEST          (0x7E0U)
#define CAN_ID_UDS_RESPONSE         (0x7E8U)

/** @brief Backwards compatibility aliases */
#define CAN_ID_BCU_CURRENT          CAN_ID_BCU_CURRENT_1
#define CAN_ID_BCU_DIAGNOSTICS      CAN_ID_BCU_BTT_DIAG_1

/** @brief Message transmission periods */
#define CAN_TX_PERIOD_STATUS_MS     (100U)
#define CAN_TX_PERIOD_CURRENT_MS    (50U)
#define CAN_TX_PERIOD_VOLTAGE_MS    (100U)
#define CAN_TX_PERIOD_TEMP_MS       (1000U)
#define CAN_TX_PERIOD_IO_MS         (100U)
#define CAN_TX_PERIOD_BTT_DIAG_MS   (100U)
#define CAN_TX_PERIOD_RUL_MS        (1000U)

/*============================================================================*/
/* TYPES                                                                      */
/*============================================================================*/

/** @brief System command codes (for CAN_ID_CMD_SYSTEM) */
typedef enum {
    CMD_SYS_NOP             = 0x00U,  /**< No operation */
    CMD_SYS_RESET           = 0x01U,  /**< Software reset */
    CMD_SYS_SAFE_STATE      = 0x02U,  /**< Enter safe state */
    CMD_SYS_CLEAR_FAULTS    = 0x03U,  /**< Clear all faults */
    CMD_SYS_ENABLE_OUTPUTS  = 0x10U,  /**< Enable output drivers */
    CMD_SYS_DISABLE_OUTPUTS = 0x11U   /**< Disable output drivers */
} SystemCommand_t;

/** @brief UDS service IDs */
typedef enum {
    UDS_SID_DIAGNOSTIC_SESSION       = 0x10,
    UDS_SID_ECU_RESET               = 0x11,
    UDS_SID_READ_DATA_BY_ID         = 0x22,
    UDS_SID_READ_MEMORY_BY_ADDRESS  = 0x23,
    UDS_SID_SECURITY_ACCESS         = 0x27,
    UDS_SID_WRITE_DATA_BY_ID        = 0x2E,
    UDS_SID_IO_CONTROL              = 0x2F,
    UDS_SID_ROUTINE_CONTROL         = 0x31,
    UDS_SID_REQUEST_DOWNLOAD        = 0x34,
    UDS_SID_TRANSFER_DATA           = 0x36,
    UDS_SID_REQUEST_TRANSFER_EXIT   = 0x37,
    UDS_SID_CLEAR_DTC               = 0x14,
    UDS_SID_READ_DTC                = 0x19,
    UDS_SID_TESTER_PRESENT          = 0x3E
} UDS_ServiceID_t;

/** @brief UDS negative response codes */
typedef enum {
    UDS_NRC_POSITIVE_RESPONSE               = 0x00,
    UDS_NRC_SERVICE_NOT_SUPPORTED           = 0x11,
    UDS_NRC_SUBFUNCTION_NOT_SUPPORTED       = 0x12,
    UDS_NRC_INCORRECT_MESSAGE_LENGTH        = 0x13,
    UDS_NRC_CONDITIONS_NOT_CORRECT          = 0x22,
    UDS_NRC_REQUEST_OUT_OF_RANGE            = 0x31,
    UDS_NRC_SECURITY_ACCESS_DENIED          = 0x33,
    UDS_NRC_INVALID_KEY                     = 0x35,
    UDS_NRC_EXCEEDED_ATTEMPTS               = 0x36,
    UDS_NRC_REQUIRED_TIME_NOT_EXPIRED       = 0x37,
    UDS_NRC_REQUEST_SEQUENCE_ERROR          = 0x24,
    UDS_NRC_GENERAL_REJECT                  = 0x10
} UDS_NRC_t;

/** @brief Data identifiers for ReadDataByID service */
typedef enum {
    DID_FIRMWARE_VERSION        = 0xF100,
    DID_HARDWARE_VERSION        = 0xF101,
    DID_SERIAL_NUMBER           = 0xF102,
    DID_SYSTEM_STATUS           = 0xF200,
    DID_LEM_CURRENTS            = 0xF201,
    DID_POWER_VOLTAGES          = 0xF202,
    DID_TEMPERATURE             = 0xF203,
    DID_OUTPUT_STATES           = 0xF204,
    DID_INPUT_STATES            = 0xF205,
    DID_ACTIVE_DTCS             = 0xF206,
    DID_CALIBRATION_STATUS      = 0xF207,
    DID_UPTIME                  = 0xF208
} DataIdentifier_t;

/** @brief Protocol statistics */
typedef struct {
    uint32_t txStatusCount;
    uint32_t txCurrentCount;
    uint32_t txVoltageCount;
    uint32_t txTempCount;
    uint32_t txIOCount;
    uint32_t txFaultCount;
    uint32_t rxUDSCount;
    uint32_t rxUnknownCount;
    uint32_t txErrors;
    uint32_t rxErrors;
} CANProto_Statistics_t;

/*============================================================================*/
/* PUBLIC FUNCTION PROTOTYPES                                                 */
/*============================================================================*/

/**
 * @brief Initialize CAN protocol stack
 * @return Status code
 */
Status_t CANProto_Init(void);

/**
 * @brief Process received CAN messages
 * @return Status code
 */
Status_t CANProto_ProcessRxMessages(void);

/**
 * @brief Transmit periodic status messages
 * @return Status code
 */
Status_t CANProto_TransmitPeriodic(void);

/**
 * @brief Transmit system status message
 * @return Status code
 */
Status_t CANProto_SendStatus(void);

/**
 * @brief Transmit current measurements
 * @return Status code
 */
Status_t CANProto_SendCurrents(void);

/**
 * @brief Transmit voltage measurements
 * @return Status code
 */
Status_t CANProto_SendVoltages(void);

/**
 * @brief Transmit temperature measurement
 * @return Status code
 */
Status_t CANProto_SendTemperature(void);

/**
 * @brief Transmit I/O states
 * @return Status code
 */
Status_t CANProto_SendIOStates(void);

/**
 * @brief Transmit active faults
 * @return Status code
 */
Status_t CANProto_SendFaults(void);

/**
 * @brief Transmit digital input states
 * @return Status code
 */
Status_t CANProto_SendInputStates(void);

/**
 * @brief Transmit BTT6200 diagnostics (all ICs)
 * @return Status code
 */
Status_t CANProto_SendBTTDiagnostics(void);

/**
 * @brief Transmit BTT6200 sense currents (all ICs)
 * @return Status code
 */
Status_t CANProto_SendBTTCurrents(void);

/**
 * @brief Transmit all LEM sensor currents (10 channels)
 * @return Status code
 */
Status_t CANProto_SendAllCurrents(void);

/**
 * @brief Transmit RUL status message
 * @param[in] pPrediction Pointer to RUL prediction data
 * @param[in] soh State of Health percentage (0-10000 = 0-100%)
 * @return Status code
 */
Status_t CANProto_SendRULStatus(const RUL_Prediction_t *pPrediction, Percentage_t soh);

/**
 * @brief Transmit scheduler timing message
 * @param[in] cycleTime_us Current cycle time in microseconds
 * @param[in] maxCycleTime_us Maximum cycle time in microseconds
 * @return Status code
 */
Status_t CANProto_SendTiming(uint32_t cycleTime_us, uint32_t maxCycleTime_us);

/**
 * @brief Get protocol statistics
 * @param[out] pStats Pointer to statistics structure
 * @return Status code
 */
Status_t CANProto_GetStatistics(CANProto_Statistics_t *pStats);

/**
 * @brief Reset protocol statistics
 * @return Status code
 */
Status_t CANProto_ResetStatistics(void);

/**
 * @brief De-initialize CAN protocol stack
 * @return Status code
 */
Status_t CANProto_DeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PROTOCOL_H */

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
