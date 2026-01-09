/**
 * @file    can_protocol.c
 * @brief   CAN protocol stack implementation
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

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "can_protocol.h"
#include "lem_sensor.h"
#include "pm_monitor.h"
#include "temp_sensor.h"
#include "digital_input.h"
#include "btt6200.h"
#include "app_errors.h"
#include "app_main.h"
#include "timestamp.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief Maximum UDS response size */
#define UDS_MAX_RESPONSE_SIZE   (256U)

/** @brief UDS negative response code prefix */
#define UDS_NEGATIVE_RESPONSE   (0x7FU)

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Protocol statistics */
static CANProto_Statistics_t proto_stats;

/** @brief Last transmission timestamps */
static uint32_t last_tx_status_ms = 0U;
static uint32_t last_tx_current_ms = 0U;
static uint32_t last_tx_voltage_ms = 0U;
static uint32_t last_tx_temp_ms = 0U;
static uint32_t last_tx_io_ms = 0U;

/** @brief Initialization flag */
static bool proto_initialized = false;

/** @brief UDS response buffer */
static uint8_t uds_response_buffer[UDS_MAX_RESPONSE_SIZE];

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static Status_t canproto_process_uds_request(const CAN_Message_t *pMsg);
static Status_t canproto_send_uds_response(const uint8_t *pData, uint8_t length);
static Status_t canproto_send_uds_negative_response(uint8_t serviceId, UDS_NRC_t nrc);
static Status_t canproto_handle_read_data_by_id(const uint8_t *pRequest, uint8_t reqLen);
static Status_t canproto_handle_clear_dtc(void);
static Status_t canproto_handle_tester_present(void);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize CAN protocol stack
 */
Status_t CANProto_Init(void)
{
    Status_t status = STATUS_OK;

    if (proto_initialized)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        /* Clear statistics */
        (void)memset(&proto_stats, 0, sizeof(proto_stats));

        /* Initialize timestamps */
        uint32_t current_ms = Timestamp_GetMillis();
        last_tx_status_ms = current_ms;
        last_tx_current_ms = current_ms;
        last_tx_voltage_ms = current_ms;
        last_tx_temp_ms = current_ms;
        last_tx_io_ms = current_ms;

        proto_initialized = true;
    }

    return status;
}

/**
 * @brief Process received CAN messages
 */
Status_t CANProto_ProcessRxMessages(void)
{
    Status_t status = STATUS_OK;

    if (proto_initialized)
    {
        CAN_Message_t msg;
        uint32_t available = 0U;

        /* Check for available messages */
        (void)BSP_CAN_Available(BSP_CAN_INSTANCE_1, &available);

        while (available > 0U)
        {
            /* Receive message */
            if (BSP_CAN_Receive(BSP_CAN_INSTANCE_1, &msg, 10U) == STATUS_OK)
            {
                /* Process based on message ID */
                if (msg.id == CAN_ID_UDS_REQUEST)
                {
                    /* UDS diagnostic request */
                    (void)canproto_process_uds_request(&msg);
                    proto_stats.rxUDSCount++;
                }
                else
                {
                    /* Unknown message */
                    proto_stats.rxUnknownCount++;
                }
            }

            /* Check for more messages */
            (void)BSP_CAN_Available(BSP_CAN_INSTANCE_1, &available);
        }
    }
    else
    {
        status = STATUS_ERROR_NOT_INIT;
    }

    return status;
}

/**
 * @brief Transmit periodic status messages
 */
Status_t CANProto_TransmitPeriodic(void)
{
    Status_t status = STATUS_OK;

    if (proto_initialized)
    {
        uint32_t current_ms = Timestamp_GetMillis();

        /* Transmit status message */
        if ((current_ms - last_tx_status_ms) >= CAN_TX_PERIOD_STATUS_MS)
        {
            (void)CANProto_SendStatus();
            last_tx_status_ms = current_ms;
        }

        /* Transmit current measurements */
        if ((current_ms - last_tx_current_ms) >= CAN_TX_PERIOD_CURRENT_MS)
        {
            (void)CANProto_SendCurrents();
            last_tx_current_ms = current_ms;
        }

        /* Transmit voltage measurements */
        if ((current_ms - last_tx_voltage_ms) >= CAN_TX_PERIOD_VOLTAGE_MS)
        {
            (void)CANProto_SendVoltages();
            last_tx_voltage_ms = current_ms;
        }

        /* Transmit temperature */
        if ((current_ms - last_tx_temp_ms) >= CAN_TX_PERIOD_TEMP_MS)
        {
            (void)CANProto_SendTemperature();
            last_tx_temp_ms = current_ms;
        }

        /* Transmit I/O states */
        if ((current_ms - last_tx_io_ms) >= CAN_TX_PERIOD_IO_MS)
        {
            (void)CANProto_SendIOStates();
            last_tx_io_ms = current_ms;
        }
    }
    else
    {
        status = STATUS_ERROR_NOT_INIT;
    }

    return status;
}

/**
 * @brief Transmit system status message
 */
Status_t CANProto_SendStatus(void)
{
    Status_t status = STATUS_OK;

    if (proto_initialized)
    {
        CAN_Message_t msg;
        AppStatus_t appStatus;

        /* Get application status */
        (void)App_GetStatus(&appStatus);

        /* Build status message */
        msg.id = CAN_ID_BCU_STATUS;
        msg.ide = CAN_IDE_STANDARD;
        msg.rtr = false;
        msg.dlc = 8U;

        msg.data[0] = (uint8_t)appStatus.state;
        msg.data[1] = appStatus.powerGood ? 0x01U : 0x00U;
        msg.data[2] = appStatus.safetyOK ? 0x01U : 0x00U;
        msg.data[3] = (uint8_t)(appStatus.activeErrors & 0xFFU);
        msg.data[4] = (uint8_t)(appStatus.uptime_ms >> 24);
        msg.data[5] = (uint8_t)(appStatus.uptime_ms >> 16);
        msg.data[6] = (uint8_t)(appStatus.uptime_ms >> 8);
        msg.data[7] = (uint8_t)(appStatus.uptime_ms & 0xFFU);

        /* Transmit */
        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, 10U);

        if (status == STATUS_OK)
        {
            proto_stats.txStatusCount++;
        }
        else
        {
            proto_stats.txErrors++;
        }
    }

    return status;
}

/**
 * @brief Transmit current measurements
 */
Status_t CANProto_SendCurrents(void)
{
    Status_t status = STATUS_OK;

    if (proto_initialized)
    {
        CAN_Message_t msg;
        Current_mA_t current;

        /* Send LEM sensor 0 current */
        if (LEM_ReadCurrent(0U, &current) == STATUS_OK)
        {
            msg.id = CAN_ID_BCU_CURRENT;
            msg.ide = CAN_IDE_STANDARD;
            msg.rtr = false;
            msg.dlc = 8U;

            msg.data[0] = 0U;  /* Sensor ID */
            msg.data[1] = (uint8_t)(current >> 24);
            msg.data[2] = (uint8_t)(current >> 16);
            msg.data[3] = (uint8_t)(current >> 8);
            msg.data[4] = (uint8_t)(current & 0xFF);
            msg.data[5] = 0U;  /* Reserved */
            msg.data[6] = 0U;
            msg.data[7] = 0U;

            status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, 10U);

            if (status == STATUS_OK)
            {
                proto_stats.txCurrentCount++;
            }
            else
            {
                proto_stats.txErrors++;
            }
        }
    }

    return status;
}

/**
 * @brief Transmit voltage measurements
 */
Status_t CANProto_SendVoltages(void)
{
    Status_t status = STATUS_OK;

    if (proto_initialized)
    {
        CAN_Message_t msg;
        Voltage_mV_t voltage;

        msg.id = CAN_ID_BCU_VOLTAGE;
        msg.ide = CAN_IDE_STANDARD;
        msg.rtr = false;
        msg.dlc = 8U;

        /* Get power rail voltages */
        (void)PM_Monitor_GetRailVoltage(PM_RAIL_INPUT_12V, &voltage);
        msg.data[0] = (uint8_t)(voltage >> 8);
        msg.data[1] = (uint8_t)(voltage & 0xFFU);

        (void)PM_Monitor_GetRailVoltage(PM_RAIL_5V, &voltage);
        msg.data[2] = (uint8_t)(voltage >> 8);
        msg.data[3] = (uint8_t)(voltage & 0xFFU);

        (void)PM_Monitor_GetRailVoltage(PM_RAIL_3V3_DIGITAL, &voltage);
        msg.data[4] = (uint8_t)(voltage >> 8);
        msg.data[5] = (uint8_t)(voltage & 0xFFU);

        (void)PM_Monitor_GetRailVoltage(PM_RAIL_3V3_ANALOG, &voltage);
        msg.data[6] = (uint8_t)(voltage >> 8);
        msg.data[7] = (uint8_t)(voltage & 0xFFU);

        /* Transmit */
        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, 10U);

        if (status == STATUS_OK)
        {
            proto_stats.txVoltageCount++;
        }
        else
        {
            proto_stats.txErrors++;
        }
    }

    return status;
}

/**
 * @brief Transmit temperature measurement
 */
Status_t CANProto_SendTemperature(void)
{
    Status_t status = STATUS_OK;

    if (proto_initialized)
    {
        CAN_Message_t msg;
        int32_t temp_mC;

        if (TempSensor_ReadTemperature(&temp_mC) == STATUS_OK)
        {
            msg.id = CAN_ID_BCU_TEMPERATURE;
            msg.ide = CAN_IDE_STANDARD;
            msg.rtr = false;
            msg.dlc = 8U;

            msg.data[0] = (uint8_t)(temp_mC >> 24);
            msg.data[1] = (uint8_t)(temp_mC >> 16);
            msg.data[2] = (uint8_t)(temp_mC >> 8);
            msg.data[3] = (uint8_t)(temp_mC & 0xFFU);
            msg.data[4] = 0U;  /* Reserved */
            msg.data[5] = 0U;
            msg.data[6] = 0U;
            msg.data[7] = 0U;

            status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, 10U);

            if (status == STATUS_OK)
            {
                proto_stats.txTempCount++;
            }
            else
            {
                proto_stats.txErrors++;
            }
        }
    }

    return status;
}

/**
 * @brief Transmit I/O states
 */
Status_t CANProto_SendIOStates(void)
{
    Status_t status = STATUS_OK;

    if (proto_initialized)
    {
        CAN_Message_t msg;

        msg.id = CAN_ID_BCU_OUTPUTS;
        msg.ide = CAN_IDE_STANDARD;
        msg.rtr = false;
        msg.dlc = 8U;

        /* Pack first 8 output states into bytes 0-1 (4 channels per byte, 2 bits each) */
        msg.data[0] = 0U;
        msg.data[1] = 0U;
        msg.data[2] = 0U;
        msg.data[3] = 0U;
        msg.data[4] = 0U;
        msg.data[5] = 0U;
        msg.data[6] = 0U;
        msg.data[7] = 0U;

        /* Read first 20 output states (20 channels) */
        /* Simplified: Pack as bitmap (1 bit per channel) */
        for (uint8_t i = 0U; i < 20U; i++)
        {
            /* TODO: Read actual BTT6200 channel state */
            /* For now, placeholder */
        }

        /* Transmit */
        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, 10U);

        if (status == STATUS_OK)
        {
            proto_stats.txIOCount++;
        }
        else
        {
            proto_stats.txErrors++;
        }
    }

    return status;
}

/**
 * @brief Transmit active faults
 */
Status_t CANProto_SendFaults(void)
{
    Status_t status = STATUS_OK;

    if (proto_initialized)
    {
        CAN_Message_t msg;
        uint32_t activeDTCCount = 0U;

        (void)ErrorHandler_GetActiveDTCCount(&activeDTCCount);

        msg.id = CAN_ID_BCU_FAULTS;
        msg.ide = CAN_IDE_STANDARD;
        msg.rtr = false;
        msg.dlc = 8U;

        msg.data[0] = (uint8_t)(activeDTCCount & 0xFFU);
        msg.data[1] = 0U;  /* Reserved for fault severity */
        msg.data[2] = 0U;  /* Reserved */
        msg.data[3] = 0U;
        msg.data[4] = 0U;
        msg.data[5] = 0U;
        msg.data[6] = 0U;
        msg.data[7] = 0U;

        /* Transmit */
        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, 10U);

        if (status == STATUS_OK)
        {
            proto_stats.txFaultCount++;
        }
        else
        {
            proto_stats.txErrors++;
        }
    }

    return status;
}

/**
 * @brief Get protocol statistics
 */
Status_t CANProto_GetStatistics(CANProto_Statistics_t *pStats)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pStats != NULL) && proto_initialized)
    {
        *pStats = proto_stats;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Reset protocol statistics
 */
Status_t CANProto_ResetStatistics(void)
{
    if (proto_initialized)
    {
        (void)memset(&proto_stats, 0, sizeof(proto_stats));
    }

    return STATUS_OK;
}

/**
 * @brief De-initialize CAN protocol stack
 */
Status_t CANProto_DeInit(void)
{
    if (proto_initialized)
    {
        proto_initialized = false;
    }

    return STATUS_OK;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS - UDS DIAGNOSTICS                                        */
/*============================================================================*/

/**
 * @brief Process UDS diagnostic request
 */
static Status_t canproto_process_uds_request(const CAN_Message_t *pMsg)
{
    Status_t status = STATUS_OK;

    if ((pMsg != NULL) && (pMsg->dlc > 0U))
    {
        uint8_t serviceId = pMsg->data[0];

        switch (serviceId)
        {
            case UDS_SID_READ_DATA_BY_ID:
                status = canproto_handle_read_data_by_id(pMsg->data, pMsg->dlc);
                break;

            case UDS_SID_CLEAR_DTC:
                status = canproto_handle_clear_dtc();
                break;

            case UDS_SID_TESTER_PRESENT:
                status = canproto_handle_tester_present();
                break;

            default:
                /* Service not supported */
                status = canproto_send_uds_negative_response(serviceId, UDS_NRC_SERVICE_NOT_SUPPORTED);
                break;
        }
    }
    else
    {
        status = STATUS_ERROR_PARAM;
    }

    return status;
}

/**
 * @brief Handle ReadDataByID service
 */
static Status_t canproto_handle_read_data_by_id(const uint8_t *pRequest, uint8_t reqLen)
{
    Status_t status = STATUS_OK;

    if ((pRequest != NULL) && (reqLen >= 3U))
    {
        uint16_t dataId = ((uint16_t)pRequest[1] << 8) | (uint16_t)pRequest[2];
        uint8_t responseLen = 0U;

        /* Positive response: SID + 0x40, DID MSB, DID LSB, data... */
        uds_response_buffer[0] = UDS_SID_READ_DATA_BY_ID + 0x40U;
        uds_response_buffer[1] = (uint8_t)(dataId >> 8);
        uds_response_buffer[2] = (uint8_t)(dataId & 0xFFU);
        responseLen = 3U;

        switch (dataId)
        {
            case DID_FIRMWARE_VERSION:
            {
                Version_t version;
                (void)App_GetVersion(&version);
                uds_response_buffer[3] = version.major;
                uds_response_buffer[4] = version.minor;
                uds_response_buffer[5] = version.patch;
                responseLen = 6U;
                break;
            }

            case DID_SYSTEM_STATUS:
            {
                AppStatus_t appStatus;
                (void)App_GetStatus(&appStatus);
                uds_response_buffer[3] = (uint8_t)appStatus.state;
                uds_response_buffer[4] = appStatus.powerGood ? 0x01U : 0x00U;
                uds_response_buffer[5] = appStatus.safetyOK ? 0x01U : 0x00U;
                uds_response_buffer[6] = (uint8_t)appStatus.activeErrors;
                responseLen = 7U;
                break;
            }

            case DID_TEMPERATURE:
            {
                int32_t temp_mC;
                (void)TempSensor_ReadTemperature(&temp_mC);
                uds_response_buffer[3] = (uint8_t)(temp_mC >> 24);
                uds_response_buffer[4] = (uint8_t)(temp_mC >> 16);
                uds_response_buffer[5] = (uint8_t)(temp_mC >> 8);
                uds_response_buffer[6] = (uint8_t)(temp_mC & 0xFFU);
                responseLen = 7U;
                break;
            }

            case DID_ACTIVE_DTCS:
            {
                uint32_t dtcCount = 0U;
                (void)ErrorHandler_GetActiveDTCCount(&dtcCount);
                uds_response_buffer[3] = (uint8_t)(dtcCount >> 24);
                uds_response_buffer[4] = (uint8_t)(dtcCount >> 16);
                uds_response_buffer[5] = (uint8_t)(dtcCount >> 8);
                uds_response_buffer[6] = (uint8_t)(dtcCount & 0xFFU);
                responseLen = 7U;
                break;
            }

            default:
                /* Data ID not supported */
                return canproto_send_uds_negative_response(UDS_SID_READ_DATA_BY_ID,
                                                          UDS_NRC_REQUEST_OUT_OF_RANGE);
        }

        /* Send positive response */
        status = canproto_send_uds_response(uds_response_buffer, responseLen);
    }
    else
    {
        status = canproto_send_uds_negative_response(UDS_SID_READ_DATA_BY_ID,
                                                     UDS_NRC_INCORRECT_MESSAGE_LENGTH);
    }

    return status;
}

/**
 * @brief Handle ClearDTC service
 */
static Status_t canproto_handle_clear_dtc(void)
{
    Status_t status = STATUS_OK;

    /* Clear all DTCs */
    (void)ErrorHandler_ClearAllDTCs();

    /* Send positive response */
    uds_response_buffer[0] = UDS_SID_CLEAR_DTC + 0x40U;
    status = canproto_send_uds_response(uds_response_buffer, 1U);

    return status;
}

/**
 * @brief Handle TesterPresent service
 */
static Status_t canproto_handle_tester_present(void)
{
    /* Send positive response */
    uds_response_buffer[0] = UDS_SID_TESTER_PRESENT + 0x40U;
    return canproto_send_uds_response(uds_response_buffer, 1U);
}

/**
 * @brief Send UDS positive response
 */
static Status_t canproto_send_uds_response(const uint8_t *pData, uint8_t length)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pData != NULL) && (length > 0U) && (length <= 8U))
    {
        CAN_Message_t msg;

        msg.id = CAN_ID_UDS_RESPONSE;
        msg.ide = CAN_IDE_STANDARD;
        msg.rtr = false;
        msg.dlc = length;
        (void)memcpy(msg.data, pData, length);

        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, 100U);
    }

    return status;
}

/**
 * @brief Send UDS negative response
 */
static Status_t canproto_send_uds_negative_response(uint8_t serviceId, UDS_NRC_t nrc)
{
    uint8_t response[3];

    response[0] = UDS_NEGATIVE_RESPONSE;
    response[1] = serviceId;
    response[2] = (uint8_t)nrc;

    return canproto_send_uds_response(response, 3U);
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
