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
#include "bsp_adc.h"
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

/* Command handlers */
static Status_t canproto_handle_output_set(const CAN_Message_t *pMsg);
static Status_t canproto_handle_output_all(const CAN_Message_t *pMsg);
static Status_t canproto_handle_output_pwm(const CAN_Message_t *pMsg);
static Status_t canproto_handle_system_cmd(const CAN_Message_t *pMsg);

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
        (void)BSP_CAN_IsMessageAvailable(BSP_CAN_INSTANCE_1, 0U, &available);

        while (available > 0U)
        {
            /* Receive message */
            if (BSP_CAN_Receive(BSP_CAN_INSTANCE_1, 0U, &msg, 10U) == STATUS_OK)
            {
                /* Process based on message ID */
                switch (msg.id)
                {
                    case CAN_ID_UDS_REQUEST:
                        /* UDS diagnostic request */
                        (void)canproto_process_uds_request(&msg);
                        proto_stats.rxUDSCount++;
                        break;

                    case CAN_ID_CMD_OUTPUT_SET:
                        /* Set single output ON/OFF */
                        (void)canproto_handle_output_set(&msg);
                        break;

                    case CAN_ID_CMD_OUTPUT_ALL:
                        /* Set all outputs via bitmap */
                        (void)canproto_handle_output_all(&msg);
                        break;

                    case CAN_ID_CMD_OUTPUT_PWM:
                        /* Set output PWM duty cycle */
                        (void)canproto_handle_output_pwm(&msg);
                        break;

                    case CAN_ID_CMD_SYSTEM:
                        /* System command */
                        (void)canproto_handle_system_cmd(&msg);
                        break;

                    default:
                        /* Unknown message */
                        proto_stats.rxUnknownCount++;
                        break;
                }
            }

            /* Check for more messages */
            (void)BSP_CAN_IsMessageAvailable(BSP_CAN_INSTANCE_1, 0U, &available);
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
        msg.frameType = CAN_FRAME_STANDARD;
        msg.frameFormat = CAN_FRAME_DATA;
        msg.dataLength =8U;

        msg.data[0] = (uint8_t)appStatus.state;
        msg.data[1] = appStatus.powerGood ? 0x01U : 0x00U;
        msg.data[2] = appStatus.safetyOK ? 0x01U : 0x00U;
        msg.data[3] = (uint8_t)(appStatus.activeErrors & 0xFFU);
        msg.data[4] = (uint8_t)(appStatus.uptime_ms >> 24);
        msg.data[5] = (uint8_t)(appStatus.uptime_ms >> 16);
        msg.data[6] = (uint8_t)(appStatus.uptime_ms >> 8);
        msg.data[7] = (uint8_t)(appStatus.uptime_ms & 0xFFU);

        /* Transmit */
        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);

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
        Current_mA_t currents[4] = {0, 0, 0, 0};

        /* Read LEM sensors (ignore errors - send 0 if sensor fails) */
        (void)LEM_ReadCurrent(0U, &currents[0]);
        (void)LEM_ReadCurrent(1U, &currents[1]);
        (void)LEM_ReadCurrent(2U, &currents[2]);
        (void)LEM_ReadCurrent(3U, &currents[3]);

        /* Build BCU_Current_1 message (0x101) - LEM0-3, big-endian signed 16-bit */
        msg.id = CAN_ID_BCU_CURRENT_1;
        msg.frameType = CAN_FRAME_STANDARD;
        msg.frameFormat = CAN_FRAME_DATA;
        msg.dataLength = 8U;

        /* Pack 4x 16-bit signed currents (factor 10mA, big-endian) */
        int16_t scaled0 = (int16_t)(currents[0] / 10);
        int16_t scaled1 = (int16_t)(currents[1] / 10);
        int16_t scaled2 = (int16_t)(currents[2] / 10);
        int16_t scaled3 = (int16_t)(currents[3] / 10);

        msg.data[0] = (uint8_t)(scaled0 >> 8);
        msg.data[1] = (uint8_t)(scaled0 & 0xFF);
        msg.data[2] = (uint8_t)(scaled1 >> 8);
        msg.data[3] = (uint8_t)(scaled1 & 0xFF);
        msg.data[4] = (uint8_t)(scaled2 >> 8);
        msg.data[5] = (uint8_t)(scaled2 & 0xFF);
        msg.data[6] = (uint8_t)(scaled3 >> 8);
        msg.data[7] = (uint8_t)(scaled3 & 0xFF);

        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);

        if (status == STATUS_OK)
        {
            proto_stats.txCurrentCount++;
        }
        else
        {
            proto_stats.txErrors++;
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
        msg.frameType = CAN_FRAME_STANDARD;
        msg.frameFormat = CAN_FRAME_DATA;
        msg.dataLength =8U;

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
        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);

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
        int32_t temp_mC = 0;  /* Default to 0 if sensor fails */

        /* Try to read temperature, ignore errors (send 0 if sensor fails) */
        (void)TempSensor_ReadTemperature(&temp_mC);

        msg.id = CAN_ID_BCU_TEMPERATURE;
        msg.frameType = CAN_FRAME_STANDARD;
        msg.frameFormat = CAN_FRAME_DATA;
        msg.dataLength = 8U;

        /* Big-endian 32-bit signed temperature in milli-Celsius */
        msg.data[0] = (uint8_t)(temp_mC >> 24);
        msg.data[1] = (uint8_t)(temp_mC >> 16);
        msg.data[2] = (uint8_t)(temp_mC >> 8);
        msg.data[3] = (uint8_t)(temp_mC & 0xFFU);
        msg.data[4] = 0U;  /* Reserved */
        msg.data[5] = 0U;
        msg.data[6] = 0U;
        msg.data[7] = 0U;

        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);

        if (status == STATUS_OK)
        {
            proto_stats.txTempCount++;
        }
        else
        {
            proto_stats.txErrors++;
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
        msg.frameType = CAN_FRAME_STANDARD;
        msg.frameFormat = CAN_FRAME_DATA;
        msg.dataLength =8U;

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
        /* Pack as bitmap (1 bit per channel) in first 3 bytes */
        for (uint8_t i = 0U; i < 20U; i++)
        {
            BTT6200_ChannelState_t channelState;
            if (BTT6200_GetChannelState(i, &channelState) == STATUS_OK)
            {
                /* Set bit if channel is ON or PWM (active states) */
                if ((channelState == BTT6200_STATE_ON) || (channelState == BTT6200_STATE_PWM))
                {
                    uint8_t byteIndex = i / 8U;
                    uint8_t bitIndex = i % 8U;
                    msg.data[byteIndex] |= (1U << bitIndex);
                }
            }
        }

        /* Transmit */
        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);

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
        msg.frameType = CAN_FRAME_STANDARD;
        msg.frameFormat = CAN_FRAME_DATA;
        msg.dataLength =8U;

        msg.data[0] = (uint8_t)(activeDTCCount & 0xFFU);
        msg.data[1] = 0U;  /* Reserved for fault severity */
        msg.data[2] = 0U;  /* Reserved */
        msg.data[3] = 0U;
        msg.data[4] = 0U;
        msg.data[5] = 0U;
        msg.data[6] = 0U;
        msg.data[7] = 0U;

        /* Transmit */
        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);

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

    if ((pMsg != NULL) && (pMsg->dataLength > 0U))
    {
        uint8_t serviceId = pMsg->data[0];

        switch (serviceId)
        {
            case UDS_SID_READ_DATA_BY_ID:
                status = canproto_handle_read_data_by_id(pMsg->data, pMsg->dataLength);
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
        msg.frameType = CAN_FRAME_STANDARD;
        msg.frameFormat = CAN_FRAME_DATA;
        msg.dataLength =length;
        (void)memcpy(msg.data, pData, length);

        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 100U);
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
/* COMMAND HANDLERS                                                           */
/*============================================================================*/

/**
 * @brief Handle output set command
 * @details Message format: [channel_id, state, 0, 0, 0, 0, 0, 0]
 *          channel_id: 0-19 (output channel number)
 *          state: 0=OFF, 1=ON
 */
static Status_t canproto_handle_output_set(const CAN_Message_t *pMsg)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pMsg != NULL) && (pMsg->dataLength >= 2U))
    {
        uint8_t channel = pMsg->data[0];
        uint8_t state = pMsg->data[1];

        if (channel < 20U)
        {
            if (state == 0U)
            {
                status = BTT6200_ChannelOff(channel);
            }
            else
            {
                status = BTT6200_ChannelOn(channel);
            }
        }
    }

    return status;
}

/**
 * @brief Handle output all command (bitmap)
 * @details Message format: [out0-7, out8-15, out16-19, 0, 0, 0, 0, 0]
 *          Each bit represents one output: 0=OFF, 1=ON
 */
static Status_t canproto_handle_output_all(const CAN_Message_t *pMsg)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pMsg != NULL) && (pMsg->dataLength >= 3U))
    {
        /* Extract bitmap from message */
        uint32_t bitmap = ((uint32_t)pMsg->data[2] << 16) |
                          ((uint32_t)pMsg->data[1] << 8) |
                          ((uint32_t)pMsg->data[0]);

        /* Set each output according to bitmap */
        for (uint8_t i = 0U; i < 20U; i++)
        {
            if ((bitmap & (1UL << i)) != 0U)
            {
                (void)BTT6200_ChannelOn(i);
            }
            else
            {
                (void)BTT6200_ChannelOff(i);
            }
        }

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Handle output PWM command
 * @details Message format: [channel_id, duty_cycle, freq_idx, 0, 0, 0, 0, 0]
 *          channel_id: 0-19 (output channel number)
 *          duty_cycle: 0-100 (percent)
 *          freq_idx: reserved for future use (frequency selection)
 */
static Status_t canproto_handle_output_pwm(const CAN_Message_t *pMsg)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pMsg != NULL) && (pMsg->dataLength >= 2U))
    {
        uint8_t channel = pMsg->data[0];
        uint8_t duty = pMsg->data[1];

        if ((channel < 20U) && (duty <= 100U))
        {
            /* PWM mode not fully implemented in BTT6200 driver yet */
            /* For now, treat >50% as ON, <=50% as OFF */
            if (duty > 50U)
            {
                status = BTT6200_ChannelOn(channel);
            }
            else if (duty == 0U)
            {
                status = BTT6200_ChannelOff(channel);
            }
            else
            {
                /* Future: Implement actual PWM */
                status = STATUS_OK;
            }
        }
    }

    return status;
}

/**
 * @brief Handle system command
 * @details Message format: [command_id, param1, param2, 0, 0, 0, 0, 0]
 */
static Status_t canproto_handle_system_cmd(const CAN_Message_t *pMsg)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pMsg != NULL) && (pMsg->dataLength >= 1U))
    {
        uint8_t cmd = pMsg->data[0];

        switch (cmd)
        {
            case CMD_SYS_NOP:
                status = STATUS_OK;
                break;

            case CMD_SYS_RESET:
                /* Software reset - use NVIC system reset */
                NVIC_SystemReset();
                /* Should not reach here */
                status = STATUS_OK;
                break;

            case CMD_SYS_SAFE_STATE:
                App_EnterSafeState();
                status = STATUS_OK;
                break;

            case CMD_SYS_CLEAR_FAULTS:
                (void)ErrorHandler_ClearAllDTCs();
                status = STATUS_OK;
                break;

            case CMD_SYS_ENABLE_OUTPUTS:
                status = BTT6200_EnableOutputs(true);
                break;

            case CMD_SYS_DISABLE_OUTPUTS:
                status = BTT6200_EnableOutputs(false);
                break;

            default:
                status = STATUS_ERROR_PARAM;
                break;
        }
    }

    return status;
}

/*============================================================================*/
/* NEW TX FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Transmit digital input states
 */
Status_t CANProto_SendInputStates(void)
{
    Status_t status = STATUS_OK;

    if (proto_initialized)
    {
        CAN_Message_t msg;

        msg.id = CAN_ID_BCU_INPUTS;
        msg.frameType = CAN_FRAME_STANDARD;
        msg.frameFormat = CAN_FRAME_DATA;
        msg.dataLength = 8U;

        /* Pack 20 digital inputs into 3 bytes (bitmap) */
        msg.data[0] = 0U;
        msg.data[1] = 0U;
        msg.data[2] = 0U;
        msg.data[3] = 0U;  /* Reserved */
        msg.data[4] = 0U;
        msg.data[5] = 0U;
        msg.data[6] = 0U;
        msg.data[7] = 0U;

        for (uint8_t i = 0U; i < 20U; i++)
        {
            bool state = false;
            if (DI_ReadInput(i, &state) == STATUS_OK)
            {
                if (state)
                {
                    uint8_t byteIndex = i / 8U;
                    uint8_t bitIndex = i % 8U;
                    msg.data[byteIndex] |= (1U << bitIndex);
                }
            }
        }

        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);
    }

    return status;
}

/**
 * @brief Transmit BTT6200 diagnostics (fault status for all ICs)
 */
Status_t CANProto_SendBTTDiagnostics(void)
{
    Status_t status = STATUS_OK;

    if (proto_initialized)
    {
        CAN_Message_t msg;

        /* Message 1: IC0-1 diagnostics */
        msg.id = CAN_ID_BCU_BTT_DIAG_1;
        msg.frameType = CAN_FRAME_STANDARD;
        msg.frameFormat = CAN_FRAME_DATA;
        msg.dataLength = 8U;

        /* Each IC: [channel_states (4 bits), fault_flags (4 bits)] = 1 byte per IC */
        /* Pack 4 channels per IC into fault status bytes */
        for (uint8_t ic = 0U; ic < 2U; ic++)
        {
            uint8_t states = 0U;
            uint8_t faults = 0U;

            for (uint8_t ch = 0U; ch < 4U; ch++)
            {
                uint8_t globalCh = (ic * 4U) + ch;
                BTT6200_ChannelState_t chState;

                if (BTT6200_GetChannelState(globalCh, &chState) == STATUS_OK)
                {
                    if ((chState == BTT6200_STATE_ON) || (chState == BTT6200_STATE_PWM))
                    {
                        states |= (1U << ch);
                    }
                    if (chState == BTT6200_STATE_FAULT)
                    {
                        faults |= (1U << ch);
                    }
                }
            }

            msg.data[ic * 2U] = states;
            msg.data[(ic * 2U) + 1U] = faults;
        }

        /* Reserved bytes */
        msg.data[4] = 0U;
        msg.data[5] = 0U;
        msg.data[6] = 0U;
        msg.data[7] = 0U;

        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);

        /* Message 2: IC2-3 diagnostics */
        if (status == STATUS_OK)
        {
            msg.id = CAN_ID_BCU_BTT_DIAG_2;

            for (uint8_t ic = 2U; ic < 4U; ic++)
            {
                uint8_t states = 0U;
                uint8_t faults = 0U;
                uint8_t localIc = ic - 2U;

                for (uint8_t ch = 0U; ch < 4U; ch++)
                {
                    uint8_t globalCh = (ic * 4U) + ch;
                    BTT6200_ChannelState_t chState;

                    if (BTT6200_GetChannelState(globalCh, &chState) == STATUS_OK)
                    {
                        if ((chState == BTT6200_STATE_ON) || (chState == BTT6200_STATE_PWM))
                        {
                            states |= (1U << ch);
                        }
                        if (chState == BTT6200_STATE_FAULT)
                        {
                            faults |= (1U << ch);
                        }
                    }
                }

                msg.data[localIc * 2U] = states;
                msg.data[(localIc * 2U) + 1U] = faults;
            }

            msg.data[4] = 0U;
            msg.data[5] = 0U;
            msg.data[6] = 0U;
            msg.data[7] = 0U;

            status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);
        }

        /* Message 3: IC4 diagnostics */
        if (status == STATUS_OK)
        {
            msg.id = CAN_ID_BCU_BTT_DIAG_3;

            uint8_t states = 0U;
            uint8_t faults = 0U;

            for (uint8_t ch = 0U; ch < 4U; ch++)
            {
                uint8_t globalCh = (4U * 4U) + ch;  /* IC4 starts at channel 16 */
                BTT6200_ChannelState_t chState;

                if (BTT6200_GetChannelState(globalCh, &chState) == STATUS_OK)
                {
                    if ((chState == BTT6200_STATE_ON) || (chState == BTT6200_STATE_PWM))
                    {
                        states |= (1U << ch);
                    }
                    if (chState == BTT6200_STATE_FAULT)
                    {
                        faults |= (1U << ch);
                    }
                }
            }

            msg.data[0] = states;
            msg.data[1] = faults;
            msg.data[2] = 0U;
            msg.data[3] = 0U;
            msg.data[4] = 0U;
            msg.data[5] = 0U;
            msg.data[6] = 0U;
            msg.data[7] = 0U;

            status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);
        }
    }

    return status;
}

/**
 * @brief Transmit BTT6200 sense currents (from ADC)
 */
Status_t CANProto_SendBTTCurrents(void)
{
    Status_t status = STATUS_OK;

    if (proto_initialized)
    {
        CAN_Message_t msg;
        uint16_t adcValue;

        /* Message 1: IC0-1 currents (4 x 16-bit values = 8 bytes) */
        msg.id = CAN_ID_BCU_BTT_CURRENT_1;
        msg.frameType = CAN_FRAME_STANDARD;
        msg.frameFormat = CAN_FRAME_DATA;
        msg.dataLength = 8U;

        /* Read ADC channels for IS_0 and IS_1 */
        /* IS_0 = ADC channel 11 (index in our config), IS_1 = ADC channel 12 */
        for (uint8_t i = 0U; i < 2U; i++)
        {
            if (BSP_ADC_ReadChannel(11U + i, &adcValue, 10U) == STATUS_OK)
            {
                msg.data[i * 2U] = (uint8_t)(adcValue >> 8);
                msg.data[(i * 2U) + 1U] = (uint8_t)(adcValue & 0xFFU);
            }
            else
            {
                msg.data[i * 2U] = 0xFFU;
                msg.data[(i * 2U) + 1U] = 0xFFU;
            }
        }

        msg.data[4] = 0U;
        msg.data[5] = 0U;
        msg.data[6] = 0U;
        msg.data[7] = 0U;

        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);

        /* Message 2: IC2-3 currents */
        if (status == STATUS_OK)
        {
            msg.id = CAN_ID_BCU_BTT_CURRENT_2;

            for (uint8_t i = 0U; i < 2U; i++)
            {
                if (BSP_ADC_ReadChannel(13U + i, &adcValue, 10U) == STATUS_OK)
                {
                    msg.data[i * 2U] = (uint8_t)(adcValue >> 8);
                    msg.data[(i * 2U) + 1U] = (uint8_t)(adcValue & 0xFFU);
                }
                else
                {
                    msg.data[i * 2U] = 0xFFU;
                    msg.data[(i * 2U) + 1U] = 0xFFU;
                }
            }

            msg.data[4] = 0U;
            msg.data[5] = 0U;
            msg.data[6] = 0U;
            msg.data[7] = 0U;

            status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);
        }

        /* Message 3: IC4 current */
        if (status == STATUS_OK)
        {
            msg.id = CAN_ID_BCU_BTT_CURRENT_3;

            if (BSP_ADC_ReadChannel(15U, &adcValue, 10U) == STATUS_OK)
            {
                msg.data[0] = (uint8_t)(adcValue >> 8);
                msg.data[1] = (uint8_t)(adcValue & 0xFFU);
            }
            else
            {
                msg.data[0] = 0xFFU;
                msg.data[1] = 0xFFU;
            }

            msg.data[2] = 0U;
            msg.data[3] = 0U;
            msg.data[4] = 0U;
            msg.data[5] = 0U;
            msg.data[6] = 0U;
            msg.data[7] = 0U;

            status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);
        }
    }

    return status;
}

/**
 * @brief Transmit all LEM sensor currents (10 sensors)
 */
Status_t CANProto_SendAllCurrents(void)
{
    Status_t status = STATUS_OK;

    if (proto_initialized)
    {
        CAN_Message_t msg;
        Current_mA_t current;

        /* Message 1: LEM sensors 0-3 (4 x 16-bit = 8 bytes) */
        msg.id = CAN_ID_BCU_CURRENT_1;
        msg.frameType = CAN_FRAME_STANDARD;
        msg.frameFormat = CAN_FRAME_DATA;
        msg.dataLength = 8U;

        for (uint8_t i = 0U; i < 4U; i++)
        {
            if (LEM_ReadCurrent(i, &current) == STATUS_OK)
            {
                /* Send current in 10mA resolution (16-bit signed) */
                int16_t current_10mA = (int16_t)(current / 10);
                msg.data[i * 2U] = (uint8_t)(current_10mA >> 8);
                msg.data[(i * 2U) + 1U] = (uint8_t)(current_10mA & 0xFFU);
            }
            else
            {
                msg.data[i * 2U] = 0x7FU;
                msg.data[(i * 2U) + 1U] = 0xFFU;  /* Invalid marker */
            }
        }

        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);

        /* Message 2: LEM sensors 4-7 */
        if (status == STATUS_OK)
        {
            msg.id = CAN_ID_BCU_CURRENT_2;

            for (uint8_t i = 0U; i < 4U; i++)
            {
                if (LEM_ReadCurrent(4U + i, &current) == STATUS_OK)
                {
                    int16_t current_10mA = (int16_t)(current / 10);
                    msg.data[i * 2U] = (uint8_t)(current_10mA >> 8);
                    msg.data[(i * 2U) + 1U] = (uint8_t)(current_10mA & 0xFFU);
                }
                else
                {
                    msg.data[i * 2U] = 0x7FU;
                    msg.data[(i * 2U) + 1U] = 0xFFU;
                }
            }

            status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);
        }

        /* Message 3: LEM sensors 8-9 */
        if (status == STATUS_OK)
        {
            msg.id = CAN_ID_BCU_CURRENT_3;

            for (uint8_t i = 0U; i < 2U; i++)
            {
                if (LEM_ReadCurrent(8U + i, &current) == STATUS_OK)
                {
                    int16_t current_10mA = (int16_t)(current / 10);
                    msg.data[i * 2U] = (uint8_t)(current_10mA >> 8);
                    msg.data[(i * 2U) + 1U] = (uint8_t)(current_10mA & 0xFFU);
                }
                else
                {
                    msg.data[i * 2U] = 0x7FU;
                    msg.data[(i * 2U) + 1U] = 0xFFU;
                }
            }

            /* Padding */
            msg.data[4] = 0U;
            msg.data[5] = 0U;
            msg.data[6] = 0U;
            msg.data[7] = 0U;

            status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);
        }

        proto_stats.txCurrentCount++;
    }

    return status;
}

/**
 * @brief Transmit RUL status message (0x120)
 * @details Message format:
 *          Byte 0-1: SoH percentage (0.01% resolution, big-endian)
 *          Byte 2-3: Remaining cycles (big-endian)
 *          Byte 4-5: Remaining days (big-endian)
 *          Byte 6:   Confidence (0-100%)
 *          Byte 7:   Valid flag (0/1)
 */
Status_t CANProto_SendRULStatus(const RUL_Prediction_t *pPrediction, Percentage_t soh)
{
    Status_t status = STATUS_OK;

    if (proto_initialized && (pPrediction != NULL))
    {
        CAN_Message_t msg;

        msg.id = CAN_ID_BCU_RUL_STATUS;
        msg.frameType = CAN_FRAME_STANDARD;
        msg.frameFormat = CAN_FRAME_DATA;
        msg.dataLength = 8U;

        /* SoH percentage in 0.01% units (big-endian) */
        msg.data[0] = (uint8_t)(soh >> 8);
        msg.data[1] = (uint8_t)(soh & 0xFFU);

        /* Remaining cycles (big-endian) */
        uint16_t cycles = (pPrediction->remainingCycles > 65535U) ?
                          65535U : (uint16_t)pPrediction->remainingCycles;
        msg.data[2] = (uint8_t)(cycles >> 8);
        msg.data[3] = (uint8_t)(cycles & 0xFFU);

        /* Remaining days (big-endian) */
        uint16_t days = (pPrediction->remainingDays > 65535U) ?
                        65535U : (uint16_t)pPrediction->remainingDays;
        msg.data[4] = (uint8_t)(days >> 8);
        msg.data[5] = (uint8_t)(days & 0xFFU);

        /* Confidence (0-100%) */
        uint8_t confidence = (pPrediction->confidence > 10000U) ?
                             100U : (uint8_t)(pPrediction->confidence / 100U);
        msg.data[6] = confidence;

        /* Valid flag */
        msg.data[7] = pPrediction->isValid ? 1U : 0U;

        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);
    }
    else
    {
        status = STATUS_ERROR_PARAM;
    }

    return status;
}

/**
 * @brief Transmit scheduler timing message (0x121)
 * @details Message format:
 *          Byte 0-3: Current cycle time in microseconds (big-endian)
 *          Byte 4-7: Maximum cycle time in microseconds (big-endian)
 */
Status_t CANProto_SendTiming(uint32_t cycleTime_us, uint32_t maxCycleTime_us)
{
    Status_t status = STATUS_OK;

    if (proto_initialized)
    {
        CAN_Message_t msg;

        msg.id = CAN_ID_BCU_TIMING;
        msg.frameType = CAN_FRAME_STANDARD;
        msg.frameFormat = CAN_FRAME_DATA;
        msg.dataLength = 8U;

        /* Current cycle time (big-endian) */
        msg.data[0] = (uint8_t)(cycleTime_us >> 24);
        msg.data[1] = (uint8_t)(cycleTime_us >> 16);
        msg.data[2] = (uint8_t)(cycleTime_us >> 8);
        msg.data[3] = (uint8_t)(cycleTime_us & 0xFFU);

        /* Max cycle time (big-endian) */
        msg.data[4] = (uint8_t)(maxCycleTime_us >> 24);
        msg.data[5] = (uint8_t)(maxCycleTime_us >> 16);
        msg.data[6] = (uint8_t)(maxCycleTime_us >> 8);
        msg.data[7] = (uint8_t)(maxCycleTime_us & 0xFFU);

        status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, NULL, 10U);
    }

    return status;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
