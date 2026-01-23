/**
 * @file    bsp_can.c
 * @brief   BSP CAN driver implementation for STM32F4
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Dual CAN bus support (CAN1 and CAN2)
 * @note    ISO 11898 CAN 2.0B standard
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "bsp_can.h"
#include "bsp_gpio.h"
#include "ringbuffer.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief CAN RX FIFO size */
#define CAN_RX_FIFO_SIZE        (128U)

/** @brief CAN TX FIFO size */
#define CAN_TX_FIFO_SIZE        (64U)

/** @brief CAN filter bank count */
#define CAN_FILTER_BANKS        (14U)

/*============================================================================*/
/* PRIVATE TYPES                                                              */
/*============================================================================*/

/** @brief CAN instance data */
typedef struct {
    CAN_HandleTypeDef hcan;
    RingBuffer_t rxFifo;
    RingBuffer_t txFifo;
    uint8_t rxBuffer[CAN_RX_FIFO_SIZE * sizeof(CAN_Message_t)];
    uint8_t txBuffer[CAN_TX_FIFO_SIZE * sizeof(CAN_Message_t)];
    CAN_Statistics_t stats;
    CAN_RxCallback_t rxCallback;
    CAN_TxCallback_t txCallback;
    CAN_ErrorCallback_t errorCallback;
    bool initialized;
} CAN_Instance_t;

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief CAN1 instance */
static CAN_Instance_t can1_instance;

/** @brief CAN2 instance */
static CAN_Instance_t can2_instance;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static Status_t can_configure_bit_timing(CAN_HandleTypeDef *hcan, uint32_t baudrate);
static Status_t can_configure_filters(CAN_HandleTypeDef *hcan, uint8_t instance);
static CAN_Instance_t* can_get_instance(uint8_t instance);
static void can_process_rx_message(uint8_t instance, CAN_Instance_t *pInst, CAN_RxHeaderTypeDef *pRxHeader, uint8_t *pData);
static void can_process_errors(uint8_t instance, CAN_Instance_t *pInst, uint32_t errorCode);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize CAN interface
 */
Status_t BSP_CAN_Init(uint8_t instance, const CAN_Config_t *pConfig)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance <= BSP_CAN_INSTANCE_2) && (pConfig != NULL))
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if (pInst == NULL)
        {
            status = STATUS_ERROR_PARAM;
        }
        else if (pInst->initialized)
        {
            status = STATUS_ERROR_ALREADY_INIT;
        }
        else
        {
            /* Initialize ring buffers */
            (void)RingBuffer_Init(&pInst->rxFifo, pInst->rxBuffer, sizeof(pInst->rxBuffer));
            (void)RingBuffer_Init(&pInst->txFifo, pInst->txBuffer, sizeof(pInst->txBuffer));

            /* Clear statistics */
            (void)memset(&pInst->stats, 0, sizeof(CAN_Statistics_t));

            /* Configure CAN peripheral */
            if (instance == BSP_CAN_INSTANCE_1)
            {
                pInst->hcan.Instance = CAN1;
                __HAL_RCC_CAN1_CLK_ENABLE();
            }
            else
            {
                pInst->hcan.Instance = CAN2;
                __HAL_RCC_CAN1_CLK_ENABLE();  /* CAN1 clock must be enabled for CAN2 */
                __HAL_RCC_CAN2_CLK_ENABLE();
            }

            /* Configure basic parameters based on mode */
            switch (pConfig->mode)
            {
                case BSP_CAN_MODE_LOOPBACK:
                    pInst->hcan.Init.Mode = CAN_MODE_LOOPBACK;
                    break;
                case BSP_CAN_MODE_SILENT:
                    pInst->hcan.Init.Mode = CAN_MODE_SILENT;
                    break;
                case BSP_CAN_MODE_SILENT_LOOPBACK:
                    pInst->hcan.Init.Mode = CAN_MODE_SILENT_LOOPBACK;
                    break;
                case BSP_CAN_MODE_NORMAL:
                default:
                    pInst->hcan.Init.Mode = CAN_MODE_NORMAL;
                    break;
            }
            pInst->hcan.Init.TimeTriggeredMode = DISABLE;
            pInst->hcan.Init.AutoBusOff = ENABLE;  /* Enable automatic bus-off recovery */
            pInst->hcan.Init.AutoWakeUp = ENABLE;
            pInst->hcan.Init.AutoRetransmission = (pConfig->autoRetransmit) ? ENABLE : DISABLE;
            pInst->hcan.Init.ReceiveFifoLocked = (pConfig->rxFifo0Overrun) ? ENABLE : DISABLE;
            pInst->hcan.Init.TransmitFifoPriority = (pConfig->txPriority > 0U) ? ENABLE : DISABLE;

            /* Configure bit timing */
            status = can_configure_bit_timing(&pInst->hcan, pConfig->bitrate);

            if (status == STATUS_OK)
            {
                /* Initialize CAN peripheral */
                HAL_StatusTypeDef halStatus = HAL_CAN_Init(&pInst->hcan);

                if (halStatus != HAL_OK)
                {
                    status = STATUS_ERROR;
                }
            }

            if (status == STATUS_OK)
            {
                /* Configure filters */
                status = can_configure_filters(&pInst->hcan, instance);
            }

            if (status == STATUS_OK)
            {
                /* Enable interrupts */
                HAL_StatusTypeDef halStatus;

                halStatus = HAL_CAN_ActivateNotification(&pInst->hcan,
                    CAN_IT_RX_FIFO0_MSG_PENDING |
                    CAN_IT_RX_FIFO1_MSG_PENDING |
                    CAN_IT_TX_MAILBOX_EMPTY |
                    CAN_IT_ERROR |
                    CAN_IT_BUSOFF |
                    CAN_IT_LAST_ERROR_CODE);

                if (halStatus == HAL_OK)
                {
                    /* Start CAN peripheral */
                    halStatus = HAL_CAN_Start(&pInst->hcan);

                    if (halStatus == HAL_OK)
                    {
                        pInst->initialized = true;
                        status = STATUS_OK;
                    }
                    else
                    {
                        status = STATUS_ERROR;
                    }
                }
                else
                {
                    status = STATUS_ERROR;
                }
            }
        }
    }

    return status;
}

/**
 * @brief Transmit CAN message
 */
Status_t BSP_CAN_Transmit(uint8_t instance, const CAN_Message_t *pMessage,
                          uint32_t *pMailbox, uint32_t timeout_ms)
{
    Status_t status = STATUS_ERROR_PARAM;
    (void)timeout_ms;  /* Currently not used for blocking wait */

    if ((instance <= BSP_CAN_INSTANCE_2) && (pMessage != NULL))
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            CAN_TxHeaderTypeDef txHeader;
            uint32_t txMailbox;

            /* Build TX header */
            if (pMessage->frameType == CAN_FRAME_EXTENDED)
            {
                txHeader.ExtId = pMessage->id;
                txHeader.IDE = CAN_ID_EXT;
            }
            else
            {
                txHeader.StdId = pMessage->id;
                txHeader.IDE = CAN_ID_STD;
            }

            txHeader.RTR = (pMessage->frameFormat == CAN_FRAME_REMOTE) ? CAN_RTR_REMOTE : CAN_RTR_DATA;
            txHeader.DLC = pMessage->dataLength;
            txHeader.TransmitGlobalTime = DISABLE;

            /* Attempt to send */
            HAL_StatusTypeDef halStatus = HAL_CAN_AddTxMessage(&pInst->hcan,
                                                               &txHeader,
                                                               (uint8_t *)pMessage->data,
                                                               &txMailbox);

            if (halStatus == HAL_OK)
            {
                pInst->stats.txCount++;
                if (pMailbox != NULL)
                {
                    *pMailbox = txMailbox;
                }
                status = STATUS_OK;
            }
            else if (halStatus == HAL_BUSY)
            {
                /* Mailboxes full */
                pInst->stats.txMailboxFullCount++;
                status = STATUS_ERROR_BUSY;
            }
            else
            {
                pInst->stats.txErrorCount++;
                status = STATUS_ERROR;
            }
        }
    }

    return status;
}

/**
 * @brief Receive CAN message from FIFO
 */
Status_t BSP_CAN_Receive(uint8_t instance, uint8_t fifo,
                         CAN_Message_t *pMessage, uint32_t timeout_ms)
{
    Status_t status = STATUS_ERROR_PARAM;
    (void)fifo;       /* Currently using single RX FIFO buffer */
    (void)timeout_ms; /* Currently not used for blocking wait */

    if ((instance <= BSP_CAN_INSTANCE_2) && (pMessage != NULL))
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            /* Try to read from RX FIFO */
            status = RingBuffer_Read(&pInst->rxFifo, (uint8_t *)pMessage,
                                     sizeof(CAN_Message_t));
            /* RingBuffer_Read returns STATUS_ERROR_UNDERFLOW if not enough data */
        }
    }

    return status;
}


/**
 * @brief Register RX callback
 */
Status_t BSP_CAN_RegisterRxCallback(uint8_t instance, CAN_RxCallback_t callback)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (instance <= BSP_CAN_INSTANCE_2)
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if (pInst != NULL)
        {
            pInst->rxCallback = callback;
            status = STATUS_OK;
        }
    }

    return status;
}

/**
 * @brief Register error callback
 */
Status_t BSP_CAN_RegisterErrorCallback(uint8_t instance, CAN_ErrorCallback_t callback)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (instance <= BSP_CAN_INSTANCE_2)
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if (pInst != NULL)
        {
            pInst->errorCallback = callback;
            status = STATUS_OK;
        }
    }

    return status;
}

/**
 * @brief Get CAN statistics
 */
Status_t BSP_CAN_GetStatistics(uint8_t instance, CAN_Statistics_t *pStats)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance <= BSP_CAN_INSTANCE_2) && (pStats != NULL))
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            *pStats = pInst->stats;
            status = STATUS_OK;
        }
    }

    return status;
}

/**
 * @brief Reset CAN statistics
 */
Status_t BSP_CAN_ResetStatistics(uint8_t instance)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (instance <= BSP_CAN_INSTANCE_2)
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            (void)memset(&pInst->stats, 0, sizeof(CAN_Statistics_t));
            status = STATUS_OK;
        }
    }

    return status;
}

/**
 * @brief Get CAN bus state
 */
Status_t BSP_CAN_GetBusState(uint8_t instance, CAN_BusState_t *pState)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance <= BSP_CAN_INSTANCE_2) && (pState != NULL))
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            uint32_t canState = HAL_CAN_GetState(&pInst->hcan);
            uint32_t canError = HAL_CAN_GetError(&pInst->hcan);

            if (canError & HAL_CAN_ERROR_BOF)
            {
                *pState = CAN_STATE_BUS_OFF;
            }
            else if (canError & HAL_CAN_ERROR_EPV)
            {
                *pState = CAN_STATE_ERROR_PASSIVE;
            }
            else if (canError & HAL_CAN_ERROR_EWG)
            {
                *pState = CAN_STATE_ERROR_ACTIVE;
            }
            else if (canState == HAL_CAN_STATE_READY)
            {
                *pState = CAN_STATE_READY;
            }
            else
            {
                *pState = CAN_STATE_UNINITIALIZED;
            }

            status = STATUS_OK;
        }
    }

    return status;
}

/**
 * @brief De-initialize CAN interface
 */
Status_t BSP_CAN_DeInit(uint8_t instance)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (instance <= BSP_CAN_INSTANCE_2)
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            HAL_CAN_Stop(&pInst->hcan);
            HAL_CAN_DeInit(&pInst->hcan);
            pInst->initialized = false;
            status = STATUS_OK;
        }
    }

    return status;
}

/**
 * @brief Start CAN bus communication
 */
Status_t BSP_CAN_Start(uint8_t instance)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (instance <= BSP_CAN_INSTANCE_2)
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            if (HAL_CAN_Start(&pInst->hcan) == HAL_OK)
            {
                status = STATUS_OK;
            }
            else
            {
                status = STATUS_ERROR;
            }
        }
    }

    return status;
}

/**
 * @brief Stop CAN bus communication
 */
Status_t BSP_CAN_Stop(uint8_t instance)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (instance <= BSP_CAN_INSTANCE_2)
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            if (HAL_CAN_Stop(&pInst->hcan) == HAL_OK)
            {
                status = STATUS_OK;
            }
            else
            {
                status = STATUS_ERROR;
            }
        }
    }

    return status;
}

/**
 * @brief Check if CAN message is available in FIFO
 */
bool BSP_CAN_IsMessageAvailable(uint8_t instance, uint8_t fifo, uint32_t *pCount)
{
    bool available = false;
    (void)fifo;  /* Using internal ring buffer instead of hardware FIFO */

    if (instance <= BSP_CAN_INSTANCE_2)
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            uint32_t count = RingBuffer_Available(&pInst->rxFifo) / sizeof(CAN_Message_t);

            if (pCount != NULL)
            {
                *pCount = count;
            }

            available = (count > 0U);
        }
    }

    return available;
}

/**
 * @brief Configure CAN filter
 */
Status_t BSP_CAN_ConfigureFilter(uint8_t instance, const CAN_Filter_t *pFilter)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance <= BSP_CAN_INSTANCE_2) && (pFilter != NULL))
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            CAN_FilterTypeDef filterConfig;

            filterConfig.FilterIdHigh = (uint16_t)((pFilter->id >> 13U) & 0xFFFFU);
            filterConfig.FilterIdLow = (uint16_t)((pFilter->id << 3U) & 0xFFF8U);
            filterConfig.FilterMaskIdHigh = (uint16_t)((pFilter->mask >> 13U) & 0xFFFFU);
            filterConfig.FilterMaskIdLow = (uint16_t)((pFilter->mask << 3U) & 0xFFF8U);
            filterConfig.FilterFIFOAssignment = (pFilter->fifo == 0U) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
            filterConfig.FilterBank = pFilter->filterBank;
            filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
            filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
            filterConfig.FilterActivation = pFilter->enabled ? CAN_FILTER_ENABLE : CAN_FILTER_DISABLE;
            filterConfig.SlaveStartFilterBank = 14U;

            if (HAL_CAN_ConfigFilter(&pInst->hcan, &filterConfig) == HAL_OK)
            {
                status = STATUS_OK;
            }
            else
            {
                status = STATUS_ERROR;
            }
        }
    }

    return status;
}

/**
 * @brief Disable CAN filter
 */
Status_t BSP_CAN_DisableFilter(uint8_t instance, uint8_t filterBank)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (instance <= BSP_CAN_INSTANCE_2)
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            CAN_FilterTypeDef filterConfig;

            filterConfig.FilterBank = filterBank;
            filterConfig.FilterActivation = CAN_FILTER_DISABLE;
            filterConfig.SlaveStartFilterBank = 14U;

            if (HAL_CAN_ConfigFilter(&pInst->hcan, &filterConfig) == HAL_OK)
            {
                status = STATUS_OK;
            }
            else
            {
                status = STATUS_ERROR;
            }
        }
    }

    return status;
}

/**
 * @brief Get CAN error counters
 */
Status_t BSP_CAN_GetErrorCounters(uint8_t instance, uint8_t *pTxErrors, uint8_t *pRxErrors)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (instance <= BSP_CAN_INSTANCE_2)
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            uint32_t esr = pInst->hcan.Instance->ESR;

            if (pTxErrors != NULL)
            {
                *pTxErrors = (uint8_t)((esr >> 16U) & 0xFFU);  /* TEC field */
            }

            if (pRxErrors != NULL)
            {
                *pRxErrors = (uint8_t)((esr >> 24U) & 0xFFU);  /* REC field */
            }

            status = STATUS_OK;
        }
    }

    return status;
}

/**
 * @brief Abort pending CAN transmission
 */
Status_t BSP_CAN_AbortTransmission(uint8_t instance, uint32_t mailbox)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance <= BSP_CAN_INSTANCE_2) && (mailbox < BSP_CAN_TX_MAILBOXES))
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            if (HAL_CAN_AbortTxRequest(&pInst->hcan, (1UL << mailbox)) == HAL_OK)
            {
                status = STATUS_OK;
            }
            else
            {
                status = STATUS_ERROR;
            }
        }
    }

    return status;
}

/**
 * @brief Register TX callback function
 */
Status_t BSP_CAN_RegisterTxCallback(uint8_t instance, CAN_TxCallback_t callback)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (instance <= BSP_CAN_INSTANCE_2)
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if (pInst != NULL)
        {
            pInst->txCallback = callback;
            status = STATUS_OK;
        }
    }

    return status;
}

/**
 * @brief Enable/disable CAN transceiver standby mode
 */
Status_t BSP_CAN_SetStandbyMode(uint8_t instance, bool enable)
{
    Status_t status = STATUS_ERROR_PARAM;
    (void)enable;  /* Placeholder - implement based on TCAN3404 GPIO control */

    if (instance <= BSP_CAN_INSTANCE_2)
    {
        /* TCAN3404 standby control would be implemented via GPIO */
        /* For now, just return OK as placeholder */
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Check CAN transceiver fault status
 */
Status_t BSP_CAN_GetTransceiverFault(uint8_t instance, bool *pFault)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance <= BSP_CAN_INSTANCE_2) && (pFault != NULL))
    {
        /* TCAN3404 fault detection would be implemented via GPIO */
        /* For now, return no fault as placeholder */
        *pFault = false;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Build standard CAN message (11-bit ID)
 */
Status_t BSP_CAN_BuildStandardMessage(CAN_Message_t *pMessage, uint16_t id,
                                      const uint8_t *pData, uint8_t dataLength)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pMessage != NULL) && (dataLength <= BSP_CAN_MAX_DATA_LEN))
    {
        pMessage->id = (uint32_t)id & 0x7FFU;  /* 11-bit mask */
        pMessage->frameType = CAN_FRAME_STANDARD;
        pMessage->frameFormat = CAN_FRAME_DATA;
        pMessage->dataLength = dataLength;
        pMessage->timestamp_ms = 0U;

        if ((pData != NULL) && (dataLength > 0U))
        {
            (void)memcpy(pMessage->data, pData, dataLength);
        }
        else
        {
            (void)memset(pMessage->data, 0, BSP_CAN_MAX_DATA_LEN);
        }

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Build extended CAN message (29-bit ID)
 */
Status_t BSP_CAN_BuildExtendedMessage(CAN_Message_t *pMessage, uint32_t id,
                                      const uint8_t *pData, uint8_t dataLength)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pMessage != NULL) && (dataLength <= BSP_CAN_MAX_DATA_LEN))
    {
        pMessage->id = id & 0x1FFFFFFFU;  /* 29-bit mask */
        pMessage->frameType = CAN_FRAME_EXTENDED;
        pMessage->frameFormat = CAN_FRAME_DATA;
        pMessage->dataLength = dataLength;
        pMessage->timestamp_ms = 0U;

        if ((pData != NULL) && (dataLength > 0U))
        {
            (void)memcpy(pMessage->data, pData, dataLength);
        }
        else
        {
            (void)memset(pMessage->data, 0, BSP_CAN_MAX_DATA_LEN);
        }

        status = STATUS_OK;
    }

    return status;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Configure CAN bit timing
 */
static Status_t can_configure_bit_timing(CAN_HandleTypeDef *hcan, uint32_t baudrate)
{
    Status_t status = STATUS_OK;

    /* APB1 clock = 50 MHz (assuming 100 MHz system clock with APB1 divider /2) */
    uint32_t apb1_clk = HAL_RCC_GetPCLK1Freq();

    /* Calculate bit timing parameters */
    /* Standard timing: 1 TQ = 1 bit time / (1 + TS1 + TS2) */
    /* Sample point at 87.5%: TS1 = 13, TS2 = 2, SJW = 1 */

    switch (baudrate)
    {
        case 1000000:  /* 1 Mbps */
            hcan->Init.Prescaler = apb1_clk / (baudrate * 16U);
            hcan->Init.TimeSeg1 = CAN_BS1_13TQ;
            hcan->Init.TimeSeg2 = CAN_BS2_2TQ;
            hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
            break;

        case 500000:  /* 500 kbps */
            hcan->Init.Prescaler = apb1_clk / (baudrate * 16U);
            hcan->Init.TimeSeg1 = CAN_BS1_13TQ;
            hcan->Init.TimeSeg2 = CAN_BS2_2TQ;
            hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
            break;

        case 250000:  /* 250 kbps */
            hcan->Init.Prescaler = apb1_clk / (baudrate * 16U);
            hcan->Init.TimeSeg1 = CAN_BS1_13TQ;
            hcan->Init.TimeSeg2 = CAN_BS2_2TQ;
            hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
            break;

        case 125000:  /* 125 kbps */
            hcan->Init.Prescaler = apb1_clk / (baudrate * 16U);
            hcan->Init.TimeSeg1 = CAN_BS1_13TQ;
            hcan->Init.TimeSeg2 = CAN_BS2_2TQ;
            hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
            break;

        default:
            status = STATUS_ERROR_PARAM;
            break;
    }

    return status;
}

/**
 * @brief Configure CAN filters
 */
static Status_t can_configure_filters(CAN_HandleTypeDef *hcan, uint8_t instance)
{
    Status_t status = STATUS_OK;
    CAN_FilterTypeDef filterConfig;

    /* Configure filter to accept all messages */
    filterConfig.FilterIdHigh = 0x0000U;
    filterConfig.FilterIdLow = 0x0000U;
    filterConfig.FilterMaskIdHigh = 0x0000U;
    filterConfig.FilterMaskIdLow = 0x0000U;
    filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filterConfig.FilterBank = (instance == BSP_CAN_INSTANCE_1) ? 0U : 14U;
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    filterConfig.FilterActivation = CAN_FILTER_ENABLE;
    filterConfig.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(hcan, &filterConfig) != HAL_OK)
    {
        status = STATUS_ERROR;
    }

    return status;
}

/**
 * @brief Get CAN instance pointer
 */
static CAN_Instance_t* can_get_instance(uint8_t instance)
{
    CAN_Instance_t *pInst = NULL;

    if (instance == BSP_CAN_INSTANCE_1)
    {
        pInst = &can1_instance;
    }
    else if (instance == BSP_CAN_INSTANCE_2)
    {
        pInst = &can2_instance;
    }

    return pInst;
}

/**
 * @brief Process received CAN message
 */
static void can_process_rx_message(uint8_t instance, CAN_Instance_t *pInst, CAN_RxHeaderTypeDef *pRxHeader, uint8_t *pData)
{
    if ((pInst != NULL) && (pRxHeader != NULL) && (pData != NULL))
    {
        CAN_Message_t message;

        /* Build message structure */
        if (pRxHeader->IDE == CAN_ID_EXT)
        {
            message.id = pRxHeader->ExtId;
            message.frameType = CAN_FRAME_EXTENDED;
        }
        else
        {
            message.id = pRxHeader->StdId;
            message.frameType = CAN_FRAME_STANDARD;
        }

        message.frameFormat = (pRxHeader->RTR == CAN_RTR_REMOTE) ? CAN_FRAME_REMOTE : CAN_FRAME_DATA;
        message.dataLength = (uint8_t)pRxHeader->DLC;
        (void)memcpy(message.data, pData, message.dataLength);
        message.timestamp_ms = HAL_GetTick();

        /* Add to RX FIFO */
        if (RingBuffer_Write(&pInst->rxFifo, (const uint8_t *)&message,
                            sizeof(CAN_Message_t)) == STATUS_OK)
        {
            pInst->stats.rxCount++;

            /* Call RX callback if registered */
            if (pInst->rxCallback != NULL)
            {
                pInst->rxCallback(instance, &message);
            }
        }
        else
        {
            pInst->stats.rxOverrunCount++;
        }
    }
}

/**
 * @brief Process CAN errors
 */
static void can_process_errors(uint8_t instance, CAN_Instance_t *pInst, uint32_t errorCode)
{
    if (pInst != NULL)
    {
        /* Store last error code */
        pInst->stats.lastErrorCode = errorCode;

        /* Increment bus-off counter */
        if ((errorCode & HAL_CAN_ERROR_BOF) != 0U)
        {
            pInst->stats.busOffCount++;
        }

        /* Increment TX error counter for TX-related errors */
        if ((errorCode & (HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_TERR0 |
                          HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_TERR1 |
                          HAL_CAN_ERROR_TX_ALST2 | HAL_CAN_ERROR_TX_TERR2)) != 0U)
        {
            pInst->stats.txErrorCount++;
        }

        /* Increment RX error counter for RX-related errors */
        if ((errorCode & (HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_RX_FOV1)) != 0U)
        {
            pInst->stats.rxErrorCount++;
        }

        /* Protocol errors increment general error counters */
        if ((errorCode & (HAL_CAN_ERROR_STF | HAL_CAN_ERROR_FOR |
                          HAL_CAN_ERROR_ACK | HAL_CAN_ERROR_BR |
                          HAL_CAN_ERROR_BD | HAL_CAN_ERROR_CRC)) != 0U)
        {
            pInst->stats.rxErrorCount++;
        }

        /* Call error callback if registered */
        if (pInst->errorCallback != NULL)
        {
            pInst->errorCallback(instance, errorCode);
        }
    }
}

/*============================================================================*/
/* HAL CALLBACKS                                                              */
/*============================================================================*/

/**
 * @brief RX FIFO 0 message pending callback
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    CAN_Instance_t *pInst = NULL;
    uint8_t instance = 0U;

    /* Determine which instance */
    if (hcan->Instance == CAN1)
    {
        pInst = &can1_instance;
        instance = BSP_CAN_INSTANCE_1;
    }
    else if (hcan->Instance == CAN2)
    {
        pInst = &can2_instance;
        instance = BSP_CAN_INSTANCE_2;
    }

    if (pInst != NULL)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
        {
            can_process_rx_message(instance, pInst, &rxHeader, rxData);
        }
    }
}

/**
 * @brief RX FIFO 1 message pending callback
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    CAN_Instance_t *pInst = NULL;
    uint8_t instance = 0U;

    /* Determine which instance */
    if (hcan->Instance == CAN1)
    {
        pInst = &can1_instance;
        instance = BSP_CAN_INSTANCE_1;
    }
    else if (hcan->Instance == CAN2)
    {
        pInst = &can2_instance;
        instance = BSP_CAN_INSTANCE_2;
    }

    if (pInst != NULL)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) == HAL_OK)
        {
            can_process_rx_message(instance, pInst, &rxHeader, rxData);
        }
    }
}

/**
 * @brief CAN error callback
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Instance_t *pInst = NULL;
    uint8_t instance = 0U;
    uint32_t errorCode = HAL_CAN_GetError(hcan);

    /* Determine which instance */
    if (hcan->Instance == CAN1)
    {
        pInst = &can1_instance;
        instance = BSP_CAN_INSTANCE_1;
    }
    else if (hcan->Instance == CAN2)
    {
        pInst = &can2_instance;
        instance = BSP_CAN_INSTANCE_2;
    }

    if (pInst != NULL)
    {
        can_process_errors(instance, pInst, errorCode);
    }
}

/**
 * @brief TX mailbox 0 complete callback
 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Instance_t *pInst = NULL;
    uint8_t instance = 0U;

    /* Determine which instance */
    if (hcan->Instance == CAN1)
    {
        pInst = &can1_instance;
        instance = BSP_CAN_INSTANCE_1;
    }
    else if (hcan->Instance == CAN2)
    {
        pInst = &can2_instance;
        instance = BSP_CAN_INSTANCE_2;
    }

    if ((pInst != NULL) && (pInst->txCallback != NULL))
    {
        pInst->txCallback(instance, 0U);
    }
}

/**
 * @brief TX mailbox 1 complete callback
 */
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Instance_t *pInst = NULL;
    uint8_t instance = 0U;

    /* Determine which instance */
    if (hcan->Instance == CAN1)
    {
        pInst = &can1_instance;
        instance = BSP_CAN_INSTANCE_1;
    }
    else if (hcan->Instance == CAN2)
    {
        pInst = &can2_instance;
        instance = BSP_CAN_INSTANCE_2;
    }

    if ((pInst != NULL) && (pInst->txCallback != NULL))
    {
        pInst->txCallback(instance, 1U);
    }
}

/**
 * @brief TX mailbox 2 complete callback
 */
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Instance_t *pInst = NULL;
    uint8_t instance = 0U;

    /* Determine which instance */
    if (hcan->Instance == CAN1)
    {
        pInst = &can1_instance;
        instance = BSP_CAN_INSTANCE_1;
    }
    else if (hcan->Instance == CAN2)
    {
        pInst = &can2_instance;
        instance = BSP_CAN_INSTANCE_2;
    }

    if ((pInst != NULL) && (pInst->txCallback != NULL))
    {
        pInst->txCallback(instance, 2U);
    }
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
