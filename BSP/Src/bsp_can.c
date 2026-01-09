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
static void can_process_rx_message(CAN_Instance_t *pInst, CAN_RxHeaderTypeDef *pRxHeader, uint8_t *pData);
static void can_process_errors(CAN_Instance_t *pInst, uint32_t errorCode);

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

            /* Configure basic parameters */
            pInst->hcan.Init.Mode = (pConfig->loopback) ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL;
            pInst->hcan.Init.TimeTriggeredMode = DISABLE;
            pInst->hcan.Init.AutoBusOff = (pConfig->autoBusOff) ? ENABLE : DISABLE;
            pInst->hcan.Init.AutoWakeUp = ENABLE;
            pInst->hcan.Init.AutoRetransmission = (pConfig->autoRetransmit) ? ENABLE : DISABLE;
            pInst->hcan.Init.ReceiveFifoLocked = DISABLE;
            pInst->hcan.Init.TransmitFifoPriority = DISABLE;

            /* Configure bit timing */
            status = can_configure_bit_timing(&pInst->hcan, pConfig->baudrate);

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
Status_t BSP_CAN_Transmit(uint8_t instance, const CAN_Message_t *pMessage, uint32_t timeout_ms)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance <= BSP_CAN_INSTANCE_2) && (pMessage != NULL))
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            CAN_TxHeaderTypeDef txHeader;
            uint32_t txMailbox;

            /* Build TX header */
            if (pMessage->ide == CAN_IDE_EXTENDED)
            {
                txHeader.ExtId = pMessage->id;
                txHeader.IDE = CAN_ID_EXT;
            }
            else
            {
                txHeader.StdId = pMessage->id;
                txHeader.IDE = CAN_ID_STD;
            }

            txHeader.RTR = (pMessage->rtr) ? CAN_RTR_REMOTE : CAN_RTR_DATA;
            txHeader.DLC = pMessage->dlc;
            txHeader.TransmitGlobalTime = DISABLE;

            /* Attempt to send */
            HAL_StatusTypeDef halStatus = HAL_CAN_AddTxMessage(&pInst->hcan,
                                                               &txHeader,
                                                               (uint8_t *)pMessage->data,
                                                               &txMailbox);

            if (halStatus == HAL_OK)
            {
                pInst->stats.txCount++;
                status = STATUS_OK;
            }
            else if (halStatus == HAL_BUSY)
            {
                /* Queue to TX FIFO if mailboxes full */
                if (RingBuffer_Write(&pInst->txFifo, (const uint8_t *)pMessage,
                                    sizeof(CAN_Message_t)) == STATUS_OK)
                {
                    status = STATUS_OK;
                }
                else
                {
                    pInst->stats.txDropped++;
                    status = STATUS_ERROR_OVERFLOW;
                }
            }
            else
            {
                pInst->stats.txErrors++;
                status = STATUS_ERROR;
            }
        }
    }

    return status;
}

/**
 * @brief Receive CAN message
 */
Status_t BSP_CAN_Receive(uint8_t instance, CAN_Message_t *pMessage, uint32_t timeout_ms)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance <= BSP_CAN_INSTANCE_2) && (pMessage != NULL))
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            /* Try to read from RX FIFO */
            status = RingBuffer_Read(&pInst->rxFifo, (uint8_t *)pMessage, sizeof(CAN_Message_t));

            if (status != STATUS_OK)
            {
                status = STATUS_ERROR_NO_DATA;
            }
        }
    }

    return status;
}

/**
 * @brief Get number of messages available
 */
Status_t BSP_CAN_Available(uint8_t instance, uint32_t *pCount)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance <= BSP_CAN_INSTANCE_2) && (pCount != NULL))
    {
        CAN_Instance_t *pInst = can_get_instance(instance);

        if ((pInst != NULL) && pInst->initialized)
        {
            *pCount = pInst->rxFifo.count / sizeof(CAN_Message_t);
            status = STATUS_OK;
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
                *pState = CAN_BUS_OFF;
            }
            else if (canError & HAL_CAN_ERROR_EPV)
            {
                *pState = CAN_ERROR_PASSIVE;
            }
            else if (canError & HAL_CAN_ERROR_EWG)
            {
                *pState = CAN_ERROR_WARNING;
            }
            else if (canState == HAL_CAN_STATE_READY)
            {
                *pState = CAN_BUS_ACTIVE;
            }
            else
            {
                *pState = CAN_BUS_OFF;
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
static void can_process_rx_message(CAN_Instance_t *pInst, CAN_RxHeaderTypeDef *pRxHeader, uint8_t *pData)
{
    if ((pInst != NULL) && (pRxHeader != NULL) && (pData != NULL))
    {
        CAN_Message_t message;

        /* Build message structure */
        if (pRxHeader->IDE == CAN_ID_EXT)
        {
            message.id = pRxHeader->ExtId;
            message.ide = CAN_IDE_EXTENDED;
        }
        else
        {
            message.id = pRxHeader->StdId;
            message.ide = CAN_IDE_STANDARD;
        }

        message.rtr = (pRxHeader->RTR == CAN_RTR_REMOTE);
        message.dlc = (uint8_t)pRxHeader->DLC;
        (void)memcpy(message.data, pData, message.dlc);
        message.timestamp_ms = HAL_GetTick();

        /* Add to RX FIFO */
        if (RingBuffer_Write(&pInst->rxFifo, (const uint8_t *)&message,
                            sizeof(CAN_Message_t)) == STATUS_OK)
        {
            pInst->stats.rxCount++;

            /* Call RX callback if registered */
            if (pInst->rxCallback != NULL)
            {
                pInst->rxCallback(&message);
            }
        }
        else
        {
            pInst->stats.rxDropped++;
        }
    }
}

/**
 * @brief Process CAN errors
 */
static void can_process_errors(CAN_Instance_t *pInst, uint32_t errorCode)
{
    if (pInst != NULL)
    {
        if ((errorCode & HAL_CAN_ERROR_EWG) != 0U)
        {
            pInst->stats.errorWarning++;
        }

        if ((errorCode & HAL_CAN_ERROR_EPV) != 0U)
        {
            pInst->stats.errorPassive++;
        }

        if ((errorCode & HAL_CAN_ERROR_BOF) != 0U)
        {
            pInst->stats.busOff++;
        }

        if ((errorCode & HAL_CAN_ERROR_STF) != 0U)
        {
            pInst->stats.stuffError++;
        }

        if ((errorCode & HAL_CAN_ERROR_FOR) != 0U)
        {
            pInst->stats.formError++;
        }

        if ((errorCode & HAL_CAN_ERROR_ACK) != 0U)
        {
            pInst->stats.ackError++;
        }

        if ((errorCode & HAL_CAN_ERROR_BR) != 0U)
        {
            pInst->stats.bitError++;
        }

        if ((errorCode & HAL_CAN_ERROR_CRC) != 0U)
        {
            pInst->stats.crcError++;
        }

        /* Call error callback if registered */
        if (pInst->errorCallback != NULL)
        {
            pInst->errorCallback(errorCode);
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

    /* Determine which instance */
    if (hcan->Instance == CAN1)
    {
        pInst = &can1_instance;
    }
    else if (hcan->Instance == CAN2)
    {
        pInst = &can2_instance;
    }

    if (pInst != NULL)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
        {
            can_process_rx_message(pInst, &rxHeader, rxData);
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

    /* Determine which instance */
    if (hcan->Instance == CAN1)
    {
        pInst = &can1_instance;
    }
    else if (hcan->Instance == CAN2)
    {
        pInst = &can2_instance;
    }

    if (pInst != NULL)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) == HAL_OK)
        {
            can_process_rx_message(pInst, &rxHeader, rxData);
        }
    }
}

/**
 * @brief CAN error callback
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Instance_t *pInst = NULL;
    uint32_t errorCode = HAL_CAN_GetError(hcan);

    /* Determine which instance */
    if (hcan->Instance == CAN1)
    {
        pInst = &can1_instance;
    }
    else if (hcan->Instance == CAN2)
    {
        pInst = &can2_instance;
    }

    if (pInst != NULL)
    {
        can_process_errors(pInst, errorCode);
    }
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
