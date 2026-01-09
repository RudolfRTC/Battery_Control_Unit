/**
 * @file    data_logger.h
 * @brief   Data logger header
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    FRAM-based circular buffer data logging
 *
 * @copyright Copyright (c) 2026
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"

/*============================================================================*/
/* CONSTANTS                                                                  */
/*============================================================================*/

/** @brief Log entry size */
#define DATALOG_ENTRY_SIZE      (64U)

/** @brief Maximum log entries (from FRAM allocation) */
#define DATALOG_MAX_ENTRIES     (128U)  /* 8KB / 64 bytes */

/*============================================================================*/
/* TYPES                                                                      */
/*============================================================================*/

/** @brief Log entry structure */
typedef struct {
    uint32_t timestamp_ms;

    /* LEM current measurements (first 4 channels) */
    int32_t lem_current_mA[4];

    /* Power rail voltages */
    uint16_t voltage_12v_mV;
    uint16_t voltage_5v_mV;
    uint16_t voltage_3v3d_mV;
    uint16_t voltage_3v3a_mV;

    /* Temperature */
    int16_t temperature_dC;  /* DeciCelsius (0.1°C) */

    /* System status */
    uint8_t app_state;
    uint8_t power_good;
    uint8_t safety_ok;
    uint8_t active_dtcs;

    /* Output states (bitmap - 20 outputs, 3 bytes) */
    uint8_t output_states[3];

    /* Input states (bitmap - 20 inputs, 3 bytes) */
    uint8_t input_states[3];

    /* Reserved for future use */
    uint8_t reserved[12];
} __attribute__((packed)) DataLog_Entry_t;

/** @brief Data logger statistics */
typedef struct {
    uint32_t totalEntries;
    uint32_t currentIndex;
    uint32_t writeCount;
    uint32_t readCount;
    uint32_t writeErrors;
    uint32_t overruns;
} DataLog_Statistics_t;

/*============================================================================*/
/* PUBLIC FUNCTION PROTOTYPES                                                 */
/*============================================================================*/

/**
 * @brief Initialize data logger
 * @return Status code
 */
Status_t DataLog_Init(void);

/**
 * @brief Log current system data
 * @return Status code
 */
Status_t DataLog_LogData(void);

/**
 * @brief Read log entry by index
 * @param[in] index Log entry index
 * @param[out] pEntry Pointer to entry structure
 * @return Status code
 */
Status_t DataLog_ReadEntry(uint32_t index, DataLog_Entry_t *pEntry);

/**
 * @brief Get latest log entry
 * @param[out] pEntry Pointer to entry structure
 * @return Status code
 */
Status_t DataLog_GetLatest(DataLog_Entry_t *pEntry);

/**
 * @brief Clear all log entries
 * @return Status code
 */
Status_t DataLog_Clear(void);

/**
 * @brief Get data logger statistics
 * @param[out] pStats Pointer to statistics structure
 * @return Status code
 */
Status_t DataLog_GetStatistics(DataLog_Statistics_t *pStats);

/**
 * @brief Export log entries
 * @param[out] pBuffer Output buffer
 * @param[in] bufferSize Buffer size
 * @param[in] startIndex Start index
 * @param[in] count Number of entries to export
 * @param[out] pExported Number of entries actually exported
 * @return Status code
 */
Status_t DataLog_Export(uint8_t *pBuffer, uint32_t bufferSize,
                       uint32_t startIndex, uint32_t count, uint32_t *pExported);

/**
 * @brief De-initialize data logger
 * @return Status code
 */
Status_t DataLog_DeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* DATA_LOGGER_H */

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
