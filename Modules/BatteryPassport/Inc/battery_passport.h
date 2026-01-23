/**
 * @file    battery_passport.h
 * @brief   Battery Passport compatibility layer - identity, usage, health tracking
 * @author  Battery Control Unit Development Team
 * @date    2026-01-20
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B compatible
 *
 * @details This module implements a Battery Passport data model that:
 *          - Collects static battery identity (provisioned once)
 *          - Tracks runtime usage and health metrics
 *          - Logs compliance events and fault history
 *          - Persists data to FRAM with CRC32 protection
 *          - Exposes data via block-based read/write API for future UDS integration
 */

#ifndef BATTERY_PASSPORT_H
#define BATTERY_PASSPORT_H

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* PUBLIC CONSTANTS                                                           */
/*============================================================================*/

/** @brief Battery Passport version */
#define BATTERY_PASSPORT_VERSION_MAJOR      (1U)
#define BATTERY_PASSPORT_VERSION_MINOR      (0U)

/** @brief FRAM record magic number for validation */
#define BATTERY_PASSPORT_MAGIC              (0xBCB0A55U)

/** @brief String field sizes (fixed for MISRA compliance) */
#define BATTERY_PASSPORT_UUID_SIZE          (16U)   /**< 128-bit UUID */
#define BATTERY_PASSPORT_MFR_ID_SIZE        (16U)   /**< Manufacturer ID */
#define BATTERY_PASSPORT_MODEL_ID_SIZE      (16U)   /**< Model/Part number */

/** @brief FRAM write rate limit (milliseconds between commits) */
#define BATTERY_PASSPORT_COMMIT_INTERVAL_MS (10000U)  /**< 10 seconds */

/** @brief Threshold for "significant change" triggering early commit */
#define BATTERY_PASSPORT_SOC_CHANGE_THRESH  (50U)    /**< 5% SOC change */
#define BATTERY_PASSPORT_TEMP_CHANGE_THRESH (100U)   /**< 10°C change */

/*============================================================================*/
/* BLOCK ID DEFINITIONS (for UDS/diagnostics)                                 */
/*============================================================================*/

/**
 * @brief Data Identifier (DID) / Block ID mapping for diagnostics
 * @note  Based on ISO 14229-1 (UDS) standard DID ranges:
 *        0xF180-0xF19F: Vehicle manufacturer specific
 */
typedef enum {
    BATTERY_PASSPORT_BLOCK_IDENTITY     = 0xF190U,  /**< Static identification block */
    BATTERY_PASSPORT_BLOCK_USAGE        = 0xF191U,  /**< Usage & health metrics */
    BATTERY_PASSPORT_BLOCK_EVENTS       = 0xF192U,  /**< Events & compliance log */
    BATTERY_PASSPORT_BLOCK_FULL         = 0xF193U,  /**< Complete passport dataset */
    BATTERY_PASSPORT_BLOCK_VERSION      = 0xF194U,  /**< Passport version info */
    BATTERY_PASSPORT_BLOCK_STATS        = 0xF195U   /**< Statistics & metadata */
} BatteryPassport_BlockId_t;

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief Battery chemistry types
 */
typedef enum {
    BATTERY_CHEM_UNKNOWN = 0U,
    BATTERY_CHEM_LFP     = 1U,  /**< Lithium Iron Phosphate */
    BATTERY_CHEM_NMC     = 2U,  /**< Nickel Manganese Cobalt */
    BATTERY_CHEM_NCA     = 3U,  /**< Nickel Cobalt Aluminum */
    BATTERY_CHEM_LTO     = 4U,  /**< Lithium Titanate */
    BATTERY_CHEM_LMO     = 5U,  /**< Lithium Manganese Oxide */
    BATTERY_CHEM_CUSTOM  = 0xFFU
} BatteryPassport_Chemistry_t;

/**
 * @brief Packed manufacture date (year-month-day)
 * @note  Year stored as offset from 2000 (0 = 2000, 255 = 2255)
 */
typedef struct {
    uint8_t year;   /**< Year - 2000 (0-255) */
    uint8_t month;  /**< Month (1-12) */
    uint8_t day;    /**< Day (1-31) */
} __attribute__((packed)) BatteryPassport_Date_t;

/**
 * @brief Static battery identification (provisioned once at manufacturing)
 */
typedef struct {
    uint8_t  passport_version_major;                      /**< Passport data format version */
    uint8_t  passport_version_minor;
    uint8_t  battery_unique_id[BATTERY_PASSPORT_UUID_SIZE];  /**< 128-bit UUID */
    char     manufacturer_id[BATTERY_PASSPORT_MFR_ID_SIZE];  /**< Null-terminated */
    char     model_id[BATTERY_PASSPORT_MODEL_ID_SIZE];       /**< Null-terminated */

    BatteryPassport_Chemistry_t chemistry_type;           /**< Battery chemistry */
    uint32_t nominal_voltage_mV;                          /**< Nominal pack voltage */
    uint32_t nominal_capacity_mAh;                        /**< Nominal capacity */

    BatteryPassport_Date_t manufacture_date;              /**< Production date */

    uint8_t  cells_series;                                /**< Cells in series */
    uint8_t  cells_parallel;                              /**< Cells in parallel */
    uint8_t  reserved[2];                                 /**< Alignment padding */
} __attribute__((packed)) BatteryPassport_Identity_t;

/**
 * @brief Runtime usage and health metrics (updated periodically)
 */
typedef struct {
    uint16_t soc_permille;          /**< State of Charge (0-1000 = 0-100%) */
    uint16_t soh_permille;          /**< State of Health (0-1000 = 0-100%) */

    uint32_t rul_cycles_remaining;  /**< Remaining Useful Life in cycles */
    uint32_t cycle_count;           /**< Total charge/discharge cycles */

    uint64_t lifetime_throughput_mAh;  /**< Cumulative Ah throughput */

    /* Temperature tracking (deci-Celsius: value/10 = °C) */
    int16_t  min_temp_dC;           /**< Minimum recorded temperature */
    int16_t  max_temp_dC;           /**< Maximum recorded temperature */
    int16_t  avg_temp_dC;           /**< Rolling average temperature */

    /* Voltage tracking */
    uint32_t min_pack_voltage_mV;   /**< Minimum pack voltage */
    uint32_t max_pack_voltage_mV;   /**< Maximum pack voltage */

    /* Current tracking */
    int32_t  min_current_mA;        /**< Minimum (most negative) current */
    int32_t  max_current_mA;        /**< Maximum current */

    uint32_t total_runtime_hours;   /**< Total operating time */
    uint8_t  reserved[4];           /**< Alignment padding */
} __attribute__((packed)) BatteryPassport_Usage_t;

/**
 * @brief Events and compliance log
 */
typedef struct {
    uint32_t last_fault_code;       /**< Most recent DTC (Diagnostic Trouble Code) */
    uint32_t fault_counter_total;   /**< Total fault events logged */

    BatteryPassport_Date_t last_service_date;  /**< Last maintenance date */

    uint8_t  firmware_version_major;  /**< BCU firmware version */
    uint8_t  firmware_version_minor;
    uint8_t  firmware_version_patch;

    uint32_t last_update_timestamp_ms;  /**< Timestamp of last update */
    bool     timestamp_valid;           /**< True if timestamp is valid */

    uint8_t  reserved[7];              /**< Alignment padding */
} __attribute__((packed)) BatteryPassport_Events_t;

/**
 * @brief Complete battery passport dataset
 */
typedef struct {
    BatteryPassport_Identity_t  identity;   /**< Static identification */
    BatteryPassport_Usage_t     usage;      /**< Usage & health */
    BatteryPassport_Events_t    events;     /**< Events & compliance */
} __attribute__((packed)) BatteryPassport_Data_t;

/**
 * @brief FRAM record structure with header, data, and CRC protection
 */
typedef struct {
    /* Header */
    uint32_t magic;                 /**< Magic number (BATTERY_PASSPORT_MAGIC) */
    uint8_t  version;               /**< Record format version */
    uint8_t  reserved[3];           /**< Alignment */
    uint32_t data_length;           /**< Payload length in bytes */
    uint32_t write_counter;         /**< Monotonic write counter */

    /* Payload */
    BatteryPassport_Data_t data;

    /* Footer */
    uint32_t crc32;                 /**< CRC32 over header + data */
} __attribute__((packed)) BatteryPassport_FRAMRecord_t;

/**
 * @brief Statistics and metadata
 */
typedef struct {
    uint32_t fram_write_count;      /**< Total FRAM commits */
    uint32_t crc_error_count;       /**< CRC validation failures */
    uint32_t last_commit_timestamp_ms;  /**< Last successful FRAM commit */
    uint32_t update_count;          /**< Total measurement updates */
    bool     data_valid;            /**< True if passport data is valid */
    bool     provisioned;           /**< True if identity data provisioned */
} BatteryPassport_Stats_t;

/*============================================================================*/
/* PUBLIC FUNCTION PROTOTYPES                                                 */
/*============================================================================*/

/**
 * @brief Initialize Battery Passport module
 * @details Loads passport data from FRAM, validates CRC, initializes defaults if needed
 * @return STATUS_OK on success, error code otherwise
 * @note  Must be called before any other passport functions
 */
Status_t BatteryPassport_Init(void);

/**
 * @brief De-initialize Battery Passport module
 * @details Commits pending changes to FRAM and releases resources
 * @return STATUS_OK on success
 */
Status_t BatteryPassport_DeInit(void);

/**
 * @brief Update runtime measurements
 * @details Feed in current sensor readings to update usage & health metrics
 * @param[in] packVoltage_mV    Pack voltage in millivolts
 * @param[in] packCurrent_mA    Pack current in milliamps (+ = charge, - = discharge)
 * @param[in] temp_dC           Temperature in deci-Celsius (value/10 = °C)
 * @param[in] soc_permille      State of Charge (0-1000 = 0-100.0%)
 * @param[in] soh_permille      State of Health (0-1000 = 0-100.0%)
 * @param[in] rul_cycles        Remaining Useful Life in cycles
 * @return STATUS_OK on success, STATUS_ERROR_PARAM if inputs out of range
 * @note  Call periodically (e.g., 100ms task) to update metrics
 */
Status_t BatteryPassport_UpdateMeasurements(
    uint32_t packVoltage_mV,
    int32_t  packCurrent_mA,
    int16_t  temp_dC,
    uint16_t soc_permille,
    uint16_t soh_permille,
    uint32_t rul_cycles
);

/**
 * @brief Commit passport data to FRAM if needed
 * @details Rate-limited write (respects COMMIT_INTERVAL_MS), or forces on significant change
 * @return STATUS_OK if committed or not needed, error code on FRAM failure
 * @note  Call from slow periodic task (e.g., 100ms or 1s)
 */
Status_t BatteryPassport_CommitIfNeeded(void);

/**
 * @brief Force immediate commit to FRAM
 * @return STATUS_OK on success, error code otherwise
 * @note  Use sparingly (e.g., before shutdown)
 */
Status_t BatteryPassport_ForceCommit(void);

/**
 * @brief Provision static identity data (manufacturing/service use)
 * @param[in] pIdentity  Pointer to identity structure
 * @return STATUS_OK on success, STATUS_ERROR_PARAM if NULL or invalid
 * @note  This function should be called once during manufacturing or service
 */
Status_t BatteryPassport_ProvisionIdentity(const BatteryPassport_Identity_t *pIdentity);

/**
 * @brief Reset passport data to factory defaults
 * @details Clears all usage/health data, preserves identity if provisioned
 * @param[in] resetIdentity  If true, also reset identity to defaults
 * @return STATUS_OK on success
 * @note  Use for service/refurbishment scenarios
 */
Status_t BatteryPassport_ResetToDefaults(bool resetIdentity);

/**
 * @brief Read data block by ID (for diagnostics/UDS)
 * @param[in]  blockId   Block identifier (see BatteryPassport_BlockId_t)
 * @param[out] pOutBuf   Buffer to receive data
 * @param[in]  bufSize   Size of output buffer
 * @param[out] pOutLen   Actual bytes written (may be NULL)
 * @return STATUS_OK on success, STATUS_ERROR_PARAM if invalid block/buffer
 * @note  Thread-safe: returns snapshot of data
 */
Status_t BatteryPassport_ReadBlock(
    uint16_t blockId,
    uint8_t  *pOutBuf,
    uint16_t bufSize,
    uint16_t *pOutLen
);

/**
 * @brief Write data block by ID (for provisioning/service)
 * @param[in] blockId   Block identifier
 * @param[in] pInBuf    Data to write
 * @param[in] inLen     Data length in bytes
 * @return STATUS_OK on success, STATUS_ERROR_PARAM if invalid
 * @note  Only identity/events blocks are writable (usage is read-only)
 */
Status_t BatteryPassport_WriteBlock(
    uint16_t blockId,
    const uint8_t *pInBuf,
    uint16_t inLen
);

/**
 * @brief UDS Read Data By Identifier handler (future integration)
 * @param[in]  did      Data Identifier (matches BlockId)
 * @param[out] pOutBuf  Response buffer
 * @param[in]  bufSize  Buffer size
 * @param[out] pOutLen  Response length
 * @return STATUS_OK on success
 * @note  Call from UDS service 0x22 handler
 */
Status_t BatteryPassport_UDS_ReadDataByIdentifier(
    uint16_t did,
    uint8_t  *pOutBuf,
    uint16_t bufSize,
    uint16_t *pOutLen
);

/**
 * @brief UDS Write Data By Identifier handler (future integration)
 * @param[in] did     Data Identifier
 * @param[in] pInBuf  Data to write
 * @param[in] inLen   Data length
 * @return STATUS_OK on success
 * @note  Call from UDS service 0x2E handler
 */
Status_t BatteryPassport_UDS_WriteDataByIdentifier(
    uint16_t did,
    const uint8_t *pInBuf,
    uint16_t inLen
);

/**
 * @brief Get passport statistics
 * @param[out] pStats  Pointer to statistics structure
 * @return STATUS_OK on success, STATUS_ERROR_PARAM if NULL
 */
Status_t BatteryPassport_GetStatistics(BatteryPassport_Stats_t *pStats);

/**
 * @brief Self-test function (validates FRAM read/write and CRC)
 * @return STATUS_OK if all tests pass, error code otherwise
 * @note  Non-destructive test, safe to call at runtime
 */
Status_t BatteryPassport_SelfTest(void);

/**
 * @brief Log a fault event
 * @param[in] faultCode  DTC fault code
 * @return STATUS_OK on success
 * @note  Updates fault counter and last fault code
 */
Status_t BatteryPassport_LogFault(uint32_t faultCode);

/**
 * @brief Update service date
 * @param[in] pDate  Service date
 * @return STATUS_OK on success, STATUS_ERROR_PARAM if NULL
 */
Status_t BatteryPassport_UpdateServiceDate(const BatteryPassport_Date_t *pDate);

/*============================================================================*/
/* INLINE HELPER FUNCTIONS                                                   */
/*============================================================================*/

/**
 * @brief Check if passport data is valid
 * @return true if data loaded and CRC validated
 */
static inline bool BatteryPassport_IsValid(void)
{
    /* Implementation will check internal state */
    extern bool g_passport_initialized;
    return g_passport_initialized;
}

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_PASSPORT_H */
