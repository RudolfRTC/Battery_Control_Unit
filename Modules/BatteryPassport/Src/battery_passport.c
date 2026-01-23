/**
 * @file    battery_passport.c
 * @brief   Battery Passport implementation - FRAM-backed identity, usage, health tracking
 * @author  Battery Control Unit Development Team
 * @date    2026-01-20
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B compatible
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "battery_passport.h"
#include "fram_driver.h"
#include "crc.h"
#include "timestamp.h"
#include "app_config.h"
#include <string.h>
#include <stddef.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief FRAM address for passport record (user data region start) */
#define PASSPORT_FRAM_ADDRESS       (0x1000U)

/** @brief FRAM record format version */
#define PASSPORT_RECORD_VERSION     (1U)

/** @brief Default values for unprovisioned batteries */
#define DEFAULT_NOMINAL_VOLTAGE_MV  (48000U)    /**< 48V nominal */
#define DEFAULT_NOMINAL_CAPACITY_MAH (50000U)   /**< 50Ah nominal */

/** @brief Temperature filter window for average calculation */
#define TEMP_FILTER_WINDOW_SIZE     (10U)

/** @brief Throughput accumulator scaling (milliseconds per hour) */
#define MS_PER_HOUR                 (3600000UL)

/*============================================================================*/
/* PRIVATE TYPES                                                              */
/*============================================================================*/

/**
 * @brief Module state
 */
typedef enum {
    PASSPORT_STATE_UNINITIALIZED = 0U,
    PASSPORT_STATE_READY         = 1U,
    PASSPORT_STATE_ERROR         = 2U
} PassportState_t;

/**
 * @brief Temperature filter state for rolling average
 */
typedef struct {
    int16_t  samples[TEMP_FILTER_WINDOW_SIZE];
    uint8_t  index;
    uint8_t  count;
    int32_t  sum;
} TempFilter_t;

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Module initialization state */
static PassportState_t g_passport_state = PASSPORT_STATE_UNINITIALIZED;

/** @brief Passport data (working copy in RAM) */
static BatteryPassport_Data_t g_passport_data;

/** @brief Statistics and metadata */
static BatteryPassport_Stats_t g_passport_stats;

/** @brief Last commit timestamp for rate limiting */
static uint32_t g_last_commit_time_ms = 0U;

/** @brief Timestamp of last update */
static uint32_t g_last_update_time_ms = 0U;

/** @brief Previous SOC for change detection */
static uint16_t g_prev_soc_permille = 0U;

/** @brief Previous temperature for change detection */
static int16_t g_prev_temp_dC = 0;

/** @brief Temperature filter state */
static TempFilter_t g_temp_filter;

/** @brief Throughput accumulator (intermediate value) */
static int64_t g_throughput_accumulator_mAs = 0;  /**< mA*s */

/** @brief Last throughput update timestamp */
static uint32_t g_last_throughput_time_ms = 0U;

/** @brief Dirty flag (changes pending commit) */
static bool g_data_dirty = false;

/** @brief Global initialization flag (for inline helper) */
bool g_passport_initialized = false;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static Status_t passport_load_from_fram(void);
static Status_t passport_save_to_fram(void);
static Status_t passport_validate_record(const BatteryPassport_FRAMRecord_t *pRecord);
static void passport_init_defaults(void);
static void passport_update_minmax(uint32_t voltage_mV, int32_t current_mA, int16_t temp_dC);
static void passport_update_temp_filter(int16_t temp_dC);
static int16_t passport_get_avg_temp(void);
static void passport_accumulate_throughput(int32_t current_mA, uint32_t now_ms);
static uint32_t passport_calculate_crc(const BatteryPassport_FRAMRecord_t *pRecord);
static bool passport_is_significant_change(void);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize Battery Passport module
 */
Status_t BatteryPassport_Init(void)
{
    Status_t status = STATUS_OK;

    if (g_passport_state == PASSPORT_STATE_READY)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        /* Clear working data */
        (void)memset(&g_passport_data, 0, sizeof(g_passport_data));
        (void)memset(&g_passport_stats, 0, sizeof(g_passport_stats));
        (void)memset(&g_temp_filter, 0, sizeof(g_temp_filter));

        g_throughput_accumulator_mAs = 0;
        g_data_dirty = false;
        g_prev_soc_permille = 0U;
        g_prev_temp_dC = 0;

        /* Load from FRAM (or initialize defaults if invalid) */
        status = passport_load_from_fram();

        if (status == STATUS_OK)
        {
            g_passport_state = PASSPORT_STATE_READY;
            g_passport_initialized = true;
            g_last_commit_time_ms = Timestamp_GetMillis();
            g_last_update_time_ms = g_last_commit_time_ms;
            g_last_throughput_time_ms = g_last_commit_time_ms;
        }
        else
        {
            g_passport_state = PASSPORT_STATE_ERROR;
            g_passport_initialized = false;
        }
    }

    return status;
}

/**
 * @brief De-initialize Battery Passport module
 */
Status_t BatteryPassport_DeInit(void)
{
    Status_t status = STATUS_OK;

    if (g_passport_state == PASSPORT_STATE_READY)
    {
        /* Commit any pending changes */
        status = BatteryPassport_ForceCommit();

        g_passport_state = PASSPORT_STATE_UNINITIALIZED;
        g_passport_initialized = false;
    }

    return status;
}

/**
 * @brief Update runtime measurements
 */
Status_t BatteryPassport_UpdateMeasurements(
    uint32_t packVoltage_mV,
    int32_t  packCurrent_mA,
    int16_t  temp_dC,
    uint16_t soc_permille,
    uint16_t soh_permille,
    uint32_t rul_cycles
)
{
    Status_t status = STATUS_OK;
    uint32_t now_ms = Timestamp_GetMillis();

    if (g_passport_state != PASSPORT_STATE_READY)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else if (soc_permille > 1000U)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (soh_permille > 1000U)
    {
        status = STATUS_ERROR_PARAM;
    }
    else
    {
        /* Update state of charge and health */
        g_passport_data.usage.soc_permille = soc_permille;
        g_passport_data.usage.soh_permille = soh_permille;
        g_passport_data.usage.rul_cycles_remaining = rul_cycles;

        /* Update min/max tracking */
        passport_update_minmax(packVoltage_mV, packCurrent_mA, temp_dC);

        /* Update temperature filter and average */
        passport_update_temp_filter(temp_dC);
        g_passport_data.usage.avg_temp_dC = passport_get_avg_temp();

        /* Accumulate throughput (integrate current over time) */
        passport_accumulate_throughput(packCurrent_mA, now_ms);

        /* Update timestamp */
        g_passport_data.events.last_update_timestamp_ms = now_ms;
        g_passport_data.events.timestamp_valid = true;

        /* Update statistics */
        g_passport_stats.update_count++;
        g_last_update_time_ms = now_ms;
        g_data_dirty = true;
    }

    return status;
}

/**
 * @brief Commit passport data to FRAM if needed
 */
Status_t BatteryPassport_CommitIfNeeded(void)
{
    Status_t status = STATUS_OK;
    uint32_t now_ms = Timestamp_GetMillis();
    uint32_t elapsed_ms;
    bool should_commit = false;

    if (g_passport_state != PASSPORT_STATE_READY)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Check if data is dirty */
        if (g_data_dirty)
        {
            /* Calculate time since last commit */
            elapsed_ms = now_ms - g_last_commit_time_ms;

            /* Commit if interval elapsed */
            if (elapsed_ms >= BATTERY_PASSPORT_COMMIT_INTERVAL_MS)
            {
                should_commit = true;
            }
            /* Or if significant change detected */
            else if (passport_is_significant_change())
            {
                should_commit = true;
            }
            else
            {
                /* No commit needed yet */
            }

            if (should_commit)
            {
                status = passport_save_to_fram();

                if (status == STATUS_OK)
                {
                    g_data_dirty = false;
                    g_last_commit_time_ms = now_ms;
                    g_passport_stats.last_commit_timestamp_ms = now_ms;
                    g_passport_stats.fram_write_count++;

                    /* Update previous values for change detection */
                    g_prev_soc_permille = g_passport_data.usage.soc_permille;
                    g_prev_temp_dC = g_passport_data.usage.avg_temp_dC;
                }
            }
        }
    }

    return status;
}

/**
 * @brief Force immediate commit to FRAM
 */
Status_t BatteryPassport_ForceCommit(void)
{
    Status_t status = STATUS_OK;
    uint32_t now_ms = Timestamp_GetMillis();

    if (g_passport_state != PASSPORT_STATE_READY)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        status = passport_save_to_fram();

        if (status == STATUS_OK)
        {
            g_data_dirty = false;
            g_last_commit_time_ms = now_ms;
            g_passport_stats.last_commit_timestamp_ms = now_ms;
            g_passport_stats.fram_write_count++;
        }
    }

    return status;
}

/**
 * @brief Provision static identity data
 */
Status_t BatteryPassport_ProvisionIdentity(const BatteryPassport_Identity_t *pIdentity)
{
    Status_t status = STATUS_OK;

    if (g_passport_state != PASSPORT_STATE_READY)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else if (pIdentity == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else
    {
        /* Copy identity data */
        (void)memcpy(&g_passport_data.identity, pIdentity, sizeof(BatteryPassport_Identity_t));

        /* Mark as provisioned */
        g_passport_stats.provisioned = true;
        g_data_dirty = true;

        /* Force immediate commit */
        status = BatteryPassport_ForceCommit();
    }

    return status;
}

/**
 * @brief Reset passport data to factory defaults
 */
Status_t BatteryPassport_ResetToDefaults(bool resetIdentity)
{
    Status_t status = STATUS_OK;

    if (g_passport_state != PASSPORT_STATE_READY)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        if (resetIdentity)
        {
            /* Clear everything */
            passport_init_defaults();
            g_passport_stats.provisioned = false;
        }
        else
        {
            /* Preserve identity, reset usage and events */
            BatteryPassport_Identity_t saved_identity;
            (void)memcpy(&saved_identity, &g_passport_data.identity, sizeof(saved_identity));

            passport_init_defaults();

            (void)memcpy(&g_passport_data.identity, &saved_identity, sizeof(saved_identity));
        }

        /* Reset filters and accumulators */
        (void)memset(&g_temp_filter, 0, sizeof(g_temp_filter));
        g_throughput_accumulator_mAs = 0;

        g_data_dirty = true;

        /* Force commit */
        status = BatteryPassport_ForceCommit();
    }

    return status;
}

/**
 * @brief Read data block by ID
 */
Status_t BatteryPassport_ReadBlock(
    uint16_t blockId,
    uint8_t  *pOutBuf,
    uint16_t bufSize,
    uint16_t *pOutLen
)
{
    Status_t status = STATUS_OK;
    uint16_t dataLen = 0U;
    const uint8_t *pSrc = NULL;

    if (g_passport_state != PASSPORT_STATE_READY)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else if (pOutBuf == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else
    {
        /* Map block ID to data structure */
        switch (blockId)
        {
            case BATTERY_PASSPORT_BLOCK_IDENTITY:
                pSrc = (const uint8_t *)&g_passport_data.identity;
                dataLen = sizeof(BatteryPassport_Identity_t);
                break;

            case BATTERY_PASSPORT_BLOCK_USAGE:
                pSrc = (const uint8_t *)&g_passport_data.usage;
                dataLen = sizeof(BatteryPassport_Usage_t);
                break;

            case BATTERY_PASSPORT_BLOCK_EVENTS:
                pSrc = (const uint8_t *)&g_passport_data.events;
                dataLen = sizeof(BatteryPassport_Events_t);
                break;

            case BATTERY_PASSPORT_BLOCK_FULL:
                pSrc = (const uint8_t *)&g_passport_data;
                dataLen = sizeof(BatteryPassport_Data_t);
                break;

            case BATTERY_PASSPORT_BLOCK_VERSION:
            {
                /* Return version as 2-byte structure: [major, minor] */
                if (bufSize >= 2U)
                {
                    pOutBuf[0] = BATTERY_PASSPORT_VERSION_MAJOR;
                    pOutBuf[1] = BATTERY_PASSPORT_VERSION_MINOR;
                    dataLen = 2U;
                }
                else
                {
                    status = STATUS_ERROR_OVERFLOW;
                }
                break;
            }

            case BATTERY_PASSPORT_BLOCK_STATS:
                pSrc = (const uint8_t *)&g_passport_stats;
                dataLen = sizeof(BatteryPassport_Stats_t);
                break;

            default:
                status = STATUS_ERROR_PARAM;
                break;
        }

        /* Copy data if valid */
        if ((status == STATUS_OK) && (pSrc != NULL))
        {
            if (bufSize >= dataLen)
            {
                (void)memcpy(pOutBuf, pSrc, dataLen);
            }
            else
            {
                status = STATUS_ERROR_OVERFLOW;
                dataLen = 0U;
            }
        }

        /* Return length if requested */
        if (pOutLen != NULL)
        {
            *pOutLen = dataLen;
        }
    }

    return status;
}

/**
 * @brief Write data block by ID
 */
Status_t BatteryPassport_WriteBlock(
    uint16_t blockId,
    const uint8_t *pInBuf,
    uint16_t inLen
)
{
    Status_t status = STATUS_OK;
    uint16_t expectedLen = 0U;
    uint8_t *pDst = NULL;

    if (g_passport_state != PASSPORT_STATE_READY)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else if (pInBuf == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else
    {
        /* Only allow writing identity and events blocks (usage is runtime-managed) */
        switch (blockId)
        {
            case BATTERY_PASSPORT_BLOCK_IDENTITY:
                pDst = (uint8_t *)&g_passport_data.identity;
                expectedLen = sizeof(BatteryPassport_Identity_t);
                break;

            case BATTERY_PASSPORT_BLOCK_EVENTS:
                pDst = (uint8_t *)&g_passport_data.events;
                expectedLen = sizeof(BatteryPassport_Events_t);
                break;

            default:
                status = STATUS_ERROR_NOT_SUPPORTED;  /* Read-only or invalid block */
                break;
        }

        if ((status == STATUS_OK) && (pDst != NULL))
        {
            if (inLen == expectedLen)
            {
                (void)memcpy(pDst, pInBuf, expectedLen);
                g_data_dirty = true;
            }
            else
            {
                status = STATUS_ERROR_PARAM;
            }
        }
    }

    return status;
}

/**
 * @brief UDS Read Data By Identifier handler
 */
Status_t BatteryPassport_UDS_ReadDataByIdentifier(
    uint16_t did,
    uint8_t  *pOutBuf,
    uint16_t bufSize,
    uint16_t *pOutLen
)
{
    /* Direct mapping: DID = BlockId */
    return BatteryPassport_ReadBlock(did, pOutBuf, bufSize, pOutLen);
}

/**
 * @brief UDS Write Data By Identifier handler
 */
Status_t BatteryPassport_UDS_WriteDataByIdentifier(
    uint16_t did,
    const uint8_t *pInBuf,
    uint16_t inLen
)
{
    /* Direct mapping: DID = BlockId */
    return BatteryPassport_WriteBlock(did, pInBuf, inLen);
}

/**
 * @brief Get passport statistics
 */
Status_t BatteryPassport_GetStatistics(BatteryPassport_Stats_t *pStats)
{
    Status_t status = STATUS_OK;

    if (pStats == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (g_passport_state != PASSPORT_STATE_READY)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        (void)memcpy(pStats, &g_passport_stats, sizeof(BatteryPassport_Stats_t));
    }

    return status;
}

/**
 * @brief Self-test function
 */
Status_t BatteryPassport_SelfTest(void)
{
    Status_t status = STATUS_OK;
    BatteryPassport_FRAMRecord_t testRecord;
    uint32_t calculatedCrc;

    if (g_passport_state != PASSPORT_STATE_READY)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Test 1: Read current record from FRAM */
        status = FRAM_Read(PASSPORT_FRAM_ADDRESS, (uint8_t *)&testRecord,
                          sizeof(BatteryPassport_FRAMRecord_t));

        if (status == STATUS_OK)
        {
            /* Test 2: Validate CRC */
            calculatedCrc = passport_calculate_crc(&testRecord);

            if (calculatedCrc != testRecord.crc32)
            {
                status = STATUS_ERROR_CRC;
            }
        }

        /* Test 3: Validate magic number */
        if (status == STATUS_OK)
        {
            if (testRecord.magic != BATTERY_PASSPORT_MAGIC)
            {
                status = STATUS_ERROR_RANGE;
            }
        }

        /* Update stats */
        if (status != STATUS_OK)
        {
            g_passport_stats.crc_error_count++;
        }
    }

    return status;
}

/**
 * @brief Log a fault event
 */
Status_t BatteryPassport_LogFault(uint32_t faultCode)
{
    Status_t status = STATUS_OK;

    if (g_passport_state != PASSPORT_STATE_READY)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        g_passport_data.events.last_fault_code = faultCode;
        g_passport_data.events.fault_counter_total++;
        g_data_dirty = true;
    }

    return status;
}

/**
 * @brief Update service date
 */
Status_t BatteryPassport_UpdateServiceDate(const BatteryPassport_Date_t *pDate)
{
    Status_t status = STATUS_OK;

    if (g_passport_state != PASSPORT_STATE_READY)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else if (pDate == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else
    {
        (void)memcpy(&g_passport_data.events.last_service_date, pDate,
                     sizeof(BatteryPassport_Date_t));
        g_data_dirty = true;
    }

    return status;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Load passport data from FRAM
 */
static Status_t passport_load_from_fram(void)
{
    Status_t status = STATUS_OK;
    BatteryPassport_FRAMRecord_t record;

    /* Read FRAM record */
    status = FRAM_Read(PASSPORT_FRAM_ADDRESS, (uint8_t *)&record,
                      sizeof(BatteryPassport_FRAMRecord_t));

    if (status == STATUS_OK)
    {
        /* Validate record */
        status = passport_validate_record(&record);

        if (status == STATUS_OK)
        {
            /* Copy valid data to working copy */
            (void)memcpy(&g_passport_data, &record.data, sizeof(BatteryPassport_Data_t));

            g_passport_stats.data_valid = true;
            g_passport_stats.provisioned = (record.data.identity.passport_version_major != 0U);

            /* Initialize previous values for change detection */
            g_prev_soc_permille = g_passport_data.usage.soc_permille;
            g_prev_temp_dC = g_passport_data.usage.avg_temp_dC;
        }
        else
        {
            /* CRC error or invalid data - initialize defaults */
            passport_init_defaults();
            g_passport_stats.data_valid = false;
            g_passport_stats.crc_error_count++;

            /* Write defaults to FRAM */
            status = passport_save_to_fram();
        }
    }
    else
    {
        /* FRAM read error - initialize defaults */
        passport_init_defaults();
        g_passport_stats.data_valid = false;
    }

    return status;
}

/**
 * @brief Save passport data to FRAM
 */
static Status_t passport_save_to_fram(void)
{
    Status_t status = STATUS_OK;
    BatteryPassport_FRAMRecord_t record;
    uint32_t crc;

    /* Prepare record header */
    record.magic = BATTERY_PASSPORT_MAGIC;
    record.version = PASSPORT_RECORD_VERSION;
    (void)memset(record.reserved, 0, sizeof(record.reserved));
    record.data_length = sizeof(BatteryPassport_Data_t);
    record.write_counter = g_passport_stats.fram_write_count + 1U;

    /* Copy data */
    (void)memcpy(&record.data, &g_passport_data, sizeof(BatteryPassport_Data_t));

    /* Calculate CRC over header + data (excluding CRC field itself) */
    crc = passport_calculate_crc(&record);
    record.crc32 = crc;

    /* Write to FRAM */
    status = FRAM_Write(PASSPORT_FRAM_ADDRESS, (uint8_t *)&record,
                       sizeof(BatteryPassport_FRAMRecord_t));

    return status;
}

/**
 * @brief Validate FRAM record
 */
static Status_t passport_validate_record(const BatteryPassport_FRAMRecord_t *pRecord)
{
    Status_t status = STATUS_OK;
    uint32_t calculatedCrc;

    if (pRecord == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else
    {
        /* Check magic number */
        if (pRecord->magic != BATTERY_PASSPORT_MAGIC)
        {
            status = STATUS_ERROR_RANGE;
        }
        /* Check version */
        else if (pRecord->version != PASSPORT_RECORD_VERSION)
        {
            status = STATUS_ERROR_RANGE;
        }
        /* Verify CRC */
        else
        {
            calculatedCrc = passport_calculate_crc(pRecord);

            if (calculatedCrc != pRecord->crc32)
            {
                status = STATUS_ERROR_CRC;
            }
        }
    }

    return status;
}

/**
 * @brief Initialize passport data to defaults
 */
static void passport_init_defaults(void)
{
    /* Clear all data */
    (void)memset(&g_passport_data, 0, sizeof(BatteryPassport_Data_t));

    /* Set identity defaults */
    g_passport_data.identity.passport_version_major = BATTERY_PASSPORT_VERSION_MAJOR;
    g_passport_data.identity.passport_version_minor = BATTERY_PASSPORT_VERSION_MINOR;
    g_passport_data.identity.chemistry_type = BATTERY_CHEM_UNKNOWN;
    g_passport_data.identity.nominal_voltage_mV = DEFAULT_NOMINAL_VOLTAGE_MV;
    g_passport_data.identity.nominal_capacity_mAh = DEFAULT_NOMINAL_CAPACITY_MAH;

    /* Set usage defaults (SOH = 100%) */
    g_passport_data.usage.soh_permille = 1000U;

    /* Initialize min/max to reasonable extremes (will be overwritten on first update) */
    g_passport_data.usage.min_temp_dC = 32767;      /* Max int16 */
    g_passport_data.usage.max_temp_dC = -32768;     /* Min int16 */
    g_passport_data.usage.min_pack_voltage_mV = UINT32_MAX;
    g_passport_data.usage.max_pack_voltage_mV = 0U;
    g_passport_data.usage.min_current_mA = INT32_MAX;
    g_passport_data.usage.max_current_mA = INT32_MIN;

    /* Set firmware version from build config (if available) */
    #ifdef FIRMWARE_VERSION_MAJOR
    g_passport_data.events.firmware_version_major = FIRMWARE_VERSION_MAJOR;
    #else
    g_passport_data.events.firmware_version_major = 1U;
    #endif

    #ifdef FIRMWARE_VERSION_MINOR
    g_passport_data.events.firmware_version_minor = FIRMWARE_VERSION_MINOR;
    #else
    g_passport_data.events.firmware_version_minor = 0U;
    #endif

    #ifdef FIRMWARE_VERSION_PATCH
    g_passport_data.events.firmware_version_patch = FIRMWARE_VERSION_PATCH;
    #else
    g_passport_data.events.firmware_version_patch = 0U;
    #endif
}

/**
 * @brief Update min/max tracking
 */
static void passport_update_minmax(uint32_t voltage_mV, int32_t current_mA, int16_t temp_dC)
{
    /* Update voltage min/max */
    if (voltage_mV < g_passport_data.usage.min_pack_voltage_mV)
    {
        g_passport_data.usage.min_pack_voltage_mV = voltage_mV;
    }

    if (voltage_mV > g_passport_data.usage.max_pack_voltage_mV)
    {
        g_passport_data.usage.max_pack_voltage_mV = voltage_mV;
    }

    /* Update current min/max */
    if (current_mA < g_passport_data.usage.min_current_mA)
    {
        g_passport_data.usage.min_current_mA = current_mA;
    }

    if (current_mA > g_passport_data.usage.max_current_mA)
    {
        g_passport_data.usage.max_current_mA = current_mA;
    }

    /* Update temperature min/max */
    if (temp_dC < g_passport_data.usage.min_temp_dC)
    {
        g_passport_data.usage.min_temp_dC = temp_dC;
    }

    if (temp_dC > g_passport_data.usage.max_temp_dC)
    {
        g_passport_data.usage.max_temp_dC = temp_dC;
    }
}

/**
 * @brief Update temperature filter
 */
static void passport_update_temp_filter(int16_t temp_dC)
{
    /* Remove oldest sample from sum if buffer is full */
    if (g_temp_filter.count >= TEMP_FILTER_WINDOW_SIZE)
    {
        g_temp_filter.sum -= (int32_t)g_temp_filter.samples[g_temp_filter.index];
    }
    else
    {
        g_temp_filter.count++;
    }

    /* Add new sample */
    g_temp_filter.samples[g_temp_filter.index] = temp_dC;
    g_temp_filter.sum += (int32_t)temp_dC;

    /* Advance index */
    g_temp_filter.index++;
    if (g_temp_filter.index >= TEMP_FILTER_WINDOW_SIZE)
    {
        g_temp_filter.index = 0U;
    }
}

/**
 * @brief Get average temperature from filter
 */
static int16_t passport_get_avg_temp(void)
{
    int16_t avg = 0;

    if (g_temp_filter.count > 0U)
    {
        avg = (int16_t)(g_temp_filter.sum / (int32_t)g_temp_filter.count);
    }

    return avg;
}

/**
 * @brief Accumulate throughput (integrate current over time)
 */
static void passport_accumulate_throughput(int32_t current_mA, uint32_t now_ms)
{
    uint32_t delta_ms;
    int64_t delta_mAs;

    if (g_last_throughput_time_ms != 0U)
    {
        /* Calculate time delta */
        delta_ms = now_ms - g_last_throughput_time_ms;

        /* Calculate charge transferred: Q = I * t (mA * ms = mA*ms) */
        delta_mAs = (int64_t)current_mA * (int64_t)delta_ms;

        /* Add to accumulator (absolute value for throughput) */
        if (delta_mAs < 0)
        {
            g_throughput_accumulator_mAs -= delta_mAs;
        }
        else
        {
            g_throughput_accumulator_mAs += delta_mAs;
        }

        /* Convert accumulated mA*ms to mAh and add to total */
        /* 1 mAh = 3600000 mA*ms */
        while (g_throughput_accumulator_mAs >= (int64_t)MS_PER_HOUR)
        {
            g_passport_data.usage.lifetime_throughput_mAh++;
            g_throughput_accumulator_mAs -= (int64_t)MS_PER_HOUR;
        }
    }

    g_last_throughput_time_ms = now_ms;
}

/**
 * @brief Calculate CRC32 over record (header + data, excluding CRC field)
 */
static uint32_t passport_calculate_crc(const BatteryPassport_FRAMRecord_t *pRecord)
{
    uint32_t crc;
    size_t length;

    /* Calculate CRC over everything except the CRC field itself */
    length = offsetof(BatteryPassport_FRAMRecord_t, crc32);

    crc = CRC_Calculate32((const uint8_t *)pRecord, (uint32_t)length);

    return crc;
}

/**
 * @brief Check if significant change occurred (triggers early commit)
 */
static bool passport_is_significant_change(void)
{
    bool significant = false;
    uint16_t soc_delta;
    int16_t temp_delta;

    /* Check SOC change */
    if (g_passport_data.usage.soc_permille > g_prev_soc_permille)
    {
        soc_delta = g_passport_data.usage.soc_permille - g_prev_soc_permille;
    }
    else
    {
        soc_delta = g_prev_soc_permille - g_passport_data.usage.soc_permille;
    }

    if (soc_delta >= BATTERY_PASSPORT_SOC_CHANGE_THRESH)
    {
        significant = true;
    }

    /* Check temperature change */
    if (!significant)
    {
        if (g_passport_data.usage.avg_temp_dC > g_prev_temp_dC)
        {
            temp_delta = g_passport_data.usage.avg_temp_dC - g_prev_temp_dC;
        }
        else
        {
            temp_delta = g_prev_temp_dC - g_passport_data.usage.avg_temp_dC;
        }

        if (temp_delta >= BATTERY_PASSPORT_TEMP_CHANGE_THRESH)
        {
            significant = true;
        }
    }

    return significant;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
