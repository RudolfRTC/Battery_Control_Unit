/**
 * @file    battery_passport_integration_example.c
 * @brief   Integration example for Battery Passport module
 * @author  Battery Control Unit Development Team
 * @date    2026-01-20
 * @version 1.0.0
 *
 * @note    This file demonstrates how to integrate the Battery Passport module
 *          into your main application. Copy the relevant sections to your
 *          application initialization and scheduler task registration code.
 *
 * @warning This is an EXAMPLE ONLY - do not add this file to the build!
 *          Integrate the code snippets below into your existing application files.
 */

/*============================================================================*/
/* INTEGRATION STEP 1: Add includes to your main application                 */
/*============================================================================*/

#include "battery_passport.h"
#include "bcu_scheduler.h"
#include "lem_sensor.h"
#include "temp_sensor.h"
#include "fram_driver.h"
/* Add other necessary includes from your application */

/*============================================================================*/
/* INTEGRATION STEP 2: Add initialization in your main() or app_init()       */
/*============================================================================*/

/**
 * @brief Example initialization sequence
 * @note  Call this from your main() function or application initialization
 */
void example_application_init(void)
{
    Status_t status;

    /* Initialize HAL and peripherals first */
    /* ... your existing HAL_Init(), clock config, etc. ... */

    /* Initialize I2C (required for FRAM and TMP1075) */
    /* ... your existing BSP_I2C_Init() ... */

    /* Initialize FRAM driver */
    status = FRAM_Init();
    if (status != STATUS_OK)
    {
        /* Handle error - FRAM is critical for passport persistence */
        Error_Handler(ERROR_FRAM_I2C_FAULT);
    }

    /* Initialize Battery Passport module */
    status = BatteryPassport_Init();
    if (status != STATUS_OK)
    {
        /* Handle error - may continue with defaults, but log the issue */
        Error_Handler(ERROR_DATA_STORAGE);
    }

    /* Initialize sensor drivers (required for measurements) */
    status = LEM_Init();
    status |= TempSensor_Init();
    if (status != STATUS_OK)
    {
        Error_Handler(ERROR_SENSOR_INIT);
    }

    /* Initialize scheduler */
    status = Scheduler_Init();
    if (status != STATUS_OK)
    {
        Error_Handler(ERROR_SCHEDULER_INIT);
    }

    /* Register all periodic tasks (see Step 3 below) */
    example_register_passport_tasks();

    /* Optional: Run self-test */
    status = BatteryPassport_SelfTest();
    if (status != STATUS_OK)
    {
        /* Log warning but continue - data may recover from defaults */
        /* Consider re-provisioning if self-test fails repeatedly */
    }

    /* Optional: Provision identity if first boot or unprovisioned */
    BatteryPassport_Stats_t stats;
    BatteryPassport_GetStatistics(&stats);
    if (!stats.provisioned)
    {
        /* Battery not provisioned - call provisioning function */
        /* This should typically be done at manufacturing or via diagnostics */
        /* example_provision_battery_identity(); */
    }
}

/*============================================================================*/
/* INTEGRATION STEP 3: Register periodic tasks with scheduler                */
/*============================================================================*/

/**
 * @brief Register Battery Passport periodic tasks
 * @note  Call this during application initialization
 */
void example_register_passport_tasks(void)
{
    Status_t status;

    /* Task 1: Update measurements every 100ms */
    status = Scheduler_RegisterJob(
        "Passport_Update",              /* Task name */
        example_passport_update_task,   /* Task function */
        100U,                           /* Period: 100ms */
        SCHED_PRIORITY_SLOW             /* Priority: SLOW (non-critical) */
    );

    if (status != STATUS_OK)
    {
        Error_Handler(ERROR_SCHEDULER_REGISTRATION);
    }

    /* Task 2: Commit to FRAM every 1 second (rate-limited internally) */
    status = Scheduler_RegisterJob(
        "Passport_Commit",              /* Task name */
        example_passport_commit_task,   /* Task function */
        1000U,                          /* Period: 1000ms (1 second) */
        SCHED_PRIORITY_VERY_SLOW        /* Priority: VERY_SLOW (background) */
    );

    if (status != STATUS_OK)
    {
        Error_Handler(ERROR_SCHEDULER_REGISTRATION);
    }
}

/*============================================================================*/
/* INTEGRATION STEP 4: Implement periodic task functions                     */
/*============================================================================*/

/**
 * @brief Periodic task to update passport measurements (100ms)
 * @param[in] now_ms  Current system timestamp
 */
void example_passport_update_task(uint32_t now_ms)
{
    Status_t status;
    Current_mA_t pack_current_mA = 0;
    Voltage_mV_t pack_voltage_mV = 0U;
    int32_t temp_mC = 0;
    uint16_t soc_permille = 0U;
    uint16_t soh_permille = 1000U;  /* Default: 100% SOH */
    uint32_t rul_cycles = 0U;

    /* 1. Read pack current from LEM sensor (main pack current, e.g., channel 0) */
    status = LEM_ReadCurrent(0U, &pack_current_mA);
    if (status != STATUS_OK)
    {
        /* Handle sensor error - use last known value or zero */
        pack_current_mA = 0;
    }

    /* 2. Read pack voltage from power monitoring module */
    /* Replace with your actual power monitoring function */
    /* Example: status = PM_GetPackVoltage(&pack_voltage_mV); */
    pack_voltage_mV = 48000U;  /* Placeholder: 48V nominal */

    /* 3. Read temperature from TMP1075 sensor */
    status = TempSensor_ReadTemperature(&temp_mC);
    if (status != STATUS_OK)
    {
        /* Handle sensor error - use safe default */
        temp_mC = 25000;  /* 25°C default */
    }

    /* 4. Get SOC from your battery management algorithm */
    /* Replace with your actual SOC calculation function */
    /* Example: soc_permille = BatteryManagement_GetSOC(); */
    soc_permille = 850U;  /* Placeholder: 85.0% SOC */

    /* 5. Get SOH from your battery health estimation algorithm */
    /* Replace with your actual SOH estimation function */
    /* Example: soh_permille = BatteryManagement_GetSOH(); */
    soh_permille = 980U;  /* Placeholder: 98.0% SOH */

    /* 6. Get RUL (Remaining Useful Life) estimate */
    /* Replace with your actual RUL calculation */
    /* Example: rul_cycles = BatteryManagement_GetRUL(); */
    rul_cycles = 5000U;  /* Placeholder: 5000 cycles remaining */

    /* 7. Update Battery Passport with all measurements */
    status = BatteryPassport_UpdateMeasurements(
        pack_voltage_mV,
        pack_current_mA,
        (int16_t)(temp_mC / 100),  /* Convert milliCelsius to deciCelsius */
        soc_permille,
        soh_permille,
        rul_cycles
    );

    if (status != STATUS_OK)
    {
        /* Log error but continue - non-critical failure */
        /* Error_LogEvent(ERROR_PASSPORT_UPDATE_FAILED); */
    }

    (void)now_ms;  /* Suppress unused parameter warning */
}

/**
 * @brief Periodic task to commit passport data to FRAM (1 second)
 * @param[in] now_ms  Current system timestamp
 */
void example_passport_commit_task(uint32_t now_ms)
{
    Status_t status;

    /* Commit if needed (rate-limited internally to 10s or on significant change) */
    status = BatteryPassport_CommitIfNeeded();

    if (status != STATUS_OK)
    {
        /* Log FRAM write error - may indicate I2C bus issue */
        /* Error_LogEvent(ERROR_FRAM_WRITE_FAILED); */
    }

    (void)now_ms;  /* Suppress unused parameter warning */
}

/*============================================================================*/
/* INTEGRATION STEP 5: Provisioning function (manufacturing/service)         */
/*============================================================================*/

/**
 * @brief Provision battery identity data (call once at manufacturing)
 * @note  This function should be called via:
 *        - Manufacturing test fixture (UART/USB command)
 *        - Service tool via CAN diagnostics (UDS WriteDataByIdentifier)
 *        - One-time initialization script
 */
void example_provision_battery_identity(void)
{
    BatteryPassport_Identity_t identity;
    BatteryPassport_Date_t manufacture_date;
    Status_t status;

    /* Clear structure */
    (void)memset(&identity, 0, sizeof(identity));

    /* Set passport version */
    identity.passport_version_major = BATTERY_PASSPORT_VERSION_MAJOR;
    identity.passport_version_minor = BATTERY_PASSPORT_VERSION_MINOR;

    /* Set unique ID (128-bit UUID) */
    /* TODO: Replace with actual UUID from EEPROM, serial number, or generated */
    const uint8_t example_uuid[16] = {
        0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88
    };
    (void)memcpy(identity.battery_unique_id, example_uuid, 16U);

    /* Set manufacturer ID */
    (void)strncpy(identity.manufacturer_id, "ACME Battery Co", 15U);
    identity.manufacturer_id[15] = '\0';  /* Ensure null termination */

    /* Set model/part number */
    (void)strncpy(identity.model_id, "LFP-48V-50Ah", 15U);
    identity.model_id[15] = '\0';

    /* Set battery chemistry */
    identity.chemistry_type = BATTERY_CHEM_LFP;  /* LiFePO4 */

    /* Set nominal ratings */
    identity.nominal_voltage_mV = 48000U;      /* 48V nominal */
    identity.nominal_capacity_mAh = 50000U;    /* 50Ah capacity */

    /* Set manufacture date (example: 2026-01-15) */
    manufacture_date.year = 26U;   /* 2026 - 2000 */
    manufacture_date.month = 1U;   /* January */
    manufacture_date.day = 15U;    /* 15th */
    (void)memcpy(&identity.manufacture_date, &manufacture_date, sizeof(BatteryPassport_Date_t));

    /* Set cell configuration */
    identity.cells_series = 15U;    /* 15S (15 cells in series, 15 × 3.2V ≈ 48V) */
    identity.cells_parallel = 1U;   /* 1P (1 cell in parallel) */

    /* Write to passport (will force commit to FRAM) */
    status = BatteryPassport_ProvisionIdentity(&identity);

    if (status == STATUS_OK)
    {
        /* Log success */
        /* printf("Battery provisioned successfully\n"); */
    }
    else
    {
        /* Handle error */
        Error_Handler(ERROR_PASSPORT_PROVISION_FAILED);
    }
}

/*============================================================================*/
/* INTEGRATION STEP 6: Fault event handler integration                       */
/*============================================================================*/

/**
 * @brief Example fault handler with passport logging
 * @param[in] fault_code  DTC fault code
 */
void example_fault_handler(uint32_t fault_code)
{
    /* Log fault to passport */
    (void)BatteryPassport_LogFault(fault_code);

    /* Force immediate commit on critical faults */
    if ((fault_code & 0xFF00U) == 0x0800U)  /* Safety-related faults */
    {
        (void)BatteryPassport_ForceCommit();
    }

    /* Continue with your existing fault handling */
    /* ... */
}

/*============================================================================*/
/* INTEGRATION STEP 7: Shutdown/sleep handler integration                    */
/*============================================================================*/

/**
 * @brief Example shutdown handler
 * @note  Call before entering sleep mode or power-off
 */
void example_system_shutdown(void)
{
    /* Force commit passport data before shutdown */
    (void)BatteryPassport_ForceCommit();

    /* De-initialize passport module */
    (void)BatteryPassport_DeInit();

    /* Continue with other shutdown tasks */
    /* ... */
}

/*============================================================================*/
/* INTEGRATION STEP 8: UDS/diagnostics integration (optional, future)        */
/*============================================================================*/

/**
 * @brief Example UDS ReadDataByIdentifier handler (service 0x22)
 * @param[in]  request     UDS request buffer
 * @param[in]  request_len Request length
 * @param[out] response    UDS response buffer
 * @param[out] response_len Response length
 * @return UDS response code
 */
uint8_t example_uds_read_data_handler(
    const uint8_t *request,
    uint16_t request_len,
    uint8_t *response,
    uint16_t *response_len
)
{
    Status_t status;
    uint16_t did;
    uint16_t data_len;

    /* Check minimum request length */
    if (request_len < 3U)
    {
        return 0x13U;  /* UDS NRC: Incorrect message length */
    }

    /* Extract DID (Data Identifier) */
    did = ((uint16_t)request[1] << 8) | (uint16_t)request[2];

    /* Check if DID is a Battery Passport block */
    if ((did >= BATTERY_PASSPORT_BLOCK_IDENTITY) &&
        (did <= BATTERY_PASSPORT_BLOCK_STATS))
    {
        /* Call Battery Passport UDS handler */
        status = BatteryPassport_UDS_ReadDataByIdentifier(
            did,
            &response[3],       /* Data starts after service ID + DID */
            256U,               /* Max response size */
            &data_len
        );

        if (status == STATUS_OK)
        {
            /* Build positive response */
            response[0] = 0x62U;  /* Positive response: 0x22 + 0x40 */
            response[1] = (uint8_t)(did >> 8);
            response[2] = (uint8_t)(did & 0xFFU);
            *response_len = 3U + data_len;
            return 0x00U;  /* Positive response */
        }
        else
        {
            return 0x31U;  /* UDS NRC: Request out of range */
        }
    }

    /* Not a passport DID - handle other DIDs */
    return 0x31U;  /* UDS NRC: Request out of range */
}

/**
 * @brief Example UDS WriteDataByIdentifier handler (service 0x2E)
 * @param[in] request      UDS request buffer
 * @param[in] request_len  Request length
 * @return UDS response code
 * @note  Should only be allowed in programming/service session with security access!
 */
uint8_t example_uds_write_data_handler(
    const uint8_t *request,
    uint16_t request_len
)
{
    Status_t status;
    uint16_t did;

    /* Check minimum request length */
    if (request_len < 4U)
    {
        return 0x13U;  /* UDS NRC: Incorrect message length */
    }

    /* TODO: Check security access and session type */
    /* if (!UDS_IsSecurityUnlocked()) return 0x33; // Security access denied */
    /* if (UDS_GetSession() != UDS_SESSION_PROGRAMMING) return 0x7F; // Service not supported in active session */

    /* Extract DID */
    did = ((uint16_t)request[1] << 8) | (uint16_t)request[2];

    /* Check if DID is a Battery Passport block */
    if ((did >= BATTERY_PASSPORT_BLOCK_IDENTITY) &&
        (did <= BATTERY_PASSPORT_BLOCK_STATS))
    {
        /* Call Battery Passport UDS handler */
        status = BatteryPassport_UDS_WriteDataByIdentifier(
            did,
            &request[3],           /* Data starts after service ID + DID */
            request_len - 3U       /* Data length */
        );

        if (status == STATUS_OK)
        {
            return 0x00U;  /* Positive response */
        }
        else if (status == STATUS_ERROR_NOT_SUPPORTED)
        {
            return 0x33U;  /* UDS NRC: Security access denied (read-only) */
        }
        else
        {
            return 0x31U;  /* UDS NRC: Request out of range */
        }
    }

    /* Not a passport DID */
    return 0x31U;  /* UDS NRC: Request out of range */
}

/*============================================================================*/
/* INTEGRATION STEP 9: Diagnostics/debug functions (optional)                */
/*============================================================================*/

/**
 * @brief Print passport data to console (debug/diagnostics)
 */
void example_print_passport_data(void)
{
    BatteryPassport_Identity_t identity;
    BatteryPassport_Usage_t usage;
    BatteryPassport_Events_t events;
    BatteryPassport_Stats_t stats;
    uint16_t len;
    Status_t status;

    /* Read identity block */
    status = BatteryPassport_ReadBlock(
        BATTERY_PASSPORT_BLOCK_IDENTITY,
        (uint8_t *)&identity,
        sizeof(identity),
        &len
    );

    if (status == STATUS_OK)
    {
        /* Print identity information */
        /* printf("Battery Passport Identity:\n"); */
        /* printf("  Manufacturer: %s\n", identity.manufacturer_id); */
        /* printf("  Model: %s\n", identity.model_id); */
        /* printf("  Chemistry: %u\n", identity.chemistry_type); */
        /* printf("  Nominal Voltage: %lu mV\n", identity.nominal_voltage_mV); */
        /* printf("  Nominal Capacity: %lu mAh\n", identity.nominal_capacity_mAh); */
    }

    /* Read usage block */
    status = BatteryPassport_ReadBlock(
        BATTERY_PASSPORT_BLOCK_USAGE,
        (uint8_t *)&usage,
        sizeof(usage),
        &len
    );

    if (status == STATUS_OK)
    {
        /* Print usage metrics */
        /* printf("\nBattery Passport Usage:\n"); */
        /* printf("  SOC: %u.%u%%\n", usage.soc_permille / 10, usage.soc_permille % 10); */
        /* printf("  SOH: %u.%u%%\n", usage.soh_permille / 10, usage.soh_permille % 10); */
        /* printf("  Cycles: %lu\n", usage.cycle_count); */
        /* printf("  Throughput: %llu mAh\n", usage.lifetime_throughput_mAh); */
    }

    /* Read events block */
    status = BatteryPassport_ReadBlock(
        BATTERY_PASSPORT_BLOCK_EVENTS,
        (uint8_t *)&events,
        sizeof(events),
        &len
    );

    if (status == STATUS_OK)
    {
        /* Print event log */
        /* printf("\nBattery Passport Events:\n"); */
        /* printf("  Last Fault: 0x%08lX\n", events.last_fault_code); */
        /* printf("  Total Faults: %lu\n", events.fault_counter_total); */
        /* printf("  Firmware: %u.%u.%u\n", events.firmware_version_major, */
        /*         events.firmware_version_minor, events.firmware_version_patch); */
    }

    /* Read statistics */
    status = BatteryPassport_GetStatistics(&stats);
    if (status == STATUS_OK)
    {
        /* printf("\nBattery Passport Statistics:\n"); */
        /* printf("  FRAM Writes: %lu\n", stats.fram_write_count); */
        /* printf("  CRC Errors: %lu\n", stats.crc_error_count); */
        /* printf("  Data Valid: %s\n", stats.data_valid ? "Yes" : "No"); */
        /* printf("  Provisioned: %s\n", stats.provisioned ? "Yes" : "No"); */
    }
}

/*============================================================================*/
/* END OF INTEGRATION EXAMPLE                                                */
/*============================================================================*/

/**
 * @note  INTEGRATION CHECKLIST:
 *
 *  [ ] 1. Add battery_passport.h to your includes
 *  [ ] 2. Call BatteryPassport_Init() during application initialization
 *  [ ] 3. Register periodic tasks with scheduler (100ms update, 1s commit)
 *  [ ] 4. Implement measurement update task with real sensor data
 *  [ ] 5. Implement FRAM commit task
 *  [ ] 6. Add provisioning function (called once at manufacturing)
 *  [ ] 7. Integrate fault logging (BatteryPassport_LogFault)
 *  [ ] 8. Add force commit to shutdown handler
 *  [ ] 9. (Optional) Integrate UDS ReadDataByIdentifier/WriteDataByIdentifier
 *  [ ] 10. (Optional) Add debug/diagnostics print functions
 *  [ ] 11. Build and verify no warnings
 *  [ ] 12. Test persistence across power cycles
 *  [ ] 13. Verify FRAM write rate is acceptable (check stats.fram_write_count)
 */
