# Battery Passport Module

## Overview

The Battery Passport module provides a comprehensive data collection and storage system for tracking battery identity, usage, health, and compliance information throughout the battery lifecycle. This module is designed for **ISO 26262 ASIL-B** compliance and follows **MISRA C:2012** coding standards.

## Features

- **Static Identity Tracking**: Manufacturer, model, chemistry, capacity (provisioned once)
- **Runtime Usage Metrics**: SOC, SOH, RUL, cycle count, throughput
- **Health Monitoring**: Min/max/average temperature, voltage, current
- **Event Logging**: Fault codes, service dates, firmware version
- **Persistent Storage**: FRAM-backed with CRC32 protection
- **Deterministic Operation**: Fixed-size data structures, no dynamic allocation
- **Diagnostics Ready**: Block-based API for UDS/ISO-TP integration

---

## Data Model

### Group A: Static Identification (Provisioned Once)

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `passport_version` | uint8[2] | - | Major.Minor format version |
| `battery_unique_id` | uint8[16] | - | 128-bit UUID |
| `manufacturer_id` | char[16] | - | Manufacturer name/code |
| `model_id` | char[16] | - | Model/part number |
| `chemistry_type` | enum | - | LFP, NMC, NCA, etc. |
| `nominal_voltage_mV` | uint32 | mV | Nominal pack voltage |
| `nominal_capacity_mAh` | uint32 | mAh | Rated capacity |
| `manufacture_date` | struct | - | Year-Month-Day |
| `cells_series` | uint8 | - | Cells in series |
| `cells_parallel` | uint8 | - | Cells in parallel |

### Group B: Usage & Health (Runtime Updated)

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `soc_permille` | uint16 | 0.1% | State of Charge (0-1000 = 0-100%) |
| `soh_permille` | uint16 | 0.1% | State of Health (0-1000 = 0-100%) |
| `rul_cycles_remaining` | uint32 | cycles | Remaining Useful Life |
| `cycle_count` | uint32 | cycles | Total charge/discharge cycles |
| `lifetime_throughput_mAh` | uint64 | mAh | Cumulative Ah throughput |
| `min_temp_dC` | int16 | 0.1°C | Minimum recorded temperature |
| `max_temp_dC` | int16 | 0.1°C | Maximum recorded temperature |
| `avg_temp_dC` | int16 | 0.1°C | Rolling average temperature |
| `min_pack_voltage_mV` | uint32 | mV | Minimum pack voltage |
| `max_pack_voltage_mV` | uint32 | mV | Maximum pack voltage |
| `min_current_mA` | int32 | mA | Minimum (most negative) current |
| `max_current_mA` | int32 | mA | Maximum current |
| `total_runtime_hours` | uint32 | hours | Total operating time |

### Group C: Events & Compliance

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `last_fault_code` | uint32 | - | Most recent DTC code |
| `fault_counter_total` | uint32 | - | Total fault events |
| `last_service_date` | struct | - | Last maintenance date |
| `firmware_version` | uint8[3] | - | Major.Minor.Patch |
| `last_update_timestamp_ms` | uint32 | ms | Timestamp of last update |
| `timestamp_valid` | bool | - | Timestamp validity flag |

---

## FRAM Storage

### Memory Layout

- **Address**: `0x1000` (user data region start)
- **Size**: ~280 bytes (header + data + CRC)
- **Endurance**: Effectively unlimited (FRAM 10^14 cycles)

### Record Structure

```c
struct FRAMRecord {
    // Header (16 bytes)
    uint32_t magic;           // 0xBCUPA55
    uint8_t  version;         // Record format version
    uint8_t  reserved[3];
    uint32_t data_length;
    uint32_t write_counter;   // Monotonic counter

    // Payload (~256 bytes)
    BatteryPassport_Data_t data;

    // Footer (4 bytes)
    uint32_t crc32;           // CRC over header + data
};
```

### Write Policy

**Rate Limiting:**
- Default commit interval: **10 seconds**
- Early commit triggers:
  - SOC change ≥ **5%** (50 permille)
  - Temperature change ≥ **10°C** (100 deci-Celsius)
  - Fault event logged
  - Service date updated

**Why Rate Limit?**
Although FRAM has near-unlimited endurance, rate limiting:
- Reduces I2C bus traffic
- Improves determinism (predictable timing)
- Batches related updates together
- Prevents "write storms" during rapid state changes

**Force Commit:**
Use `BatteryPassport_ForceCommit()` before:
- System shutdown/sleep
- Critical fault recovery
- Manual service operations

---

## Block ID / Data Identifier Mapping

For UDS/diagnostics access via ISO 14229-1:

| Block ID | DID (hex) | Description | Size (bytes) | Access |
|----------|-----------|-------------|--------------|--------|
| `BLOCK_IDENTITY` | 0xF190 | Static identification | 64 | Read/Write |
| `BLOCK_USAGE` | 0xF191 | Usage & health metrics | 80 | Read-only |
| `BLOCK_EVENTS` | 0xF192 | Events & compliance | 32 | Read/Write |
| `BLOCK_FULL` | 0xF193 | Complete passport | 176 | Read-only |
| `BLOCK_VERSION` | 0xF194 | Passport version | 2 | Read-only |
| `BLOCK_STATS` | 0xF195 | Statistics/metadata | 24 | Read-only |

### UDS Service Integration (Future)

```c
// UDS 0x22: ReadDataByIdentifier
if (service_id == 0x22) {
    uint16_t did = request[1] << 8 | request[2];
    BatteryPassport_UDS_ReadDataByIdentifier(did, response, sizeof(response), &len);
}

// UDS 0x2E: WriteDataByIdentifier (security access required!)
if (service_id == 0x2E) {
    uint16_t did = request[1] << 8 | request[2];
    BatteryPassport_UDS_WriteDataByIdentifier(did, &request[3], request_len - 3);
}
```

---

## API Usage

### Initialization

```c
#include "battery_passport.h"

void main_init(void) {
    Status_t status;

    // Initialize FRAM driver first
    status = FRAM_Init();
    assert(status == STATUS_OK);

    // Initialize Battery Passport
    status = BatteryPassport_Init();
    assert(status == STATUS_OK);
}
```

### Periodic Updates

Register periodic tasks with the scheduler:

```c
#include "bcu_scheduler.h"

// 100ms task: Update measurements
void task_passport_update_100ms(uint32_t now_ms) {
    Current_mA_t pack_current_mA;
    Voltage_mV_t pack_voltage_mV;
    int32_t temp_mC;
    uint16_t soc, soh;

    // Gather measurements from other modules
    LEM_ReadCurrent(0, &pack_current_mA);  // Main pack current sensor
    PM_GetPackVoltage(&pack_voltage_mV);   // Power monitoring
    TempSensor_ReadTemperature(&temp_mC);  // TMP1075 sensor

    // Get SOC/SOH from battery management algorithm
    soc = BatteryManagement_GetSOC();      // Your SOC calculation
    soh = BatteryManagement_GetSOH();      // Your SOH estimation

    // Update passport
    BatteryPassport_UpdateMeasurements(
        pack_voltage_mV,
        pack_current_mA,
        temp_mC / 100,  // Convert milli-Celsius to deci-Celsius
        soc,
        soh,
        5000  // RUL estimate (placeholder)
    );
}

// 1000ms task: Commit to FRAM
void task_passport_commit_1s(uint32_t now_ms) {
    BatteryPassport_CommitIfNeeded();  // Rate-limited commit
}

void scheduler_register_passport_tasks(void) {
    Scheduler_RegisterJob("Passport_Update", task_passport_update_100ms,
                          100, SCHED_PRIORITY_SLOW);
    Scheduler_RegisterJob("Passport_Commit", task_passport_commit_1s,
                          1000, SCHED_PRIORITY_VERY_SLOW);
}
```

### Provisioning (Manufacturing/Service)

```c
void provision_battery_identity(void) {
    BatteryPassport_Identity_t identity;
    BatteryPassport_Date_t manufacture_date;
    Status_t status;

    // Clear structure
    memset(&identity, 0, sizeof(identity));

    // Set passport version
    identity.passport_version_major = 1;
    identity.passport_version_minor = 0;

    // Set unique ID (example: generate or read from EEPROM)
    const uint8_t uuid[16] = {
        0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88
    };
    memcpy(identity.battery_unique_id, uuid, 16);

    // Set manufacturer and model
    strncpy(identity.manufacturer_id, "ACME Battery Co", 15);
    strncpy(identity.model_id, "LFP-48V-50Ah", 15);

    // Set chemistry and ratings
    identity.chemistry_type = BATTERY_CHEM_LFP;
    identity.nominal_voltage_mV = 48000;      // 48V
    identity.nominal_capacity_mAh = 50000;    // 50Ah

    // Set manufacture date (2026-01-15)
    manufacture_date.year = 26;   // 2026 - 2000
    manufacture_date.month = 1;
    manufacture_date.day = 15;
    memcpy(&identity.manufacture_date, &manufacture_date, sizeof(BatteryPassport_Date_t));

    // Set cell configuration
    identity.cells_series = 15;    // 15S (15 * 3.2V = 48V LFP)
    identity.cells_parallel = 1;   // 1P

    // Write to passport (will force commit to FRAM)
    status = BatteryPassport_ProvisionIdentity(&identity);

    if (status == STATUS_OK) {
        printf("Battery provisioned successfully\n");
    }
}
```

### Reading Data Blocks

```c
void read_passport_usage(void) {
    BatteryPassport_Usage_t usage;
    uint16_t len;
    Status_t status;

    status = BatteryPassport_ReadBlock(
        BATTERY_PASSPORT_BLOCK_USAGE,
        (uint8_t *)&usage,
        sizeof(usage),
        &len
    );

    if (status == STATUS_OK) {
        printf("SOC: %u.%u%%\n", usage.soc_permille / 10, usage.soc_permille % 10);
        printf("SOH: %u.%u%%\n", usage.soh_permille / 10, usage.soh_permille % 10);
        printf("Cycles: %lu\n", usage.cycle_count);
        printf("Throughput: %llu mAh\n", usage.lifetime_throughput_mAh);
        printf("Temp: min=%.1f°C, max=%.1f°C, avg=%.1f°C\n",
               usage.min_temp_dC / 10.0f,
               usage.max_temp_dC / 10.0f,
               usage.avg_temp_dC / 10.0f);
    }
}
```

### Fault Logging

```c
void handle_fault_event(uint32_t fault_code) {
    // Log to passport
    BatteryPassport_LogFault(fault_code);

    // Fault events trigger immediate commit
    BatteryPassport_ForceCommit();
}
```

### Pre-Shutdown Commit

```c
void system_shutdown(void) {
    // Ensure all passport data is saved before power-off
    BatteryPassport_ForceCommit();

    // Other shutdown tasks...
}
```

---

## Unit Conventions

| Parameter | Unit | Conversion | Example |
|-----------|------|------------|---------|
| Voltage | millivolts (mV) | V × 1000 | 48.5V → 48500 mV |
| Current | milliamps (mA) | A × 1000 | 12.5A → 12500 mA |
| Capacity | milliamp-hours (mAh) | Ah × 1000 | 50Ah → 50000 mAh |
| Temperature | deci-Celsius (dC) | °C × 10 | 25.5°C → 255 dC |
| SOC/SOH | permille (‰) | % × 10 | 85.3% → 853 ‰ |
| Time | milliseconds (ms) | - | System timestamp |

**Why these units?**
- Avoids floating-point math (MISRA, determinism)
- Sufficient precision for battery applications
- Fits in fixed-width integer types

---

## Self-Test

```c
void system_self_test(void) {
    Status_t status;

    status = BatteryPassport_SelfTest();

    if (status == STATUS_OK) {
        printf("Passport self-test PASSED\n");
    } else {
        printf("Passport self-test FAILED: 0x%02X\n", status);
        // Handle error (e.g., corrupted FRAM)
    }
}
```

**Self-test validates:**
- FRAM read/write operations
- CRC integrity
- Magic number presence
- Record version compatibility

---

## Statistics

```c
void print_passport_stats(void) {
    BatteryPassport_Stats_t stats;
    BatteryPassport_GetStatistics(&stats);

    printf("Passport Statistics:\n");
    printf("  FRAM writes: %lu\n", stats.fram_write_count);
    printf("  CRC errors: %lu\n", stats.crc_error_count);
    printf("  Last commit: %lu ms ago\n", Timestamp_GetMs() - stats.last_commit_timestamp_ms);
    printf("  Updates: %lu\n", stats.update_count);
    printf("  Data valid: %s\n", stats.data_valid ? "YES" : "NO");
    printf("  Provisioned: %s\n", stats.provisioned ? "YES" : "NO");
}
```

---

## Safety Considerations

### MISRA C:2012 Compliance

- ✅ No dynamic memory allocation (`malloc`/`free`)
- ✅ No recursion
- ✅ Fixed-width integer types (`uint8_t`, `int32_t`)
- ✅ All public functions return `Status_t`
- ✅ Pointer validation before dereferencing
- ✅ Array bounds checking
- ✅ Explicit type casts

### ISO 26262 ASIL-B

- ✅ Deterministic timing (no long loops)
- ✅ CRC-protected persistent storage
- ✅ Atomic data updates (copy-out pattern)
- ✅ Graceful degradation (defaults on CRC failure)
- ✅ Self-test capability

### Thread Safety

The module is **NOT thread-safe** by design (single-threaded superloop). If future RTOS migration is needed:
- Add mutex/semaphore protection around `g_passport_data`
- Use atomic operations for statistics counters

---

## Performance

### CPU Usage

Typical execution times (STM32F413 @ 100 MHz):

| Function | WCET (approx) |
|----------|---------------|
| `UpdateMeasurements()` | ~50 µs |
| `CommitIfNeeded()` (no-op) | ~10 µs |
| `CommitIfNeeded()` (write) | ~2 ms (I2C transfer) |
| `ForceCommit()` | ~2 ms |
| `ReadBlock()` | ~20 µs |

### Memory Usage

- **RAM**: ~600 bytes (data + stats + filters)
- **FRAM**: 280 bytes
- **Code**: ~4 KB (compiled with `-Os`)

---

## Troubleshooting

### CRC Errors on Boot

**Symptom**: Passport initializes to defaults every boot

**Causes**:
1. First boot (FRAM empty) - **Normal**
2. FRAM corruption - Check I2C bus, power stability
3. Software version change - Record format incompatible

**Solution**:
```c
// Check statistics
BatteryPassport_Stats_t stats;
BatteryPassport_GetStatistics(&stats);

if (stats.crc_error_count > 0) {
    // Re-provision if needed
    provision_battery_identity();
}
```

### Data Not Persisting

**Symptom**: Changes lost after reboot

**Causes**:
1. `CommitIfNeeded()` not called periodically
2. Commit interval too long and no forced commit before shutdown
3. FRAM write failure (I2C fault)

**Solution**:
- Ensure 1s periodic task calls `CommitIfNeeded()`
- Call `ForceCommit()` before shutdown/sleep
- Check FRAM statistics for write failures

### Incorrect Throughput

**Symptom**: `lifetime_throughput_mAh` not accumulating

**Causes**:
1. `UpdateMeasurements()` not called regularly
2. Incorrect current polarity/scaling
3. Timestamp rollover (should be handled internally)

**Solution**:
- Call `UpdateMeasurements()` at **consistent** intervals (100ms recommended)
- Verify current sensor calibration and sign convention

---

## Future Enhancements

### Planned Features

- [ ] Cycle counting algorithm (charge/discharge detection)
- [ ] Runtime hours tracking (integrate uptime)
- [ ] Periodic CAN broadcast of key metrics
- [ ] Compressed event log (circular buffer in FRAM)
- [ ] Export to JSON/XML via UART/USB

### UDS Integration Roadmap

1. Implement ISO-TP layer (multi-frame transport)
2. Add UDS security access (seed/key for write operations)
3. Register DIDs in UDS service 0x22/0x2E handlers
4. Add session control (programming vs. diagnostic session)

---

## References

- **ISO 14229-1**: Unified Diagnostic Services (UDS)
- **ISO 26262**: Functional Safety for Road Vehicles
- **MISRA C:2012**: Guidelines for C in Critical Systems
- **CY15B256J Datasheet**: Cypress FRAM specifications
- **Battery Passport Initiative**: EU Battery Regulation 2023/1542

---

## License

Copyright © 2026 Battery Control Unit Development Team
All rights reserved.

---

**Author**: BCU Development Team
**Version**: 1.0.0
**Last Updated**: 2026-01-20
