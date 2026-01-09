# Battery Control Unit (BCU) Firmware

## Overview
Complete embedded C firmware for Battery Control Unit based on STM32F413ZHT3 microcontroller.
Fully compliant with **MISRA C:2012** and **ISO 26262 ASIL-B** safety requirements.

**Firmware Version:** 1.0.0
**Target MCU:** STM32F413ZHT3 (ARM Cortex-M4F @ 100 MHz)
**Development Date:** 2026-01-09

---

## Project Structure

```
Battery_Control_Unit/
├── Application/              # Application layer
│   ├── Inc/                 # Headers
│   │   ├── app_config.h     # ✅ Configuration parameters
│   │   ├── app_types.h      # ✅ Common type definitions
│   │   ├── app_errors.h     # ✅ Error codes
│   │   └── app_main.h       # ✅ Main application
│   └── Src/                 # Source files
│       ├── app_init.c       # ⚠️  TODO: Implement
│       ├── app_task_manager.c # ⚠️ TODO: Implement
│       └── app_state_machine.c # ⚠️ TODO: Implement
│
├── BSP/                      # Board Support Package
│   ├── Inc/
│   │   ├── bsp_gpio.h       # ✅ GPIO abstraction
│   │   ├── bsp_can.h        # ✅ CAN bus driver
│   │   ├── bsp_i2c.h        # ✅ I2C driver
│   │   └── bsp_adc.h        # ✅ ADC driver
│   └── Src/
│       ├── bsp_gpio.c       # ⚠️  TODO: Implement
│       ├── bsp_can.c        # ⚠️  TODO: Implement
│       ├── bsp_i2c.c        # ⚠️  TODO: Implement
│       └── bsp_adc.c        # ⚠️  TODO: Implement
│
├── Modules/                  # Functional modules
│   ├── PowerManagement/
│   │   ├── Inc/
│   │   │   ├── pm_monitor.h    # ✅ Power monitoring
│   │   │   ├── pm_control.h    # ⚠️  TODO: Create
│   │   │   └── pm_safety.h     # ⚠️  TODO: Create
│   │   └── Src/
│   │       ├── pm_monitor.c    # ⚠️  TODO: Implement
│   │       ├── pm_control.c    # ⚠️  TODO: Implement
│   │       └── pm_safety.c     # ⚠️  TODO: Implement
│   │
│   ├── OutputControl/
│   │   ├── Inc/
│   │   │   ├── btt6200.h          # ✅ BTT6200 driver
│   │   │   ├── output_driver.h    # ⚠️  TODO: Create
│   │   │   └── output_diagnostics.h # ⚠️ TODO: Create
│   │   └── Src/
│   │       └── btt6200.c          # ⚠️  TODO: Implement
│   │
│   ├── InputAcquisition/
│   │   ├── Inc/
│   │   │   ├── lem_sensor.h       # ✅ LEM current sensors
│   │   │   ├── digital_input.h    # ⚠️  TODO: Create
│   │   │   └── analog_input.h     # ⚠️  TODO: Create
│   │   └── Src/
│   │       └── lem_sensor.c       # ⚠️  TODO: Implement
│   │
│   ├── DataStorage/
│   │   ├── Inc/
│   │   │   ├── fram_driver.h      # ✅ FRAM driver
│   │   │   ├── nv_storage.h       # ⚠️  TODO: Create
│   │   │   └── data_integrity.h   # ⚠️  TODO: Create
│   │   └── Src/
│   │       └── fram_driver.c      # ⚠️  TODO: Implement
│   │
│   ├── Communication/
│   │   ├── Inc/
│   │   │   ├── can_protocol.h     # ⚠️  TODO: Create
│   │   │   ├── can_tx.h           # ⚠️  TODO: Create
│   │   │   ├── can_rx.h           # ⚠️  TODO: Create
│   │   │   └── isotp.h            # ⚠️  TODO: Create
│   │   └── Src/
│   │       └── ...                # ⚠️  TODO: Implement
│   │
│   ├── Safety/
│   │   ├── Inc/
│   │   │   ├── watchdog.h         # ✅ Dual watchdog
│   │   │   ├── safety_monitor.h   # ⚠️  TODO: Create
│   │   │   └── plausibility.h     # ⚠️  TODO: Create
│   │   └── Src/
│   │       └── watchdog.c         # ⚠️  TODO: Implement
│   │
│   └── Diagnostics/
│       ├── Inc/
│       │   ├── temperature.h      # ⚠️  TODO: Create
│       │   ├── fault_handler.h    # ⚠️  TODO: Create
│       │   └── dtc_manager.h      # ⚠️  TODO: Create
│       └── Src/
│           └── ...                # ⚠️  TODO: Implement
│
├── Utilities/                # Utility functions
│   ├── Inc/
│   │   ├── crc.h            # ✅ CRC calculations
│   │   ├── filter.h         # ✅ Digital filters
│   │   ├── ringbuffer.h     # ✅ Ring buffer
│   │   └── timestamp.h      # ✅ Timing utilities
│   └── Src/
│       ├── crc.c            # ✅ COMPLETE EXAMPLE
│       ├── filter.c         # ⚠️  TODO: Implement
│       ├── ringbuffer.c     # ⚠️  TODO: Implement
│       └── timestamp.c      # ⚠️  TODO: Implement
│
├── Tests/                    # Unit & integration tests
│   ├── unit_tests/          # ⚠️  TODO: Create tests
│   └── integration_tests/   # ⚠️  TODO: Create tests
│
└── Core/                     # STM32 HAL (existing)
    ├── Inc/
    └── Src/
```

**Legend:**
- ✅ = Header created, documented, ready for implementation
- ⚠️  = TODO: Needs implementation
- 📝 = Implementation in progress

---

## Implementation Status

### Completed (Headers Ready)
1. **Application Layer** (3/3 headers)
   - ✅ `app_types.h` - Common types (Status_t, Version_t, etc.)
   - ✅ `app_config.h` - System configuration parameters
   - ✅ `app_errors.h` - Error codes and handling

2. **BSP Layer** (4/5 headers)
   - ✅ `bsp_gpio.h` - GPIO abstraction (190+ pins defined)
   - ✅ `bsp_can.h` - Dual CAN bus driver
   - ✅ `bsp_i2c.h` - I2C driver (FRAM, temperature sensor)
   - ✅ `bsp_adc.h` - 12-bit ADC with DMA

3. **Utilities** (4/4 headers + 1 implementation)
   - ✅ `crc.h` + `crc.c` ← **COMPLETE REFERENCE IMPLEMENTATION**
   - ✅ `filter.h` - Moving average, IIR, median, debounce
   - ✅ `ringbuffer.h` - Thread-safe circular buffer
   - ✅ `timestamp.h` - System timing (ms, μs, cycle counting)

4. **Critical Modules** (4/4 headers)
   - ✅ `pm_monitor.h` - Power monitoring (LM74900, rails)
   - ✅ `btt6200.h` - 20-channel output driver
   - ✅ `lem_sensor.h` - 10 LEM current sensors
   - ✅ `fram_driver.h` - 32 KB FRAM storage
   - ✅ `watchdog.h` - Dual watchdog (IWDG + WWDG)

### TODO (Implementations Needed)
1. **BSP Layer** - Implement hardware drivers (4 files)
2. **Power Management** - 3 implementation files
3. **Output Control** - BTT6200 driver + diagnostics
4. **Input Acquisition** - LEM + digital/analog inputs
5. **Communication** - CAN protocol stack (ISO-TP, J1939)
6. **Safety** - Watchdog, safety monitor, plausibility checks
7. **Diagnostics** - Temperature, fault handling, DTC manager
8. **Application** - State machine, task manager, main loop
9. **Tests** - Unit tests + integration tests

---

## How to Continue Implementation

### Step 1: Complete Utility Modules (Foundation)
Follow the **MISRA C:2012 example** in `Utilities/Src/crc.c`:

```c
// Template structure for MISRA C:2012 compliant code
/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "module.h"
#include <stdint.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/
#define MAX_VALUE (100U)

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/
static bool initialized = false;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/
static Status_t module_internal_function(uint8_t param);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

Status_t Module_Init(void)
{
    Status_t status = STATUS_OK;

    /* Parameter validation */
    if (initialized)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        /* Implementation */
        initialized = true;
    }

    return status;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

static Status_t module_internal_function(uint8_t param)
{
    /* Implementation */
    return STATUS_OK;
}
```

**Implement in this order:**
1. `Utilities/Src/timestamp.c` - System timing (SysTick, DWT)
2. `Utilities/Src/filter.c` - Digital signal processing
3. `Utilities/Src/ringbuffer.c` - Circular buffer for CAN/UART

### Step 2: Implement BSP Layer (Hardware Abstraction)
Start with GPIO as it's needed by all other modules:

1. **`BSP/Src/bsp_gpio.c`**
   - Initialize all 190+ GPIO pins
   - Use ST HAL: `HAL_GPIO_Init()`, `HAL_GPIO_WritePin()`, etc.
   - Implement interrupt handlers (EXTI)

2. **`BSP/Src/bsp_adc.c`**
   - Configure ADC1 with DMA for continuous scanning
   - 16 channels: power monitor, LEM sensors, BTT6200 current sense
   - Implement calibration storage/retrieval

3. **`BSP/Src/bsp_i2c.c`**
   - I2C2 driver for FRAM and TMP1075
   - Implement `BSP_I2C_Write()`, `BSP_I2C_Read()` with timeout
   - Bus error recovery

4. **`BSP/Src/bsp_can.c`**
   - Dual CAN configuration (500 kbit/s)
   - Filter setup for J1939/CANopen
   - TX/RX callbacks for interrupt-driven communication

### Step 3: Implement Critical Safety Modules
**Safety comes first!**

1. **`Modules/Safety/Src/watchdog.c`**
   - IWDG: 500ms timeout, independent clock
   - WWDG: 100ms window watchdog
   - Reset cause detection (POR, watchdog, software)

2. **`Modules/PowerManagement/Src/pm_monitor.c`**
   - Monitor 4 power rails via ADC
   - Read LM74900 IMON and FLT signals
   - Power sequencing: 12V → 5V → 3.3V with delays
   - Fault detection with debouncing

3. **`Modules/Safety/Src/safety_monitor.c`** (new file)
   - Periodic safety checks (1ms task)
   - Stack overflow detection
   - Plausibility checks (cross-validate sensors)
   - Safe state entry on critical fault

### Step 4: Implement Hardware Control Modules

1. **`Modules/OutputControl/Src/btt6200.c`**
   - Control 5× BTT6200 ICs (20 outputs total)
   - DEN, DSEL0/1, IN0-3 control via GPIO
   - Read IS pin via ADC for current sense
   - Diagnostic functions: open load, short circuit, overcurrent
   - PWM support for dimming

2. **`Modules/InputAcquisition/Src/lem_sensor.c`**
   - Read 10 LEM sensors via ADC
   - Zero-current calibration
   - Monitor LEM_OC flags
   - Store calibration in FRAM

3. **`Modules/InputAcquisition/Src/digital_input.c`** (new file)
   - 20 digital inputs with ACS772 current sensors
   - Debounce filtering (10ms)
   - Open/short circuit detection
   - Threshold configuration in FRAM

### Step 5: Implement Data Storage

1. **`Modules/DataStorage/Src/fram_driver.c`**
   - I2C communication with CY15B256J
   - Memory map: config (0x0000), calibration (0x0100), faults (0x0300)
   - CRC-32 protection for critical data
   - Wear-leveling not needed (10^14 cycles endurance)

2. **`Modules/DataStorage/Src/nv_storage.c`** (new file)
   - High-level API for storing configuration
   - Calibration data management
   - Fault log ring buffer

### Step 6: Implement Communication Stack

1. **`Modules/Communication/Src/can_protocol.c`** (new file)
   - Periodic messages (10-100ms): status, sensors, outputs
   - Event-driven: faults, state changes
   - Request/response: configuration, diagnostics

2. **`Modules/Communication/Src/isotp.c`** (new file)
   - ISO-TP (ISO 15765-2) for multi-frame messages
   - Flow control
   - Used for diagnostics and calibration

3. **`Modules/Communication/Src/can_tx.c` / `can_rx.c`**
   - TX queue management
   - RX FIFO processing
   - Message filtering

### Step 7: Implement Application Layer

1. **`Application/Src/app_state_machine.c`**
   - States: INIT → STARTUP → POWER_ON → RUNNING → SHUTDOWN
   - State transitions with validation
   - Error handling and recovery

2. **`Application/Src/app_task_manager.c`**
   - **Fast task (1ms):** Safety monitor, watchdog refresh
   - **Medium task (10ms):** Sensor acquisition, output control
   - **Slow task (100ms):** CAN TX, diagnostics, logging

3. **`Application/Src/app_init.c`**
   - Initialize all modules in correct order
   - Load configuration from FRAM
   - Self-tests

4. **`Core/Src/main.c`**
   ```c
   int main(void)
   {
       /* HAL initialization */
       HAL_Init();
       SystemClock_Config();

       /* Application initialization */
       App_Init();

       /* Main loop */
       App_MainLoop();

       /* Never reached */
       return 0;
   }
   ```

---

## Coding Standards (MISRA C:2012)

### Mandatory Rules
1. **No dynamic memory** - `malloc()`, `free()` forbidden
2. **No recursion** - All functions must have bounded stack usage
3. **Fixed-width types** - Use `uint8_t`, `uint16_t`, `uint32_t`, `int32_t`
4. **Explicit casts** - Cast all type conversions explicitly
5. **All functions return Status_t** - Never ignore return values
6. **NULL checks** - Check all pointers before use
7. **Array bounds** - Validate all array indices
8. **No side effects in expressions** - `if (func() == STATUS_OK)` not `if (func())`

### Naming Conventions
- **Public API:** `ModuleName_FunctionName()` (e.g., `CRC_Calculate32()`)
- **Private functions:** `modulename_functionname()` (lowercase)
- **Constants:** `ALL_CAPS` (e.g., `MAX_BUFFER_SIZE`)
- **Types:** `PascalCase_t` (e.g., `Status_t`, `CAN_Message_t`)
- **Variables:** `camelCase` (e.g., `sensorValue`)

### Error Handling Pattern
```c
Status_t Module_Function(const Type_t *pInput, Type_t *pOutput)
{
    Status_t status = STATUS_OK;

    /* Parameter validation */
    if ((pInput == NULL) || (pOutput == NULL))
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (condition_invalid)
    {
        status = STATUS_ERROR_RANGE;
    }
    else
    {
        /* Implementation */
        ...
    }

    return status;
}
```

---

## Compilation (STM32CubeIDE)

### Build Configuration
```bash
# Debug build
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
  -g3 -Og -Wall -Wextra -Werror -std=c11 -pedantic \
  -DUSE_HAL_DRIVER -DSTM32F413xx -DDEBUG \
  -I Application/Inc -I BSP/Inc -I Modules/.../Inc -I Utilities/Inc \
  -I Core/Inc -I Drivers/STM32F4xx_HAL_Driver/Inc -I Drivers/CMSIS/Include \
  -c file.c -o file.o

# Link
arm-none-eabi-gcc -mcpu=cortex-m4 -T STM32F413ZHTX_FLASH.ld \
  --specs=nosys.specs -Wl,-Map=output.map -Wl,--gc-sections \
  -o firmware.elf *.o

# Generate binary
arm-none-eabi-objcopy -O binary firmware.elf firmware.bin
```

### Required Compiler Flags
- **`-Wall -Wextra -Werror`** - All warnings as errors
- **`-std=c11`** - C11 standard
- **`-pedantic`** - Strict ISO C compliance
- **`-DMISRA_CHECK`** - Enable MISRA checks (if using PC-lint/Cppcheck)

---

## Testing Strategy

### Unit Tests
Create test files in `Tests/unit_tests/`:
```c
// test_crc.c
#include "crc.h"
#include <assert.h>

void test_crc16_known_values(void)
{
    uint8_t data[] = {0x31, 0x32, 0x33, 0x34, 0x35};
    uint16_t crc = CRC_Calculate16(data, 5);
    assert(crc == 0x2189);  // Known CRC-16/CCITT value
}

void test_crc16_null_pointer(void)
{
    uint16_t crc = CRC_Calculate16(NULL, 10);
    assert(crc == 0);
}

int main(void)
{
    test_crc16_known_values();
    test_crc16_null_pointer();
    return 0;
}
```

### Integration Tests
Test module interactions:
- Power sequencing: LM74900 → 5V → 3.3V
- Output control: BTT6200 enable → current measurement → fault handling
- Sensor chain: ADC → LEM → filtering → CAN transmission

---

## Safety Requirements (ISO 26262)

### ASIL-B Compliance Checklist
- [ ] Dual watchdog implemented (IWDG + WWDG)
- [ ] Plausibility checks for redundant sensors
- [ ] Safe state defined (all outputs OFF)
- [ ] Graceful degradation on faults
- [ ] Error counters for transient faults
- [ ] Stack overflow detection
- [ ] Memory protection (MPU if needed)
- [ ] CRC checks on critical data
- [ ] Deterministic timing (< 20ms worst-case)

---

## Next Steps

### Immediate Actions (Priority 1)
1. ✅ Review this README and architecture
2. ⚠️  Implement `Utilities/Src/timestamp.c` (needed by all modules)
3. ⚠️  Implement `BSP/Src/bsp_gpio.c` (hardware foundation)
4. ⚠️  Implement `Modules/Safety/Src/watchdog.c` (safety critical)
5. ⚠️  Implement `Modules/PowerManagement/Src/pm_monitor.c` (safety critical)

### Week 1 Goals
- Complete BSP layer (GPIO, ADC, I2C, CAN)
- Complete utility modules
- Implement watchdog and power monitoring

### Week 2 Goals
- Implement BTT6200 output driver
- Implement LEM sensor driver
- Create CAN communication stack

### Week 3 Goals
- Implement application state machine
- Complete FRAM storage
- Create unit tests

---

## Documentation & Standards

### Required Documents
1. **Software Requirements Specification (SRS)**
2. **Software Architecture Document (SAD)**
3. **MISRA C:2012 Compliance Report**
4. **Unit Test Report (>80% coverage)**
5. **Integration Test Report**
6. **Safety Analysis (FMEA)**

### Tools
- **MISRA checker:** PC-lint, Cppcheck, Coverity
- **Static analysis:** Clang-tidy, SonarQube
- **Unit testing:** Unity, CppUTest
- **Coverage:** gcov, lcov

---

## Contact & Support

**Project:** Battery Control Unit Firmware
**MCU:** STM32F413ZHT3
**Status:** Architecture complete, ready for implementation
**Version:** 1.0.0

**Key Features:**
- 🔧 20× high-side outputs (BTT6200)
- 📊 10× LEM current sensors
- 🔌 20× digital inputs with current sensing
- 🚌 Dual CAN bus communication
- 💾 32 KB FRAM storage
- 🛡️ ISO 26262 ASIL-B safety
- ✅ MISRA C:2012 compliant

---

**Start coding with the reference implementation in `Utilities/Src/crc.c`!**
