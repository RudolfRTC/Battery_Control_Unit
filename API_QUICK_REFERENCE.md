# Battery Control Unit - API Quick Reference

This document provides a quick reference for all implemented module APIs.

---

## 📦 BSP Layer APIs

### GPIO (bsp_gpio.h)
```c
Status_t BSP_GPIO_Init(void);
Status_t BSP_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin, GPIO_State_t *pState);
Status_t BSP_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_State_t state);
Status_t BSP_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);
```

### ADC (bsp_adc.h)
```c
Status_t BSP_ADC_Init(const ADC_Config_t *pConfig);
Status_t BSP_ADC_Start(void);
Status_t BSP_ADC_ReadChannel(uint8_t channel, uint16_t *pValue, uint32_t timeout_ms);
uint32_t BSP_ADC_ToMillivolts(uint16_t adcValue);
Status_t BSP_ADC_Calibrate(void);
```

### I2C (bsp_i2c.h)
```c
Status_t BSP_I2C_Init(uint8_t instance, const I2C_Config_t *pConfig);
Status_t BSP_I2C_Read(uint8_t instance, uint8_t devAddr, uint16_t regAddr,
                      uint8_t *pData, uint16_t length, uint32_t timeout_ms);
Status_t BSP_I2C_Write(uint8_t instance, uint8_t devAddr, uint16_t regAddr,
                       const uint8_t *pData, uint16_t length, uint32_t timeout_ms);
```

### CAN (bsp_can.h)
```c
Status_t BSP_CAN_Init(uint8_t instance, const CAN_Config_t *pConfig);
Status_t BSP_CAN_Transmit(uint8_t instance, const CAN_Message_t *pMessage, uint32_t timeout_ms);
Status_t BSP_CAN_Receive(uint8_t instance, CAN_Message_t *pMessage, uint32_t timeout_ms);
Status_t BSP_CAN_Available(uint8_t instance, uint32_t *pCount);
Status_t BSP_CAN_GetBusState(uint8_t instance, CAN_BusState_t *pState);
Status_t BSP_CAN_RegisterRxCallback(uint8_t instance, CAN_RxCallback_t callback);
```

---

## 🛡️ Safety Module APIs

### Watchdog (watchdog.h)
```c
Status_t Watchdog_Init(void);
Status_t Watchdog_StartIWDG(void);  // Independent watchdog (500ms)
Status_t Watchdog_StartWWDG(void);  // Window watchdog (100ms, 30-100ms window)
Status_t Watchdog_RefreshIWDG(void);
Status_t Watchdog_RefreshWWDG(void);
Status_t Watchdog_RefreshAll(void);
Status_t Watchdog_GetResetCause(ResetCause_t *pCause);
```

### Safety Monitor (safety_monitor.h)
```c
Status_t SafetyMonitor_Init(void);
Status_t SafetyMonitor_Execute(void);  // Call every 1ms
bool SafetyMonitor_IsSafe(void);
Status_t SafetyMonitor_GetState(SafetyMonitor_State_t *pState);
Status_t SafetyMonitor_UpdateTiming(uint32_t loopTime_us);
Status_t SafetyMonitor_ForceSafeState(void);
Status_t SafetyMonitor_RegisterFaultCallback(SafetyMonitor_FaultCallback_t callback);
```

**Safety Checks Performed**:
- Watchdog health (IWDG + WWDG running)
- Timing constraints (loop time < 10ms)
- Stack integrity (watermark check)
- RAM integrity (pattern test)
- Power supply health
- Temperature limits
- Communication health

---

## 📡 Sensor Module APIs

### LEM HOYS Sensors (lem_sensor.h)
```c
Status_t LEM_Init(void);
Status_t LEM_EnableSupply(bool enable);
Status_t LEM_ReadCurrent(uint8_t sensorId, Current_mA_t *pCurrent_mA);
Status_t LEM_CalibrateSensor(uint8_t sensorId);  // Zero-current calibration
Current_mA_t LEM_ADCToCurrent(uint16_t rawADC, const LEM_Calibration_t *pCalibration);
Status_t LEM_GetMeasurement(uint8_t sensorId, LEM_Measurement_t *pMeasurement);
Status_t LEM_GetStatistics(uint8_t sensorId, LEM_Statistics_t *pStats);
Status_t LEM_Update(void);  // Call every 1ms (1 kHz sampling)
Status_t LEM_RegisterOvercurrentCallback(LEM_OvercurrentCallback_t callback);
```

**Sensor IDs**: 0-9 (10 channels total)
**Sample Rate**: 1 kHz
**Calibration**: Zero-current offset, gain factor

### Digital Inputs (digital_input.h)
```c
Status_t DI_Init(void);
Status_t DI_ConfigureInput(uint8_t inputId, const DI_InputConfig_t *pConfig);
Status_t DI_ReadInput(uint8_t inputId, bool *pState);
Status_t DI_GetInputState(uint8_t inputId, DI_InputState_t *pState);
Status_t DI_ReadInputCurrent(uint8_t inputId, Current_mA_t *pCurrent_mA);  // ACS772
Status_t DI_GetEdgeDetection(uint8_t inputId, bool *pRisingEdge, bool *pFallingEdge);
Status_t DI_Update(void);  // Call every 10ms
Status_t DI_RegisterCallback(uint8_t inputId, DI_StateChangeCallback_t callback);
```

**Input IDs**: 0-19 (20 channels total)
**Features**: Debouncing (20ms), edge detection, current sensing (ACS772)

### Temperature Sensor (temp_sensor.h)
```c
Status_t TempSensor_Init(void);
Status_t TempSensor_ReadTemperature(int32_t *pTemp_mC);  // milliCelsius
Status_t TempSensor_GetData(TempSensor_Data_t *pData);
Status_t TempSensor_Configure(const TempSensor_Config_t *pConfig);
bool TempSensor_IsValid(void);  // Check if recent measurement
Status_t TempSensor_RegisterAlarmCallback(TempSensor_AlarmCallback_t callback);
```

**Sensor**: TI TMP1075 (I2C address 0x48)
**Resolution**: 12-bit (0.0625°C)
**Accuracy**: ±0.5°C

---

## 🔌 Output Control APIs

### BTT6200 Driver (btt6200.h)
```c
Status_t BTT6200_Init(void);
Status_t BTT6200_ChannelOn(uint8_t channel);
Status_t BTT6200_ChannelOff(uint8_t channel);
Status_t BTT6200_ReadChannelCurrent(uint8_t channel, Current_mA_t *pCurrent_mA);
Status_t BTT6200_DiagnoseChannel(uint8_t channel, BTT6200_ChannelDiag_t *pDiag);
Status_t BTT6200_SetSafeState(void);  // Turn off all channels
Status_t BTT6200_Update(void);  // Call every 10ms for diagnostics
Current_mA_t BTT6200_ADCToCurrent(uint16_t adcValue);
Status_t BTT6200_RegisterFaultCallback(BTT6200_FaultCallback_t callback);
```

**Channels**: 0-19 (20 outputs via 5 ICs)
**Max Current**: 3.5A per channel
**Diagnostics**: Overcurrent, open load, short circuit detection

---

## ⚡ Power Management APIs

### Power Monitor (pm_monitor.h)
```c
Status_t PM_Monitor_Init(void);
Status_t PM_Monitor_Update(void);  // Call every 10ms
Status_t PM_Monitor_GetLM74900Status(PM_LM74900_Status_t *pStatus);
Status_t PM_Monitor_GetRailStatus(uint8_t railId, PM_RailStatus_t *pStatus);
Status_t PM_Monitor_GetRailVoltage(uint8_t railId, Voltage_mV_t *pVoltage_mV);
Status_t PM_Monitor_IsSystemPowerGood(bool *pPowerGood);
Status_t PM_Monitor_GetInputCurrent(Current_mA_t *pCurrent_mA);
Status_t PM_Monitor_GetUptime(uint32_t *pUptimeSeconds);
Status_t PM_Monitor_RegisterFaultCallback(PM_FaultCallback_t callback);
```

**Power Rails**:
- `PM_RAIL_INPUT_12V` - Main 12V input
- `PM_RAIL_5V` - 5V rail (with PG signal)
- `PM_RAIL_3V3_DIGITAL` - 3.3V digital rail
- `PM_RAIL_3V3_ANALOG` - 3.3V analog rail (with PG signal)

**LM74900 Monitoring**: IMON current, FLT fault detection

---

## 💾 Storage APIs

### FRAM Driver (fram_driver.h)
```c
Status_t FRAM_Init(void);
Status_t FRAM_Read(uint16_t address, uint8_t *pData, uint16_t length);
Status_t FRAM_Write(uint16_t address, const uint8_t *pData, uint16_t length);
Status_t FRAM_Erase(uint16_t address, uint16_t length);  // Fill with 0xFF

/* Configuration */
Status_t FRAM_SaveConfig(const uint8_t *pConfig, uint16_t length);
Status_t FRAM_LoadConfig(uint8_t *pConfig, uint16_t maxLength, uint16_t *pActualLength);

/* Calibration */
Status_t FRAM_SaveCalibration(const uint8_t *pCalibration, uint16_t length);
Status_t FRAM_LoadCalibration(uint8_t *pCalibration, uint16_t length);

/* Fault Logging */
Status_t FRAM_WriteFaultLog(uint16_t logIndex, const FRAM_FaultLogEntry_t *pEntry);
Status_t FRAM_ReadFaultLog(uint16_t logIndex, FRAM_FaultLogEntry_t *pEntry);

/* Diagnostics */
Status_t FRAM_TestIntegrity(void);
Status_t FRAM_GetStatistics(FRAM_Statistics_t *pStats);
Status_t FRAM_SetWriteProtection(bool enable);
```

**Memory Map**:
- 0x0000-0x03FF: Configuration (1 KB, CRC protected)
- 0x0400-0x0BFF: Calibration (2 KB)
- 0x0C00-0x1BFF: Fault log (4 KB, circular)
- 0x1C00-0x3BFF: Data log (8 KB, circular)
- 0x3C00-0x7FFF: User/reserved (17 KB)

---

## 🐛 Error Handling APIs

### Error Handler (app_errors.h)
```c
Status_t ErrorHandler_Init(void);
Status_t ErrorHandler_LogError(ErrorCode_t code, uint32_t param1, uint32_t param2, uint32_t param3);
Status_t ErrorHandler_GetDTC(ErrorCode_t code, DTC_Info_t *pInfo);
Status_t ErrorHandler_ClearDTC(ErrorCode_t code);
Status_t ErrorHandler_ClearAllDTCs(void);
Status_t ErrorHandler_GetActiveDTCCount(uint32_t *pCount);
bool ErrorHandler_IsDTCActive(ErrorCode_t code);
Status_t ErrorHandler_Update(void);  // Call every 100ms for DTC aging
void ErrorHandler_SafeState(void);  // Emergency safe state
void ErrorHandler_HardFault(void);  // Hard fault handler
const char* ErrorHandler_GetErrorName(ErrorCode_t code);
Status_t ErrorHandler_RegisterCallback(ErrorCallback_t callback);
```

**Error Codes** (app_errors.h):
```c
ERROR_NONE                          = 0x00
ERROR_SAFETY_WATCHDOG              = 0x01
ERROR_SAFETY_TIMING_VIOLATION      = 0x02
ERROR_SAFETY_STACK_OVERFLOW        = 0x03
ERROR_SAFETY_MEMORY_CORRUPTION     = 0x04
ERROR_POWER_FAULT                  = 0x05
ERROR_TEMPERATURE_FAULT            = 0x06
ERROR_CAN_BUS_OFF                  = 0x07
ERROR_SENSOR_FAULT                 = 0x08
ERROR_ACTUATOR_FAULT               = 0x09
ERROR_SAFETY_CRITICAL_FAULT        = 0x0A
ERROR_SAFETY_MONITOR_FAULT         = 0x0B
ERROR_HARDWARE_FAULT               = 0x0C
```

**DTC Status**:
- `DTC_STATUS_PENDING` - Error occurred but not confirmed
- `DTC_STATUS_CONFIRMED` - Error occurred ≥3 times
- `DTC_STATUS_CLEARED` - Error cleared or aged out

**DTC Aging**: Pending DTCs cleared after 60 seconds without occurrence

---

## 🛠️ Utility APIs

### Timestamp (timestamp.h)
```c
Status_t Timestamp_Init(void);
uint32_t Timestamp_GetMillis(void);  // Milliseconds since startup
uint64_t Timestamp_GetMicros(void);  // Microseconds (DWT cycle counter)
uint64_t Timestamp_GetNanos(void);   // Nanoseconds (DWT cycle counter)
void Timestamp_SysTick_Handler(void);  // Call from SysTick ISR
```

### CRC (crc.h)
```c
Status_t CRC_Init(void);
uint16_t CRC_Calculate16(const uint8_t *pData, uint32_t length);  // Software
uint32_t CRC_Calculate32(const uint8_t *pData, uint32_t length);  // Hardware accelerated
uint16_t CRC_Calculate16_Incremental(uint16_t previousCRC, const uint8_t *pData, uint32_t length);
uint32_t CRC_Calculate32_Incremental(uint32_t previousCRC, const uint8_t *pData, uint32_t length);
```

### Ring Buffer (ringbuffer.h)
```c
Status_t RingBuffer_Init(RingBuffer_t *pRingBuf, uint8_t *pBuffer, uint32_t size);
Status_t RingBuffer_WriteByte(RingBuffer_t *pRingBuf, uint8_t data);
Status_t RingBuffer_ReadByte(RingBuffer_t *pRingBuf, uint8_t *pData);
Status_t RingBuffer_Write(RingBuffer_t *pRingBuf, const uint8_t *pData, uint32_t length);
Status_t RingBuffer_Read(RingBuffer_t *pRingBuf, uint8_t *pData, uint32_t length);
uint32_t RingBuffer_Available(const RingBuffer_t *pRingBuf);
void RingBuffer_Clear(RingBuffer_t *pRingBuf);
```

### Filters (filter.h)
```c
/* Moving Average */
Status_t Filter_MA_Init(Filter_MovingAverage_t *pFilter, uint8_t windowSize);
int32_t Filter_MA_Update(Filter_MovingAverage_t *pFilter, int32_t sample);

/* IIR Low-Pass */
Status_t Filter_IIR_Init(Filter_IIR_t *pFilter, uint32_t cutoffFreq_Hz, uint32_t sampleRate_Hz);
int32_t Filter_IIR_Update(Filter_IIR_t *pFilter, int32_t sample);

/* Median */
Status_t Filter_Median_Init(Filter_Median_t *pFilter, uint8_t windowSize);
int32_t Filter_Median_Update(Filter_Median_t *pFilter, int32_t sample);

/* Debounce */
Status_t Filter_Debounce_Init(Filter_Debounce_t *pFilter, uint32_t debounceTime_ms);
bool Filter_Debounce_Update(Filter_Debounce_t *pFilter, bool input);
```

---

## 🚀 Application APIs

### Main Application (app_main.h)
```c
Status_t App_Init(void);
void App_MainLoop(void);  // Never returns
void App_FastTasks(void);  // 1ms period (safety critical)
void App_MediumTasks(void);  // 10ms period (control & I/O)
void App_SlowTasks(void);  // 100ms period (communication & diagnostics)
Status_t App_GetState(AppState_t *pState);
Status_t App_GetStatus(AppStatus_t *pStatus);
Status_t App_RequestStateChange(AppState_t newState);
Status_t App_Shutdown(void);
void App_EnterSafeState(void);
Status_t App_GetVersion(Version_t *pVersion);
```

**Application States**:
```c
APP_STATE_INIT      // Initializing
APP_STATE_STARTUP   // Startup sequence
APP_STATE_POWER_ON  // Powering on systems
APP_STATE_RUNNING   // Normal operation
APP_STATE_IDLE      // Low power idle
APP_STATE_SHUTDOWN  // Shutdown sequence
APP_STATE_ERROR     // Error state
APP_STATE_SAFE      // Safe state (all outputs off)
```

---

## 📋 Common Types (app_types.h)

### Status Codes
```c
typedef enum {
    STATUS_OK = 0x00,
    STATUS_ERROR = 0x01,
    STATUS_ERROR_PARAM = 0x02,
    STATUS_ERROR_TIMEOUT = 0x03,
    STATUS_ERROR_BUSY = 0x04,
    STATUS_ERROR_NOT_INIT = 0x05,
    STATUS_ERROR_ALREADY_INIT = 0x06,
    STATUS_ERROR_OVERFLOW = 0x07,
    STATUS_ERROR_UNDERFLOW = 0x08,
    STATUS_ERROR_NO_DATA = 0x09,
    STATUS_ERROR_CRC = 0x0A,
    STATUS_ERROR_NOT_FOUND = 0x0B,
    STATUS_ERROR_INVALID_STATE = 0x0C,
    STATUS_ERROR_WRITE_PROTECTED = 0x0D,
    STATUS_ERROR_INVALID_DATA = 0x0E,
    STATUS_ERROR_DISABLED = 0x0F
} Status_t;
```

### Common Units
```c
typedef int32_t Current_mA_t;    // Current in milliamperes
typedef uint32_t Voltage_mV_t;   // Voltage in millivolts
typedef int32_t Temperature_mC_t; // Temperature in milliCelsius
```

---

## 🔧 Configuration Constants (app_config.h)

### System Configuration
```c
#define MCU_CLOCK_FREQ_HZ           100000000UL  // 100 MHz
#define SYSTICK_FREQ_HZ             1000UL       // 1 kHz
#define TASK_PERIOD_SAFETY_MS       1U           // 1ms
#define TASK_PERIOD_FAST_MS         10U          // 10ms
#define TASK_PERIOD_SLOW_MS         100U         // 100ms
#define SAFETY_MAX_LOOP_TIME_MS     10U          // 10ms
```

### Power Specifications
```c
#define POWER_INPUT_VOLTAGE_NOM_MV  12000U   // 12V
#define POWER_INPUT_VOLTAGE_MIN_MV  9000U    // 9V
#define POWER_INPUT_VOLTAGE_MAX_MV  16000U   // 16V
#define POWER_5V_RAIL_NOM_MV        5000U
#define POWER_3V3_DIGITAL_NOM_MV    3300U
#define POWER_3V3_ANALOG_NOM_MV     3300U
```

### Sensor Configuration
```c
#define LEM_SENSOR_COUNT            10U
#define LEM_MAX_CURRENT_A           50          // 50A per sensor
#define LEM_SENSITIVITY_MV_PER_A    625         // 62.5 mV/A
#define BTT6200_IC_COUNT            5U
#define BTT6200_TOTAL_CHANNELS      20U
#define BTT6200_MAX_CURRENT_MA      3500
```

### Communication Configuration
```c
#define CAN1_BAUDRATE               500000UL    // 500 kbps
#define CAN2_BAUDRATE               500000UL
#define I2C_FRAM_ADDRESS            0x50U
#define I2C_TEMP_SENSOR_ADDRESS     0x48U
```

---

## 💡 Usage Examples

### Example 1: Reading LEM Sensor Current
```c
Current_mA_t current;
Status_t status = LEM_ReadCurrent(0, &current);  // Sensor 0
if (status == STATUS_OK) {
    printf("Current: %ld mA\n", current);
}
```

### Example 2: Controlling BTT6200 Output
```c
// Turn on channel 5
Status_t status = BTT6200_ChannelOn(5);
if (status == STATUS_OK) {
    // Wait a bit
    HAL_Delay(100);

    // Read current
    Current_mA_t current;
    BTT6200_ReadChannelCurrent(5, &current);

    // Turn off
    BTT6200_ChannelOff(5);
}
```

### Example 3: Reading Temperature
```c
int32_t temp_mC;
Status_t status = TempSensor_ReadTemperature(&temp_mC);
if (status == STATUS_OK) {
    float temp_C = (float)temp_mC / 1000.0f;
    printf("Temperature: %.2f °C\n", temp_C);
}
```

### Example 4: Transmitting CAN Message
```c
CAN_Message_t msg = {
    .id = 0x123,
    .ide = CAN_IDE_STANDARD,
    .rtr = false,
    .dlc = 8,
    .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
};
Status_t status = BSP_CAN_Transmit(BSP_CAN_INSTANCE_1, &msg, 100);
```

### Example 5: Saving to FRAM
```c
uint8_t config[256] = { /* configuration data */ };
Status_t status = FRAM_SaveConfig(config, sizeof(config));
if (status == STATUS_OK) {
    // Config saved with CRC
}
```

### Example 6: Registering Error Callback
```c
void my_error_handler(ErrorCode_t code, uint32_t p1, uint32_t p2, uint32_t p3) {
    printf("Error: 0x%02X, params: %lu, %lu, %lu\n", code, p1, p2, p3);
}

ErrorHandler_RegisterCallback(my_error_handler);
```

---

## 🎯 Typical Initialization Sequence

```c
int main(void) {
    // 1. HAL Init
    HAL_Init();
    SystemClock_Config();

    // 2. Initialize application
    Status_t status = App_Init();
    if (status != STATUS_OK) {
        ErrorHandler_SafeState();
    }

    // 3. Start main loop (never returns)
    App_MainLoop();

    return 0;
}
```

**App_Init() does**:
1. Initialize timestamp module
2. Initialize peripherals (GPIO, ADC, I2C, CAN)
3. Initialize error handler & safety monitor
4. Initialize FRAM storage
5. Initialize all sensor/actuator modules
6. Start dual watchdog
7. Enter STARTUP state

**App_MainLoop() executes**:
1. Fast tasks every 1ms (safety monitor, watchdog, LEM sensors)
2. Medium tasks every 10ms (digital inputs, BTT6200, power monitoring)
3. Slow tasks every 100ms (temperature, CAN, DTCs, logging)
4. State machine
5. Status updates
6. Timing violation detection
7. Sleep until next cycle (__WFI)

---

## 📚 Additional Resources

- **Full Documentation**: See `FIRMWARE_README.md`
- **Continuation Guide**: See `CONTINUATION_INSTRUCTIONS.md`
- **GPIO Mapping**: See `GPIO_PIN_MAP_FROM_SCHEMATIC.md`
- **Header Files**: All in `Application/Inc/`, `BSP/Inc/`, `Modules/*/Inc/`, `Utilities/Inc/`

---

*Last Updated: 2026-01-09*
*Firmware Version: 1.0.0*
*Commit: 12816cc*
