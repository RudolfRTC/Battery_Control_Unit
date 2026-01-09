# GPIO Pin Mapping - Battery Control Unit (BCU)

**MCU:** STM32F413ZHT3 (144-pin LQFP)
**Total GPIO:** 190+ pins configured

---

## Power Management

| Pin     | Function          | Direction | Description                    |
|---------|-------------------|-----------|--------------------------------|
| PA0     | POWER_5V_EN       | Output    | Enable 5V rail (ADPL16000B)   |
| PA1     | POWER_3V3_EN      | Output    | Enable 3.3V rail (TPSM365R3)  |
| PA2     | POWER_SLEEP       | Output    | LM74900 sleep control         |
| PA3     | POWER_IMON        | Analog    | LM74900 current monitor       |
| PA4     | POWER_FLT         | Input     | LM74900 fault signal          |
| PA5     | POWER_5V_PG       | Input     | 5V power good                 |
| PA6     | POWER_3V3A_PG     | Input     | 3.3V analog power good        |

---

## CAN Bus (Dual)

### CAN1 (TCAN3404 #1)
| Pin     | Function    | Direction | Description              |
|---------|-------------|-----------|--------------------------|
| PA11    | CAN1_RX     | AF9       | CAN1 receive             |
| PA12    | CAN1_TX     | AF9       | CAN1 transmit            |
| PB0     | CAN1_STB    | Output    | CAN1 standby control     |

### CAN2 (TCAN3404 #2)
| Pin     | Function    | Direction | Description              |
|---------|-------------|-----------|--------------------------|
| PB12    | CAN2_RX     | AF9       | CAN2 receive             |
| PB13    | CAN2_TX     | AF9       | CAN2 transmit            |
| PB1     | CAN2_STB    | Output    | CAN2 standby control     |

---

## SPI4 (Isolated - LTC6820)

| Pin     | Function    | Direction | Description              |
|---------|-------------|-----------|--------------------------|
| PE2     | SPI4_SCK    | AF5       | SPI4 clock               |
| PE5     | SPI4_MISO   | AF5       | SPI4 master in slave out |
| PE6     | SPI4_MOSI   | AF5       | SPI4 master out slave in |
| PE4     | SPI4_CS     | Output    | SPI4 chip select         |

---

## I2C2 (FRAM + Temperature Sensor)

| Pin     | Function    | Direction | Description              |
|---------|-------------|-----------|--------------------------|
| PB10    | I2C2_SCL    | AF4       | I2C2 clock               |
| PB11    | I2C2_SDA    | AF4       | I2C2 data                |

**Connected devices:**
- CY15B256J FRAM (0x50)
- TMP1075DGKR Temperature Sensor (0x48)

---

## UART1 (Debug)

| Pin     | Function    | Direction | Description              |
|---------|-------------|-----------|--------------------------|
| PA9     | UART1_TX    | AF7       | UART1 transmit (115200)  |
| PA10    | UART1_RX    | AF7       | UART1 receive            |

---

## BTT6200-4ESA Output Control (5 ICs = 20 Outputs)

### BTT6200 IC0 (Channels 0-3)
| Pin     | Function      | Direction | Description              |
|---------|---------------|-----------|--------------------------|
| PC0     | BTT_DEN_0     | Output    | IC0 diagnostic enable    |
| PC1     | BTT_DSEL0_0   | Output    | IC0 diag select bit 0    |
| PC2     | BTT_DSEL1_0   | Output    | IC0 diag select bit 1    |
| PC3     | BTT_IN0_0     | Output    | IC0 input 0 (OUT0)       |
| PC4     | BTT_IN1_0     | Output    | IC0 input 1 (OUT1)       |
| PC5     | BTT_IN2_0     | Output    | IC0 input 2 (OUT2)       |
| PC6     | BTT_IN3_0     | Output    | IC0 input 3 (OUT3)       |
| PC7     | BTT_IS_0      | Analog    | IC0 current sense (ADC)  |

### BTT6200 IC1 (Channels 4-7)
| Pin     | Function      | Direction | Description              |
|---------|---------------|-----------|--------------------------|
| PC8     | BTT_DEN_1     | Output    | IC1 diagnostic enable    |
| PC9     | BTT_DSEL0_1   | Output    | IC1 diag select bit 0    |
| PC10    | BTT_DSEL1_1   | Output    | IC1 diag select bit 1    |
| PC11    | BTT_IN0_1     | Output    | IC1 input 0 (OUT4)       |
| PC12    | BTT_IN1_1     | Output    | IC1 input 1 (OUT5)       |
| PC13    | BTT_IN2_1     | Output    | IC1 input 2 (OUT6)       |
| PC14    | BTT_IN3_1     | Output    | IC1 input 3 (OUT7)       |
| PC15    | BTT_IS_1      | Analog    | IC1 current sense (ADC)  |

### BTT6200 IC2 (Channels 8-11)
| Pin     | Function      | Direction | Description              |
|---------|---------------|-----------|--------------------------|
| PD0     | BTT_DEN_2     | Output    | IC2 diagnostic enable    |
| PD1     | BTT_DSEL0_2   | Output    | IC2 diag select bit 0    |
| PD2     | BTT_DSEL1_2   | Output    | IC2 diag select bit 1    |
| PD3     | BTT_IN0_2     | Output    | IC2 input 0 (OUT8)       |
| PD4     | BTT_IN1_2     | Output    | IC2 input 1 (OUT9)       |
| PD5     | BTT_IN2_2     | Output    | IC2 input 2 (OUT10)      |
| PD6     | BTT_IN3_2     | Output    | IC2 input 3 (OUT11)      |
| PD7     | BTT_IS_2      | Analog    | IC2 current sense (ADC)  |

### BTT6200 IC3 (Channels 12-15)
| Pin     | Function      | Direction | Description              |
|---------|---------------|-----------|--------------------------|
| PD8     | BTT_DEN_3     | Output    | IC3 diagnostic enable    |
| PD9     | BTT_DSEL0_3   | Output    | IC3 diag select bit 0    |
| PD10    | BTT_DSEL1_3   | Output    | IC3 diag select bit 1    |
| PD11    | BTT_IN0_3     | Output    | IC3 input 0 (OUT12)      |
| PD12    | BTT_IN1_3     | Output    | IC3 input 1 (OUT13)      |
| PD13    | BTT_IN2_3     | Output    | IC3 input 2 (OUT14)      |
| PD14    | BTT_IN3_3     | Output    | IC3 input 3 (OUT15)      |
| PD15    | BTT_IS_3      | Analog    | IC3 current sense (ADC)  |

### BTT6200 IC4 (Channels 16-19)
| Pin     | Function      | Direction | Description              |
|---------|---------------|-----------|--------------------------|
| PE0     | BTT_DEN_4     | Output    | IC4 diagnostic enable    |
| PE1     | BTT_DSEL0_4   | Output    | IC4 diag select bit 0    |
| PE3     | BTT_DSEL1_4   | Output    | IC4 diag select bit 1    |
| PE7     | BTT_IN0_4     | Output    | IC4 input 0 (OUT16)      |
| PE8     | BTT_IN1_4     | Output    | IC4 input 1 (OUT17)      |
| PE9     | BTT_IN2_4     | Output    | IC4 input 2 (OUT18)      |
| PE10    | BTT_IN3_4     | Output    | IC4 input 3 (OUT19)      |
| PE11    | BTT_IS_4      | Analog    | IC4 current sense (ADC)  |

---

## LEM Current Sensors (10 Channels)

| Pin     | Function      | Direction | Description              |
|---------|---------------|-----------|--------------------------|
| PF0     | LEM_SUPPLY    | Output    | LEM sensor supply enable |
| PF1     | LEM_OC_0      | Input     | LEM 0 overcurrent flag   |
| PF2     | LEM_OUT_0     | Analog    | LEM 0 current output (ADC)|
| PF3     | LEM_OC_1      | Input     | LEM 1 overcurrent flag   |
| PF4     | LEM_OUT_1     | Analog    | LEM 1 current output     |
| PF5     | LEM_OC_2      | Input     | LEM 2 overcurrent flag   |
| PF6     | LEM_OUT_2     | Analog    | LEM 2 current output     |
| PF7     | LEM_OC_3      | Input     | LEM 3 overcurrent flag   |
| PF8     | LEM_OUT_3     | Analog    | LEM 3 current output     |
| PF9     | LEM_OC_4      | Input     | LEM 4 overcurrent flag   |
| PF10    | LEM_OUT_4     | Analog    | LEM 4 current output     |
| PF11    | LEM_OC_5      | Input     | LEM 5 overcurrent flag   |
| PF12    | LEM_OUT_5     | Analog    | LEM 5 current output     |
| PF13    | LEM_OC_6      | Input     | LEM 6 overcurrent flag   |
| PF14    | LEM_OUT_6     | Analog    | LEM 6 current output     |
| PF15    | LEM_OC_7      | Input     | LEM 7 overcurrent flag   |
| PG0     | LEM_OUT_7     | Analog    | LEM 7 current output     |
| PG1     | LEM_OC_8      | Input     | LEM 8 overcurrent flag   |
| PG2     | LEM_OUT_8     | Analog    | LEM 8 current output     |
| PG3     | LEM_OC_9      | Input     | LEM 9 overcurrent flag   |
| PG4     | LEM_OUT_9     | Analog    | LEM 9 current output     |

---

## Digital Inputs (20 Channels with ACS772)

| Pin     | Function      | Direction | Description                    |
|---------|---------------|-----------|--------------------------------|
| PG5     | DIN_0         | Analog    | Digital input 0 (ACS772 VIOUT) |
| PG6     | DIN_1         | Analog    | Digital input 1                |
| PG7     | DIN_2         | Analog    | Digital input 2                |
| PG8     | DIN_3         | Analog    | Digital input 3                |
| PG9     | DIN_4         | Analog    | Digital input 4                |
| PG10    | DIN_5         | Analog    | Digital input 5                |
| PG11    | DIN_6         | Analog    | Digital input 6                |
| PG12    | DIN_7         | Analog    | Digital input 7                |
| PG13    | DIN_8         | Analog    | Digital input 8                |
| PG14    | DIN_9         | Analog    | Digital input 9                |
| PG15    | DIN_10        | Analog    | Digital input 10               |
| PH0     | DIN_11        | Analog    | Digital input 11               |
| PH1     | DIN_12        | Analog    | Digital input 12               |
| PH2     | DIN_13        | Analog    | Digital input 13               |
| PH3     | DIN_14        | Analog    | Digital input 14               |
| PH4     | DIN_15        | Analog    | Digital input 15               |
| PH5     | DIN_16        | Analog    | Digital input 16               |
| PH6     | DIN_17        | Analog    | Digital input 17               |
| PH7     | DIN_18        | Analog    | Digital input 18               |
| PH8     | DIN_19        | Analog    | Digital input 19               |

---

## Status LEDs

| Pin     | Function      | Direction | Description              |
|---------|---------------|-----------|--------------------------|
| PH9     | LED_STATUS    | Output    | Status LED (green)       |
| PH10    | LED_ERROR     | Output    | Error LED (red)          |
| PH11    | LED_CAN1      | Output    | CAN1 activity LED        |
| PH12    | LED_CAN2      | Output    | CAN2 activity LED        |

---

## Debug Interface

| Pin     | Function      | Direction | Description              |
|---------|---------------|-----------|--------------------------|
| PA13    | SWDIO         | AF0       | SWD data I/O             |
| PA14    | SWCLK         | AF0       | SWD clock                |
| PB3     | SWO           | AF0       | SWD trace output         |

**Connector:** TC2050-IDC (10-pin Tag-Connect)

---

## ADC Channel Assignment

### ADC1 (DMA continuous scan mode)

| ADC Ch  | GPIO Pin | Signal        | Description              |
|---------|----------|---------------|--------------------------|
| 0       | PA3      | POWER_IMON    | LM74900 current monitor  |
| 1       | PF2      | LEM_OUT_0     | LEM sensor 0 output      |
| 2       | PF4      | LEM_OUT_1     | LEM sensor 1 output      |
| 3       | PF6      | LEM_OUT_2     | LEM sensor 2 output      |
| 4       | PF8      | LEM_OUT_3     | LEM sensor 3 output      |
| 5       | PF10     | LEM_OUT_4     | LEM sensor 4 output      |
| 6       | PF12     | LEM_OUT_5     | LEM sensor 5 output      |
| 7       | PF14     | LEM_OUT_6     | LEM sensor 6 output      |
| 8       | PG0      | LEM_OUT_7     | LEM sensor 7 output      |
| 9       | PG2      | LEM_OUT_8     | LEM sensor 8 output      |
| 10      | PG4      | LEM_OUT_9     | LEM sensor 9 output      |
| 11      | PC7      | BTT_IS_0      | BTT6200 IC0 current sense|
| 12      | PC15     | BTT_IS_1      | BTT6200 IC1 current sense|
| 13      | PD7      | BTT_IS_2      | BTT6200 IC2 current sense|
| 14      | PD15     | BTT_IS_3      | BTT6200 IC3 current sense|
| 15      | PE11     | BTT_IS_4      | BTT6200 IC4 current sense|
| 17      | -        | VREFINT       | Internal reference       |
| 18      | -        | TEMPSENSOR    | Internal temperature     |

---

## Power Domains

### 3.3V Digital Domain
- MCU core logic
- All GPIO pins
- CAN transceivers
- I2C bus

### 3.3V Analog Domain (isolated)
- ADC reference (REF1933)
- LEM sensors
- ACS772 sensors
- BTT6200 current sense

### 5V Domain
- Power rail for DC/DC converters
- CAN transceiver supply

### 12V Input Domain
- Battery input (9-16V range)
- LM74900 ideal diode controller
- Protected by reverse polarity

---

## GPIO Configuration Notes

### Pull-up/Pull-down
- **CAN standby pins:** Pull-down (active high)
- **Power good signals:** Pull-down (active high)
- **LEM overcurrent flags:** Pull-down (active high)
- **Fault signals:** Pull-up (active low)
- **All other inputs:** No pull (external pull resistors)

### Output Drive Strength
- **LED pins:** Low speed (2 MHz)
- **Digital control:** Medium speed (25 MHz)
- **CAN, SPI, I2C:** High speed (50 MHz)

### Interrupt Capability (EXTI)
- **POWER_FLT:** EXTI4 (falling edge)
- **LEM_OC_x:** EXTI (any edge, debounced in software)
- **CAN_RX:** Built-in peripheral interrupt

---

## Recommended Initialization Order

1. **System Clock** - Configure PLL to 100 MHz
2. **GPIO Clocks** - Enable all GPIO port clocks
3. **Power Control** - Configure power enable pins (outputs low)
4. **Status LEDs** - Configure LED pins (outputs low)
5. **Debug UART** - Initialize UART1 for early logging
6. **Power Sequencing** - Enable 5V, wait 50ms, enable 3.3V
7. **CAN Transceivers** - Disable standby mode
8. **I2C Bus** - Initialize for FRAM/temperature sensor
9. **ADC** - Configure for continuous DMA scanning
10. **BTT6200** - Initialize all ICs to safe state (outputs off)
11. **LEM Sensors** - Enable supply, calibrate zero-current
12. **Digital Inputs** - Configure ADC channels, debounce filters
13. **Application** - Start main state machine

---

**Revision:** 1.0
**Date:** 2026-01-09
**Status:** Preliminary (subject to hardware changes)
