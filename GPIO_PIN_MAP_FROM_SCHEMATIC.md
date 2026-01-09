# GPIO Pin Mapping - Battery Control Unit (BCU) - FROM SCHEMATIC

**Architecture:** DUAL-MCU System
**MCU:** 2× STM32F413ZHT3 (144-pin LQFP)
**IC4A:** Primary MCU (LEM sensors, outputs, debug)
**IC4B:** Secondary MCU (power control, digital inputs, CAN, I2C)

---

## IC4A (Primary MCU) - Pin Mapping

### LEM Current Sensor Outputs (Analog ADC)
| Pin     | Signal       | IC4A Pin | Direction | Description              |
|---------|--------------|----------|-----------|--------------------------|
| PA0     | LEM_OUT1     | 34       | Analog    | LEM sensor 1 output      |
| PA1     | LEM_OUT2     | 35       | Analog    | LEM sensor 2 output      |
| PA2     | LEM_OUT3     | 36       | Analog    | LEM sensor 3 output      |
| PA3     | LEM_OUT4     | 37       | Analog    | LEM sensor 4 output      |
| PA4     | LEM_OUT5     | 40       | Analog    | LEM sensor 5 output      |
| PA5     | LEM_OUT6     | 41       | Analog    | LEM sensor 6 output      |
| PA6     | LEM_OUT7     | 42       | Analog    | LEM sensor 7 output      |
| PA7     | LEM_OUT8     | 43       | Analog    | LEM sensor 8 output      |

### Output Control Group 1 & 2
| Pin     | Signal       | IC4A Pin | Direction | Description              |
|---------|--------------|----------|-----------|--------------------------|
| PB0     | OUT2_0       | 46       | Output    | Output group 2, channel 0|
| PB1     | OUT2_1       | 47       | Output    | Output group 2, channel 1|
| PB2     | OUT2_2       | 48       | Output    | Output group 2, channel 2|
| PB3     | OUT2_3       | 89       | Output    | Output group 2, channel 3|
| PB4     | OUT1_0       | 90       | Output    | Output group 1, channel 0|
| PB5     | OUT1_1       | 91       | Output    | Output group 1, channel 1|
| PB6     | OUT1_2       | 92       | Output    | Output group 1, channel 2|
| PB7     | OUT1_3       | 93       | Output    | Output group 1, channel 3|

### Misc Control Signals (Port C)
| Pin     | Signal       | IC4A Pin | Direction | Description              |
|---------|--------------|----------|-----------|--------------------------|
| PC0     | LEM_OC1      | 15       | Input     | LEM 1 overcurrent flag   |
| PC13    | SWCLK        | 3        | Output    | Clock signal?            |
| PC14    | OUT4         | 4        | Output    | Output channel 4         |
| PC14-OSC32_IN | -     | 5        | Crystal   | 32.768 kHz crystal       |
| PC15-OSC32_OUT | -    | 6        | Crystal   | 32.768 kHz crystal       |

### Port D - LEM outputs, Power, Debug UART, Digital Inputs
| Pin     | Signal         | IC4A Pin | Direction | Description              |
|---------|----------------|----------|-----------|--------------------------|
| PD0     | LEM_OUT9       | 114      | Analog    | LEM sensor 9 output      |
| PD1     | LEM_OUT10      | 115      | Analog    | LEM sensor 10 output     |
| PD2     | PWR_FLT        | 116      | Input     | Power fault signal       |
| PD5     | DBG_UART_TX    | 119      | AF7       | Debug UART transmit      |
| PD6     | DBG_UART_RX    | 122      | AF7       | Debug UART receive       |
| PD8     | IN_8           | 55       | Input     | Digital input 8          |
| PD9     | IN_9           | 56       | Input     | Digital input 9          |
| PD10    | IN_10          | 57       | Input     | Digital input 10         |
| PD11    | IN_11          | 58       | Input     | Digital input 11         |
| PD12    | IN_12          | 59       | Input     | Digital input 12         |
| PD13    | IN_13          | 60       | Input     | Digital input 13         |
| PD14    | IN_14          | 61       | Input     | Digital input 14         |

### Port E - CAN, Output Control
| Pin     | Signal       | IC4A Pin | Direction | Description              |
|---------|--------------|----------|-----------|--------------------------|
| PE0     | OUT5_0       | 97       | Output    | Output group 5, channel 0|
| PE1     | CAN2_RXD     | 98       | AF9       | CAN2 receive             |
| PE2     | CAN2_TXD     | 1        | AF9       | CAN2 transmit            |
| PE3     | LEM_OC5      | 2        | Input     | LEM 5 overcurrent flag   |
| PE4     | DSEL1_1      | 3        | Output    | Diag select 1, bit 1     |

---

## IC4B (Secondary MCU) - Pin Mapping

### Digital Inputs (Port G)
| Pin     | Signal       | IC4B Pin | Direction | Description              |
|---------|--------------|----------|-----------|--------------------------|
| PG12    | IN_10        | 2        | Input     | Digital input 10         |
| PG14    | IN_11        | 1        | Input     | Digital input 11         |

### Port E - SPI, Power Control, Output Control
| Pin     | Signal       | IC4B Pin | Direction | Description              |
|---------|--------------|----------|-----------|--------------------------|
| PE1     | SCK          | 4        | AF5       | SPI clock                |
| PE1     | LEM_OC6      | 4        | Input     | LEM 6 overcurrent (alt)  |
| PE5     | MISO         | 5        | AF5       | SPI MISO                 |
| PE6     | MOSI         | 6        | AF5       | SPI MOSI                 |
| PE7     | 5V_EN        | 38       | Output    | 5V rail enable           |
| PE8     | EN_3V3A      | 39       | Output    | 3.3V analog enable       |
| PE9     | PWR_24V_EN   | 40       | Output    | 24V power enable         |
| PE10    | ISOSPI_EN    | 41       | Output    | Isolated SPI enable      |

### Port G - Output Control Signals
| Pin     | Signal       | IC4B Pin | Direction | Description              |
|---------|--------------|----------|-----------|--------------------------|
| PG0     | CAN_RXD      | 56       | AF9       | CAN receive (main CAN)   |
| PG1     | CAN_TXD      | 57       | AF9       | CAN transmit             |
| PG2     | IN_20        | 87       | Input     | Digital input 20         |
| PG3     | IN_19        | 88       | Input     | Digital input 19         |
| PG4     | IN_18        | 89       | Input     | Digital input 18         |
| PG5     | IN_17        | 90       | Input     | Digital input 17         |
| PG6     | DSEL0_0      | 65       | Output    | Diag select 0, bit 0     |
| PG7     | IN_16        | 66       | Input     | Digital input 16         |
| PG8     | DSEL1_0      | 68       | Output    | Diag select 1, bit 0     |
| PG9     | OUT2_0       | 69       | Output    | Output group 2, ch 0     |
| PG10    | OUT3_1       | 70       | Output    | Output group 3, ch 1     |
| PG11    | OUT3_0       | 71       | Output    | Output group 3, ch 0     |
| PG12    | DEN_0        | 72       | Output    | Diag enable 0            |
| PG13    | OUT1_0       | 73       | Output    | Output group 1, ch 0     |

### Port F - I2C, Digital Inputs, Power Signals
| Pin     | Signal       | IC4B Pin | Direction | Description              |
|---------|--------------|----------|-----------|--------------------------|
| PF0     | I2C_SDA      | 10       | AF4       | I2C data (FRAM/temp)     |
| PF1     | I2C_SCL      | 11       | AF4       | I2C clock                |
| PF2     | IN_9         | 12       | Input     | Digital input 9          |
| PF3     | IN_8         | 13       | Input     | Digital input 8          |
| PF4     | IN_7         | 14       | Input     | Digital input 7          |
| PF5     | IN_6         | 15       | Input     | Digital input 6          |
| PF6     | IN_5         | 16       | Input     | Digital input 5          |
| PF7     | IN_4         | 17       | Input     | Digital input 4          |
| PF8     | IN_3         | 18       | Input     | Digital input 3          |
| PF9     | IN_2         | 19       | Input     | Digital input 2          |
| PF10    | IN_1         | 20       | Input     | Digital input 1          |
| PF11    | 5V_PG        | 49       | Input     | 5V power good            |
| PF12    | 3V3A_PG      | 50       | Input     | 3.3V analog power good   |
| PF13    | PWR_SLEEP    | 51       | Output    | Power sleep control      |
| PF14    | LEM_OC8      | 54       | Input     | LEM 8 overcurrent flag   |
| PF15    | LEM_OC10     | 55       | Input     | LEM 10 overcurrent flag  |

### Port D - Additional Signals (IC4B)
| Pin     | Signal       | IC4B Pin | Direction | Description              |
|---------|---------------|----------|-----------|--------------------------|
| PD11    | IN_15        | 58       | Input     | Digital input 15         |
| PD13    | LED          | 60       | Output    | Status LED               |

### Port H - Boot Control (IC4B)
| Pin     | Signal       | IC4B Pin | Direction | Description              |
|---------|--------------|----------|-----------|--------------------------|
| PH0-OSC_IN | -         | 23       | Crystal   | External crystal input   |
| PH1-OSC_OUT | -        | 24       | Crystal   | External crystal output  |

### Power & Boot (IC4B)
| Pin     | Signal       | IC4B Pin | Direction | Description              |
|---------|--------------|----------|-----------|--------------------------|
| BOOT0   | CPU_BOOT     | 138      | Input     | Boot mode selection      |
| NRST    | NRST         | 25       | Input     | System reset (active low)|
| PDR_ON  | -            | 143      | Input     | Power domain control?    |

---

## Special Signals

### Debug Interface (SWD)
**IC4B:**
| Pin     | Signal       | IC4B Pin | Direction | Description              |
|---------|--------------|----------|-----------|--------------------------|
| PA13    | PH0-OSC_IN   | 23       | AF0       | SWD data I/O             |
| PA14    | PH1-OSC_OUT  | 24       | AF0       | SWD clock                |

---

## ADC Channel Assignment (Tentative)

### IC4A - ADC1 (LEM Sensors)
| ADC Ch  | GPIO Pin | Signal      | Description              |
|---------|----------|-------------|--------------------------|
| 0       | PA0      | LEM_OUT1    | LEM sensor 1             |
| 1       | PA1      | LEM_OUT2    | LEM sensor 2             |
| 2       | PA2      | LEM_OUT3    | LEM sensor 3             |
| 3       | PA3      | LEM_OUT4    | LEM sensor 4             |
| 4       | PA4      | LEM_OUT5    | LEM sensor 5             |
| 5       | PA5      | LEM_OUT6    | LEM sensor 6             |
| 6       | PA6      | LEM_OUT7    | LEM sensor 7             |
| 7       | PA7      | LEM_OUT8    | LEM sensor 8             |
| 8       | PD0      | LEM_OUT9    | LEM sensor 9             |
| 9       | PD1      | LEM_OUT10   | LEM sensor 10            |

### IC4B - ADC1 (Digital Inputs - if analog)
| ADC Ch  | GPIO Pin | Signal      | Description              |
|---------|----------|-------------|--------------------------|
| Various | PF2-PF10 | IN_1-IN_9   | Digital inputs 1-9       |
| Various | PG2-PG7  | IN_16-IN_20 | Digital inputs 16-20     |

---

## Inter-MCU Communication

**Note:** The two MCUs likely communicate via:
- CAN bus (CAN1 ↔ CAN2)
- SPI (isolated or direct)
- Shared signals (e.g., power good flags)

**Power Sequencing:**
- IC4B controls power rails (5V_EN, EN_3V3A, PWR_24V_EN)
- IC4A monitors power status (PWR_FLT)
- Both MCUs monitor power good signals (5V_PG, 3V3A_PG)

---

## Key Differences from Previous Mapping

1. **DUAL-MCU Architecture** - Two separate STM32F413ZHT3 controllers
2. **IC4A handles:** LEM sensors (10× analog), some outputs, debug UART
3. **IC4B handles:** Power control, digital inputs (20×), CAN, I2C (FRAM/temp)
4. **LEM sensors:** PA0-PA7 + PD0-PD1 (IC4A)
5. **Digital inputs:** Spread across PF2-PF10, PG2-PG7 (IC4B)
6. **Power control:** PE7-PE9 (IC4B)
7. **I2C:** PF0-PF1 (IC4B) for FRAM and temperature sensor

---

**Revision:** 2.0 - FROM SCHEMATIC
**Date:** 2026-01-09
**Status:** Based on actual hardware schematic
