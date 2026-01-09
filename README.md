Battery Control Unit (BCU)
Overview

The Battery Control Unit (BCU) is an industrial-grade embedded system designed for monitoring, protection, and control of battery-powered systems. It integrates robust power management, a high-performance STM32 microcontroller, smart high-side outputs, multiple analog and digital inputs, and reliable CAN communication.

The BCU is suitable for applications such as:

Battery management systems (BMS support units)

Industrial power distribution

Electric and hybrid vehicles

Energy storage systems

Automation and control systems

Key Features
Power Management

Nominal input voltage: 12 V DC (automotive / industrial)

Multi-stage power architecture:

24 V / battery input

5 V digital rail

3.3 V digital rail

3.3 V analog rail (separate analog domain)

Power-good signals for system supervision

Undervoltage, overvoltage, overcurrent, and fault monitoring

Current monitoring (IMON)

Microcontroller

STM32F413 (ARM Cortex-M4)

External clocks:

24 MHz system clock

32.768 kHz RTC clock

Debug interfaces:

SWD / SWO

Debug UART

Interfaces:

SPI

Isolated SPI (ISO-SPI)

I²C

Dual CAN

External FRAM memory

Integrated temperature monitoring

Communication Interfaces

2× CAN bus

Automotive-grade CAN transceivers (TCAN3404)

Standby and normal operating modes

ESD and EMC protection

Isolated SPI (ISO-SPI)

Galvanically isolated communication

Suitable for isolated measurement modules

Standard SPI and I²C interfaces for peripherals and expansion

Analog Inputs

Designed for LEM current sensors

Configurable reference voltage:

External reference: 0.5 V – 1.7 V

Internal reference: 1.65 V

Features:

Overcurrent detection

Analog filtering (RC + ferrite beads)

ESD protection

Separate analog ground for improved signal integrity

Digital Inputs

Up to 20 digital input channels

Protected against ESD and noise

Suitable for:

Switches

Status signals

Diagnostic inputs

Hardware filtering for reliable operation in harsh environments

Outputs

Smart high-side power outputs (BTT6200-4ESA)

Multiple output groups (OUTA, OUTB, OUTC, OUTD)

Integrated protection:

Overcurrent

Short-circuit

Thermal shutdown

Diagnostic feedback via current sense

Typical use cases:

Relays

Valves

Heaters

Motors and inductive loads

Safety & Diagnostics

Comprehensive fault detection

Power supply supervision

Current and temperature monitoring

Extensive test points for debugging and validation

Clear separation of power, digital, and analog domains

Project Structure

The schematic is organized hierarchically for clarity and maintainability:

Block Diagram – System-level overview

Top Level – Interconnection of all subsystems

Power Supply – Power conversion and protection

MCU – Microcontroller and peripherals

Communication – CAN, SPI, ISO-SPI

Analog Inputs – Current and analog sensing

Digital Inputs – Digital signal acquisition

Outputs – High-side power drivers

Project Status

Revision: A0

Status: DRAFT

This project is currently in an early development stage. The overall architecture and functionality are defined, while details may change during further validation and testing.

Author

Rudolf-Leon Filip
