# CoreXY CNC Control System

This project implements a CoreXY kinematics system for a 3D printer or CNC machine running on an STM32G431 microcontroller. The system provides G-code parsing capabilities, stepper motor control, and basic machine operation.

## Project Structure

### Main Application Module (`main.c`)
- Entry point for the application
- Initializes hardware components (UART, GPIO, etc.)
- Implements a command-line interface for receiving G-code commands via serial
- Processes user input and delegates command execution to appropriate modules

### CoreXY Kinematics Module (`corexy.c`, `corexy.h`)
- Implements CoreXY kinematics for translating Cartesian coordinates to stepper motor movements
- Provides motion planning and control for 3-axis system (X, Y, Z)
- Manages machine dimensions and physical constraints
- Handles homing procedures and endstop detection
- Processes G-code movement commands (G0, G1, G28)

### G-code Parser Module (`parser.c`, `parser.h`) 
- Parses G-code strings into structured command objects
- Extracts command codes and parameters (X, Y, Z, F values)
- Validates command syntax
- Supports various G-code commands (G0, G1, G28, G90, G91)
- Provides callback mechanism for command execution

### Stepper Motor Control Module (`stepper_motor.c`, `stepper_motor.h`)
- Provides low-level control for stepper motors
- Handles timing and pulse generation for smooth motor movement
- Supports acceleration and deceleration profiles
- Manages motor enable/disable functionality
- Controls motor direction and step signals

### Hardware Abstraction Layer
- **GPIO Control** (`stm32g4_gpio.h`) - Manages GPIO pins configuration and control
- **UART Communication** (`stm32g4_uart.h`) - Handles serial communication
- **System Functions** (`stm32g4_sys.h`) - Core system functionality and clock configuration
- **ADC Control** (`stm32g4_adc.h`) - Analog-to-digital conversion functionality
- **Timer Control** (`stm32g4_timer.h`) - Manages hardware timers for precise timing

## Hardware Configuration
- **Microcontroller**: STM32G431 - NUCLEO-G431KB
- **Communication**: UART at 115200 baud
- **Status Indicator**: Green LED
- **Build Volume**: 220x220x250mm (configurable)
- **Endstops**: Configurable endstops for X, Y, Z axes (optional)

## Supported Commands
- **G0/G1**: Linear movement
- **G28**: Home axes
- **G90**: Absolute positioning
- **G91**: Relative positioning
- **Special commands**: HOME, STOP, POS

## Usage
1. Connect to the device via serial terminal at 115200 baud
2. Send G-code commands (e.g., "G1 X10 Y20 F1000")
3. System will execute commands and return feedback
4. Use "HOME" command to home axes, "STOP" to halt operations, "POS" to query position

## Building and Flashing
The project is configured for STM32Cube development environment. It can be built using:
- STM32CubeIDE
- Visual Studio Code with Embedded tools

See the [Setup Guide](/home/ol/Github-Repos/Modular_3D_Printer/Kinematics_Prototype/SOFT/README.md) for detailed development environment setup instructions.