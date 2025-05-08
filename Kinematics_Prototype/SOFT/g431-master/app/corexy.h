/**
 *******************************************************************************
 * @file 	corexy.h
 * @author 	naej, your name
 * @date 	Current Date
 * @brief	CoreXY kinematics for CNC control
 *******************************************************************************
 */
#ifndef COREXY_H
#define COREXY_H

#include "stm32g4xx_hal.h"
#include "stm32g4_utils.h"
#include "stepper_motor.h"
#include "parser.h"

// Initialize the CoreXY system
void CoreXY_Init(void);

// Process a movement command
void CoreXY_Move(float x, float y, float z, float feedrate);

// Move to absolute coordinates
void CoreXY_MoveTo(float x, float y, float z, float feedrate);

// Process a G-code command
void CoreXY_ProcessGCodeCommand(GCodeCommand *cmd);

// Home the machine
void CoreXY_Home(void);

// Get current position
void CoreXY_GetPosition(float *x, float *y, float *z);

// Emergency stop
void CoreXY_EmergencyStop(void);

// Function to set absolute positioning mode (G90)
void CoreXY_SetAbsolutePositioning(void);

// Function to set relative positioning mode (G91)
void CoreXY_SetRelativePositioning(void);

// Function to get current positioning mode
bool CoreXY_IsAbsolutePositioning(void);

// Machine dimensions - these define the maximum possible travel distance
typedef struct
{
    float max_x_mm; // Maximum X dimension in mm
    float max_y_mm; // Maximum Y dimension in mm
    float max_z_mm; // Maximum Z dimension in mm
} CoreXY_Dimensions_t;

// Function to set the machine dimensions
void CoreXY_SetMachineDimensions(float max_x_mm, float max_y_mm, float max_z_mm);

// Function to get the machine dimensions
CoreXY_Dimensions_t CoreXY_GetMachineDimensions(void);

// Function to configure endstops
#if ENDSTOP_ENABLED
void CoreXY_ConfigureEndstops(
    // X axis min endstop
    GPIO_TypeDef *x_min_port, uint16_t x_min_pin, uint8_t x_min_trigger_level,
    // X axis max endstop (optional)
    GPIO_TypeDef *x_max_port, uint16_t x_max_pin, uint8_t x_max_trigger_level,
    // Y axis min endstop
    GPIO_TypeDef *y_min_port, uint16_t y_min_pin, uint8_t y_min_trigger_level,
    // Y axis max endstop (optional)
    GPIO_TypeDef *y_max_port, uint16_t y_max_pin, uint8_t y_max_trigger_level,
    // Z axis min endstop
    GPIO_TypeDef *z_min_port, uint16_t z_min_pin, uint8_t z_min_trigger_level,
    // Z axis max endstop (optional)
    GPIO_TypeDef *z_max_port, uint16_t z_max_pin, uint8_t z_max_trigger_level);

// Home all axes
bool CoreXY_HomeAll(float homing_speed_mm_s);
#endif

#endif /* COREXY_H */