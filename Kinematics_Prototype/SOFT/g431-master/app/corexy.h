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
void CoreXY_ProcessGCodeCommand(GCodeCommand* cmd);

// Home the machine
void CoreXY_Home(void);

// Get current position
void CoreXY_GetPosition(float* x, float* y, float* z);

// Emergency stop
void CoreXY_EmergencyStop(void);

#endif /* COREXY_H */