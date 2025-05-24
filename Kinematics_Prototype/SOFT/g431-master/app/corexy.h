/**
 *******************************************************************************
 * @file    corexy.h
 * @author  Your Name
 * @date    Current Date
 * @brief   CoreXY kinematics for 3D printer control with NEMA 17 motors
 *******************************************************************************
 */
#ifndef COREXY_H
#define COREXY_H

#include "config.h"
#include "stepper_motor.h"
#include "parser.h"

// CoreXY states
typedef enum
{
    COREXY_STATE_IDLE = 0, // System idle, ready to receive commands
    COREXY_STATE_MOVING,   // System executing a move command
    COREXY_STATE_HOMING_Z, // Homing Z axis
    COREXY_STATE_HOMING_X, // Homing X axis
    COREXY_STATE_HOMING_Y, // Homing Y axis
    COREXY_STATE_ERROR     // Error state
} corexy_state_t;

// CoreXY move operation types
typedef enum
{
    COREXY_MOVE_RAPID = 0, // G0 - rapid positioning
    COREXY_MOVE_LINEAR,    // G1 - linear move
    COREXY_MOVE_ARC_CW,    // G2 - clockwise arc
    COREXY_MOVE_ARC_CCW    // G3 - counterclockwise arc
} corexy_move_type_t;

// CoreXY callback function type
typedef void (*corexy_callback_t)(bool success);

// Machine dimensions - these define the maximum possible travel distance
typedef struct
{
    float max_x_mm; // Maximum X dimension in mm
    float max_y_mm; // Maximum Y dimension in mm
    float max_z_mm; // Maximum Z dimension in mm
} corexy_dimensions_t;

/**
 * Initialize the CoreXY system
 * This configures the stepper motors for a CoreXY setup
 */
void CoreXY_Init(void);

/**
 * Process pending movements and state transitions
 * Call this regularly in the main loop
 */
void CoreXY_Process(void);

/**
 * Move to the specified absolute coordinates
 * @param x X coordinate in mm
 * @param y Y coordinate in mm
 * @param z Z coordinate in mm
 * @param feedrate Speed in mm/min
 * @param callback Function to call when movement completes (can be NULL)
 * @return true if movement started, false if error
 */
bool CoreXY_MoveTo(float x, float y, float z, float feedrate, corexy_callback_t callback);

/**
 * Move by the specified relative distances
 * @param x X distance in mm
 * @param y Y distance in mm
 * @param z Z distance in mm
 * @param feedrate Speed in mm/min
 * @param callback Function to call when movement completes (can be NULL)
 * @return true if movement started, false if error
 */
bool CoreXY_MoveBy(float x, float y, float z, float feedrate, corexy_callback_t callback);

/**
 * Process a parsed G-code command
 * @param cmd The parsed command structure
 * @return true if command was processed successfully, false if error
 */
bool CoreXY_ProcessGCodeCommand(parser_command_t *cmd);

/**
 * Home all axes or specific axes
 * @param home_x Whether to home X axis
 * @param home_y Whether to home Y axis
 * @param home_z Whether to home Z axis
 * @param callback Function to call when homing completes (can be NULL)
 * @return true if homing started, false if error
 */
bool CoreXY_Home(bool home_x, bool home_y, bool home_z, corexy_callback_t callback);

/**
 * Emergency stop - immediately halt all motion
 */
void CoreXY_EmergencyStop(void);

/**
 * Get current position
 * @param x Pointer to store X position
 * @param y Pointer to store Y position
 * @param z Pointer to store Z position
 */
void CoreXY_GetPosition(float *x, float *y, float *z);

/**
 * Set absolute positioning mode (G90)
 */
void CoreXY_SetAbsolutePositioning(void);

/**
 * Set relative positioning mode (G91)
 */
void CoreXY_SetRelativePositioning(void);

/**
 * Get current positioning mode
 * @return true for absolute, false for relative
 */
bool CoreXY_IsAbsolutePositioning(void);

/**
 * Set the machine dimensions (build volume)
 * @param max_x_mm Maximum X travel in mm
 * @param max_y_mm Maximum Y travel in mm
 * @param max_z_mm Maximum Z travel in mm
 */
void CoreXY_SetMachineDimensions(float max_x_mm, float max_y_mm, float max_z_mm);

/**
 * Get the current machine dimensions
 * @return Structure containing max X, Y, Z dimensions
 */
corexy_dimensions_t CoreXY_GetMachineDimensions(void);

/**
 * Get the current CoreXY state
 * @return Current state enum value
 */
corexy_state_t CoreXY_GetState(void);

/**
 * Configure endstops for CoreXY system
 * @param x_min_port Port for X min endstop
 * @param x_min_pin Pin for X min endstop
 * @param x_min_trigger_level Trigger level for X min endstop
 * @param y_min_port Port for Y min endstop
 * @param y_min_pin Pin for Y min endstop
 * @param y_min_trigger_level Trigger level for Y min endstop
 * @param z_min_port Port for Z min endstop
 * @param z_min_pin Pin for Z min endstop
 * @param z_min_trigger_level Trigger level for Z min endstop
 */
#if ENDSTOP_ENABLED
void CoreXY_ConfigureEndstops(
    // X min endstop
    GPIO_TypeDef *x_min_port, uint16_t x_min_pin, uint8_t x_min_trigger_level,
    // Y min endstop
    GPIO_TypeDef *y_min_port, uint16_t y_min_pin, uint8_t y_min_trigger_level,
    // Z min endstop
    GPIO_TypeDef *z_min_port, uint16_t z_min_pin, uint8_t z_min_trigger_level);
#endif

#endif /* COREXY_H */