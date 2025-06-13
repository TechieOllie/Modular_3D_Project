/**
 *******************************************************************************
 * @file    kinematics.h
 * @author  Ol, naej, Fabs
 * @date    May 27, 2025
 * @brief   Cartesian flying gantry kinematics for 2-axis CNC system
 *******************************************************************************
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "stepper_motor.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Position structure for cartesian coordinates
 */
typedef struct
{
    float x; // X-axis position in mm
    float y; // Y-axis position in mm
} position_t;

/**
 * @brief Velocity structure for cartesian velocities
 */
typedef struct
{
    float x; // X-axis velocity in mm/s
    float y; // Y-axis velocity in mm/s
} velocity_t;

/**
 * @brief Machine configuration structure
 */
typedef struct
{
    float steps_per_mm_x;     // Steps per millimeter for X-axis
    float steps_per_mm_y;     // Steps per millimeter for Y-axis
    float max_velocity_x;     // Maximum velocity for X-axis in mm/s
    float max_velocity_y;     // Maximum velocity for Y-axis in mm/s
    float max_acceleration_x; // Maximum acceleration for X-axis in mm/s²
    float max_acceleration_y; // Maximum acceleration for Y-axis in mm/s²
    float x_max;              // Maximum X position in mm
    float y_max;              // Maximum Y position in mm
    uint8_t x_motor_id;       // Motor ID for X-axis
    uint8_t y_motor_id;       // Motor ID for Y-axis
} machine_config_t;

/**
 * @brief Movement state enumeration
 */
typedef enum
{
    MOVE_STATE_IDLE,
    MOVE_STATE_MOVING,
    MOVE_STATE_HOMING
} move_state_t;

/* Exported constants --------------------------------------------------------*/
#define KINEMATICS_MAX_VELOCITY_DEFAULT 50.0f      // mm/s
#define KINEMATICS_MAX_ACCELERATION_DEFAULT 100.0f // mm/s²
#define KINEMATICS_STEPS_PER_MM_DEFAULT 80.0f      // Typical for 1.8° stepper with 20-tooth pulley and GT2 belt

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize the kinematics system
 * @param config Pointer to machine configuration structure
 * @return true if initialization successful, false otherwise
 */
bool kinematics_init(const machine_config_t *config);

/**
 * @brief Get machine configuration
 * @return Pointer to machine configuration structure, or NULL if not initialized
 */
machine_config_t *kinematics_get_config(void);

/**
 * @brief Move to absolute position in cartesian coordinates
 * @param target_pos Target position in mm
 * @param feedrate Feedrate in mm/min
 * @return true if move started successfully, false otherwise
 */
bool kinematics_move_to(const position_t *target_pos, float feedrate);

/**
 * @brief Move relative to current position
 * @param delta_pos Relative movement in mm
 * @param feedrate Feedrate in mm/min
 * @return true if move started successfully, false otherwise
 */
bool kinematics_move_relative(const position_t *delta_pos, float feedrate);

/**
 * @brief Home all axes
 * @return true if homing started successfully, false otherwise
 */
bool kinematics_home_all(void);

/**
 * @brief Home specific axis
 * @param axis 'X' or 'Y'
 * @return true if homing started successfully, false otherwise
 */
bool kinematics_home_axis(char axis);

/**
 * @brief Get current position
 * @return Current position in mm
 */
position_t kinematics_get_position(void);

/**
 * @brief Set current position (for homing or calibration)
 * @param pos Position to set in mm
 */
void kinematics_set_position(const position_t *pos);

/**
 * @brief Get current movement state
 * @return Current movement state
 */
move_state_t kinematics_get_state(void);

/**
 * @brief Stop all movement immediately
 */
void kinematics_stop(void);

/**
 * @brief Update kinematics system - call periodically in main loop
 */
void kinematics_update(void);

/**
 * @brief Check if position is within machine limits
 * @param pos Position to check
 * @return true if position is valid, false otherwise
 */
bool kinematics_is_position_valid(const position_t *pos);

/**
 * @brief Convert mm to steps for X-axis
 * @param mm Distance in millimeters
 * @return Number of steps
 */
uint32_t kinematics_mm_to_steps_x(float mm);

/**
 * @brief Convert mm to steps for Y-axis
 * @param mm Distance in millimeters
 * @return Number of steps
 */
uint32_t kinematics_mm_to_steps_y(float mm);

/**
 * @brief Convert steps to mm for X-axis
 * @param steps Number of steps
 * @return Distance in millimeters
 */
float kinematics_steps_to_mm_x(uint32_t steps);

/**
 * @brief Convert steps to mm for Y-axis
 * @param steps Number of steps
 * @return Distance in millimeters
 */
float kinematics_steps_to_mm_y(uint32_t steps);

/**
 * @brief Convert feedrate (mm/min) to steps per second for given distance
 * @param feedrate Feedrate in mm/min
 * @param distance_mm Distance in mm
 * @param axis 'X' or 'Y'
 * @return Steps per second
 */
uint32_t kinematics_feedrate_to_steps_per_sec(float feedrate, float distance_mm, char axis);

#endif /* KINEMATICS_H */
