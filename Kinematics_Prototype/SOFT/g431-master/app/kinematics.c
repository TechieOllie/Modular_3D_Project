/**
 *******************************************************************************
 * @file    kinematics.c
 * @author  GitHub Copilot
 * @date    May 27, 2025
 * @brief   Cartesian flying gantry kinematics implementation
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "kinematics.h"
#include "stepper_motor.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static machine_config_t machine_config;
static position_t current_position = {0.0f, 0.0f};
static position_t target_position = {0.0f, 0.0f};
static move_state_t current_state = MOVE_STATE_IDLE;
static bool kinematics_initialized = false;

/* Private function declarations ---------------------------------------------*/
static bool execute_move(const position_t *start_pos, const position_t *end_pos, float feedrate);
static float calculate_move_distance(const position_t *start_pos, const position_t *end_pos);

/* Public functions implementations ------------------------------------------*/

/**
 * @brief Initialize the kinematics system
 */
bool kinematics_init(const machine_config_t *config)
{
    if (config == NULL)
    {
        printf("ERROR: Kinematics config is NULL\n");
        return false;
    }

    // Copy configuration
    memcpy(&machine_config, config, sizeof(machine_config_t));

    // Validate configuration
    if (machine_config.steps_per_mm_x <= 0 || machine_config.steps_per_mm_y <= 0)
    {
        printf("ERROR: Invalid steps per mm configuration\n");
        return false;
    }

    if (machine_config.max_velocity_x <= 0 || machine_config.max_velocity_y <= 0)
    {
        printf("ERROR: Invalid max velocity configuration\n");
        return false;
    }

    // Initialize position to origin
    current_position.x = 0.0f;
    current_position.y = 0.0f;
    target_position.x = 0.0f;
    target_position.y = 0.0f;

    current_state = MOVE_STATE_IDLE;
    kinematics_initialized = true;

    printf("Kinematics initialized:\n");
    printf("  X: %.1f steps/mm, max vel: %.1f mm/s, motor ID: %d\n",
           machine_config.steps_per_mm_x, machine_config.max_velocity_x, machine_config.x_motor_id);
    printf("  Y: %.1f steps/mm, max vel: %.1f mm/s, motor ID: %d\n",
           machine_config.steps_per_mm_y, machine_config.max_velocity_y, machine_config.y_motor_id);
    printf("  Work area: %.1f x %.1f mm\n", machine_config.x_max, machine_config.y_max);

    return true;
}

/**
 * @brief Get machine configuration
 */
machine_config_t *kinematics_get_config(void)
{
    if (!kinematics_initialized)
    {
        return NULL;
    }

    return &machine_config;
}

/**
 * @brief Move to absolute position in cartesian coordinates
 */
bool kinematics_move_to(const position_t *target_pos, float feedrate)
{
    if (!kinematics_initialized || target_pos == NULL)
    {
        printf("ERROR: Kinematics not initialized or target position is NULL\n");
        return false;
    }

    if (current_state != MOVE_STATE_IDLE)
    {
        printf("ERROR: Machine is busy (state: %d)\n", current_state);
        return false;
    }

    if (!kinematics_is_position_valid(target_pos))
    {
        printf("ERROR: Target position (%.2f, %.2f) is out of bounds\n", target_pos->x, target_pos->y);
        return false;
    }

    printf("Moving to position: X=%.2f, Y=%.2f at %.1f mm/min\n",
           target_pos->x, target_pos->y, feedrate);

    // Execute the move
    if (execute_move(&current_position, target_pos, feedrate))
    {
        target_position = *target_pos;
        current_state = MOVE_STATE_MOVING;
        return true;
    }

    return false;
}

/**
 * @brief Move relative to current position
 */
bool kinematics_move_relative(const position_t *delta_pos, float feedrate)
{
    if (!kinematics_initialized || delta_pos == NULL)
    {
        return false;
    }

    position_t new_target;
    new_target.x = current_position.x + delta_pos->x;
    new_target.y = current_position.y + delta_pos->y;

    printf("Relative move: delta X=%.2f, Y=%.2f (target: X=%.2f, Y=%.2f)\n",
           delta_pos->x, delta_pos->y, new_target.x, new_target.y);

    return kinematics_move_to(&new_target, feedrate);
}

/**
 * @brief Home all axes
 */
bool kinematics_home_all(void)
{
    if (!kinematics_initialized)
    {
        return false;
    }

    if (current_state != MOVE_STATE_IDLE)
    {
        printf("ERROR: Cannot home while machine is busy\n");
        return false;
    }

    printf("Starting homing sequence for all axes\n");
    current_state = MOVE_STATE_HOMING;

    // For now, just set position to origin
    // In a real implementation, you would move to limit switches
    current_position.x = 0.0f;
    current_position.y = 0.0f;
    target_position.x = 0.0f;
    target_position.y = 0.0f;

    current_state = MOVE_STATE_IDLE;
    printf("Homing complete - position set to origin\n");

    return true;
}

/**
 * @brief Home specific axis
 */
bool kinematics_home_axis(char axis)
{
    if (!kinematics_initialized)
    {
        return false;
    }

    if (current_state != MOVE_STATE_IDLE)
    {
        printf("ERROR: Cannot home while machine is busy\n");
        return false;
    }

    printf("Homing %c axis\n", axis);
    current_state = MOVE_STATE_HOMING;

    // For now, just set the specific axis to origin
    if (axis == 'X' || axis == 'x')
    {
        current_position.x = 0.0f;
        target_position.x = 0.0f;
    }
    else if (axis == 'Y' || axis == 'y')
    {
        current_position.y = 0.0f;
        target_position.y = 0.0f;
    }
    else
    {
        printf("ERROR: Invalid axis '%c'\n", axis);
        current_state = MOVE_STATE_IDLE;
        return false;
    }

    current_state = MOVE_STATE_IDLE;
    printf("%c axis homing complete\n", axis);

    return true;
}

/**
 * @brief Get current position
 */
position_t kinematics_get_position(void)
{
    return current_position;
}

/**
 * @brief Set current position (for homing or calibration)
 */
void kinematics_set_position(const position_t *pos)
{
    if (pos != NULL)
    {
        current_position = *pos;
        target_position = *pos;
        printf("Position set to: X=%.2f, Y=%.2f\n", pos->x, pos->y);
    }
}

/**
 * @brief Get current movement state
 */
move_state_t kinematics_get_state(void)
{
    return current_state;
}

/**
 * @brief Stop all movement immediately
 */
void kinematics_stop(void)
{
    if (!kinematics_initialized)
    {
        return;
    }

    printf("Emergency stop - stopping all motors\n");

    // Stop both motors
    stepper_motor_stop(machine_config.x_motor_id);
    stepper_motor_stop(machine_config.y_motor_id);

    current_state = MOVE_STATE_IDLE;
}

/**
 * @brief Update kinematics system - call periodically in main loop
 */
void kinematics_update(void)
{
    if (!kinematics_initialized)
    {
        return;
    }

    if (current_state == MOVE_STATE_MOVING)
    {
        // Check if both motors have finished moving
        stepper_motor_state_t x_state = stepper_motor_get_state(machine_config.x_motor_id);
        stepper_motor_state_t y_state = stepper_motor_get_state(machine_config.y_motor_id);

        if (x_state == MOTOR_STATE_IDLE && y_state == MOTOR_STATE_IDLE)
        {
            // Movement complete - update current position to target
            current_position = target_position;
            current_state = MOVE_STATE_IDLE;

            printf("Move completed - position: X=%.2f, Y=%.2f\n",
                   current_position.x, current_position.y);
        }
    }
}

/**
 * @brief Check if position is within machine limits
 */
bool kinematics_is_position_valid(const position_t *pos)
{
    if (pos == NULL)
    {
        return false;
    }

    return (pos->x >= 0.0f && pos->x <= machine_config.x_max &&
            pos->y >= 0.0f && pos->y <= machine_config.y_max);
}

/**
 * @brief Convert mm to steps for X-axis
 */
uint32_t kinematics_mm_to_steps_x(float mm)
{
    return (uint32_t)(fabsf(mm) * machine_config.steps_per_mm_x);
}

/**
 * @brief Convert mm to steps for Y-axis
 */
uint32_t kinematics_mm_to_steps_y(float mm)
{
    return (uint32_t)(fabsf(mm) * machine_config.steps_per_mm_y);
}

/**
 * @brief Convert steps to mm for X-axis
 */
float kinematics_steps_to_mm_x(uint32_t steps)
{
    return (float)steps / machine_config.steps_per_mm_x;
}

/**
 * @brief Convert steps to mm for Y-axis
 */
float kinematics_steps_to_mm_y(uint32_t steps)
{
    return (float)steps / machine_config.steps_per_mm_y;
}

/**
 * @brief Convert feedrate (mm/min) to steps per second for given axis
 */
uint32_t kinematics_feedrate_to_steps_per_sec(float feedrate, float distance_mm, char axis)
{
    // Convert mm/min to mm/s
    float mm_per_sec = feedrate / 60.0f;

    // Get steps per mm for the axis
    float steps_per_mm;
    if (axis == 'X' || axis == 'x')
    {
        steps_per_mm = machine_config.steps_per_mm_x;
    }
    else if (axis == 'Y' || axis == 'y')
    {
        steps_per_mm = machine_config.steps_per_mm_y;
    }
    else
    {
        return 0;
    }

    // Calculate steps per second
    uint32_t steps_per_sec = (uint32_t)(mm_per_sec * steps_per_mm);

    return steps_per_sec;
}

/* Private functions implementations ------------------------------------------*/

/**
 * @brief Execute a move between two positions
 */
static bool execute_move(const position_t *start_pos, const position_t *end_pos, float feedrate)
{
    // Calculate movement distances
    float delta_x = end_pos->x - start_pos->x;
    float delta_y = end_pos->y - start_pos->y;

    printf("Move deltas: X=%.2f mm, Y=%.2f mm\n", delta_x, delta_y);

    // Convert to steps
    uint32_t steps_x = kinematics_mm_to_steps_x(delta_x);
    uint32_t steps_y = kinematics_mm_to_steps_y(delta_y);

    // Determine directions
    stepper_motor_dir_t dir_x = (delta_x >= 0) ? MOTOR_DIR_CLOCKWISE : MOTOR_DIR_COUNTERCLOCKWISE;
    stepper_motor_dir_t dir_y = (delta_y >= 0) ? MOTOR_DIR_CLOCKWISE : MOTOR_DIR_COUNTERCLOCKWISE;

    printf("Steps: X=%lu (%s), Y=%lu (%s)\n",
           steps_x, (dir_x == MOTOR_DIR_CLOCKWISE) ? "CW" : "CCW",
           steps_y, (dir_y == MOTOR_DIR_CLOCKWISE) ? "CW" : "CCW");

    // Calculate move time and speeds for each axis
    float total_distance = calculate_move_distance(start_pos, end_pos);

    if (total_distance < 0.001f)
    {
        printf("Move distance too small, ignoring\n");
        return true; // No movement needed
    }

    // Calculate time for the move in seconds
    float move_time_sec = (total_distance / feedrate) * 60.0f; // feedrate is in mm/min

    // Calculate steps per second for each axis
    uint32_t steps_per_sec_x = 0;
    uint32_t steps_per_sec_y = 0;

    if (steps_x > 0)
    {
        steps_per_sec_x = (uint32_t)(steps_x / move_time_sec);
        if (steps_per_sec_x < 1)
            steps_per_sec_x = 1; // Minimum speed
    }

    if (steps_y > 0)
    {
        steps_per_sec_y = (uint32_t)(steps_y / move_time_sec);
        if (steps_per_sec_y < 1)
            steps_per_sec_y = 1; // Minimum speed
    }

    printf("Move time: %.2f sec, Speeds: X=%lu Hz, Y=%lu Hz\n",
           move_time_sec, steps_per_sec_x, steps_per_sec_y);

    // Start movement for both axes simultaneously
    bool success = true;

    if (steps_x > 0)
    {
        if (!stepper_motor_move(machine_config.x_motor_id, steps_x, steps_per_sec_x, dir_x))
        {
            printf("ERROR: Failed to start X-axis movement\n");
            success = false;
        }
    }

    if (steps_y > 0 && success)
    {
        if (!stepper_motor_move(machine_config.y_motor_id, steps_y, steps_per_sec_y, dir_y))
        {
            printf("ERROR: Failed to start Y-axis movement\n");
            stepper_motor_stop(machine_config.x_motor_id); // Stop X if Y failed
            success = false;
        }
    }

    return success;
}

/**
 * @brief Calculate total move distance (Euclidean distance)
 */
static float calculate_move_distance(const position_t *start_pos, const position_t *end_pos)
{
    float delta_x = end_pos->x - start_pos->x;
    float delta_y = end_pos->y - start_pos->y;

    return sqrtf(delta_x * delta_x + delta_y * delta_y);
}
