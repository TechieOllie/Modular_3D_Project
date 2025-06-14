/**
 *******************************************************************************
 * @file    kinematics.c
 * @author  Ol, naej, Fabs
 * @date    May 27, 2025
 * @brief   Cartesian gantry kinematics implementation
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "kinematics.h"
#include "stepper_motor.h"
#include "stm32g4xx_hal.h" // Fixed: Added missing include for HAL_GetTick()
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
bool kinematics_move_to(const position_t *target, float feedrate_mm_min)
{
    if (!kinematics_initialized || !target)
    {
        return false;
    }

    if (current_state != MOVE_STATE_IDLE)
    {
        printf("Warning: Movement already in progress\n");
        return false;
    }

    // Calculate movement vector
    float dx = target->x - current_position.x;
    float dy = target->y - current_position.y;
    float distance = sqrtf(dx * dx + dy * dy);

    if (distance < 0.001f) // Very small movement, ignore
    {
        printf("Movement too small, ignored\n");
        return true;
    }

    // Calculate movement time based on feedrate
    float move_time_min = distance / feedrate_mm_min; // Time in minutes
    float move_time_sec = move_time_min * 60.0f;      // Time in seconds

    // Calculate step counts for each axis
    uint32_t x_steps = (uint32_t)(fabsf(dx) * machine_config.steps_per_mm_x);
    uint32_t y_steps = (uint32_t)(fabsf(dy) * machine_config.steps_per_mm_y);

    // Calculate frequencies for each motor to complete move in same time
    uint32_t x_freq = 0;
    uint32_t y_freq = 0;

    if (x_steps > 0 && move_time_sec > 0.001f)
    {
        x_freq = (uint32_t)(x_steps / move_time_sec);
        if (x_freq < STEPPER_MIN_FREQUENCY)
            x_freq = STEPPER_MIN_FREQUENCY;
        if (x_freq > STEPPER_MAX_FREQUENCY)
            x_freq = STEPPER_MAX_FREQUENCY;
    }

    if (y_steps > 0 && move_time_sec > 0.001f)
    {
        y_freq = (uint32_t)(y_steps / move_time_sec);
        if (y_freq < STEPPER_MIN_FREQUENCY)
            y_freq = STEPPER_MIN_FREQUENCY;
        if (y_freq > STEPPER_MAX_FREQUENCY)
            y_freq = STEPPER_MAX_FREQUENCY;
    }

    printf("Coordinated move: dx=%.2f dy=%.2f, dist=%.2f, time=%.2fs\n",
           dx, dy, distance, move_time_sec);
    printf("Steps: X=%lu@%luHz, Y=%lu@%luHz\n",
           x_steps, x_freq, y_steps, y_freq);

    // Determine directions
    stepper_motor_dir_t x_dir = (dx >= 0) ? MOTOR_DIR_CLOCKWISE : MOTOR_DIR_COUNTERCLOCKWISE;
    stepper_motor_dir_t y_dir = (dy >= 0) ? MOTOR_DIR_CLOCKWISE : MOTOR_DIR_COUNTERCLOCKWISE;

    // Store target for tracking
    target_position = *target;
    current_state = MOVE_STATE_MOVING;

    // Start both motors simultaneously
    bool x_started = false;
    bool y_started = false;

    if (x_steps > 0)
    {
        x_started = stepper_motor_move_accel(machine_config.x_motor_id, x_steps, x_freq, x_dir);
        if (!x_started)
        {
            printf("Failed to start X motor\n");
        }
    }
    else
    {
        x_started = true; // No X movement needed
    }

    if (y_steps > 0)
    {
        y_started = stepper_motor_move_accel(machine_config.y_motor_id, y_steps, y_freq, y_dir);
        if (!y_started)
        {
            printf("Failed to start Y motor\n");
        }
    }
    else
    {
        y_started = true; // No Y movement needed
    }

    if (!x_started || !y_started)
    {
        // Stop any motors that did start
        stepper_motor_stop(machine_config.x_motor_id);
        stepper_motor_stop(machine_config.y_motor_id);
        current_state = MOVE_STATE_IDLE;
        return false;
    }

    printf("Both motors started successfully\n");
    return true;
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

    // Home Y axis first (safer to home Y first)
    printf(" Homing Y-axis...\n");
    if (!stepper_motor_home_precision(machine_config.y_motor_id, 2000, 400)) // 2kHz fast, 400Hz slow
    {
        printf("ERROR: Failed to start Y axis homing\n");
        current_state = MOVE_STATE_IDLE;
        return false;
    }

    while (stepper_motor_get_state(machine_config.y_motor_id) != MOTOR_STATE_IDLE)
    {
        stepper_motor_update();
        HAL_Delay(10);
    }
    printf("Y axis homing complete\n");

    // Home X axis next
    printf("Homing X-axis...\n");
    if (!stepper_motor_home_precision(machine_config.x_motor_id, 2000, 400)) // 2kHz fast, 400Hz slow
    {
        printf("ERROR: Failed to start X axis homing\n");
        current_state = MOVE_STATE_IDLE;
        return false;
    }

    while (stepper_motor_get_state(machine_config.x_motor_id) != MOTOR_STATE_IDLE)
    {
        stepper_motor_update();
        HAL_Delay(10);
    }
    printf("X axis homing complete\n");

    // Set current position to origin after homing
    current_position.x = 0.0f;
    current_position.y = 0.0f;
    target_position = current_position;

    current_state = MOVE_STATE_IDLE;
    printf("Homing complete\n");

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

    if (axis == 'X' || axis == 'x')
    {
        if (!stepper_motor_home_precision(machine_config.x_motor_id, 2000, 400))
        {
            printf("ERROR: Failed to home X axis\n");
            current_state = MOVE_STATE_IDLE;
            return false;
        }

        while (stepper_motor_get_state(machine_config.x_motor_id) != MOTOR_STATE_IDLE)
        {
            stepper_motor_update();
            HAL_Delay(10); // Wait for X motor to finish homing
        }

        current_position.x = 0.0f; // Reset position after homing
        target_position.x = 0.0f;
    }
    else if (axis == 'Y' || axis == 'y')
    {
        if (!stepper_motor_home_precision(machine_config.x_motor_id, 2000, 400))
        {
            printf("ERROR: Failed to home Y axis\n");
            current_state = MOVE_STATE_IDLE;
            return false;
        }

        while (stepper_motor_get_state(machine_config.y_motor_id) != MOTOR_STATE_IDLE)
        {
            stepper_motor_update();
            HAL_Delay(10); // Wait for Y motor to finish homing
        }

        current_position.y = 0.0f; // Reset position after homing
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
    return &current_position;
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
    return &current_state;
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

    static uint32_t last_update = 0;
    uint32_t now = HAL_GetTick();

    // Update every 50ms
    if (now - last_update < 50)
    {
        return;
    }
    last_update = now;

    switch (current_state)
    {
    case MOVE_STATE_MOVING:
    {
        // Check if both motors have completed their moves
        stepper_motor_state_t x_state = stepper_motor_get_state(machine_config.x_motor_id);
        stepper_motor_state_t y_state = stepper_motor_get_state(machine_config.y_motor_id);

        bool x_done = (x_state == MOTOR_STATE_IDLE);
        bool y_done = (y_state == MOTOR_STATE_IDLE);

        // Debug output every 1 second during movement
        static uint32_t last_debug = 0;
        if (now - last_debug > 1000)
        {
            last_debug = now;
            uint32_t x_completed = stepper_motor_get_completed_steps(machine_config.x_motor_id);
            uint32_t y_completed = stepper_motor_get_completed_steps(machine_config.y_motor_id);
            printf("Movement progress - X: %lu steps (state %d), Y: %lu steps (state %d)\n",
                   x_completed, x_state, y_completed, y_state);
        }

        // If both motors are done, update position and state
        if (x_done && y_done)
        {
            current_position = target_position;
            current_state = MOVE_STATE_IDLE;
            printf("Coordinated movement completed - Position: X=%.2f Y=%.2f\n",
                   current_position.x, current_position.y);
        }
        break;
    }

    case MOVE_STATE_HOMING:
    {
        // Update homing state
        stepper_motor_state_t x_state = stepper_motor_get_state(machine_config.x_motor_id);
        stepper_motor_state_t y_state = stepper_motor_get_state(machine_config.y_motor_id);

        if (x_state == MOTOR_STATE_IDLE && y_state == MOTOR_STATE_IDLE)
        {
            current_position.x = 0.0f;
            current_position.y = 0.0f;
            target_position = current_position;
            current_state = MOVE_STATE_IDLE;
            printf("Homing completed\n");
        }
        break;
    }

    case MOVE_STATE_IDLE:
    default:
        // Nothing to do in idle state
        break;
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

    // Check if any movement is needed
    if (steps_x == 0 && steps_y == 0)
    {
        printf("No movement needed - already at target position\n");
        return true;
    }

    // Calculate speeds using a simplified approach for better reliability
    // Convert feedrate from mm/min to mm/s
    float feedrate_mm_per_sec = feedrate / 60.0f;

    // Calculate individual axis speeds based on feedrate and distance ratio
    float total_distance = calculate_move_distance(start_pos, end_pos);

    uint32_t steps_per_sec_x = 0;
    uint32_t steps_per_sec_y = 0;

    // Set speed limits for reliable operation
    uint32_t min_speed = 2000;     // Minimum 2000 Hz for smooth operation
    uint32_t max_speed = 8000;     // Maximum 8000 Hz for reliable operation
    uint32_t default_speed = 4000; // Default speed when calculation fails

    if (total_distance > 0.001f) // Avoid division by zero
    {
        // Calculate time for the entire move
        float move_time_sec = total_distance / feedrate_mm_per_sec;

        // Calculate speed for each axis based on the move time
        if (steps_x > 0)
        {
            steps_per_sec_x = (uint32_t)(steps_x / move_time_sec);
            // Clamp to safe limits
            if (steps_per_sec_x < min_speed)
                steps_per_sec_x = min_speed;
            if (steps_per_sec_x > max_speed)
                steps_per_sec_x = max_speed;
        }

        if (steps_y > 0)
        {
            steps_per_sec_y = (uint32_t)(steps_y / move_time_sec);
            // Clamp to safe limits
            if (steps_per_sec_y < min_speed)
                steps_per_sec_y = min_speed;
            if (steps_per_sec_y > max_speed)
                steps_per_sec_y = max_speed;
        }
    }
    else
    {
        // Fallback to default speeds
        if (steps_x > 0)
            steps_per_sec_x = default_speed;
        if (steps_y > 0)
            steps_per_sec_y = default_speed;
    }

    printf("Calculated speeds: X=%lu Hz, Y=%lu Hz (feedrate=%.1f mm/min, distance=%.2f mm)\n",
           steps_per_sec_x, steps_per_sec_y, feedrate, total_distance);

    // Start movement for both axes simultaneously
    bool success = true;
    bool x_started = false;
    bool y_started = false;

    // Start X-axis movement
    if (steps_x > 0)
    {
        printf("Starting X-axis: %lu steps at %lu Hz, direction %d\n",
               steps_x, steps_per_sec_x, dir_x);

        if (stepper_motor_move(machine_config.x_motor_id, steps_x, steps_per_sec_x, dir_x))
        {
            x_started = true;
            printf("X-axis movement started successfully\n");
        }
        else
        {
            printf("ERROR: Failed to start X-axis movement\n");
            success = false;
        }
    }
    else
    {
        printf("X-axis: No movement needed\n");
    }

    // Start Y-axis movement
    if (steps_y > 0 && success)
    {
        printf("Starting Y-axis: %lu steps at %lu Hz, direction %d\n",
               steps_y, steps_per_sec_y, dir_y);

        if (stepper_motor_move(machine_config.y_motor_id, steps_y, steps_per_sec_y, dir_y))
        {
            y_started = true;
            printf("Y-axis movement started successfully\n");
        }
        else
        {
            printf("ERROR: Failed to start Y-axis movement\n");
            // Stop X if Y failed to start
            if (x_started)
            {
                stepper_motor_stop(machine_config.x_motor_id);
                printf("Stopped X-axis due to Y-axis failure\n");
            }
            success = false;
        }
    }
    else if (steps_y == 0)
    {
        printf("Y-axis: No movement needed\n");
    }

    if (success)
    {
        printf("Movement execution started successfully\n");
    }
    else
    {
        printf("Movement execution failed\n");
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
