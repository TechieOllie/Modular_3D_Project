/**
 *******************************************************************************
 * @file    corexy.c
 * @author  Your Name
 * @date    Current Date
 * @brief   CoreXY kinematics implementation for NEMA 17 stepper motors
 *******************************************************************************
 */

#include "corexy.h"
#include "stepper_motor.h"
#include "stm32g4_gpio.h"
#include "stm32g4_timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Motor IDs for CoreXY system
static stepper_id_t motor_a = -1; // First CoreXY motor (A)
static stepper_id_t motor_b = -1; // Second CoreXY motor (B)
static stepper_id_t motor_z = -1; // Z axis motor

// Current position and state tracking
static float current_position[3] = {0.0f, 0.0f, 0.0f}; // X, Y, Z in mm
static corexy_state_t current_state = COREXY_STATE_IDLE;
static bool absolute_positioning = true; // Default is absolute positioning (G90)
static float default_feedrate = 1000.0f; // Default feedrate in mm/min

// CoreXY dimensions (build volume)
static corexy_dimensions_t machine_dimensions = {
    .max_x_mm = 220.0f,
    .max_y_mm = 220.0f,
    .max_z_mm = 250.0f};

// Movement tracking
static corexy_move_type_t current_move_type = COREXY_MOVE_RAPID;
static corexy_callback_t move_callback = NULL;
static uint32_t last_state_update = 0;
static const uint32_t STATE_UPDATE_INTERVAL_MS = 20; // State machine update interval

// Movement completion tracking
static bool motor_a_done = false;
static bool motor_b_done = false;
static bool motor_z_done = false;
static bool homing_in_progress = false;

// Forward declarations for internal functions
static void xy_to_ab(float x, float y, float *a, float *b);
static bool check_bounds(float x, float y, float z);
static void motor_a_callback(stepper_id_t id, bool success);
static void motor_b_callback(stepper_id_t id, bool success);
static void motor_z_callback(stepper_id_t id, bool success);
static void homing_completed_callback(bool success);

// Convert Cartesian XY coordinates to CoreXY AB motor positions
static void xy_to_ab(float x, float y, float *a, float *b)
{
    // CoreXY kinematics: A = X+Y, B = X-Y
    *a = x + y;
    *b = x - y;
}

// Check if coordinates are within machine bounds
static bool check_bounds(float x, float y, float z)
{
    if (x < 0 || x > machine_dimensions.max_x_mm)
    {
        printf("Error: X position %.2f out of bounds (0-%.2f)\n",
               x, machine_dimensions.max_x_mm);
        return false;
    }

    if (y < 0 || y > machine_dimensions.max_y_mm)
    {
        printf("Error: Y position %.2f out of bounds (0-%.2f)\n",
               y, machine_dimensions.max_y_mm);
        return false;
    }

    if (z < 0 || z > machine_dimensions.max_z_mm)
    {
        printf("Error: Z position %.2f out of bounds (0-%.2f)\n",
               z, machine_dimensions.max_z_mm);
        return false;
    }

    return true;
}

// Initialize the CoreXY system
void CoreXY_Init(void)
{
    printf("Initializing CoreXY system...\n");

    // Initialize stepper motors
    STEPPER_init();

    // Motor configuration for NEMA 17 at 1.5A
    float xy_steps_per_mm = 80.0f; // Steps per mm with microstepping
    uint8_t microstepping = 16;    // 1/16 microstepping
    float current_limit = 1.5f;    // 1.5A current limit for NEMA 17 motors

    // Add first CoreXY motor (A)
    // According to stm32g4_timer.c, this uses GPIO PB4 for step and PA6 for direction
    motor_a = STEPPER_add(GPIOB, GPIO_PIN_4, GPIOA, GPIO_PIN_6, TIMER3_ID, TIM_CHANNEL_1, xy_steps_per_mm, microstepping, current_limit);

    // Add second CoreXY motor (B)
    // Using GPIO PA10 for step and PA11 for direction with Timer 4
    motor_b = STEPPER_add(GPIOA, GPIO_PIN_10, GPIOA, GPIO_PIN_11, TIMER4_ID, TIM_CHANNEL_1, xy_steps_per_mm, microstepping, current_limit);

    // Z axis typically has different steps/mm due to lead screw pitch
    float z_steps_per_mm = 400.0f; // Example: 4-start 8mm pitch leadscrew with 1/16 microstepping

    // Add Z motor
    // Using GPIO PA8 for step and PA9 for direction with Timer 2
    motor_z = STEPPER_add(GPIOA, GPIO_PIN_8, GPIOA, GPIO_PIN_9, TIMER2_ID, TIM_CHANNEL_1, z_steps_per_mm, microstepping, current_limit);

    // Check if motors were successfully added
    if (motor_a < 0 || motor_b < 0 || motor_z < 0)
    {
        printf("Error: Failed to initialize one or more motors!\n");
        printf("Motor A: %d, Motor B: %d, Motor Z: %d\n", motor_a, motor_b, motor_z);
        current_state = COREXY_STATE_ERROR;
        return;
    }

    // Initialize state variables
    current_state = COREXY_STATE_IDLE;
    absolute_positioning = true;
    current_position[0] = 0.0f; // X
    current_position[1] = 0.0f; // Y
    current_position[2] = 0.0f; // Z

    printf("CoreXY system initialized with dimensions:\n");
    printf("X: %.1f mm, Y: %.1f mm, Z: %.1f mm\n",
           machine_dimensions.max_x_mm, machine_dimensions.max_y_mm, machine_dimensions.max_z_mm);
    printf("NEMA 17 motors: %d/16 microstepping, %.1fA current\n",
           microstepping, current_limit);
}

// Process the CoreXY state machine
void CoreXY_Process(void)
{
    // Only process at defined intervals to avoid hogging CPU
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_state_update < STATE_UPDATE_INTERVAL_MS)
    {
        return;
    }
    last_state_update = current_time;

    // Process based on current state
    switch (current_state)
    {
    case COREXY_STATE_IDLE:
        // Nothing to do in idle state
        break;

    case COREXY_STATE_MOVING:
        // Check if all motors have completed their moves
        if ((motor_a_done || motor_a < 0) &&
            (motor_b_done || motor_b < 0) &&
            (motor_z_done || motor_z < 0))
        {
            current_state = COREXY_STATE_IDLE;
            printf("Movement complete at X=%.2f Y=%.2f Z=%.2f\n",
                   current_position[0], current_position[1], current_position[2]);

            // Execute callback if provided
            if (move_callback != NULL)
            {
                corexy_callback_t callback = move_callback;
                move_callback = NULL;
                callback(true);
            }
        }
        break;

    case COREXY_STATE_HOMING_Z:
        if (!STEPPER_is_moving(motor_z))
        {
            current_position[2] = 0.0f; // Z is now at home position

            if (homing_in_progress)
            {
                // Z homing complete, now move to X homing
                current_state = COREXY_STATE_HOMING_X;
                printf("Z homing complete, now homing X axis...\n");

                // Start X homing
                // In a CoreXY system, homing X requires moving both A and B motors
                // For now, we'll simulate X homing by moving motor_a
                if (!STEPPER_home(motor_a, motor_a_callback))
                {
                    current_state = COREXY_STATE_ERROR;
                    printf("Failed to start X axis homing\n");
                    homing_completed_callback(false);
                }
            }
            else
            {
                // Only Z was requested to home
                current_state = COREXY_STATE_IDLE;
                printf("Z homing complete\n");
                homing_completed_callback(true);
            }
        }
        break;

    case COREXY_STATE_HOMING_X:
        if (!STEPPER_is_moving(motor_a))
        {
            current_position[0] = 0.0f; // X is now at home position

            if (homing_in_progress)
            {
                // X homing complete, now move to Y homing
                current_state = COREXY_STATE_HOMING_Y;
                printf("X homing complete, now homing Y axis...\n");

                // Start Y homing
                // In a CoreXY system, homing Y requires moving both A and B motors
                // For now, we'll simulate Y homing by moving motor_b
                if (!STEPPER_home(motor_b, motor_b_callback))
                {
                    current_state = COREXY_STATE_ERROR;
                    printf("Failed to start Y axis homing\n");
                    homing_completed_callback(false);
                }
            }
            else
            {
                // Only X was requested to home
                current_state = COREXY_STATE_IDLE;
                printf("X homing complete\n");
                homing_completed_callback(true);
            }
        }
        break;

    case COREXY_STATE_HOMING_Y:
        if (!STEPPER_is_moving(motor_b))
        {
            current_position[1] = 0.0f; // Y is now at home position
            current_state = COREXY_STATE_IDLE;
            printf("Y homing complete\n");

            // All homing is now complete
            homing_completed_callback(true);
        }
        break;

    case COREXY_STATE_ERROR:
        // Stay in error state until explicitly reset
        break;
    }
}

// Move to absolute coordinates
bool CoreXY_MoveTo(float x, float y, float z, float speed_mm_s, corexy_callback_t callback)
{
    // Check if we're already moving
    if (current_state != COREXY_STATE_IDLE)
        return false;

    // Check bounds
    if (!check_bounds(x, y, z))
    {
        printf("Error: Move target out of bounds\n");
        return false;
    }

    // Calculate steps required for each motor
    float dx = x - current_position[0];
    float dy = y - current_position[1];
    float dz = z - current_position[2];

    // Get steps per mm from the stepper motors directly
    float steps_per_mm_a = STEPPER_get_steps_per_mm(motor_a);
    float steps_per_mm_b = STEPPER_get_steps_per_mm(motor_b);
    float steps_per_mm_z = STEPPER_get_steps_per_mm(motor_z);

    // Calculate steps for CoreXY kinematics
    int32_t steps_a = (int32_t)((dx + dy) * steps_per_mm_a);
    int32_t steps_b = (int32_t)((dx - dy) * steps_per_mm_b);
    int32_t steps_z = (int32_t)(dz * steps_per_mm_z);

    printf("Movement deltas: dx=%.2f dy=%.2f dz=%.2f\n", dx, dy, dz);
    printf("Step counts: A=%ld B=%ld Z=%ld\n", steps_a, steps_b, steps_z);

    // Record target positions
    current_position[0] = x;
    current_position[1] = y;
    current_position[2] = z;

    // Record callback
    move_callback = callback;

    // Initialize motion tracking
    motor_a_done = false;
    motor_b_done = false;
    motor_z_done = false;

    // Start movement for each motor
    bool a_started = true;
    bool b_started = true;
    bool z_started = true;

    if (steps_a != 0 && motor_a >= 0)
    {
        float speed_steps = speed_mm_s * steps_per_mm_a;
        a_started = STEPPER_move_steps(motor_a, steps_a, speed_steps, motor_a_callback);
        if (!a_started)
            printf("Failed to start motor A\n");
    }
    else
    {
        motor_a_done = true; // No movement needed for this motor
    }

    if (steps_b != 0 && motor_b >= 0)
    {
        float speed_steps = speed_mm_s * steps_per_mm_b;
        b_started = STEPPER_move_steps(motor_b, steps_b, speed_steps, motor_b_callback);
        if (!b_started)
            printf("Failed to start motor B\n");
    }
    else
    {
        motor_b_done = true; // No movement needed for this motor
    }

    if (steps_z != 0 && motor_z >= 0)
    {
        float speed_steps = speed_mm_s * steps_per_mm_z;
        z_started = STEPPER_move_steps(motor_z, steps_z, speed_steps, motor_z_callback);
        if (!z_started)
            printf("Failed to start motor Z\n");
    }
    else
    {
        motor_z_done = true; // No movement needed for this motor
    }

    // Set current state if any motor is moving
    if (a_started && b_started && z_started)
    {
        if (!motor_a_done || !motor_b_done || !motor_z_done)
        {
            current_state = COREXY_STATE_MOVING;
        }
        else
        {
            // All motors are already done (no steps to move)
            if (move_callback != NULL)
            {
                corexy_callback_t callback = move_callback;
                move_callback = NULL;
                callback(true);
            }
            return true;
        }
    }
    else
    {
        // Something failed, stop any motors that did start
        CoreXY_EmergencyStop();
        return false;
    }

    return true;
}

// Move by relative distances
bool CoreXY_MoveBy(float x, float y, float z, float feedrate, corexy_callback_t callback)
{
    float new_x = current_position[0] + x;
    float new_y = current_position[1] + y;
    float new_z = current_position[2] + z;

    return CoreXY_MoveTo(new_x, new_y, new_z, feedrate, callback);
}

// Process a G-code command
bool CoreXY_ProcessGCodeCommand(parser_command_t *cmd)
{
    if (cmd == NULL)
        return false;

    switch (cmd->code)
    {
    case 0: // G0 - Rapid move
    case 1: // G1 - Linear move
    {
        float x = current_position[0];
        float y = current_position[1];
        float z = current_position[2];
        float feedrate = default_feedrate;
        bool has_movement = false;

        // Parse parameters
        for (int i = 0; i < cmd->num_params; i++)
        {
            switch (cmd->params[i].letter)
            {
            case 'X':
                x = absolute_positioning ? cmd->params[i].value : (current_position[0] + cmd->params[i].value);
                has_movement = true;
                break;
            case 'Y':
                y = absolute_positioning ? cmd->params[i].value : (current_position[1] + cmd->params[i].value);
                has_movement = true;
                break;
            case 'Z':
                z = absolute_positioning ? cmd->params[i].value : (current_position[2] + cmd->params[i].value);
                has_movement = true;
                break;
            case 'F':
                feedrate = cmd->params[i].value;
                default_feedrate = feedrate;
                break;
            }
        }

        if (has_movement)
        {
            printf("Moving to X:%.2f Y:%.2f Z:%.2f at F:%.2f mm/min\n", x, y, z, feedrate);
            float speed_mm_s = feedrate / 60.0f;
            return CoreXY_MoveTo(x, y, z, speed_mm_s, g_code_move_complete_callback);
        }
        else
        {
            printf("No movement parameters specified\n");
            return true;
        }
    }
    break;
        // ... other G-code handlers ...
    }

    return true; // Fixed: Added default return value
}

// Fixed: Moved callback function outside the switch statement and fixed its closing brace
static void g_code_move_complete_callback(bool success)
{
    float x, y, z;
    CoreXY_GetPosition(&x, &y, &z);

    printf("G-code movement %s at X=%.2f Y=%.2f Z=%.2f\n",
           success ? "complete" : "failed", x, y, z);
}

// Home specific axes
bool CoreXY_Home(bool home_x, bool home_y, bool home_z, corexy_callback_t callback)
{
    // Check if system is ready for homing
    if (current_state != COREXY_STATE_IDLE)
    {
        printf("Error: System not ready for homing (state: %d)\n", current_state);
        return false;
    }

    move_callback = callback;
    homing_in_progress = home_x || home_y; // Whether we need to continue with XY after Z

    printf("Homing");
    if (home_z)
        printf(" Z");
    if (home_x)
        printf(" X");
    if (home_y)
        printf(" Y");
    printf(" axes...\n");

    // Always home Z first for safety (to avoid collisions)
    if (home_z)
    {
        current_state = COREXY_STATE_HOMING_Z;
        if (!STEPPER_home(motor_z, motor_z_callback))
        {
            printf("Error: Failed to start Z homing\n");
            current_state = COREXY_STATE_ERROR;
            if (move_callback != NULL)
            {
                move_callback(false);
                move_callback = NULL;
            }
            return false;
        }
    }
    else if (home_x)
    {
        // If not homing Z, start with X
        current_state = COREXY_STATE_HOMING_X;
        if (!STEPPER_home(motor_a, motor_a_callback))
        {
            printf("Error: Failed to start X homing\n");
            current_state = COREXY_STATE_ERROR;
            if (move_callback != NULL)
            {
                move_callback(false);
                move_callback = NULL;
            }
            return false;
        }
    }
    else if (home_y)
    {
        // If only homing Y
        current_state = COREXY_STATE_HOMING_Y;
        if (!STEPPER_home(motor_b, motor_b_callback))
        {
            printf("Error: Failed to start Y homing\n");
            current_state = COREXY_STATE_ERROR;
            if (move_callback != NULL)
            {
                move_callback(false);
                move_callback = NULL;
            }
            return false;
        }
    }
    else
    {
        // Nothing to home!
        if (move_callback != NULL)
        {
            move_callback(true);
            move_callback = NULL;
        }
        return true;
    }

    return true;
}

// Emergency stop
void CoreXY_EmergencyStop(void)
{
    printf("EMERGENCY STOP ACTIVATED!\n");

    // Stop all motors immediately - including a direct check for motor 0
    if (motor_a >= 0)
    {
        printf("Stopping motor A (ID: %d)\n", motor_a);
        STEPPER_stop(motor_a);
    }

    if (motor_b >= 0)
    {
        printf("Stopping motor B (ID: %d)\n", motor_b);
        STEPPER_stop(motor_b);
    }

    if (motor_z >= 0)
    {
        printf("Stopping motor Z (ID: %d)\n", motor_z);
        STEPPER_stop(motor_z);
    }

    // Extra safety - stop any motor that might be running
    for (int i = 0; i < STEPPER_COUNT; i++)
    {
        STEPPER_stop(i);
    }

    // Reset state machine
    current_state = COREXY_STATE_IDLE;

    // Execute callback with failure if one is registered
    if (move_callback != NULL)
    {
        corexy_callback_t callback = move_callback;
        move_callback = NULL;
        callback(false);
    }

    printf("Emergency stop complete\n");
}

// Get current position
void CoreXY_GetPosition(float *x, float *y, float *z)
{
    if (x)
        *x = current_position[0];
    if (y)
        *y = current_position[1];
    if (z)
        *z = current_position[2];
}

// Set absolute positioning mode
void CoreXY_SetAbsolutePositioning(void)
{
    absolute_positioning = true;
    printf("Set to absolute positioning mode (G90)\n");
}

// Set relative positioning mode
void CoreXY_SetRelativePositioning(void)
{
    absolute_positioning = false;
    printf("Set to relative positioning mode (G91)\n");
}

// Get current positioning mode
bool CoreXY_IsAbsolutePositioning(void)
{
    return absolute_positioning;
}

// Set machine dimensions
void CoreXY_SetMachineDimensions(float max_x_mm, float max_y_mm, float max_z_mm)
{
    machine_dimensions.max_x_mm = max_x_mm;
    machine_dimensions.max_y_mm = max_y_mm;
    machine_dimensions.max_z_mm = max_z_mm;

    printf("Machine dimensions set to X:%.1f Y:%.1f Z:%.1f mm\n",
           max_x_mm, max_y_mm, max_z_mm);
}

// Get machine dimensions
corexy_dimensions_t CoreXY_GetMachineDimensions(void)
{
    return machine_dimensions;
}

// Get current state
corexy_state_t CoreXY_GetState(void)
{
    return current_state;
}

// Motor movement callbacks
static void motor_a_callback(stepper_id_t id, bool success)
{
    motor_a_done = true;
    if (!success)
    {
        printf("Motor A move failed\n");
        CoreXY_EmergencyStop();
    }
}

static void motor_b_callback(stepper_id_t id, bool success)
{
    motor_b_done = true;
    if (!success)
    {
        printf("Motor B move failed\n");
        CoreXY_EmergencyStop();
    }
}

static void motor_z_callback(stepper_id_t id, bool success)
{
    motor_z_done = true;
    if (!success)
    {
        printf("Motor Z move failed\n");
        CoreXY_EmergencyStop();
    }
}

// Homing completed callback
static void homing_completed_callback(bool success)
{
    homing_in_progress = false;

    if (success)
    {
        printf("Homing completed successfully\n");
    }
    else
    {
        printf("Homing failed\n");
    }

    if (move_callback != NULL)
    {
        corexy_callback_t callback = move_callback;
        move_callback = NULL;
        callback(success);
    }
}

// Configure endstops for the CoreXY system
#if ENDSTOP_ENABLED
void CoreXY_ConfigureEndstops(
    GPIO_TypeDef *x_min_port, uint16_t x_min_pin, uint8_t x_min_trigger_level,
    GPIO_TypeDef *y_min_port, uint16_t y_min_pin, uint8_t y_min_trigger_level,
    GPIO_TypeDef *z_min_port, uint16_t z_min_pin, uint8_t z_min_trigger_level)
{
    printf("Configuring CoreXY endstops...\n");

    // X axis endstops (connected to motor_a)
    if (x_min_port != NULL && motor_a >= 0)
    {
        STEPPER_config_endstop(motor_a, ENDSTOP_MIN, x_min_port, x_min_pin, x_min_trigger_level);
        printf("X min endstop configured\n");
    }

    // Y axis endstops (connected to motor_b)
    if (y_min_port != NULL && motor_b >= 0)
    {
        STEPPER_config_endstop(motor_b, ENDSTOP_MIN, y_min_port, y_min_pin, y_min_trigger_level);
        printf("Y min endstop configured\n");
    }

    // Z axis endstops
    if (z_min_port != NULL && motor_z >= 0)
    {
        STEPPER_config_endstop(motor_z, ENDSTOP_MIN, z_min_port, z_min_pin, z_min_trigger_level);
        printf("Z min endstop configured\n");
    }
}
#endif