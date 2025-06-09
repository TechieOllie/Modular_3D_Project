/**
 *******************************************************************************
 * @file 	corexy.c
 * @author 	naej, your name
 * @date 	Current Date
 * @brief	CoreXY kinematics implementation for CNC control
 *******************************************************************************
 */

#include "config.h"
#include "corexy.h"
#include "stepper_motor.h"
/* System includes - correct order is important */
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// Current position tracking
static float current_position[3] = {0.0f, 0.0f, 0.0f}; // X, Y, Z
static float default_feedrate = 500.0f;                // Default feedrate in mm/min (increased for better performance)
static bool absolute_positioning_mode = true;          // Default to absolute positioning (G90)

// Stepper motor IDs
static stepper_id_t motor_a; // First CoreXY motor
static stepper_id_t motor_b; // Second CoreXY motor
static stepper_id_t motor_z; // Z-axis motor

// Machine dimensions
static CoreXY_Dimensions_t machine_dimensions = {
    .max_x_mm = 200.0f, // Default X dimension (200mm)
    .max_y_mm = 200.0f, // Default Y dimension (200mm)
    .max_z_mm = 200.0f  // Default Z dimension (200mm)
};

// Internal IDs for stepper motors
static stepper_id_t x_stepper_id = -1;
static stepper_id_t y_stepper_id = -1;
static stepper_id_t z_stepper_id = -1;

// Convert X,Y coordinates to A,B motor movements (CoreXY kinematics)
static void xy_to_ab(float x, float y, float *a, float *b)
{
    *a = x + y; // Motor A = X + Y
    *b = x - y; // Motor B = X - Y
}

void CoreXY_Init(void)
{
    // Initialize the stepper motor module
    STEPPER_init();

    // Set default positioning mode to absolute (G90)
    absolute_positioning_mode = true;

    // Configure with NEMA 17 specific settings
    // NEMA 17 typically has 200 steps per revolution (1.8° per step)
    // For a GT2 pulley with 20 teeth (2mm pitch), one revolution = 40mm
    // So steps_per_mm = 200/40 = 5 steps/mm
    float steps_per_mm = 80.0f;
    uint8_t microstepping = 16;   // Using 1/16 microstepping for smoother motion
    float current_limit_a = 1.5f; // Set to 1.2A (typical for NEMA17)

    // Add the motors to the system
    motor_a = STEPPER_add(GPIOB, GPIO_PIN_4, GPIOA, GPIO_PIN_6,
                          TIMER3_ID, TIM_CHANNEL_1, steps_per_mm, microstepping, current_limit_a);

    //motor_b = STEPPER_add(GPIOA, GPIO_PIN_9, GPIOA, GPIO_PIN_10,
      //                    TIMER2_ID, TIM_CHANNEL_1, steps_per_mm, microstepping, current_limit_a);

    //motor_z = STEPPER_add(GPIOA, GPIO_PIN_11, GPIOA, GPIO_PIN_12,
       //                   TIMER4_ID, TIM_CHANNEL_1, steps_per_mm, microstepping, current_limit_a);

    // Store the motor IDs for later use
    x_stepper_id = motor_a;
    y_stepper_id = motor_b;
    z_stepper_id = motor_z;

    // Set default machine dimensions
    CoreXY_SetMachineDimensions(200.0f, 200.0f, 200.0f);

    printf("CoreXY system initialized with NEMA 17 stepper motors\n");
    printf("Using microstepping: 1/%d\n", microstepping);
}

// Set the machine dimensions
void CoreXY_SetMachineDimensions(float max_x_mm, float max_y_mm, float max_z_mm)
{
    machine_dimensions.max_x_mm = max_x_mm;
    machine_dimensions.max_y_mm = max_y_mm;
    machine_dimensions.max_z_mm = max_z_mm;

    printf("Machine dimensions set to: X=%.1f, Y=%.1f, Z=%.1f mm\n",
           max_x_mm, max_y_mm, max_z_mm);
}

// Get the machine dimensions
CoreXY_Dimensions_t CoreXY_GetMachineDimensions(void)
{
    return machine_dimensions;
}

// Move to the specified coordinates
void CoreXY_MoveTo(float x, float y, float z, float feedrate)
{
    float a_pos, b_pos;

    // Convert target X,Y to A,B coordinates
    xy_to_ab(x, y, &a_pos, &b_pos);

    // Convert feedrate from mm/min to mm/sec
    float speed_mm_s = feedrate / 60.0f;

    // Move the motors to their targets
    STEPPER_move_to_mm(motor_a, a_pos, speed_mm_s);
    STEPPER_move_to_mm(motor_b, b_pos, speed_mm_s);
    STEPPER_move_to_mm(motor_z, z, speed_mm_s);

    // Update position tracking
    current_position[0] = x;
    current_position[1] = y;
    current_position[2] = z;
}

// Handle positioning mode in Move function if needed
void CoreXY_Move(float x, float y, float z, float feedrate)
{
    // For the Move function, we always treat inputs as relative offsets
    // This ensures compatibility with existing code that calls Move
    CoreXY_MoveTo(
        current_position[0] + x,
        current_position[1] + y,
        current_position[2] + z,
        feedrate);
}

// Process a G-code command
void CoreXY_ProcessGCodeCommand(GCodeCommand *cmd)
{
    switch (cmd->g_code)
    {
    case 0: // G0: Rapid positioning
    case 1: // G1: Linear move
    {
        float x = current_position[0];
        float y = current_position[1];
        float z = current_position[2];
        float f = default_feedrate;

        // Update coordinates if specified
        if (cmd->has_x)
        {
            // Apply X based on positioning mode
            x = absolute_positioning_mode ? cmd->x : current_position[0] + cmd->x;
        }

        if (cmd->has_y)
        {
            // Apply Y based on positioning mode
            y = absolute_positioning_mode ? cmd->y : current_position[1] + cmd->y;
        }

        if (cmd->has_z)
        {
            // Apply Z based on positioning mode
            z = absolute_positioning_mode ? cmd->z : current_position[2] + cmd->z;
        }

        if (cmd->has_f)
        {
            f = cmd->feedrate;
        }

        CoreXY_MoveTo(x, y, z, f);

        printf("Moved to X:%.2f Y:%.2f Z:%.2f F:%.2f (%s)\n",
               x, y, z, f,
               absolute_positioning_mode ? "absolute" : "relative");
        break;
    }

    case 28: // G28: Home
    {
        // Check for axis-specific homing parameters (X, Y, Z)
        bool home_x = true;
        bool home_y = true;
        bool home_z = true;

        // If any axis is specified, only home those axes
        bool specific_axes = false;

        for (int i = 0; i < cmd->numParams; i++)
        {
            if (cmd->params[i].letter == 'X')
            {
                specific_axes = true;
            }
            else if (cmd->params[i].letter == 'Y')
            {
                specific_axes = true;
            }
            else if (cmd->params[i].letter == 'Z')
            {
                specific_axes = true;
            }
        }

        // If specific axes were mentioned, start by assuming we don't home any
        if (specific_axes)
        {
            home_x = false;
            home_y = false;
            home_z = false;

            // Then enable only the ones specified
            for (int i = 0; i < cmd->numParams; i++)
            {
                if (cmd->params[i].letter == 'X')
                {
                    home_x = true;
                }
                else if (cmd->params[i].letter == 'Y')
                {
                    home_y = true;
                }
                else if (cmd->params[i].letter == 'Z')
                {
                    home_z = true;
                }
            }
        }

        // Z should always be homed first for safety
        if (home_z)
        {
            printf("Homing Z axis...\n");
#if ENDSTOP_ENABLED
            if (z_stepper_id >= 0)
            {
                STEPPER_home_with_endstop(z_stepper_id, 5.0f, machine_dimensions.max_z_mm);
            }
#else
            if (z_stepper_id >= 0)
            {
                STEPPER_home(z_stepper_id);
            }
#endif
            current_position[2] = 0.0f;
        }

        // Then home X
        if (home_x)
        {
            printf("Homing X axis...\n");
#if ENDSTOP_ENABLED
            if (x_stepper_id >= 0)
            {
                STEPPER_home_with_endstop(x_stepper_id, 5.0f, machine_dimensions.max_x_mm);
            }
#else
            if (x_stepper_id >= 0)
            {
                STEPPER_home(x_stepper_id);
            }
#endif
            current_position[0] = 0.0f;
        }

        // Then home Y
        if (home_y)
        {
            printf("Homing Y axis...\n");
#if ENDSTOP_ENABLED
            if (y_stepper_id >= 0)
            {
                STEPPER_home_with_endstop(y_stepper_id, 5.0f, machine_dimensions.max_y_mm);
            }
#else
            if (y_stepper_id >= 0)
            {
                STEPPER_home(y_stepper_id);
            }
#endif
            current_position[1] = 0.0f;
        }

        printf("Homing complete\n");
        break;
    }

    case 90: // G90: Absolute positioning
        absolute_positioning_mode = true;
        printf("Using absolute positioning\n");
        break;

    case 91: // G91: Relative positioning
        absolute_positioning_mode = false;
        printf("Using relative positioning\n");
        break;

    default:
        printf("Unhandled G-code: G%d\n", cmd->g_code);
    }
}

// Home the machine (simple version used when not called via G28)
void CoreXY_Home(void)
{
    // Create a G28 command with no parameters to home all axes
    GCodeCommand cmd = {0};
    cmd.g_code = 28;

    // Process it using our enhanced G28 handler
    CoreXY_ProcessGCodeCommand(&cmd);
}

#if ENDSTOP_ENABLED
// Configure endstops for the CoreXY system
void CoreXY_ConfigureEndstops(
    GPIO_TypeDef *x_min_port, uint16_t x_min_pin, uint8_t x_min_trigger_level,
    GPIO_TypeDef *x_max_port, uint16_t x_max_pin, uint8_t x_max_trigger_level,
    GPIO_TypeDef *y_min_port, uint16_t y_min_pin, uint8_t y_min_trigger_level,
    GPIO_TypeDef *y_max_port, uint16_t y_max_pin, uint8_t y_max_trigger_level,
    GPIO_TypeDef *z_min_port, uint16_t z_min_pin, uint8_t z_min_trigger_level,
    GPIO_TypeDef *z_max_port, uint16_t z_max_pin, uint8_t z_max_trigger_level)
{
    printf("Configuring CoreXY endstops...\n");

    // Configure X-axis endstops
    if (x_min_port != NULL && x_stepper_id >= 0)
    {
        STEPPER_config_endstop(x_stepper_id, ENDSTOP_MIN, x_min_port, x_min_pin, x_min_trigger_level);
    }

    if (x_max_port != NULL && x_stepper_id >= 0)
    {
        STEPPER_config_endstop(x_stepper_id, ENDSTOP_MAX, x_max_port, x_max_pin, x_max_trigger_level);
    }

    // Configure Y-axis endstops
    if (y_min_port != NULL && y_stepper_id >= 0)
    {
        STEPPER_config_endstop(y_stepper_id, ENDSTOP_MIN, y_min_port, y_min_pin, y_min_trigger_level);
    }

    if (y_max_port != NULL && y_stepper_id >= 0)
    {
        STEPPER_config_endstop(y_stepper_id, ENDSTOP_MAX, y_max_port, y_max_pin, y_max_trigger_level);
    }

    // Configure Z-axis endstops
    if (z_min_port != NULL && z_stepper_id >= 0)
    {
        STEPPER_config_endstop(z_stepper_id, ENDSTOP_MIN, z_min_port, z_min_pin, z_min_trigger_level);
    }

    if (z_max_port != NULL && z_stepper_id >= 0)
    {
        STEPPER_config_endstop(z_stepper_id, ENDSTOP_MAX, z_max_port, z_max_pin, z_max_trigger_level);
    }

    printf("CoreXY endstops configured successfully\n");
}

// Home all axes with configured endstops
bool CoreXY_HomeAll(float homing_speed_mm_s)
{
    bool success = true;
    printf("Homing all CoreXY axes...\n");

    // Home Z first for safety (to avoid collision with the bed)
    if (z_stepper_id >= 0)
    {
        printf("Homing Z axis...\n");
        success &= STEPPER_home_with_endstop(z_stepper_id, homing_speed_mm_s, machine_dimensions.max_z_mm);
    }

    // Home X and Y
    if (x_stepper_id >= 0)
    {
        printf("Homing X axis...\n");
        success &= STEPPER_home_with_endstop(x_stepper_id, homing_speed_mm_s, machine_dimensions.max_x_mm);
    }

    if (y_stepper_id >= 0)
    {
        printf("Homing Y axis...\n");
        success &= STEPPER_home_with_endstop(y_stepper_id, homing_speed_mm_s, machine_dimensions.max_y_mm);
    }

    if (success)
    {
        printf("All axes homed successfully\n");
    }
    else
    {
        printf("WARNING: Homing failed for one or more axes\n");
    }

    return success;
}
#endif

// Get current position
void CoreXY_GetPosition(float *x, float *y, float *z)
{
    *x = current_position[0];
    *y = current_position[1];
    *z = current_position[2];
}

// Emergency stop
void CoreXY_EmergencyStop(void)
{
    STEPPER_stop(motor_a);
    STEPPER_stop(motor_b);
    STEPPER_stop(motor_z);
    printf("EMERGENCY STOP\n");
}

// Set absolute positioning mode
void CoreXY_SetAbsolutePositioning(void)
{
    absolute_positioning_mode = true;
}

// Set relative positioning mode
void CoreXY_SetRelativePositioning(void)
{
    absolute_positioning_mode = false;
}

// Get current positioning mode
bool CoreXY_IsAbsolutePositioning(void)
{
    return absolute_positioning_mode;
}
