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

// Stepper motor IDs
static stepper_id_t motor_a; // First CoreXY motor
static stepper_id_t motor_b; // Second CoreXY motor
static stepper_id_t motor_z; // Z-axis motor

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

    // Configure with NEMA 17 specific settings
    // NEMA 17 typically has 200 steps per revolution (1.8° per step)
    // For a GT2 pulley with 20 teeth (2mm pitch), one revolution = 40mm
    // So steps_per_mm = 200/40 = 5 steps/mm
    float steps_per_mm = 5.0f;
    uint8_t microstepping = 16;   // Using 1/16 microstepping for smoother motion
    float current_limit_a = 1.2f; // Set to 1.2A (typical for NEMA17)

    // Add the motors to the system
    motor_a = STEPPER_add(GPIOB, GPIO_PIN_4, GPIOA, GPIO_PIN_6,
                          TIMER3_ID, TIM_CHANNEL_1, steps_per_mm, microstepping, current_limit_a);

    motor_b = STEPPER_add(GPIOA, GPIO_PIN_9, GPIOA, GPIO_PIN_10,
                          TIMER2_ID, TIM_CHANNEL_1, steps_per_mm, microstepping, current_limit_a);

    motor_z = STEPPER_add(GPIOA, GPIO_PIN_11, GPIOA, GPIO_PIN_12,
                          TIMER4_ID, TIM_CHANNEL_1, steps_per_mm, microstepping, current_limit_a);

    printf("CoreXY system initialized with NEMA 17 stepper motors\n");
    printf("Using microstepping: 1/%d\n", microstepping);
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

// Move by the specified amounts
void CoreXY_Move(float x, float y, float z, float feedrate)
{
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
            x = cmd->x;
        if (cmd->has_y)
            y = cmd->y;
        if (cmd->has_z)
            z = cmd->z;
        if (cmd->has_f)
            f = cmd->feedrate;

        CoreXY_MoveTo(x, y, z, f);

        printf("Moved to X:%.2f Y:%.2f Z:%.2f F:%.2f\n", x, y, z, f);
        break;
    }

    case 28: // G28: Home
        CoreXY_Home();
        printf("Homed all axes\n");
        break;

    case 90: // G90: Absolute positioning
        // Already using absolute positioning
        printf("Using absolute positioning\n");
        break;

    case 91: // G91: Relative positioning
        // Would need to implement relative positioning mode
        printf("Relative positioning not implemented\n");
        break;

    default:
        printf("Unhandled G-code: G%d\n", cmd->g_code);
    }
}

// Home the machine
void CoreXY_Home(void)
{
    // Home each axis
    STEPPER_home(motor_z);
    STEPPER_home(motor_b);
    STEPPER_home(motor_a);

    // Reset position tracking
    current_position[0] = 0.0f;
    current_position[1] = 0.0f;
    current_position[2] = 0.0f;
}

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
