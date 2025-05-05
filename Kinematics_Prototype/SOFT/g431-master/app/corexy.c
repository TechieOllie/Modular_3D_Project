/**
 *******************************************************************************
 * @file 	corexy.c
 * @author 	naej, your name
 * @date 	Current Date
 * @brief	CoreXY kinematics implementation for CNC control
 *******************************************************************************
 */
#include "corexy.h"
#include "stepper_motor.h"
#include <stdio.h>

// Current position tracking
static float current_position[3] = {0.0f, 0.0f, 0.0f}; // X, Y, Z
static float default_feedrate = 100.0f; // Default feedrate in mm/min

// Stepper motor IDs
static stepper_id_t motor_a; // First CoreXY motor
static stepper_id_t motor_b; // Second CoreXY motor
static stepper_id_t motor_z; // Z-axis motor

void CoreXY_Init(void) {
    // Initialize the stepper motor module
    STEPPER_init();
    
    // Add the motors with NEMA 17 specific settings
    // NEMA 17 typically has 200 steps per revolution (1.8° per step)
    // For a GT2 pulley with 20 teeth (2mm pitch), one revolution = 40mm
    // So steps_per_mm = 200/40 = 5 steps/mm
    float steps_per_mm = 5.0f;
    
    motor_a = STEPPER_add(GPIOB, GPIO_PIN_4, GPIOA, GPIO_PIN_6, 
                         TIMER3_ID, TIM_CHANNEL_1, steps_per_mm);
    
    //motor_b = STEPPER_add(GPIOA, GPIO_PIN_9, GPIOA, GPIO_PIN_10, 
    //                     TIMER2_ID, TIM_CHANNEL_1, steps_per_mm);
                         
    //motor_z = STEPPER_add(GPIOA, GPIO_PIN_11, GPIOA, GPIO_PIN_12, 
            //             TIMER3_ID, TIM_CHANNEL_1, steps_per_mm);
                         
    printf("CoreXY system initialized with NEMA 17 stepper motors\n");
}

// Convert X,Y coordinates to A,B motor movements (CoreXY kinematics)
static void xy_to_ab(float x, float y, float* a, float* b) {
    *a = x + y;  // Motor A = X + Y
    *b = x - y;  // Motor B = X - Y
}

// Move to the specified coordinates
void CoreXY_MoveTo(float x, float y, float z, float feedrate) {
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
void CoreXY_Move(float x, float y, float z, float feedrate) {
    // Calculate absolute coordinates
    float new_x = current_position[0] + x;
    float new_y = current_position[1] + y;
    float new_z = current_position[2] + z;
    
    // Move to the absolute coordinates
    CoreXY_MoveTo(new_x, new_y, new_z, feedrate);
}

// Process a G-code command
void CoreXY_ProcessGCodeCommand(GCodeCommand* cmd) {
    float x = cmd->has_x ? cmd->x : current_position[0];
    float y = cmd->has_y ? cmd->y : current_position[1];
    float z = cmd->has_z ? cmd->z : current_position[2];
    float f = cmd->has_f ? cmd->feedrate : default_feedrate;
    
    switch (cmd->g_code) {
        case 0: // G0 - Rapid move
            CoreXY_MoveTo(x, y, z, default_feedrate * 3); // 3x faster than normal
            break;
            
        case 1: // G1 - Linear move
            CoreXY_MoveTo(x, y, z, f);
            break;
            
        case 28: // G28 - Home
            CoreXY_Home();
            break;
            
        default:
            printf("Unsupported G-code: G%d\n", cmd->g_code);
            break;
    }
}

void CoreXY_Home(void) {
    // Home Z first for safety (if implemented with limit switches)
    STEPPER_home(motor_z);
    
    // Then home X and Y (via A and B motors)
    STEPPER_home(motor_a);
    STEPPER_home(motor_b);
    
    // Reset position tracking
    current_position[0] = 0.0f;
    current_position[1] = 0.0f;
    current_position[2] = 0.0f;
}

void CoreXY_GetPosition(float* x, float* y, float* z) {
    if (x) *x = current_position[0];
    if (y) *y = current_position[1];
    if (z) *z = current_position[2];
}

void CoreXY_EmergencyStop(void) {
    STEPPER_stop(motor_a);
    STEPPER_stop(motor_b);
    STEPPER_stop(motor_z);
    
    printf("EMERGENCY STOP!\n");
}
