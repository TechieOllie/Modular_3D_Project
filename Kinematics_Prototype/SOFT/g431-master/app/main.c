/**
 *******************************************************************************
 * @file 	main.c
 * @author 	Ol, naej, Fabs
 * @date 	Current Date
 * @brief	Main application for CNC control system with cartesian kinematics
 *******************************************************************************
 */

#include "config.h"
#include "stm32g4_sys.h"
#include "stm32g4_systick.h"
#include "stm32g4_gpio.h"
#include "stm32g4_uart.h"
#include "stm32g4_utils.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include "stm32g4_adc.h"
#include "stm32g4_timer.h"
#include "parser.h"
#include <string.h>
#include "stepper_motor.h"
#include "kinematics.h"
#include "limit_switches.h"
#include "uart_commands.h"
#include "sd_gcode_reader.h"
#include "gcode_move_buffer.h"

#define BLINK_DELAY 100 // ms
#define MAX_CMD_LENGTH 128

// Motor ID constants
#define X_MOTOR_ID 0
#define Y_MOTOR_ID 1

// UART receive buffer
static uint8_t uart_rx_buffer[1];
static volatile bool uart_error = false;

void setup_limit_switches(void)
{
    printf("Initializing limit switches...\n");

    // Initialize limit switches system
    if (!limit_switches_init())
    {
        printf("ERROR: Failed to initialize limit switches system!\n");
        return;
    }

    // Configure limit switches for each axis
    // X-axis limits on GPIOB pins 0 (MIN) and 1 (MAX)
    limit_switch_configure(AXIS_X, LIMIT_MIN, GPIOB, GPIO_PIN_0, true); // Active low
    limit_switch_configure(AXIS_X, LIMIT_MAX, GPIOB, GPIO_PIN_1, true); // Active low

    // Y-axis limits on GPIOB pins 2 (MIN) and 3 (MAX)
    limit_switch_configure(AXIS_Y, LIMIT_MIN, GPIOB, GPIO_PIN_2, true); // Active low
    limit_switch_configure(AXIS_Y, LIMIT_MAX, GPIOB, GPIO_PIN_3, true); // Active low

    // Start with limit switches disabled for testing
    limit_switch_enable(AXIS_X, LIMIT_MIN, false);
    limit_switch_enable(AXIS_X, LIMIT_MAX, false);
    limit_switch_enable(AXIS_Y, LIMIT_MIN, false);
    limit_switch_enable(AXIS_Y, LIMIT_MAX, false);

    printf("Limit switches configured but disabled for testing\n");
}

void setup_stepper_motors(void)
{
    printf("Setting up stepper motors with acceleration profiles...\n");

    // Initialize stepper motor system
    if (!STEPPER_MOTOR_system_init())
    {
        printf("ERROR: Failed to initialize stepper motor system!\n");
        return;
    }

    // Initialize motors with correct pin assignments using new interface
    // Y-axis motor using pins PA10(PUL), PA9(DIR)
    printf("Initializing Y motor: PA10(PUL), PA9(DIR)\n");
    STEPPER_MOTOR_configure(Y_MOTOR_ID, GPIOA, 10, GPIOA, 9, NULL, 0, 16);

    // X-axis motor using pins PA12(PUL), PB0(DIR) - CORRECTED
    printf("Initializing X motor: PA12(PUL), PB0(DIR)\n");
    STEPPER_MOTOR_configure(X_MOTOR_ID, GPIOA, 12, GPIOB, 0, NULL, 0, 16);

    // Enable motors
    STEPPER_MOTOR_enable(X_MOTOR_ID, true);
    STEPPER_MOTOR_enable(Y_MOTOR_ID, true);

    // Associate motors with axes for limit switch checking
    STEPPER_MOTOR_set_axis(X_MOTOR_ID, AXIS_X);
    STEPPER_MOTOR_set_axis(Y_MOTOR_ID, AXIS_Y);

    // Disable limit checking for initial testing
    STEPPER_MOTOR_enable_limit_check(X_MOTOR_ID, false);
    STEPPER_MOTOR_enable_limit_check(Y_MOTOR_ID, false);

    // Set acceleration parameters for smooth movement (6-12kHz range)
    STEPPER_MOTOR_set_acceleration(X_MOTOR_ID, 8000, 10000); // 8000 steps/sec², max 10kHz
    STEPPER_MOTOR_set_acceleration(Y_MOTOR_ID, 8000, 10000); // 8000 steps/sec², max 10kHz

    // Enable acceleration profiles
    STEPPER_MOTOR_enable_acceleration(X_MOTOR_ID, true);
    STEPPER_MOTOR_enable_acceleration(Y_MOTOR_ID, true);

    printf("Stepper motors initialized with acceleration profiles and 6-12kHz operation\n");
}

void setup_kinematics(void)
{
    // Configure machine parameters for cartesian flying gantry - 50x50mm work area
    machine_config_t config = {
        .steps_per_mm_x = 80.0f,      // 1.8° stepper, 20-tooth pulley, GT2 belt
        .steps_per_mm_y = 80.0f,      // Same for Y axis
        .max_velocity_x = 50.0f,      // 50 mm/s maximum
        .max_velocity_y = 50.0f,      // 50 mm/s maximum
        .max_acceleration_x = 100.0f, // 100 mm/s² acceleration
        .max_acceleration_y = 100.0f, // 100 mm/s² acceleration
        .x_max = 210.0f,              //  working area in X
        .y_max = 170.0f,              //  working area in Y
        .x_motor_id = X_MOTOR_ID,
        .y_motor_id = Y_MOTOR_ID};

    if (!kinematics_init(&config))
    {
        printf("ERROR: Failed to initialize kinematics!\n");
        return;
    }

    printf("Kinematics system initialized successfully - 50x50mm work area\n");
}

void test_50x50_kinematics(void)
{
    printf("\n========== TESTING 50x50 KINEMATICS WITH ACCELERATION ==========\n");

    // Debug motor configurations first
    printf("=== Motor Configuration Debug ===\n");
    STEPPER_MOTOR_debug_config(X_MOTOR_ID);
    STEPPER_MOTOR_debug_config(Y_MOTOR_ID);

    // Test individual motor movements with acceleration
    printf("=== Testing Accelerated Motor Movements ===\n");

    // Test X motor movement with acceleration
    printf("Testing X motor: 1500 steps with acceleration to 9000 Hz\n");
    if (STEPPER_MOTOR_move_accel(X_MOTOR_ID, 1500, 9000, MOTOR_DIR_CLOCKWISE))
    {
        uint32_t start_time = HAL_GetTick();
        uint32_t last_progress = 0;
        uint32_t last_completed = 0;
        uint32_t stuck_counter = 0;

        while (STEPPER_MOTOR_get_state(X_MOTOR_ID) != MOTOR_STATE_IDLE)
        {
            STEPPER_MOTOR_update();

            uint32_t completed = STEPPER_MOTOR_get_completed_steps(X_MOTOR_ID);

            // Check if motor is stuck (not progressing)
            if (completed == last_completed)
            {
                stuck_counter++;
                if (stuck_counter > 100) // 1 second of no progress
                {
                    printf("X motor appears stuck at %lu steps, forcing stop\n", completed);
                    STEPPER_MOTOR_stop(X_MOTOR_ID);
                    break;
                }
            }
            else
            {
                stuck_counter = 0;
                last_completed = completed;
            }

            // Print progress every 1000ms
            if (HAL_GetTick() - last_progress > 1000)
            {
                last_progress = HAL_GetTick();
                uint32_t frequency = STEPPER_MOTOR_get_current_frequency(X_MOTOR_ID);
                stepper_motor_state_t state = STEPPER_MOTOR_get_state(X_MOTOR_ID);

                printf("X: %lu/1500 steps (%.1f%%), %lu Hz, state %d\n",
                       completed, (float)completed * 100.0f / 1500.0f, frequency, state);

                // Force completion check if we've reached target steps
                if (completed >= 1500)
                {
                    printf("X motor reached target steps, forcing stop\n");
                    STEPPER_MOTOR_stop(X_MOTOR_ID);
                    break;
                }
            }

            if (HAL_GetTick() - start_time > 20000) // 20s timeout
            {
                printf("X motor timeout after %lu ms! Steps: %lu\n", HAL_GetTick() - start_time, completed);
                STEPPER_MOTOR_stop(X_MOTOR_ID);
                break;
            }
            HAL_Delay(10);
        }

        // Final status
        uint32_t final_steps = STEPPER_MOTOR_get_completed_steps(X_MOTOR_ID);
        stepper_motor_state_t final_state = STEPPER_MOTOR_get_state(X_MOTOR_ID);
        printf("X motor test completed - Final steps: %lu, State: %d\n", final_steps, final_state);
    }
    else
    {
        printf("ERROR: Failed to start X motor accelerated movement!\n");
    }

    // Ensure X motor is fully stopped
    STEPPER_MOTOR_stop(X_MOTOR_ID);
    HAL_Delay(1000);

    // Test Y motor movement with acceleration
    printf("Testing Y motor: 1500 steps with acceleration to 8500 Hz\n");
    if (STEPPER_MOTOR_move_accel(Y_MOTOR_ID, 1500, 8500, MOTOR_DIR_CLOCKWISE))
    {
        uint32_t start_time = HAL_GetTick();
        uint32_t last_progress = 0;
        uint32_t last_completed = 0;
        uint32_t stuck_counter = 0;

        while (STEPPER_MOTOR_get_state(Y_MOTOR_ID) != MOTOR_STATE_IDLE)
        {
            STEPPER_MOTOR_update();

            uint32_t completed = STEPPER_MOTOR_get_completed_steps(Y_MOTOR_ID);

            // Check if motor is stuck (not progressing)
            if (completed == last_completed)
            {
                stuck_counter++;
                if (stuck_counter > 100) // 1 second of no progress
                {
                    printf("Y motor appears stuck at %lu steps, forcing stop\n", completed);
                    STEPPER_MOTOR_stop(Y_MOTOR_ID);
                    break;
                }
            }
            else
            {
                stuck_counter = 0;
                last_completed = completed;
            }

            // Print progress every 1000ms
            if (HAL_GetTick() - last_progress > 1000)
            {
                last_progress = HAL_GetTick();
                uint32_t frequency = STEPPER_MOTOR_get_current_frequency(Y_MOTOR_ID);
                stepper_motor_state_t state = STEPPER_MOTOR_get_state(Y_MOTOR_ID);

                printf("Y: %lu/1500 steps (%.1f%%), %lu Hz, state %d\n",
                       completed, (float)completed * 100.0f / 1500.0f, frequency, state);

                // Force completion check if we've reached target steps
                if (completed >= 1500)
                {
                    printf("Y motor reached target steps, forcing stop\n");
                    STEPPER_MOTOR_stop(Y_MOTOR_ID);
                    break;
                }
            }

            if (HAL_GetTick() - start_time > 20000) // 20s timeout
            {
                printf("Y motor timeout after %lu ms! Steps: %lu\n", HAL_GetTick() - start_time, completed);
                STEPPER_MOTOR_stop(Y_MOTOR_ID);
                break;
            }
            HAL_Delay(10);
        }

        // Final status
        uint32_t final_steps = STEPPER_MOTOR_get_completed_steps(Y_MOTOR_ID);
        stepper_motor_state_t final_state = STEPPER_MOTOR_get_state(Y_MOTOR_ID);
        printf("Y motor test completed - Final steps: %lu, State: %d\n", final_steps, final_state);
    }
    else
    {
        printf("ERROR: Failed to start Y motor accelerated movement!\n");
    }

    printf("Accelerated stepper motor test complete\n");

    // Ensure both motors are stopped before returning
    STEPPER_MOTOR_stop(X_MOTOR_ID);
    STEPPER_MOTOR_stop(Y_MOTOR_ID);
    HAL_Delay(1000);

    printf("Both motors confirmed stopped\n");
}

int main(void)
{
    // Initialize system
    HAL_Init();
    SystemClock_Config();
    BSP_GPIO_enable();

    BSP_UART_init(UART2_ID, 115200);
    BSP_SYS_set_std_usart(UART2_ID, UART2_ID, UART2_ID);

    // Setup limit switches first
    setup_limit_switches();

    // Setup stepper motors
    setup_stepper_motors();
    printf("Stepper motors initialized.\n");

    // Setup kinematics system
    setup_kinematics();

    // Initialize SD card G-code systems
    printf("Initializing SD card G-code systems...\n");
    if (sd_gcode_reader_init())
    {
        printf("SD card G-code reader ready\n");
    }
    else
    {
        printf("WARNING: SD card not available\n");
    }

    if (!gcode_move_buffer_init())
    {
        printf("ERROR: Failed to initialize G-code move buffer!\n");
        while (1)
            ; // Stop here if move buffer fails
    }

    // Initialize UART command system
    if (!uart_commands_init(UART2_ID))
    {
        printf("ERROR: Failed to initialize UART command system!\n");
        while (1)
            ; // Stop here if command system fails
    }

    printf("UART command system enabled\n");

    // Debug motor configurations before testing
    printf("=== Initial Motor Debug ===\n");
    STEPPER_MOTOR_debug_config(X_MOTOR_ID);
    STEPPER_MOTOR_debug_config(Y_MOTOR_ID);

    // Wait then run the test
    // HAL_Delay(2000);
    // test_50x50_kinematics();

    // Final safety stop
    printf("\n========== TEST COMPLETE ==========\n");
    STEPPER_MOTOR_emergency_stop_all();

    printf("\n========== ENTERING MAIN LOOP ==========\n");
    printf("Send commands via UART (type 'help' for available commands)...\n");

    // Main loop
    while (1)
    {
        // Static variables for timing
        static uint32_t last_pos_update = 0;
        static uint32_t last_debug = 0;

        // Update systems
        uart_commands_update(); // Process incoming commands
        kinematics_update();
        STEPPER_MOTOR_update();

        // Update parser position periodically
        if (HAL_GetTick() - last_pos_update > 100) // Every 100ms
        {
            last_pos_update = HAL_GetTick();
            position_t current_pos = kinematics_get_position();
            parser_update_position(current_pos);
        }

        HAL_Delay(10);

        // Debug output every 5 seconds
        if (HAL_GetTick() - last_debug > 5000) // Every 5 seconds
        {
            last_debug = HAL_GetTick();
            parser_state_t *parser_state = parser_get_state();
            move_state_t state = kinematics_get_state();
            position_t pos = kinematics_get_position();
            printf("Status - Position: X=%.2f, Y=%.2f, State: %d, Parser Mode: %s, Feedrate: %.1f\n",
                   pos.x, pos.y, state,
                   parser_state->absolute_mode ? "ABS" : "REL",
                   parser_state->feedrate);
        }
    }
}
