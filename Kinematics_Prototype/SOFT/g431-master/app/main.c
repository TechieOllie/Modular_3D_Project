/**
 *******************************************************************************
 * @file 	main.c
 * @author 	naej, ol, your name
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
#include <stdio.h>
#include "stm32g4_adc.h"
#include "stm32g4_timer.h"
#include "parser.h"
#include <string.h>
#include "stepper_motor.h"
#include "kinematics.h"
#include "limit_switches.h"

UART_HandleTypeDef huart2;

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

    // Enable all configured limit switches
    limit_switch_enable(AXIS_X, LIMIT_MIN, true);
    limit_switch_enable(AXIS_X, LIMIT_MAX, true);
    limit_switch_enable(AXIS_Y, LIMIT_MIN, true);
    limit_switch_enable(AXIS_Y, LIMIT_MAX, true);

    printf("Limit switches configured and enabled\n");
}

void setup_stepper_motors(void)
{
    // Initialize motors with non-conflicting timer channels
    // X-axis motor using Timer1, Channel 1 on pins PA8(PUL), PA10(DIR) with 8 microsteps
    stepper_motor_init(X_MOTOR_ID, GPIOA, GPIO_PIN_8, GPIOA, GPIO_PIN_10, TIMER1_ID, TIM_CHANNEL_1, 8);

    // Y-axis motor using Timer3, Channel 1 on pins PB4(PUL), PB5(DIR) with 8 microsteps
    stepper_motor_init(Y_MOTOR_ID, GPIOB, GPIO_PIN_4, GPIOB, GPIO_PIN_5, TIMER3_ID, TIM_CHANNEL_1, 8);

    // Associate motors with axes for limit switch checking
    stepper_motor_set_axis(X_MOTOR_ID, AXIS_X);
    stepper_motor_set_axis(Y_MOTOR_ID, AXIS_Y);

    // Enable limit checking for both motors
    stepper_motor_enable_limit_check(X_MOTOR_ID, true);
    stepper_motor_enable_limit_check(Y_MOTOR_ID, true);

    printf("Stepper motors initialized with limit switch integration\n");
}

void setup_kinematics(void)
{
    // Configure machine parameters for cartesian flying gantry
    machine_config_t config = {
        .steps_per_mm_x = 80.0f,      // 1.8° stepper, 20-tooth pulley, GT2 belt
        .steps_per_mm_y = 80.0f,      // Same for Y axis
        .max_velocity_x = 50.0f,      // 50 mm/s maximum
        .max_velocity_y = 50.0f,      // 50 mm/s maximum
        .max_acceleration_x = 100.0f, // 100 mm/s² acceleration
        .max_acceleration_y = 100.0f, // 100 mm/s² acceleration
        .x_max = 200.0f,              // 200mm working area in X
        .y_max = 200.0f,              // 200mm working area in Y
        .x_motor_id = X_MOTOR_ID,
        .y_motor_id = Y_MOTOR_ID};

    if (!kinematics_init(&config))
    {
        printf("ERROR: Failed to initialize kinematics!\n");
        return;
    }

    printf("Kinematics system initialized successfully\n");
}

void test_gcode_parser(void)
{
    printf("\n========== TESTING G-CODE PARSER ==========\n");

    // Test basic G-code commands
    const char *test_gcode[] = {
        "G28",             // Home all axes
        "G90",             // Absolute positioning
        "G1 X10 Y5 F1200", // Move to (10,5) at 1200 mm/min
        "G1 X20 F600",     // Move to X=20 at 600 mm/min
        "G1 Y15",          // Move to Y=15
        "G91",             // Relative positioning
        "G1 X-5 Y-5 F800", // Relative move
        "G90",             // Back to absolute
        "G1 X0 Y0 F1500",  // Return to origin
        "M114",            // Get position
        "M400"             // Wait for completion
    };

    uint8_t num_commands = sizeof(test_gcode) / sizeof(test_gcode[0]);

    for (uint8_t i = 0; i < num_commands; i++)
    {
        printf("\nExecuting: %s\n", test_gcode[i]);

        if (!parser_execute_gcode(test_gcode[i]))
        {
            printf("ERROR: Failed to execute G-code: %s\n", test_gcode[i]);
        }

        // Update parser with current position
        position_t current_pos = kinematics_get_position();
        parser_update_position(current_pos);

        HAL_Delay(500); // Brief pause between commands
    }

    printf("\nG-code parser test complete!\n");
}

void test_kinematics_system(void)
{
    printf("\n========== TESTING CARTESIAN KINEMATICS ==========\n");

    // Home the machine
    printf("Homing machine...\n");
    kinematics_home_all();
    HAL_Delay(1000);

    // Test moves
    position_t test_positions[] = {
        {10.0f, 10.0f}, // Move to (10,10)
        {20.0f, 10.0f}, // Move to (20,10) - X only
        {20.0f, 20.0f}, // Move to (20,20) - Y only
        {0.0f, 0.0f}    // Return to origin
    };

    float feedrates[] = {600.0f, 1200.0f, 600.0f, 300.0f}; // mm/min
    uint8_t num_moves = sizeof(test_positions) / sizeof(test_positions[0]);

    for (uint8_t i = 0; i < num_moves; i++)
    {
        printf("\nMove %d: Going to X=%.1f, Y=%.1f at %.0f mm/min\n",
               i + 1, test_positions[i].x, test_positions[i].y, feedrates[i]);

        if (kinematics_move_to(&test_positions[i], feedrates[i]))
        {
            // Wait for move to complete
            uint32_t start_time = HAL_GetTick();
            uint32_t timeout = 30000; // 30 second timeout

            while (kinematics_get_state() != MOVE_STATE_IDLE)
            {
                kinematics_update();
                stepper_motor_update();

                if (HAL_GetTick() - start_time > timeout)
                {
                    printf("Timeout waiting for move to complete!\n");
                    kinematics_stop();
                    break;
                }

                HAL_Delay(10);
            }

            position_t current = kinematics_get_position();
            printf("Move completed - Current position: X=%.2f, Y=%.2f\n",
                   current.x, current.y);
        }
        else
        {
            printf("Failed to start move!\n");
        }

        // Wait between moves
        HAL_Delay(2000);
    }

    // Test relative moves
    printf("\nTesting relative moves...\n");
    position_t relative_moves[] = {
        {5.0f, 0.0f},  // +5mm in X
        {0.0f, 5.0f},  // +5mm in Y
        {-5.0f, -5.0f} // Back diagonally
    };

    for (uint8_t i = 0; i < 3; i++)
    {
        printf("Relative move %d: X%+.1f, Y%+.1f\n",
               i + 1, relative_moves[i].x, relative_moves[i].y);

        if (kinematics_move_relative(&relative_moves[i], 600.0f))
        {
            while (kinematics_get_state() != MOVE_STATE_IDLE)
            {
                kinematics_update();
                stepper_motor_update();
                HAL_Delay(10);
            }

            position_t current = kinematics_get_position();
            printf("Relative move completed - Current position: X=%.2f, Y=%.2f\n",
                   current.x, current.y);
        }

        HAL_Delay(1000);
    }

    printf("\nKinematics test sequence complete!\n");
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

    // Setup stepper motors (now with limit switch integration)
    setup_stepper_motors();
    printf("Stepper motors initialized.\n");

    // Setup kinematics system
    setup_kinematics();

    // Initialize G-code parser
    if (!parser_init())
    {
        printf("ERROR: Failed to initialize G-code parser!\n");
        while (1)
            ; // Stop here if parser fails
    }
    // Enable UART receive interrupt for G-code input
    if (HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 1) != HAL_OK)
    {
        printf("ERROR: Failed to enable UART receive interrupt!\n");
        while (1)
            ; // Stop here if UART receive fails
    }
    printf("UART receive enabled for G-code input\n");

    // Run tests
    HAL_Delay(2000); // Wait a bit before starting tests

    // First test kinematics directly
    test_kinematics_system();

    HAL_Delay(3000);

    // Then test G-code parser
    test_gcode_parser();

    printf("\n========== ENTERING MAIN LOOP ==========\n");
    printf("Send G-code commands via UART...\n");

    // Main loop
    while (1)
    {
        // Check for UART errors and restart if needed
        if (uart_error)
        {
            printf("Recovering from UART error...\n");
            if (HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 1) == HAL_OK)
            {
                uart_error = false;
                printf("UART recovery successful\n");
            }
        }
            static uint32_t last_pos_update = 0;
            if (HAL_GetTick() - last_pos_update > 100) // Every 100ms
            {
                last_pos_update = HAL_GetTick();
                position_t current_pos = kinematics_get_position();
                parser_update_position(current_pos);
            }

            HAL_Delay(10);

            // Debug output every 5 seconds
            static uint32_t last_debug = 0;
            if (HAL_GetTick() - last_debug > 5000) // Every 5 seconds
            {
                last_debug = HAL_GetTick();
                position_t pos = kinematics_get_position();
                move_state_t state = kinematics_get_state();
                parser_state_t *parser_state = parser_get_state();

                printf("Status - Position: X=%.2f, Y=%.2f, State: %d, Parser Mode: %s, Feedrate: %.1f\n",
                       pos.x, pos.y, state,
                       parser_state->absolute_mode ? "ABS" : "REL",
                       parser_state->feedrate);
            }
        }
    }
