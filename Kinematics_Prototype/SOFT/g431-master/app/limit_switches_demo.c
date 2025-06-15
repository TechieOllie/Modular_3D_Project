/**
 *******************************************************************************
 * @file    limit_switches_demo.c
 * @author  Ol, naej, Fabs
 * @date    May 27, 2025
 * @brief   Demo application for limit switches functionality
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "limit_switches_demo.h"
#include "limit_switches.h"
#include "stepper_motor.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
static bool demo_running = false;

/* Private function declarations ---------------------------------------------*/
static void limit_switch_event_handler(axis_t axis, limit_position_t position, limit_switch_state_t state);

/* Public functions implementations ------------------------------------------*/

/**
 * @brief Initialize and configure limit switches for demo
 */
bool limit_switches_demo_init(void)
{
    printf("Initializing limit switches demo...\n");

    // Initialize limit switches system
    if (!limit_switches_init())
    {
        printf("Failed to initialize limit switches system\n");
        return false;
    }

    // Configure limit switches for each axis
    // X-axis limits on GPIOB pins 0 (MIN) and 1 (MAX)
    limit_switch_configure(AXIS_X, LIMIT_MIN, GPIOB, GPIO_PIN_0, true); // Active low
    limit_switch_configure(AXIS_X, LIMIT_MAX, GPIOB, GPIO_PIN_1, true); // Active low

    // Y-axis limits on GPIOB pins 2 (MIN) and 3 (MAX)
    limit_switch_configure(AXIS_Y, LIMIT_MIN, GPIOB, GPIO_PIN_2, true); // Active low
    limit_switch_configure(AXIS_Y, LIMIT_MAX, GPIOB, GPIO_PIN_3, true); // Active low

    // Z-axis limits on GPIOB pins 4 (MIN) and 5 (MAX)
    limit_switch_configure(AXIS_Z, LIMIT_MIN, GPIOB, GPIO_PIN_4, true); // Active low
    limit_switch_configure(AXIS_Z, LIMIT_MAX, GPIOB, GPIO_PIN_5, true); // Active low

    // Set up callback for limit switch events
    limit_switches_set_callback(limit_switch_event_handler);

    // Enable all limit switches
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        limit_switch_enable((axis_t)axis, LIMIT_MIN, true);
        limit_switch_enable((axis_t)axis, LIMIT_MAX, true);
    }

    printf("Limit switches demo initialized successfully\n");
    return true;
}

/**
 * @brief Demo for testing limit switches functionality
 */
void limit_switches_demo_run(void)
{
    printf("Starting limit switches demo...\n");
    printf("Press any limit switch to see the response\n");
    printf("Demo will monitor all switches and show their states\n\n");

    demo_running = true;
    uint32_t last_status_time = 0;

    while (demo_running)
    {
        // Update limit switches
        limit_switches_update();

        // Print status every 2 seconds
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_status_time >= 2000)
        {
            last_status_time = current_time;

            printf("=== Limit Switch Status ===\n");
            for (uint8_t axis = 0; axis < 3; axis++)
            {
                for (uint8_t pos = 0; pos < 2; pos++)
                {
                    limit_switch_state_t state = limit_switch_read((axis_t)axis, (limit_position_t)pos);
                    printf("%s_%s: %s\n",
                           limit_switch_axis_to_string((axis_t)axis),
                           limit_switch_position_to_string((limit_position_t)pos),
                           state == LIMIT_TRIGGERED ? "TRIGGERED" : "RELEASED");
                }
            }
            printf("===========================\n\n");
        }

        HAL_Delay(100);
    }
}

/**
 * @brief Demo for stepper motor with limit switches
 */
void stepper_motor_with_limits_demo(void)
{
    printf("Starting stepper motor with limit switches demo...\n");

    // Initialize stepper motors (assuming they're already configured)
    // Motor 0 = X-axis, Motor 1 = Y-axis, Motor 2 = Z-axis

    // Associate motors with axes
    stepper_motor_set_axis(0, AXIS_X); // Motor 0 -> X-axis
    stepper_motor_set_axis(1, AXIS_Y); // Motor 1 -> Y-axis
    stepper_motor_set_axis(2, AXIS_Z); // Motor 2 -> Z-axis

    // Enable limit checking for all motors
    stepper_motor_enable_limit_check(0, true);
    stepper_motor_enable_limit_check(1, true);
    stepper_motor_enable_limit_check(2, true);

    printf("Motors configured with limit switch checking\n");

    // Demo sequence
    printf("\n=== Homing Sequence ===\n");

    // Home X-axis
    printf("Homing X-axis...\n");
    stepper_motor_home_precision(0, 2000, 400); // Motor 0, 1000 steps/sec

    // Wait for homing to complete
    while (stepper_motor_get_state(0) != MOTOR_STATE_IDLE)
    {
        stepper_motor_update();
        HAL_Delay(10);
    }
    printf("X-axis homing complete\n");

    HAL_Delay(1000);

    // Home Y-axis
    printf("Homing Y-axis...\n");
    ststepper_motor_home_precision(1, 2000, 400); // Motor 1, 1000 steps/sec

    while (stepper_motor_get_state(1) != MOTOR_STATE_IDLE)
    {
        stepper_motor_update();
        HAL_Delay(10);
    }
    printf("Y-axis homing complete\n");

    HAL_Delay(1000);

    // Demo movement with limits
    printf("\n=== Movement with Limit Checking ===\n");

    printf("Moving X-axis 500 steps (should be safe)...\n");
    stepper_motor_move_with_limits(0, 500, 2000, MOTOR_DIR_CLOCKWISE);

    while (stepper_motor_get_state(0) != MOTOR_STATE_IDLE)
    {
        stepper_motor_update();
        HAL_Delay(10);
    }
    printf("X-axis movement complete\n");

    printf("Demo complete!\n");
}

/**
 * @brief Stop the demo
 */
void limit_switches_demo_stop(void)
{
    demo_running = false;
    printf("Limit switches demo stopped\n");
}

/* Private functions implementations ------------------------------------------*/

/**
 * @brief Callback function for limit switch events
 */
static void limit_switch_event_handler(axis_t axis, limit_position_t position, limit_switch_state_t state)
{
    printf("*** LIMIT SWITCH EVENT ***\n");
    printf("Axis: %s\n", limit_switch_axis_to_string(axis));
    printf("Position: %s\n", limit_switch_position_to_string(position));
    printf("State: %s\n", state == LIMIT_TRIGGERED ? "TRIGGERED" : "RELEASED");
    printf("Time: %lu ms\n", HAL_GetTick());
    printf("*************************\n\n");
}
