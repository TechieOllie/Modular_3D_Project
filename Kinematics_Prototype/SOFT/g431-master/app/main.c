/**
 *******************************************************************************
 * @file 	main.c
 * @author 	naej, ol, your name
 * @date 	Current Date
 * @brief	Main application for CNC control system
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
#include "corexy.h"

#define BLINK_DELAY 100 // ms
#define MAX_CMD_LENGTH 128

// Motor ID constants
#define X_MOTOR_ID 0
#define Y_MOTOR_ID 1

void setup_stepper_motors(void)
{
    // Initialize motors
    // X-axis motor using Timer2, Channel 1 on pin PA0 with 8 microsteps
    stepper_motor_init(X_MOTOR_ID, GPIOA, GPIO_PIN_9, GPIOA, GPIO_PIN_10, TIMER1_ID, TIM_CHANNEL_2, 8);

    // Y-axis motor using Timer3, Channel 1 on pin PA6 with 8 microsteps
    // stepper_motor_init(Y_MOTOR_ID, GPIOA, GPIO_PIN_6, TIMER3_ID, TIM_CHANNEL_1, 8);

    // Move X-axis motor 200 steps at 100 steps/second
    // stepper_motor_move(X_MOTOR_ID, 200, 100);

    // Y-axis movement could be started later
}

int main(void)
{
    // Initialize system
    HAL_Init();
    SystemClock_Config();
    BSP_GPIO_enable();

    BSP_UART_init(UART2_ID, 115200);
    BSP_SYS_set_std_usart(UART2_ID, UART2_ID, UART2_ID);

    setup_stepper_motors(); // Setup stepper motors

    // Try much higher speed - 500 steps per second
    stepper_motor_move(X_MOTOR_ID, 400, 500, MOTOR_DIR_CLOCKWISE);
    printf("Stepper motor initialized and moving at 500 steps/sec.\n");

    // Main loop
    while (1)
    {
        stepper_motor_update(); // Update stepper motors

        // Check if motor has stopped
        if (stepper_motor_get_state(X_MOTOR_ID) == MOTOR_STATE_IDLE)
        {
            // Start next movement in the opposite direction
            static stepper_motor_dir_t direction = MOTOR_DIR_CLOCKWISE;
            static uint32_t speed = 500; // Start at 500 steps/sec

            // Toggle direction
            direction = (direction == MOTOR_DIR_CLOCKWISE) ? MOTOR_DIR_COUNTERCLOCKWISE : MOTOR_DIR_CLOCKWISE;

            // Gradually increase speed to test maximum
            speed += 100;
            if (speed > 2000)
                speed = 500; // Reset after reaching 2000 steps/sec

            printf("Moving at %lu steps/sec in %s direction\n",
                   speed, (direction == MOTOR_DIR_CLOCKWISE) ? "clockwise" : "counterclockwise");

            // Move 400 steps in new direction
            stepper_motor_move(X_MOTOR_ID, 400, speed, direction);

            // Optional delay between movements
            HAL_Delay(500);
        }

        // Small delay in the loop
        HAL_Delay(10);
    }
}
