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
    stepper_motor_init(X_MOTOR_ID, GPIOA, GPIO_PIN_9, GPIOA, GPIO_PIN_10, TIMER1_ID, TIM_CHANNEL_2, 16);

    // Y-axis motor using Timer3, Channel 1 on pin PA6 with 8 microsteps
    stepper_motor_init(Y_MOTOR_ID, GPIOB, GPIO_PIN_4, GPIOB, GPIO_PIN_5, TIMER3_ID, TIM_CHANNEL_1, 16);

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
    printf("Stepper motors initialized.\n");
    printf("X Motor move 1000 steps\n");
    stepper_motor_move(X_MOTOR_ID, 10000, 8000, MOTOR_DIR_COUNTERCLOCKWISE);
    printf("Y Motor move 1000 steps\n");
    stepper_motor_move(Y_MOTOR_ID, 10000, 8000, MOTOR_DIR_COUNTERCLOCKWISE);
    // Run the speed test
    // stepper_motor_speed_test(X_MOTOR_ID);
    // printf("After speed test - Motor state: %d\n", stepper_motor_get_state(X_MOTOR_ID));

    // Main loop
    while (1)
    {
        stepper_motor_update(); // Update stepper motors
        HAL_Delay(10);
    }
}
