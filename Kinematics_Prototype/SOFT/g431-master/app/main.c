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
#include "corexy.h"
#include "parser.h"
#include <string.h>
#include "stepper_motor.h"

#define BLINK_DELAY 100 // ms
#define MAX_CMD_LENGTH 128

void write_LED(bool b) {
    HAL_GPIO_WritePin(LED_GREEN_GPIO, LED_GREEN_PIN, b);
}

// Buffer for receiving commands from serial
static char cmd_buffer[MAX_CMD_LENGTH];
static int cmd_index = 0;

// Process received G-code command
void process_command(void) {
    // Terminate the string
    cmd_buffer[cmd_index] = '\0';
    
    // Echo the command
    printf("Command: %s\n", cmd_buffer);
    
    // Parse the command
    GCodeCommand cmd;
    parse_gcode_line(cmd_buffer, &cmd);
    
    // Process the command if valid
    if (cmd.g_code >= 0) {
        CoreXY_ProcessGCodeCommand(&cmd);
    } else {
        // Check for special commands
        if (strcmp(cmd_buffer, "HOME") == 0) {
            CoreXY_Home();
        } else if (strcmp(cmd_buffer, "STOP") == 0) {
            CoreXY_EmergencyStop();
        } else if (strncmp(cmd_buffer, "POS", 3) == 0) {
            float x, y, z;
            CoreXY_GetPosition(&x, &y, &z);
            printf("Position: X:%.3f Y:%.3f Z:%.3f\n", x, y, z);
        } else if (strcmp(cmd_buffer, "NEMATEST") == 0) {
            printf("Starting NEMA 17 test sequence\n");
            STEPPER_test_nema17(0); // Test first motor
            // Uncomment to test other motors
            // STEPPER_test_nema17(1);
            // STEPPER_test_nema17(2);
            printf("NEMA 17 test sequence complete\n");
        } else if (strcmp(cmd_buffer, "GPIOTEST") == 0) {
            printf("Testing GPIO pins...\n");
            
            // Test the step pin for motor 0 directly
            // Assuming motor 0 uses GPIOA, PIN_8 for step
            for (int i = 0; i < 10; i++) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
                HAL_Delay(100);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
                HAL_Delay(100);
            }
            
            printf("GPIO test complete\n");
        } else if (strcmp(cmd_buffer, "TIMERTEST") == 0) {
            printf("Testing timer configuration...\n");
            
            // First, configure the pin as standard GPIO output
            BSP_GPIO_pin_config(GPIOB, GPIO_PIN_4, GPIO_MODE_OUTPUT_PP,
                               GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
            
            // Toggle manually to verify the pin works
            for (int i = 0; i < 5; i++) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
                HAL_Delay(200);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
                HAL_Delay(200);
            }
            
            // Now try with the timer
            printf("Configuring for timer output...\n");
            BSP_GPIO_pin_config(GPIOB, GPIO_PIN_4, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF2_TIM3);
            
            // Test the timer directly
            TIM_HandleTypeDef* htim = BSP_TIMER_get_handler(TIMER3_ID);
            if (htim == NULL) {
                printf("Timer handler is NULL!\n");
            } else {
                printf("Starting timer PWM output...\n");
                BSP_TIMER_run_us(TIMER3_ID, 1000, true);  // 1ms period (1kHz)
                BSP_TIMER_enable_PWM(TIMER3_ID, TIM_CHANNEL_1, 50, false, false);
                HAL_Delay(5000);  // Run for 5 seconds
                BSP_TIMER_disable_PWM(TIMER3_ID, TIM_CHANNEL_1);
                printf("Timer PWM test complete\n");
            }
        } else {
            printf("Unknown command\n");
        }
    }
    
    // Reset the buffer
    cmd_index = 0;
}

// Process received characters
void process_char(char c) {
    // Handle backspace
    if (c == 8 || c == 127) {
        if (cmd_index > 0) {
            cmd_index--;
        }
        return;
    }
    
    // Handle newline/carriage return
    if (c == '\n' || c == '\r') {
        if (cmd_index > 0) {
            process_command();
        }
        return;
    }
    
    // Add character to buffer if there's space
    if (cmd_index < MAX_CMD_LENGTH - 1) {
        cmd_buffer[cmd_index++] = c;
    }
}

/**
  * @brief  Main application entry point
  */
int main(void) {
    // Initialize system
    HAL_Init();
    SystemClock_Config();
    BSP_GPIO_enable();
    BSP_UART_init(UART2_ID, 115200);
    BSP_SYS_set_std_usart(UART2_ID, UART2_ID, UART2_ID);
    
    // Configure the LED for status
    BSP_GPIO_pin_config(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_MODE_OUTPUT_PP, 
                        GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
    
    // Initialize the CoreXY system
    CoreXY_Init();
    
    printf("\nCNC Control System Ready\n");
    printf("Enter G-code commands or special commands (HOME, STOP, POS)\n");
    
    // Main loop
    while (1) {
         // Check for received characters
         if (BSP_UART_data_ready(UART2_ID)) {
             char c = BSP_UART_get_next_byte(UART2_ID);
             write_LED(true);
             process_char(c);
             HAL_Delay(10);
             write_LED(false);
         }
    }
}



