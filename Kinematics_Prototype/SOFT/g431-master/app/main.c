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

void write_LED(bool b)
{
    HAL_GPIO_WritePin(LED_GREEN_GPIO, LED_GREEN_PIN, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Buffer for receiving commands from serial
static char cmd_buffer[MAX_CMD_LENGTH];
static int cmd_index = 0;

// Callback function for G-code processing
void gcode_callback(GCodeCommand *cmd)
{
    // Process the command with the CoreXY system
    CoreXY_ProcessGCodeCommand(cmd);
}

// Process received G-code command
void process_command(void)
{
    // Terminate the string
    cmd_buffer[cmd_index] = '\0';

    printf("Processing command: %s\n", cmd_buffer);

    // Check for special commands first
    if (strcmp(cmd_buffer, "HOME") == 0)
    {
        CoreXY_Home();
    }
    else if (strcmp(cmd_buffer, "STOP") == 0)
    {
        CoreXY_EmergencyStop();
    }
    else if (strncmp(cmd_buffer, "POS", 3) == 0)
    {
        float x, y, z;
        CoreXY_GetPosition(&x, &y, &z);
        printf("Position: X:%.3f Y:%.3f Z:%.3f\n", x, y, z);
    }
    else
    {
        // Process as G-code command using the callback if not a special command
        process_gcode_string(cmd_buffer, gcode_callback);
    }

    // Reset buffer for next command
    cmd_index = 0;
}

int main(void)
{
    // Initialize system
    HAL_Init();
    SystemClock_Config();
    BSP_GPIO_enable();
    BSP_UART_init(UART2_ID, 115200);
    BSP_SYS_set_std_usart(UART2_ID, UART2_ID, UART2_ID);

    // Configure the LED for status - use 0 instead of GPIO_NO_AF if it's not defined
    BSP_GPIO_pin_config(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_MODE_OUTPUT_PP,
                        GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0);

    // Initialize the CoreXY system
    CoreXY_Init();

    // Set machine dimensions (define your build volume)
    CoreXY_SetMachineDimensions(220.0f, 220.0f, 250.0f); // 220x220x250mm build volume

#if ENDSTOP_ENABLED
    // Configure endstops
    CoreXY_ConfigureEndstops(
        // X min endstop (connected to GPIOA, pin 1, active LOW)
        GPIOA, GPIO_PIN_1, ENDSTOP_TRIGGER_LOW,
        // X max endstop (not used - set to NULL)
        NULL, 0, 0,
        // Y min endstop (connected to GPIOA, pin 2, active LOW)
        GPIOA, GPIO_PIN_2, ENDSTOP_TRIGGER_LOW,
        // Y max endstop (not used)
        NULL, 0, 0,
        // Z min endstop (connected to GPIOA, pin 3, active LOW)
        GPIOA, GPIO_PIN_3, ENDSTOP_TRIGGER_LOW,
        // Z max endstop (not used)
        NULL, 0, 0);
#endif

    printf("\nCNC Control System Ready\n");
    printf("Enter G-code commands or special commands (HOME, STOP, POS)\n");

    // Main loop
    while (1)
    {
        // Use the BSP UART functions for your specific library
        if (BSP_UART_data_ready(UART2_ID))
        {
            char c = BSP_UART_get_next_byte(UART2_ID);

            // Echo character
            BSP_UART_putc(UART2_ID, c);

            // Visual feedback
            write_LED(true);

            // Handle character input
            if (c == '\n' || c == '\r')
            {
                // Process command on newline
                BSP_UART_putc(UART2_ID, '\n');
                if (cmd_index > 0)
                {
                    process_command();
                }
            }
            else if (c == '\b' && cmd_index > 0)
            {
                // Handle backspace
                cmd_index--;
                BSP_UART_putc(UART2_ID, ' ');
                BSP_UART_putc(UART2_ID, '\b');
            }
            else if (cmd_index < MAX_CMD_LENGTH - 1 && c >= 32 && c <= 126)
            {
                // Store character in buffer
                cmd_buffer[cmd_index++] = c;
            }

            // Using HAL_Delay instead of the non-blocking SYS_get_tick_ms which might not exist
            HAL_Delay(10);
            write_LED(false);
        }
    }
}
