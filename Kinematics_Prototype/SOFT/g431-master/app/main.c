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
#include "command_buffer.h"
#include "command_executor.h"
#include "stepper_motor.h"
#include "corexy.h"
#include "SD/FatFs/src/ff.h"
#include "SD/stm32g4_sd.h"

#define BLINK_DELAY 100 // ms
#define MAX_CMD_LENGTH 128

// Global command buffer
static command_buffer_t g_cmd_buffer;

// Emergency stop handler - can be called from anywhere
void emergency_stop(void)
{
    printf("EMERGENCY STOP ACTIVATED!\n");

    // Stop CoreXY movement
    CoreXY_EmergencyStop();

    // Stop any command execution
    cmd_executor_stop();

    // Visual feedback - solid LED
    HAL_GPIO_WritePin(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_PIN_SET);

    printf("Emergency stop complete\n");
    printf("Enter command or filename: ");
    fflush(stdout);
}

// Complete system reset - last resort for emergency stop
void system_reset(void)
{
    printf("!!! PERFORMING SYSTEM RESET !!!\r\n");
    fflush(stdout);
    HAL_Delay(100); // Give time for the message to be sent

    // Reset the MCU
    HAL_NVIC_SystemReset();
}

// Test function for CoreXY functionality
void test_corexy(void)
{
    printf("Testing CoreXY functionality...\n");

    // Initialize CoreXY system
    CoreXY_Init();

    // Set machine dimensions (this is just example values)
    CoreXY_SetMachineDimensions(220.0f, 220.0f, 250.0f);

    // Move to home position first
    printf("Homing machine...\n");
    CoreXY_Home(true, true, true, NULL);

    // Wait for homing to complete
    while (CoreXY_GetState() != COREXY_STATE_IDLE)
    {
        CoreXY_Process();
        STEPPER_process();
        HAL_Delay(10);
    }

    // Move to center of build volume
    printf("Moving to center position...\n");
    CoreXY_MoveTo(110.0f, 110.0f, 125.0f, 1000.0f, NULL);

    // Wait for movement to complete
    while (CoreXY_GetState() != COREXY_STATE_IDLE)
    {
        CoreXY_Process();
        STEPPER_process();
        HAL_Delay(10);
    }

    // Move back to home position
    printf("Moving back to home position...\n");
    CoreXY_MoveTo(0.0f, 0.0f, 0.0f, 1000.0f, NULL);

    // Wait for movement to complete
    while (CoreXY_GetState() != COREXY_STATE_IDLE)
    {
        CoreXY_Process();
        STEPPER_process();
        HAL_Delay(10);
    }

    printf("CoreXY test complete!\n");
}

// Test function for stepper motors (single motor test)
void test_stepper_motors(void)
{
    printf("Testing stepper motors...\n");

    // Initialize stepper motors
    STEPPER_init();

    // Add motors (using NEMA 17 configuration)
    stepper_id_t motor_x = STEPPER_add(GPIOB, GPIO_PIN_4, GPIOA, GPIO_PIN_6,
                                       TIMER3_ID, TIM_CHANNEL_1, 80.0f, 16, 1.5f);

    // Move X motor back and forth
    if (motor_x >= 0)
    {
        printf("Moving X motor forward 10mm...\n");
        STEPPER_move_mm(motor_x, 10.0f, 10.0f, NULL);

        // Wait for movement to complete
        while (STEPPER_is_moving(motor_x))
        {
            STEPPER_process();
            HAL_Delay(10);
        }

        printf("Moving X motor backward 10mm...\n");
        STEPPER_move_mm(motor_x, -10.0f, 10.0f, NULL);

        // Wait for movement to complete
        while (STEPPER_is_moving(motor_x))
        {
            STEPPER_process();
            HAL_Delay(10);
        }

        printf("Stepper motor test complete!\n");
    }
    else
    {
        printf("Failed to initialize X motor\n");
    }
}

int main(void)
{
    // Initialize system
    HAL_Init();
    SystemClock_Config();
    BSP_GPIO_enable();

    // Fix UART initialization with proper settings
    BSP_UART_init(UART2_ID, 115200);
    // If BSP_UART_config is not available, ensure UART is initialized with correct settings in BSP_UART_init
    // BSP_UART_config(UART2_ID, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE); // Removed due to missing declaration
    BSP_SYS_set_std_usart(UART2_ID, UART2_ID, UART2_ID);

    // Configure the LED for status - use 0 instead of GPIO_NO_AF if it's not defined
    BSP_GPIO_pin_config(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_MODE_OUTPUT_PP,
                        GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0);

    // Initialize SD card filesystem
    if (BSP_SD_GetStatus() != 0)
    {
        printf("Failed to initialize filesystem\n");
        Error_Handler();
    }

    // Initialize command processing
    cmd_buffer_init(&g_cmd_buffer);
    cmd_executor_init();

    // Initialize CoreXY system
    CoreXY_Init();

    printf("System initialized\n");

    // Run basic tests
    // printf("Running basic stepper motor test...\n");
    // test_stepper_motors();

    // printf("Running CoreXY test...\n");
    // test_corexy();

    printf("Enter command or filename: ");
    fflush(stdout); // Ensure the prompt is sent immediately

    char cmd_line[MAX_CMD_LENGTH];
    int cmd_pos = 0;

    // Main loop
    while (1)
    {
        // Process stepper motors (handles acceleration profiles)
        STEPPER_process();

        // Process CoreXY state machine
        CoreXY_Process();

        // Process all available characters in the UART buffer
        while (BSP_UART_data_ready(UART2_ID))
        {
            char c = BSP_UART_getc(UART2_ID);

            // Handle command completion (both CR and LF are accepted)
            if (c == '\r' || c == '\n')
            {
                // Ignore empty LF that might follow CR (prevents double-processing)
                if (c == '\n' && cmd_pos == 0)
                    continue;

                // Only process non-empty commands
                if (cmd_pos > 0)
                {
                    // Echo newline for better visual feedback
                    BSP_UART_puts(UART2_ID, "\r\n", 2);

                    // Null-terminate the command string
                    cmd_line[cmd_pos] = '\0';

                    printf("Processing command: %s\n", cmd_line);

                    // Process the command
                    if (strcmp(cmd_line, "STOP") == 0)
                    {
                        emergency_stop();
                    }
                    else if (strcmp(cmd_line, "RESET") == 0)
                    {
                        system_reset();
                    }
                    else if (strcmp(cmd_line, "PAUSE") == 0)
                    {
                        cmd_executor_pause();
                        printf("Execution paused\r\n");
                    }
                    else if (strcmp(cmd_line, "RESUME") == 0)
                    {
                        cmd_executor_resume();
                        printf("Execution resumed\r\n");
                    }
                    else if (strcmp(cmd_line, "HOME") == 0)
                    {
                        CoreXY_Home(true, true, true, NULL);
                    }
                    else if (strcmp(cmd_line, "POS") == 0)
                    {
                        float x, y, z;
                        CoreXY_GetPosition(&x, &y, &z);
                        printf("Current position: X=%.2f Y=%.2f Z=%.2f\n", x, y, z);
                    }
                    else if (cmd_line[0] == 'G' || cmd_line[0] == 'M')
                    {
                        // Parse and execute a single G-code command
                        parser_command_t cmd;
                        if (parser_parse_line(cmd_line, &cmd))
                        {
                            if (cmd.type == COMMAND_TYPE_G)
                            {
                                CoreXY_ProcessGCodeCommand(&cmd);
                            }
                            else if (cmd.type == COMMAND_TYPE_M)
                            {
                                // For M commands, handle them directly here
                                if (cmd.code == 112)
                                {
                                    // M112 - Emergency Stop
                                    CoreXY_EmergencyStop();
                                }
                                else
                                {
                                    printf("Unsupported M-code: M%d\n", cmd.code);
                                }
                            }
                        }
                        else
                        {
                            printf("Parse error: %s\n", parser_get_error());
                        }
                    }
                    else if (strcmp(cmd_line, "TEST") == 0)
                    {
                        test_corexy();
                    }
                    else
                    {
                        // Assume it's a filename and try to open it
                        if (cmd_buffer_open_file(&g_cmd_buffer, cmd_line) == BUFFER_ERROR)
                        {
                            printf("Error: %s\n", cmd_buffer_get_error(&g_cmd_buffer));
                        }
                        else
                        {
                            printf("File opened: %s\n", cmd_line);
                            cmd_executor_start(&g_cmd_buffer);
                        }
                    }

                    cmd_pos = 0;
                    printf("\r\nEnter command or filename: ");
                    fflush(stdout);
                }
                else if (c == '\r')
                {
                    // For empty command with just Enter, redisplay the prompt
                    BSP_UART_puts(UART2_ID, "\r\n", 2);
                    printf("Enter command or filename: ");
                    fflush(stdout);
                }
            }
            else if (c == 8 || c == 127) // Backspace or Delete
            {
                if (cmd_pos > 0)
                {
                    cmd_pos--;
                    // Echo backspace sequence to erase character on terminal
                    BSP_UART_puts(UART2_ID, "\b \b", 3);
                }
            }
            else if (cmd_pos < MAX_CMD_LENGTH - 1)
            {
                // Echo the character for feedback
                BSP_UART_putc(UART2_ID, c);

                // Store character
                cmd_line[cmd_pos++] = c;
            }
            else
            {
                // Buffer full - emit beep sound
                BSP_UART_putc(UART2_ID, 7); // ASCII BEL (bell)
            }
        }

        // Small delay to prevent CPU hogging and allow UART buffer to fill
        HAL_Delay(1);

        // Process command execution from file
        cmd_executor_process(&g_cmd_buffer);

        // Toggle LED based on execution status
        static uint32_t led_timer = 0;
        if (HAL_GetTick() - led_timer > BLINK_DELAY)
        {
            led_timer = HAL_GetTick();

            if (cmd_executor_is_active() || CoreXY_GetState() != COREXY_STATE_IDLE)
            {
                if (cmd_executor_is_paused())
                {
                    // Blink slowly when paused
                    if ((led_timer / 500) % 2)
                    {
                        HAL_GPIO_WritePin(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_PIN_SET);
                    }
                    else
                    {
                        HAL_GPIO_WritePin(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_PIN_RESET);
                    }
                }
                else
                {
                    // Blink quickly when executing
                    HAL_GPIO_TogglePin(LED_GREEN_GPIO, LED_GREEN_PIN);
                }
            }
            else
            {
                // Steady on when idle
                HAL_GPIO_WritePin(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_PIN_SET);
            }
        }
    }
}
