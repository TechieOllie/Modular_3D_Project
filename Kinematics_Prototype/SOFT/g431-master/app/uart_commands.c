/**
 *******************************************************************************
 * @file    uart_commands.c
 * @author  GitHub Copilot
 * @date    May 27, 2025
 * @brief   UART command handling for G-code and special commands
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "uart_commands.h"
#include "parser.h"
#include "kinematics.h"
#include "stepper_motor.h"
#include "limit_switches.h"
#include "stm32g4_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>

/* Private variables ---------------------------------------------------------*/
static uart_id_t uart_id;
static uart_cmd_state_t current_state = UART_CMD_STATE_IDLE;
static uart_cmd_stats_t stats = {0};
static char line_buffer[UART_CMD_MAX_LINE_LENGTH];
static uint16_t line_index = 0;
static bool line_ready = false;
static uint32_t last_char_time = 0; // Fixed: Added missing semicolon

/* Private function prototypes -----------------------------------------------*/
static void process_special_command(const char *line);
static void process_gcode_command(const char *line);
static void send_ok_response(void);
static void send_error_response(const char *error);
static void reset_line_buffer(void);
static bool is_special_command(const char *line);
static void uart_rx_callback(void);

/* Public function implementations -------------------------------------------*/

/**
 * @brief Initialize UART command handling
 */
bool uart_commands_init(uart_id_t uart_id_param)
{
    uart_id = uart_id_param;
    current_state = UART_CMD_STATE_IDLE;

    // Clear buffers and statistics
    memset(line_buffer, 0, sizeof(line_buffer));
    memset(&stats, 0, sizeof(stats));

    line_index = 0;
    line_ready = false;

    // Initialize G-code parser
    if (!parser_init())
    {
        printf("ERROR: Failed to initialize G-code parser\n");
        return false;
    }

    // Set up callback for UART reception (not strictly needed with polling)
    BSP_UART_set_callback(uart_id, uart_rx_callback);

    printf("UART commands initialized on UART%d\n", uart_id + 1);
    printf("Testing UART output...\n");

    // Test UART output
    uart_commands_send_response("// UART Command System Ready");
    uart_commands_send_response("// Send 'help' for available commands");
    uart_commands_send_response("// Debug mode enabled");

    return true;
}

/**
 * @brief UART RX callback - called when character is received
 */
static void uart_rx_callback(void)
{
    // This function is called from BSP UART interrupt
    // We'll process characters in the update function instead
}

/**
 * @brief Update UART command processing
 */
void uart_commands_update(void)
{
    // Check for incoming characters from BSP UART
    static uint32_t last_debug = 0;
    bool chars_available = BSP_UART_data_ready(uart_id);

    // Debug output every 5 seconds if no characters
    if (!chars_available && HAL_GetTick() - last_debug > 5000)
    {
        last_debug = HAL_GetTick();
        printf("UART Debug: No chars available, state=%d, line_ready=%d\n",
               current_state, line_ready);
    }

    while (BSP_UART_data_ready(uart_id))
    {
        char c = (char)BSP_UART_getc(uart_id);
        printf("UART RX: '%c' (0x%02X)\n", (c >= 32 && c <= 126) ? c : '.', (uint8_t)c);
        uart_commands_process_char(c);
        last_char_time = HAL_GetTick();
    }

    // Check for command timeout (if we have characters but no line ending for 2 seconds)
    if (line_index > 0 && !line_ready && (HAL_GetTick() - last_char_time > 2000))
    {
        printf("Command timeout - processing buffer: '%.*s'\n", line_index, line_buffer);
        line_buffer[line_index] = '\0';
        line_ready = true;
        current_state = UART_CMD_STATE_PROCESSING;
    }

    if (!line_ready)
    {
        // Debug output every 5 seconds if no characters and we're waiting
        if (!chars_available && line_index == 0 && HAL_GetTick() - last_debug > 5000)
        {
            last_debug = HAL_GetTick();
            printf("UART Debug: Waiting for commands, state=%d\n", current_state);
        }
        return;
    }

    // Process the complete line
    if (strlen(line_buffer) > 0)
    {
        stats.lines_processed++;

        printf("Processing command: '%s' (len=%d)\n", line_buffer, strlen(line_buffer));

        // Emergency stop handling
        if (strncmp(line_buffer, "M112", 4) == 0 || strncmp(line_buffer, "stop", 4) == 0)
        {
            uart_commands_emergency_stop();
            reset_line_buffer();
            current_state = UART_CMD_STATE_IDLE;
            return;
        }

        // Check if it's a special command or G-code
        if (is_special_command(line_buffer))
        {
            process_special_command(line_buffer);
        }
        else
        {
            process_gcode_command(line_buffer);
        }

        stats.commands_executed++;
    }
    else
    {
        printf("Empty line received\n");
        send_ok_response(); // Send OK for empty lines
    }

    // Reset for next line
    reset_line_buffer();
    current_state = UART_CMD_STATE_IDLE;
}

/**
 * @brief Process incoming character from UART
 */
void uart_commands_process_char(char c)
{
    printf("Processing char: '%c' (0x%02X), line_index=%d\n",
           (c >= 32 && c <= 126) ? c : '.', (uint8_t)c, line_index);

    // Handle line endings
    if (c == '\n' || c == '\r')
    {
        if (line_index > 0)
        {
            line_buffer[line_index] = '\0';
            line_ready = true;
            current_state = UART_CMD_STATE_PROCESSING;
            printf("Line ready: '%s'\n", line_buffer);
        }
        else
        {
            printf("Empty line detected\n");
            line_ready = true; // Process empty lines too
            current_state = UART_CMD_STATE_PROCESSING;
        }
        return;
    }

    // Ignore control characters except printable ones
    if (c < 32 && c != '\t')
    {
        printf("Ignoring control character: 0x%02X\n", (uint8_t)c);
        return;
    }

    // Check for buffer overflow
    if (line_index >= UART_CMD_MAX_LINE_LENGTH - 1)
    {
        send_error_response("Line too long");
        stats.buffer_overflows++;
        reset_line_buffer();
        current_state = UART_CMD_STATE_ERROR;
        printf("Buffer overflow!\n");
        return;
    }

    // Add character to line buffer
    line_buffer[line_index++] = c;
    current_state = UART_CMD_STATE_RECEIVING;
    printf("Added char, buffer now: '%.*s'\n", line_index, line_buffer);

    // Auto-process certain commands when we detect complete words
    // This handles cases where no line ending is sent
    if (line_index >= 4)
    {
        if (strncmp(line_buffer, "help", 4) == 0 ||
            strncmp(line_buffer, "stop", 4) == 0 ||
            strncmp(line_buffer, "home", 4) == 0 ||
            strncmp(line_buffer, "test", 4) == 0)
        {
            // Check if this is exactly the command (not part of a longer command)
            if (line_index == 4 || line_buffer[4] == ' ')
            {
                line_buffer[line_index] = '\0';
                line_ready = true;
                current_state = UART_CMD_STATE_PROCESSING;
                printf("Auto-detected complete command: '%s'\n", line_buffer);
                return;
            }
        }
    }

    // Auto-process other complete commands
    if (line_index >= 3 && strncmp(line_buffer, "pos", 3) == 0)
    {
        if (line_index == 3 || line_buffer[3] == ' ')
        {
            line_buffer[line_index] = '\0';
            line_ready = true;
            current_state = UART_CMD_STATE_PROCESSING;
            printf("Auto-detected complete command: '%s'\n", line_buffer);
            return;
        }
    }

    if (line_index >= 5 && strncmp(line_buffer, "stats", 5) == 0)
    {
        if (line_index == 5 || line_buffer[5] == ' ')
        {
            line_buffer[line_index] = '\0';
            line_ready = true;
            current_state = UART_CMD_STATE_PROCESSING;
            printf("Auto-detected complete command: '%s'\n", line_buffer);
            return;
        }
    }

    if (line_index >= 6)
    {
        if (strncmp(line_buffer, "status", 6) == 0 ||
            strncmp(line_buffer, "motors", 6) == 0 ||
            strncmp(line_buffer, "limits", 6) == 0)
        {
            if (line_index == 6 || line_buffer[6] == ' ')
            {
                line_buffer[line_index] = '\0';
                line_ready = true;
                current_state = UART_CMD_STATE_PROCESSING;
                printf("Auto-detected complete command: '%s'\n", line_buffer);
                return;
            }
        }
    }

    // Check for G-code commands (G or M followed by digits)
    if (line_index >= 2 && (line_buffer[0] == 'G' || line_buffer[0] == 'M'))
    {
        // Look for space or end of reasonable G-code command
        if (c == ' ' || (line_index >= 3 && isdigit(line_buffer[1]) && isdigit(line_buffer[2])))
        {
            // This might be a complete G-code command, let's process it
            line_buffer[line_index] = '\0';
            line_ready = true;
            current_state = UART_CMD_STATE_PROCESSING;
            printf("Auto-detected G-code command: '%s'\n", line_buffer);
            return;
        }
    }
}

/**
 * @brief Get current state
 */
uart_cmd_state_t uart_commands_get_state(void)
{
    return current_state;
}

/**
 * @brief Get statistics
 */
uart_cmd_stats_t *uart_commands_get_stats(void)
{
    return &stats;
}

/**
 * @brief Send response message
 */
void uart_commands_send_response(const char *message)
{
    if (message && strlen(message) > 0)
    {
        printf("Sending response: %s\n", message);
        BSP_UART_puts(uart_id, (const uint8_t *)message, 0);
        BSP_UART_puts(uart_id, (const uint8_t *)"\r\n", 2);

        // Add a small delay to prevent overwhelming the terminal
        HAL_Delay(1);
    }
}

/**
 * @brief Send formatted response
 */
void uart_commands_send_response_printf(const char *format, ...)
{
    char buffer[256];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    uart_commands_send_response(buffer);
}

/**
 * @brief Process emergency stop command
 */
void uart_commands_emergency_stop(void)
{
    stepper_motor_emergency_stop_all();
    kinematics_stop();
    uart_commands_send_response("// EMERGENCY STOP ACTIVATED");
}

/**
 * @brief Clear error state
 */
void uart_commands_clear_error(void)
{
    current_state = UART_CMD_STATE_IDLE;
    reset_line_buffer();
}

/* Private function implementations ------------------------------------------*/

/**
 * @brief Check if line is a special command
 */
static bool is_special_command(const char *line)
{
    // Skip whitespace
    while (*line && isspace(*line))
        line++;

    // Check for special command prefixes
    return (strncmp(line, "help", 4) == 0 ||
            strncmp(line, "status", 6) == 0 ||
            strncmp(line, "stop", 4) == 0 ||
            strncmp(line, "reset", 5) == 0 ||
            strncmp(line, "home", 4) == 0 ||
            strncmp(line, "pos", 3) == 0 ||
            strncmp(line, "stats", 5) == 0 ||
            strncmp(line, "motors", 6) == 0 ||
            strncmp(line, "limits", 6) == 0 ||
            strncmp(line, "test", 4) == 0 ||
            line[0] == '/' || line[0] == '#');
}

/**
 * @brief Process special system commands
 */
static void process_special_command(const char *line)
{
    // Skip whitespace
    while (*line && isspace(*line))
        line++;

    // Skip comment markers
    if (line[0] == '/' && line[1] == '/')
    {
        send_ok_response();
        return;
    }
    if (line[0] == '#')
    {
        send_ok_response();
        return;
    }

    if (strncmp(line, "help", 4) == 0)
    {
        uart_commands_send_response("// Available Commands:");
        uart_commands_send_response("// help - Show this help");
        uart_commands_send_response("// status - Show system status");
        uart_commands_send_response("// stop - Emergency stop");
        uart_commands_send_response("// test - Test coordinated movement");
        uart_commands_send_response("// reset - Reset system");
        uart_commands_send_response("// home - Home all axes");
        uart_commands_send_response("// pos - Show current position");
        uart_commands_send_response("// stats - Show command statistics");
        uart_commands_send_response("// motors - Show motor status");
        uart_commands_send_response("// limits - Show limit switch status");
        uart_commands_send_response("// G-code commands also supported");
        send_ok_response();
    }
    else if (strncmp(line, "test", 4) == 0)
    {
        uart_commands_send_response("// Testing coordinated movement...");
        position_t test_pos = {10.0f, 10.0f};
        if (kinematics_move_to(&test_pos, 1200.0f))
        {
            uart_commands_send_response("// Test movement started");
            send_ok_response();
        }
        else
        {
            send_error_response("Test movement failed");
        }
    }
    else if (strncmp(line, "status", 6) == 0)
    {
        position_t pos = kinematics_get_position();
        move_state_t state = kinematics_get_state();
        parser_state_t *parser_state = parser_get_state();

        uart_commands_send_response_printf("// Position: X=%.2f Y=%.2f", pos.x, pos.y);
        uart_commands_send_response_printf("// State: %d", state);
        uart_commands_send_response_printf("// Mode: %s", parser_state->absolute_mode ? "ABS" : "REL");
        uart_commands_send_response_printf("// Feedrate: %.1f mm/min", parser_state->feedrate);
        send_ok_response();
    }
    else if (strncmp(line, "stop", 4) == 0)
    {
        uart_commands_emergency_stop();
        send_ok_response();
    }
    else if (strncmp(line, "reset", 5) == 0)
    {
        uart_commands_send_response("// System reset not implemented");
        send_ok_response();
    }
    else if (strncmp(line, "home", 4) == 0)
    {
        uart_commands_send_response("// Homing all axes...");
        if (kinematics_home_all())
        {
            uart_commands_send_response("// Homing complete");
            send_ok_response();
        }
        else
        {
            send_error_response("Homing failed");
        }
    }
    else if (strncmp(line, "pos", 3) == 0)
    {
        position_t pos = kinematics_get_position();
        uart_commands_send_response_printf("X:%.2f Y:%.2f Z:0.00 E:0.00", pos.x, pos.y);
        send_ok_response();
    }
    else if (strncmp(line, "stats", 5) == 0)
    {
        uart_commands_send_response_printf("// Lines: %lu", stats.lines_processed);
        uart_commands_send_response_printf("// Commands: %lu", stats.commands_executed);
        uart_commands_send_response_printf("// Errors: %lu", stats.errors_count);
        uart_commands_send_response_printf("// Overflows: %lu", stats.buffer_overflows);
        send_ok_response();
    }
    else if (strncmp(line, "motors", 6) == 0)
    {
        uart_commands_send_response("// Motor Status:");
        uart_commands_send_response_printf("// X Motor: State %d, Steps %lu",
                                           stepper_motor_get_state(0), stepper_motor_get_completed_steps(0));
        uart_commands_send_response_printf("// Y Motor: State %d, Steps %lu",
                                           stepper_motor_get_state(1), stepper_motor_get_completed_steps(1));
        send_ok_response();
    }
    else if (strncmp(line, "limits", 6) == 0)
    {
        uart_commands_send_response("// Limit Switch Status:");
        uart_commands_send_response_printf("// X_MIN: %s",
                                           limit_switch_read(AXIS_X, LIMIT_MIN) == LIMIT_TRIGGERED ? "TRIGGERED" : "OPEN");
        uart_commands_send_response_printf("// X_MAX: %s",
                                           limit_switch_read(AXIS_X, LIMIT_MAX) == LIMIT_TRIGGERED ? "TRIGGERED" : "OPEN");
        uart_commands_send_response_printf("// Y_MIN: %s",
                                           limit_switch_read(AXIS_Y, LIMIT_MIN) == LIMIT_TRIGGERED ? "TRIGGERED" : "OPEN");
        uart_commands_send_response_printf("// Y_MAX: %s",
                                           limit_switch_read(AXIS_Y, LIMIT_MAX) == LIMIT_TRIGGERED ? "TRIGGERED" : "OPEN");
        send_ok_response();
    }
    else if (strncmp(line, "test", 4) == 0)
    {
        uart_commands_send_response("// Testing coordinated movement...");
        position_t test_pos = {10.0f, 10.0f};
        if (kinematics_move_to(&test_pos, 1200.0f))
        {
            uart_commands_send_response("// Test movement started");
            send_ok_response();
        }
        else
        {
            send_error_response("Test movement failed");
        }
    }
    else
    {
        send_error_response("Unknown special command");
    }
}

/**
 * @brief Process G-code commands
 */
static void process_gcode_command(const char *line)
{
    uart_commands_send_response_printf("// Executing: %s", line);

    if (parser_process_line(line))
    {
        // For movement commands, wait for completion with timeout
        if (line[0] == 'G' && (line[1] == '0' || line[1] == '1'))
        {
            uart_commands_send_response("// Movement started, waiting for completion...");

            uint32_t start_time = HAL_GetTick();
            uint32_t timeout = 30000; // 30 second timeout
            uint32_t last_status = 0;

            while (kinematics_get_state() != MOVE_STATE_IDLE)
            {
                kinematics_update();
                stepper_motor_update();

                // Print status every 2 seconds
                if (HAL_GetTick() - last_status > 2000)
                {
                    last_status = HAL_GetTick();
                    position_t pos = kinematics_get_position();
                    uart_commands_send_response_printf("// Moving... X=%.2f Y=%.2f", pos.x, pos.y);
                }

                // Check for timeout
                if (HAL_GetTick() - start_time > timeout)
                {
                    uart_commands_send_response("// Movement timeout - stopping motors");
                    stepper_motor_emergency_stop_all();
                    kinematics_stop();
                    send_error_response("Movement timeout");
                    stats.errors_count++;
                    return;
                }

                HAL_Delay(10);
            }

            uart_commands_send_response("// Movement complete");
        }

        send_ok_response();
    }
    else
    {
        send_error_response("G-code execution failed");
        stats.errors_count++;
    }
}

/**
 * @brief Send OK response
 */
static void send_ok_response(void)
{
    uart_commands_send_response("ok");
}

/**
 * @brief Send error response
 */
static void send_error_response(const char *error)
{
    uart_commands_send_response_printf("Error: %s", error);
}

/**
 * @brief Reset line buffer
 */
static void reset_line_buffer(void)
{
    memset(line_buffer, 0, sizeof(line_buffer));
    line_index = 0;
    line_ready = false;
}
