/**
 *******************************************************************************
 * @file    uart_commands.c
 * @author  Ol, naej, Fabs
 * @date    May 27, 2025
 * @brief   UART command handling for G-code and special commands
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "uart_commands.h"
#include "command_buffer.h"
#include "parser.h"
#include "kinematics.h"
#include "stepper_motor.h"
#include "limit_switches.h"
#include "stm32g4_uart.h"
#include "sd_gcode_reader.h"
#include "gcode_move_buffer.h"
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
static uint32_t last_char_time = 0;

/* Add SD card printing state */
static bool sd_printing = false;
static uint32_t last_sd_read_time = 0;

/* Private function prototypes -----------------------------------------------*/
static void process_special_command(const char *line);
static void process_gcode_command(const char *line);
static void send_ok_response(void);
static void send_error_response(const char *error);
static void reset_line_buffer(void);
static bool is_special_command(const char *line);
static void uart_rx_callback(void);
static void process_emergency_stop(void);

/* Add this at the top with other static variables */

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

    // Initialize command buffer
    if (!command_buffer_init())
    {
        printf("- ERROR: Failed to initialize command buffer\n");
        return false;
    }

    // Initialize G-code parser
    if (!parser_init())
    {
        printf("- ERROR: Failed to initialize G-code parser\n");
        return false;
    }

    // Initialize SD card G-code reader
    if (sd_gcode_reader_init())
    {
        printf("- SD card G-code reader initialized\n");
    }
    else
    {
        printf("- WARNING: SD card G-code reader initialization failed\n");
    }

    // Initialize G-code move buffer
    if (!gcode_move_buffer_init())
    {
        printf("- ERROR: Failed to initialize G-code move buffer\n");
        return false;
    }

    // Set up callback for UART reception (not strictly needed with polling)
    BSP_UART_set_callback(uart_id, uart_rx_callback);

    printf("- UART commands initialized on UART%d\n", uart_id + 1);

    // Test UART output
    uart_commands_send_response("|------------------------------------------|");
    uart_commands_send_response("|========UART Command System Ready=========|");
    uart_commands_send_response("|   Send 'help' for available commands     |");
    uart_commands_send_response("| Type 'M112' or 'stop' for emergency stop |");
    uart_commands_send_response("|------------------------------------------|");

    return true;
}

/**
 * @brief UART RX callback - called when character is received
 */
static void uart_rx_callback(void)
{
    // This function is called from BSP UART interrupt
}

/**
 * @brief Update UART command processing
 */
void uart_commands_update(void)
{
    // Update command buffer (processes buffered commands)
    command_buffer_update();

    // Update G-code move buffer
    gcode_move_buffer_update();

    // Handle SD card printing - continuously read and buffer moves
    if (sd_printing && sd_gcode_reader_is_ready())
    {
        uint32_t current_time = HAL_GetTick();

        // Read new lines every 100ms or when buffer has space
        if (current_time - last_sd_read_time > 100 &&
            gcode_move_buffer_get_free_space() > 10)
        {
            last_sd_read_time = current_time;

            // Read and buffer multiple lines
            char sd_line_buffer[SD_GCODE_MAX_LINE_LENGTH];
            position_t current_pos = kinematics_get_position();
            parser_state_t *parser_state = parser_get_state();

            int lines_read = 0;
            while (lines_read < 5 && !sd_gcode_reader_is_eof() &&
                   gcode_move_buffer_get_free_space() > 5)
            {
                sd_gcode_result_t result = sd_gcode_reader_read_line(
                    sd_line_buffer, sizeof(sd_line_buffer));

                if (result == SD_GCODE_OK)
                {
                    gcode_move_buffer_result_t buf_result = gcode_move_buffer_add_from_gcode(
                        sd_line_buffer, current_pos, parser_state->feedrate,
                        parser_state->absolute_mode);

                    if (buf_result == GCODE_MOVE_BUFFER_OK)
                    {
                        lines_read++;
                    }
                    else if (buf_result == GCODE_MOVE_BUFFER_FULL)
                    {
                        break; // Buffer full, try later
                    }
                }
                else if (result == SD_GCODE_END_OF_FILE)
                {
                    sd_printing = false;
                    sd_gcode_reader_close();
                    uart_commands_send_response("| SD card printing completed |");
                    break;
                }
                else
                {
                    printf("SD read error: %d\n", result);
                    break;
                }
            }
        }
    }

    // Check for incoming characters from BSP UART
    static uint32_t last_debug = 0;
    bool chars_available = BSP_UART_data_ready(uart_id);

    // Debug output every 5 seconds if no characters
    if (!chars_available && HAL_GetTick() - last_debug > 5000)
    {
        last_debug = HAL_GetTick();
        printf("- UART Debug: No chars available, state=%d, line_ready=%d\n",
               current_state, line_ready);
    }

    while (BSP_UART_data_ready(uart_id))
    {
        char c = (char)BSP_UART_getc(uart_id);
        printf("- UART RX: '%c' (0x%02X)\n", (c >= 32 && c <= 126) ? c : '.', (uint8_t)c);
        uart_commands_process_char(c);
        last_char_time = HAL_GetTick();
    }

    // Check for command timeout (if we have characters but no line ending for 2 seconds)
    if (line_index > 0 && !line_ready && (HAL_GetTick() - last_char_time > 2000))
    {
        printf("- Command timeout - processing buffer: '%.*s'\n", line_index, line_buffer);
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
            printf("- UART Debug: Waiting for commands, state=%d\n", current_state);
        }
        return;
    }

    // Process the complete line
    if (strlen(line_buffer) > 0)
    {
        stats.lines_processed++;

        printf("- Processing command: '%s' (len=%d)\n", line_buffer, strlen(line_buffer));

        // Emergency stop handling - ALWAYS execute immediately regardless of system state
        if (strncmp(line_buffer, "M112", 4) == 0 || strncmp(line_buffer, "stop", 4) == 0)
        {
            printf("- EMERGENCY STOP COMMAND DETECTED!\n");

            // Force stop all motors using the emergency stop method
            stepper_motor_emergency_stop_all();
            kinematics_stop();

            // Clear any buffers
            command_buffer_clear();
            gcode_move_buffer_clear();

            // Stop SD printing if active
            if (sd_printing)
            {
                sd_printing = false;
                sd_gcode_reader_close();
                gcode_move_buffer_set_state(GCODE_MOVE_BUFFER_STATE_IDLE);
            }

            uart_commands_send_response("|---- EMERGENCY STOP ACTIVATED ----|");
            uart_commands_send_response("|---- ALL MOTORS STOPPED ----|");
            send_ok_response();

            reset_line_buffer();
            current_state = UART_CMD_STATE_IDLE;
            return;
        }

        // Check if it's a special command or G-code
        if (is_special_command(line_buffer))
        {
            // ALL special commands should be executed immediately, not buffered
            printf("- Executing special command immediately: '%s'\n", line_buffer);
            process_special_command(line_buffer);
        }
        else
        {
            // For G-code commands, execute them immediately using the parser
            // This bypasses the command buffer for direct G-code execution
            printf("- Executing G-code directly: '%s'\n", line_buffer);

            if (parser_process_line(line_buffer))
            {
                send_ok_response();
            }
            else
            {
                send_error_response("G-code execution failed");
                stats.errors_count++;
            }
        }

        stats.commands_executed++;
    }
    else
    {
        printf("- Empty line received\n");
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
    // Check for emergency stop command at the very beginning
    if (line_index == 0 && c == 'M')
    {
        potential_emergency_stop = true;
    }
    else if (potential_emergency_stop && line_index <= 3)
    {
        // Check for "M112"
        if ((line_index == 1 && c == '1') ||
            (line_index == 2 && c == '1') ||
            (line_index == 3 && c == '2'))
        {
            // Continue checking
        }
        else
        {
            potential_emergency_stop = false;
        }

        // If we have "M112", execute emergency stop immediately
        if (potential_emergency_stop && line_index == 3 && c == '2')
        {
            printf("- EMERGENCY STOP DETECTED FROM PARTIAL COMMAND!\n");
            process_emergency_stop();
            reset_line_buffer();
            return;
        }
    }

    // Regular character processing
    printf("- Processing char: '%c' (0x%02X), line_index=%d\n",
           (c >= 32 && c <= 126) ? c : '.', (uint8_t)c, line_index);

    // Handle line endings
    if (c == '\n' || c == '\r')
    {
        if (line_index > 0)
        {
            line_buffer[line_index] = '\0';
            line_ready = true;
            current_state = UART_CMD_STATE_PROCESSING;
            printf("- Line ready: '%s'\n", line_buffer);
        }
        else
        {
            printf("- Empty line detected\n");
            line_ready = true; // Process empty lines too
            current_state = UART_CMD_STATE_PROCESSING;
        }
        return;
    }

    // Ignore control characters except printable ones
    if (c < 32 && c != '\t')
    {
        printf("- Ignoring control character: 0x%02X\n", (uint8_t)c);
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
    // printf("- Added char, buffer now: '%.*s'\n", line_index, line_buffer);

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
                printf("- Auto-detected complete command: '%s'\n", line_buffer);
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
            printf("- Auto-detected complete command: '%s'\n", line_buffer);
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
            printf("- Auto-detected complete command: '%s'\n", line_buffer);
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
                printf("- Auto-detected complete command: '%s'\n", line_buffer);
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
            printf("- Auto-detected G-code command: '%s'\n", line_buffer);
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
        // printf("Sending response: %s\n", message);
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
    STEPPER_MOTOR_emergency_stop_all();
    kinematics_stop();
    uart_commands_send_response("|---- EMERGENCY STOP ACTIVATED ----|");
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
            strncmp(line, "buffer", 6) == 0 ||
            strncmp(line, "pause", 5) == 0 ||
            strncmp(line, "resume", 6) == 0 ||
            strncmp(line, "clear", 5) == 0 ||
            strncmp(line, "sdlist", 6) == 0 ||
            strncmp(line, "sdprint", 7) == 0 ||
            strncmp(line, "sdstop", 6) == 0 ||
            strncmp(line, "sdstatus", 8) == 0 ||
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
        uart_commands_send_response("|------------------------------------|");
        uart_commands_send_response("|=========Available Commands=========|");
        uart_commands_send_response("|  help - Show this help             |");
        uart_commands_send_response("|  status - Show system status       |");
        uart_commands_send_response("|  stop - Emergency stop             |");
        uart_commands_send_response("|  test - Test coordinated movement  |");
        uart_commands_send_response("|  reset - Reset system              |");
        uart_commands_send_response("|  home - Home all axes              |");
        uart_commands_send_response("|  pos - Show current position       |");
        uart_commands_send_response("|  stats - Show command statistics   |");
        uart_commands_send_response("|  buffer - Show buffer status       |");
        uart_commands_send_response("|  pause - Pause command execution   |");
        uart_commands_send_response("|  resume - Resume command execution |");
        uart_commands_send_response("|  clear - Clear command buffer      |");
        uart_commands_send_response("|  motors - Show motor status        |");
        uart_commands_send_response("|  limits - Show limit switch status |");
        uart_commands_send_response("|  sdlist - List G-code files on SD  |");
        uart_commands_send_response("|  sdprint <file> - Print from SD    |");
        uart_commands_send_response("|  sdstatus - Show SD card status    |");
        uart_commands_send_response("|  G-code - commands also supported  |");
        uart_commands_send_response("|------------------------------------|");
        send_ok_response();
    }
    else if (strncmp(line, "test", 4) == 0)
    {
        uart_commands_send_response("| Testing coordinated movement... |");
        position_t test_pos = {10.0f, 10.0f};
        if (kinematics_move_to(&test_pos, 1200.0f))
        {
            uart_commands_send_response("| Test movement started |");
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

        uart_commands_send_response_printf("| Position: X=%.2f Y=%.2f |", pos.x, pos.y);
        uart_commands_send_response_printf("| State: %d |", state);
        uart_commands_send_response_printf("| Mode: %s |", parser_state->absolute_mode ? "ABS" : "REL");
        uart_commands_send_response_printf("| Feedrate: %.1f mm/min |", parser_state->feedrate);
        send_ok_response();
    }
    else if (strncmp(line, "stop", 4) == 0)
    {
        uart_commands_emergency_stop();
        send_ok_response();
    }
    else if (strncmp(line, "reset", 5) == 0)
    {
        uart_commands_send_response("| System reset not implemented |");
        send_ok_response();
    }
    else if (strncmp(line, "home", 4) == 0)
    {
        uart_commands_send_response("| Homing all axes... |");
        if (kinematics_home_all())
        {
            uart_commands_send_response("| Homing complete |");
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
        uart_commands_send_response_printf("| X:%.2f Y:%.2f Z:0.00 E:0.00 |", pos.x, pos.y);
        send_ok_response();
    }
    else if (strncmp(line, "stats", 5) == 0)
    {
        cmd_buffer_stats_t *buf_stats = command_buffer_get_stats();
        uart_commands_send_response_printf("| Lines: %lu |", stats.lines_processed);
        uart_commands_send_response_printf("| Commands: %lu |", stats.commands_executed);
        uart_commands_send_response_printf("| Errors: %lu |", stats.errors_count);
        uart_commands_send_response_printf("| Overflows: %lu |", stats.buffer_overflows);
        uart_commands_send_response_printf("| Buffer: %d/%d |", buf_stats->current_count, CMD_BUFFER_SIZE);
        uart_commands_send_response_printf("| Buffer Executed: %lu |", buf_stats->commands_executed);
        uart_commands_send_response_printf("| Buffer Errors: %lu |", buf_stats->execution_errors);
        send_ok_response();
    }
    else if (strncmp(line, "buffer", 6) == 0)
    {
        uart_commands_send_response("| Motor Status: |");
        uart_commands_send_response_printf("| X Motor: State %d, Steps %lu |",
                                           STEPPER_MOTOR_get_state(0), STEPPER_MOTOR_get_completed_steps(0));
        uart_commands_send_response_printf("| Y Motor: State %d, Steps %lu |",
                                           STEPPER_MOTOR_get_state(1), STEPPER_MOTOR_get_completed_steps(1));
        send_ok_response();
    }
    else if (strncmp(line, "pause", 5) == 0)
    {
        command_buffer_pause();
        uart_commands_send_response("| Command buffer paused |");
        send_ok_response();
    }
    else if (strncmp(line, "resume", 6) == 0)
    {
        command_buffer_resume();
        uart_commands_send_response("| Command buffer resumed |");
        send_ok_response();
    }
    else if (strncmp(line, "clear", 5) == 0)
    {
        command_buffer_clear();
        uart_commands_send_response("| Command buffer cleared |");
        send_ok_response();
    }
    else if (strncmp(line, "sdlist", 6) == 0)
    {
        if (sd_gcode_reader_is_ready())
        {
            char file_list[10][SD_GCODE_MAX_FILENAME_LENGTH];
            uint32_t file_count = sd_gcode_reader_list_files(file_list, 10);

            uart_commands_send_response("| G-code files on SD card: |");
            if (file_count == 0)
            {
                uart_commands_send_response("| No G-code files found |");
            }
            else
            {
                for (uint32_t i = 0; i < file_count; i++)
                {
                    uart_commands_send_response_printf("| %d: %s |", i + 1, file_list[i]);
                }
            }
            send_ok_response();
        }
        else
        {
            send_error_response("SD card not ready");
        }
    }
    else if (strncmp(line, "sdprint", 7) == 0)
    {
        // Extract filename
        const char *filename = line + 7;
        while (*filename && isspace(*filename))
            filename++; // Skip spaces

        if (strlen(filename) == 0)
        {
            send_error_response("Usage: sdprint <filename>");
            return;
        }

        if (!sd_gcode_reader_is_ready())
        {
            send_error_response("SD card not ready");
            return;
        }

        sd_gcode_result_t result = sd_gcode_reader_open(filename);
        if (result != SD_GCODE_OK)
        {
            send_error_response("Failed to open G-code file");
            return;
        }

        uart_commands_send_response_printf("| Starting print: %s |", filename);

        // Start processing G-code moves
        gcode_move_buffer_set_state(GCODE_MOVE_BUFFER_STATE_PROCESSING);
        sd_printing = true;
        last_sd_read_time = 0; // Force immediate read

        // Read and buffer initial moves
        char line_buffer[SD_GCODE_MAX_LINE_LENGTH];
        position_t current_pos = kinematics_get_position();
        parser_state_t *parser_state = parser_get_state();

        int lines_buffered = 0;
        while (!sd_gcode_reader_is_eof() && lines_buffered < 20 &&
               !gcode_move_buffer_is_full())
        {
            result = sd_gcode_reader_read_line(line_buffer, sizeof(line_buffer));
            if (result == SD_GCODE_OK)
            {
                kinematics_update();
                STEPPER_MOTOR_update();

                if (buf_result == GCODE_MOVE_BUFFER_OK)
                {
                    lines_buffered++;
                }
                else if (buf_result == GCODE_MOVE_BUFFER_FULL)
                {
                    uart_commands_send_response("| Movement timeout - stopping motors |");
                    STEPPER_MOTOR_emergency_stop_all();
                    kinematics_stop();
                    send_error_response("Movement timeout");
                    stats.errors_count++;
                    return;
                }
            }
            else if (result == SD_GCODE_END_OF_FILE)
            {
                break;
            }
        }

        uart_commands_send_response_printf("| Buffered %d moves |", lines_buffered);
        send_ok_response();
    }
    else if (strncmp(line, "sdstop", 6) == 0)
    {
        if (sd_printing)
        {
            sd_printing = false;
            sd_gcode_reader_close();
            gcode_move_buffer_set_state(GCODE_MOVE_BUFFER_STATE_IDLE);
            gcode_move_buffer_clear();
            kinematics_stop();
            uart_commands_send_response("| SD card printing stopped |");
        }
        else
        {
            uart_commands_send_response("| No SD card printing active |");
        }
        send_ok_response();
    }
    else if (strncmp(line, "sdstatus", 8) == 0)
    {
        if (sd_gcode_reader_is_ready())
        {
            if (sd_printing)
            {
                sd_gcode_file_info_t *info = sd_gcode_reader_get_info();
                if (info && info->is_open)
                {
                    uart_commands_send_response_printf("| Printing: %s |", info->filename);
                    uart_commands_send_response_printf("| Progress: %d%% |", info->progress_percent);
                    uart_commands_send_response_printf("| Lines: %lu |", info->lines_processed);
                }
                else
                {
                    uart_commands_send_response("| SD printing active but no file info |");
                }
            }
            else
            {
                uart_commands_send_response("| SD card ready, not printing |");
            }
        }
        else
        {
            uart_commands_send_response("| SD card not ready |");
        }
        send_ok_response();
    }
    else if (strncmp(line, "motors", 6) == 0)
    {
        uart_commands_send_response("| Motor Status: |");
        uart_commands_send_response("| Not implemented |");
        send_ok_response();
    }
    else if (strncmp(line, "limits", 6) == 0)
    {
        uart_commands_send_response("| Limit Switch Status: |");
        uart_commands_send_response("| Not implemented |");
        send_ok_response();
    }
    else
    {
        send_error_response("Unknown special command");
    }
}

/**
 * @brief Process G-code commands (now executes directly)
 */
static void process_gcode_command(const char *line)
{
    // Execute G-code directly using parser
    printf("Executing G-code: %s\n", line);

    if (parser_process_line(line))
    {
        send_ok_response();
    }
    else
    {
        send_error_response("G-code execution failed");
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
    uart_commands_send_response_printf("|------------------------------------|");
    uart_commands_send_response_printf("| Error: %s |", error);
    uart_commands_send_response_printf("|------------------------------------|");
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

/**
 * @brief Check if SD card printing is active
 */
bool uart_commands_is_sd_printing(void)
{
    return sd_printing;
}

/**
 * @brief Process emergency stop command
 */
static void process_emergency_stop(void)
{
    printf("- EMERGENCY STOP ACTIVATED\n");

    // Stop all motors using emergency stop function
    stepper_motor_emergency_stop_all();

    // Stop kinematics
    kinematics_stop();

    // Clear buffers
    command_buffer_clear();
    gcode_move_buffer_clear();

    // Stop SD card operations
    if (sd_printing)
    {
        sd_printing = false;
        sd_gcode_reader_close();
        gcode_move_buffer_set_state(GCODE_MOVE_BUFFER_STATE_IDLE);
    }

    uart_commands_send_response("|---- EMERGENCY STOP ACTIVATED ----|");
    uart_commands_send_response("|---- ALL MOTORS STOPPED ----|");

    // Make sure to reset the emergency stop detection flag
    potential_emergency_stop = false;
}
