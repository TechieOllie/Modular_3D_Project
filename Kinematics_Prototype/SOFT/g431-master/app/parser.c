/**
 *******************************************************************************
 * @file    parser.c
 * @author  GitHub Copilot
 * @date    May 27, 2025
 * @brief   G-code parser for 3D printer/CNC control system
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "parser.h"
#include "kinematics.h"
#include "stepper_motor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/
static parser_state_t parser_state;
static char line_buffer[MAX_LINE_LENGTH];
static uint8_t buffer_index = 0;

/* Private function prototypes -----------------------------------------------*/
static bool parse_gcode_line(const char *line);
static bool execute_g_command(gcode_command_t *cmd);
static bool execute_m_command(gcode_command_t *cmd);
static bool execute_t_command(gcode_command_t *cmd);
static float parse_parameter(const char *line, char param_char);
static bool has_parameter(const char *line, char param_char);
static void reset_command(gcode_command_t *cmd);
static bool validate_position(float x, float y);
static void send_ok_response(void);
static void send_error_response(const char *error_msg);

/* Public function implementations -------------------------------------------*/

/**
 * @brief Initialize the G-code parser
 */
bool parser_init(void)
{
    // Initialize parser state
    parser_state.current_position.x = 0.0f;
    parser_state.current_position.y = 0.0f;
    parser_state.feedrate = 1000.0f; // Default 1000 mm/min
    parser_state.absolute_mode = true;
    parser_state.units_mm = true;
    parser_state.line_number = 0;
    parser_state.tool_number = 0;
    parser_state.extruder_temp = 0;
    parser_state.bed_temp = 0;
    parser_state.fan_speed = 0;

    buffer_index = 0;
    memset(line_buffer, 0, sizeof(line_buffer));

    printf("G-code parser initialized\n");
    return true;
}

/**
 * @brief Process incoming character from serial communication
 */
bool parser_process_char(char c)
{
    // Handle different line endings
    if (c == '\n' || c == '\r')
    {
        if (buffer_index > 0)
        {
            line_buffer[buffer_index] = '\0';
            bool result = parse_gcode_line(line_buffer);

            // Reset buffer for next line
            buffer_index = 0;
            memset(line_buffer, 0, sizeof(line_buffer));

            return result;
        }
        return true;
    }

    // Ignore control characters except newlines
    if (c < 32)
    {
        return true;
    }

    // Check buffer overflow
    if (buffer_index >= MAX_LINE_LENGTH - 1)
    {
        send_error_response("Line too long");
        buffer_index = 0;
        return false;
    }

    // Add character to buffer
    line_buffer[buffer_index++] = toupper(c);
    return true;
}

/**
 * @brief Process a complete G-code line
 */
bool parser_process_line(const char *line)
{
    return parse_gcode_line(line);
}

/**
 * @brief Get current parser state
 */
parser_state_t *parser_get_state(void)
{
    return &parser_state;
}

/**
 * @brief Update parser with current machine position
 */
void parser_update_position(position_t pos)
{
    parser_state.current_position = pos;
}

/* Private function implementations ------------------------------------------*/

/**
 * @brief Parse a complete G-code line
 */
static bool parse_gcode_line(const char *line)
{
    if (!line || strlen(line) == 0)
    {
        send_ok_response();
        return true;
    }

    printf("Parsing: %s\n", line);

    gcode_command_t cmd;
    reset_command(&cmd);

    // Skip whitespace
    while (*line && isspace(*line))
        line++;

    // Skip empty lines and comments
    if (*line == '\0' || *line == ';' || *line == '(')
    {
        send_ok_response();
        return true;
    }

    // Parse line number if present
    if (*line == 'N')
    {
        parser_state.line_number = (uint32_t)parse_parameter(line, 'N');
        // Skip to next part
        while (*line && *line != ' ' && *line != 'G' && *line != 'M' && *line != 'T')
            line++;
        while (*line && isspace(*line))
            line++;
    }

    // Determine command type
    if (*line == 'G')
    {
        cmd.type = GCODE_G;
        cmd.code = (uint16_t)parse_parameter(line, 'G');
    }
    else if (*line == 'M')
    {
        cmd.type = GCODE_M;
        cmd.code = (uint16_t)parse_parameter(line, 'M');
    }
    else if (*line == 'T')
    {
        cmd.type = GCODE_T;
        cmd.code = (uint16_t)parse_parameter(line, 'T');
    }
    else
    {
        send_error_response("Unknown command");
        return false;
    }

    // Parse parameters
    cmd.has_x = has_parameter(line, 'X');
    cmd.has_y = has_parameter(line, 'Y');
    cmd.has_f = has_parameter(line, 'F');
    cmd.has_s = has_parameter(line, 'S');
    cmd.has_p = has_parameter(line, 'P');

    if (cmd.has_x)
        cmd.x = parse_parameter(line, 'X');
    if (cmd.has_y)
        cmd.y = parse_parameter(line, 'Y');
    if (cmd.has_f)
        cmd.f = parse_parameter(line, 'F');
    if (cmd.has_s)
        cmd.s = parse_parameter(line, 'S');
    if (cmd.has_p)
        cmd.p = parse_parameter(line, 'P');

    // Execute command
    bool result = false;
    switch (cmd.type)
    {
    case GCODE_G:
        result = execute_g_command(&cmd);
        break;
    case GCODE_M:
        result = execute_m_command(&cmd);
        break;
    case GCODE_T:
        result = execute_t_command(&cmd);
        break;
    default:
        send_error_response("Invalid command type");
        return false;
    }

    if (result)
    {
        send_ok_response();
    }

    return result;
}

/**
 * @brief Execute G commands (motion and machine control)
 */
static bool execute_g_command(gcode_command_t *cmd)
{
    position_t target_pos;
    float feedrate;

    switch (cmd->code)
    {
    case 0: // G0 - Rapid positioning
    case 1: // G1 - Linear interpolation
    {
        // Calculate target position
        if (parser_state.absolute_mode)
        {
            target_pos.x = cmd->has_x ? cmd->x : parser_state.current_position.x;
            target_pos.y = cmd->has_y ? cmd->y : parser_state.current_position.y;
        }
        else
        {
            target_pos.x = parser_state.current_position.x + (cmd->has_x ? cmd->x : 0.0f);
            target_pos.y = parser_state.current_position.y + (cmd->has_y ? cmd->y : 0.0f);
        }

        // Validate position
        if (!validate_position(target_pos.x, target_pos.y))
        {
            send_error_response("Position out of bounds");
            return false;
        }

        // Update feedrate if specified
        if (cmd->has_f)
        {
            parser_state.feedrate = cmd->f;
        }

        feedrate = (cmd->code == 0) ? 6000.0f : parser_state.feedrate; // G0 uses rapid speed

        printf("Moving to X=%.2f Y=%.2f at F=%.1f\n",
               target_pos.x, target_pos.y, feedrate);

        // Execute move
        if (!kinematics_move_to(&target_pos, feedrate))
        {
            send_error_response("Move failed");
            return false;
        }

        parser_state.current_position = target_pos;
        break;
    }

    case 4: // G4 - Dwell (pause)
    {
        uint32_t dwell_ms = 0;
        if (cmd->has_p)
        {
            dwell_ms = (uint32_t)(cmd->p * 1000); // P parameter in seconds
        }
        else if (cmd->has_s)
        {
            dwell_ms = (uint32_t)(cmd->s * 1000); // S parameter in seconds
        }

        printf("Dwelling for %lu ms\n", dwell_ms);
        HAL_Delay(dwell_ms);
        break;
    }

    case 28: // G28 - Home axes
    {
        printf("Homing axes\n");
        if (!kinematics_home_all())
        {
            send_error_response("Homing failed");
            return false;
        }
        parser_state.current_position.x = 0.0f;
        parser_state.current_position.y = 0.0f;
        break;
    }

    case 90: // G90 - Absolute positioning
    {
        parser_state.absolute_mode = true;
        printf("Absolute positioning mode\n");
        break;
    }

    case 91: // G91 - Relative positioning
    {
        parser_state.absolute_mode = false;
        printf("Relative positioning mode\n");
        break;
    }

    case 92: // G92 - Set position
    {
        if (cmd->has_x)
            parser_state.current_position.x = cmd->x;
        if (cmd->has_y)
            parser_state.current_position.y = cmd->y;

        printf("Position set to X=%.2f Y=%.2f\n",
               parser_state.current_position.x, parser_state.current_position.y);

        // Update kinematics with new position
        kinematics_set_position(&parser_state.current_position);
        break;
    }

    default:
        printf("Unsupported G command: G%d\n", cmd->code);
        send_error_response("Unsupported G command");
        return false;
    }

    return true;
}

/**
 * @brief Execute M commands (miscellaneous functions)
 */
static bool execute_m_command(gcode_command_t *cmd)
{
    switch (cmd->code)
    {
    case 0: // M0 - Program stop
    case 1: // M1 - Optional stop
    {
        printf("Program stop\n");
        kinematics_stop();
        break;
    }

    case 17: // M17 - Enable steppers
    {
        printf("Steppers enabled\n");
        // Note: Our stepper drivers are always enabled
        break;
    }

    case 18: // M18 - Disable steppers
    case 84: // M84 - Disable steppers
    {
        printf("Steppers disabled\n");
        kinematics_stop();
        break;
    }

    case 104: // M104 - Set extruder temperature
    {
        if (cmd->has_s)
        {
            parser_state.extruder_temp = (uint16_t)cmd->s;
            printf("Set extruder temperature to %d°C\n", parser_state.extruder_temp);
            // TODO: Implement temperature control
        }
        break;
    }

    case 105: // M105 - Get temperatures
    {
        printf("T:%d/%d B:%d/0\n",
               20, parser_state.extruder_temp, // Current temp (fake) / target
               25);                            // Bed temp (fake)
        break;
    }

    case 106: // M106 - Fan on
    {
        parser_state.fan_speed = cmd->has_s ? (uint8_t)cmd->s : 255;
        printf("Fan speed set to %d\n", parser_state.fan_speed);
        // TODO: Implement fan control
        break;
    }

    case 107: // M107 - Fan off
    {
        parser_state.fan_speed = 0;
        printf("Fan turned off\n");
        break;
    }

    case 109: // M109 - Wait for extruder temperature
    {
        if (cmd->has_s)
        {
            parser_state.extruder_temp = (uint16_t)cmd->s;
            printf("Waiting for extruder temperature %d°C\n", parser_state.extruder_temp);
            // TODO: Implement temperature waiting
            HAL_Delay(1000); // Simulate heating time
        }
        break;
    }

    case 114: // M114 - Get current position
    {
        position_t pos = kinematics_get_position();
        printf("X:%.2f Y:%.2f Z:0.00 E:0.00\n", pos.x, pos.y);
        break;
    }

    case 115: // M115 - Get firmware version
    {
        printf("FIRMWARE_NAME:CartesianCNC FIRMWARE_VERSION:1.0.0\n");
        break;
    }

    case 140: // M140 - Set bed temperature
    {
        if (cmd->has_s)
        {
            parser_state.bed_temp = (uint16_t)cmd->s;
            printf("Set bed temperature to %d°C\n", parser_state.bed_temp);
            // TODO: Implement bed temperature control
        }
        break;
    }

    case 190: // M190 - Wait for bed temperature
    {
        if (cmd->has_s)
        {
            parser_state.bed_temp = (uint16_t)cmd->s;
            printf("Waiting for bed temperature %d°C\n", parser_state.bed_temp);
            // TODO: Implement bed temperature waiting
            HAL_Delay(1000); // Simulate heating time
        }
        break;
    }

    case 400: // M400 - Wait for moves to complete
    {
        printf("Waiting for moves to complete\n");
        while (kinematics_get_state() != MOVE_STATE_IDLE)
        {
            kinematics_update();
            stepper_motor_update();
            HAL_Delay(10);
        }
        break;
    }

    default:
        printf("Unsupported M command: M%d\n", cmd->code);
        send_error_response("Unsupported M command");
        return false;
    }

    return true;
}

/**
 * @brief Execute T commands (tool selection)
 */
static bool execute_t_command(gcode_command_t *cmd)
{
    parser_state.tool_number = cmd->code;
    printf("Tool %d selected\n", parser_state.tool_number);
    // TODO: Implement tool changing logic
    return true;
}

/**
 * @brief Parse a parameter value from the command line
 */
static float parse_parameter(const char *line, char param_char)
{
    const char *ptr = strchr(line, param_char);
    if (!ptr)
    {
        return 0.0f;
    }

    ptr++; // Skip parameter character
    return strtof(ptr, NULL);
}

/**
 * @brief Check if a parameter exists in the command line
 */
static bool has_parameter(const char *line, char param_char)
{
    return strchr(line, param_char) != NULL;
}

/**
 * @brief Reset command structure to default values
 */
static void reset_command(gcode_command_t *cmd)
{
    memset(cmd, 0, sizeof(gcode_command_t));
}

/**
 * @brief Validate that position is within machine limits
 */
static bool validate_position(float x, float y)
{
    // Get actual machine limits from kinematics system
    machine_config_t *config = kinematics_get_config();
    if (!config)
    {
        // Fallback to default limits if config not available
        return (x >= 0.0f && x <= 200.0f && y >= 0.0f && y <= 200.0f);
    }

    // Check if position is within configured machine limits
    bool x_valid = (x >= 0.0f && x <= config->x_max);
    bool y_valid = (y >= 0.0f && y <= config->y_max);

    if (!x_valid)
    {
        printf("X position %.2f out of bounds (0.0 to %.2f)\n", x, config->x_max);
    }

    if (!y_valid)
    {
        printf("Y position %.2f out of bounds (0.0 to %.2f)\n", y, config->y_max);
    }

    return x_valid && y_valid;
}

/**
 * @brief Send OK response
 */
static void send_ok_response(void)
{
    printf("ok\n");
}

/**
 * @brief Send error response
 */
static void send_error_response(const char *error_msg)
{
    printf("Error: %s\n", error_msg);
}

/**
 * @brief Process G-code commands from string buffer (for testing)
 */
bool parser_execute_gcode(const char *gcode)
{
    if (!gcode)
    {
        return false;
    }

    printf("Executing G-code: %s\n", gcode);

    // Split by lines if multiple commands
    char *line_copy = malloc(strlen(gcode) + 1);
    if (!line_copy)
    {
        return false;
    }

    strcpy(line_copy, gcode);

    char *line = strtok(line_copy, "\n\r");
    bool result = true;

    while (line && result)
    {
        result = parse_gcode_line(line);

        // Wait for move to complete before next command
        if (result && kinematics_get_state() != MOVE_STATE_IDLE)
        {
            uint32_t start_time = HAL_GetTick();
            uint32_t timeout = 30000; // 30 second timeout

            while (kinematics_get_state() != MOVE_STATE_IDLE)
            {
                kinematics_update();
                stepper_motor_update();

                if (HAL_GetTick() - start_time > timeout)
                {
                    printf("Timeout waiting for move to complete\n");
                    kinematics_stop();
                    result = false;
                    break;
                }

                HAL_Delay(10);
            }
        }

        line = strtok(NULL, "\n\r");
    }

    free(line_copy);
    return result;
}
