#include "parser.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>

// Error message buffer
static char error_message[80];

// Initialize the parser
void parser_init(void)
{
    // Initialize any parser state here if needed
    error_message[0] = '\0';
}

// Get the last error message
const char *parser_get_error(void)
{
    return error_message;
}

// Helper function to clear a command struct
static void clear_command(parser_command_t *command)
{
    command->type = COMMAND_TYPE_NONE;
    command->code = 0;
    command->params.params = 0;
    command->params.x = 0.0f;
    command->params.y = 0.0f;
    command->params.z = 0.0f;
    command->params.feed_rate = 0.0f;
    command->params.i = 0.0f;
    command->params.j = 0.0f;
    command->params.p = 0.0f;
    command->valid = false;
}

// Parse a line of G-code
bool parser_parse_line(const char *line, parser_command_t *command)
{
    if (!line || !command)
    {
        sprintf(error_message, "Invalid input");
        return false;
    }

    // Clear the command
    clear_command(command);

    // Skip leading whitespace
    while (isspace((unsigned char)*line))
        line++;

    // Check for empty line or comment
    if (*line == '\0' || *line == ';' || *line == '(')
    {
        return false;
    }

    // Parse the command type (G or M)
    if (toupper((unsigned char)*line) == 'G')
    {
        command->type = COMMAND_TYPE_G;
        line++;
    }
    else if (toupper((unsigned char)*line) == 'M')
    {
        command->type = COMMAND_TYPE_M;
        line++;
    }
    else
    {
        sprintf(error_message, "Unknown command type: %c", *line);
        return false;
    }

    // Parse the command code number
    char *end;
    command->code = (uint8_t)strtol(line, &end, 10);
    if (line == end)
    {
        sprintf(error_message, "Invalid command code");
        return false;
    }
    line = end;

    // Parse parameters
    while (*line != '\0' && *line != ';')
    {
        // Skip whitespace
        while (isspace((unsigned char)*line))
            line++;

        if (*line == '\0' || *line == ';')
            break;

        // Get parameter letter
        char param = toupper((unsigned char)*line++);

        // Skip any whitespace between letter and value
        while (isspace((unsigned char)*line))
            line++;

        // Parse parameter value
        if (*line == '\0' || *line == ';')
        {
            sprintf(error_message, "Missing value for parameter %c", param);
            return false;
        }

        float value = strtof(line, &end);
        if (line == end)
        {
            sprintf(error_message, "Invalid value for parameter %c", param);
            return false;
        }
        line = end;

        // Set parameter in command
        switch (param)
        {
        case 'X':
            command->params.params |= PARAM_X;
            command->params.x = value;
            break;
        case 'Y':
            command->params.params |= PARAM_Y;
            command->params.y = value;
            break;
        case 'Z':
            command->params.params |= PARAM_Z;
            command->params.z = value;
            break;
        case 'F':
            command->params.params |= PARAM_F;
            command->params.feed_rate = value;
            break;
        case 'I':
            command->params.params |= PARAM_I;
            command->params.i = value;
            break;
        case 'J':
            command->params.params |= PARAM_J;
            command->params.j = value;
            break;
        case 'P':
            command->params.params |= PARAM_P;
            command->params.p = value;
            break;
        default:
            // Ignore unknown parameters
            break;
        }
    }

    // Command is valid
    command->valid = true;
    return true;
}