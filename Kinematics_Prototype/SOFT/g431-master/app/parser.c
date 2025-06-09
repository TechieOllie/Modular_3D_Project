/**
 *******************************************************************************
 * @file 	parser.c
 * @author 	naej, ol, your name
 * @date 	Current Date
 * @brief	G-code parser implementation
 *******************************************************************************
 */

#include "config.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include "parser.h"
#include <stdio.h>

void parse_gcode_line(const char *line, GCodeCommand *cmd)
{
    memset(cmd, 0, sizeof(GCodeCommand)); // Zero everything
    cmd->g_code = -1;                     // Invalid G-code by default
    cmd->numParams = 0;                   // Start with no parameters

    const char *ptr = line;
    while (*ptr)
    {
        // Skip whitespace and comments
        if (isspace(*ptr))
        {
            ptr++;
            continue;
        }

        // Skip comments that start with '(' or ';'
        if (*ptr == '(' || *ptr == ';')
        {
            break;
        }

        // Parse letter-number pairs
        char letter = toupper(*ptr++);

        // Skip if not a valid letter code
        if (!isalpha(letter))
        {
            continue;
        }

        // Read the number after the letter
        char buffer[32] = {0};
        int i = 0;

        while (*ptr && (isdigit(*ptr) || *ptr == '.' || *ptr == '-'))
        {
            buffer[i++] = *ptr++;
        }

        if (i == 0)
        {
            // No number after letter, add it as a parameter anyway
            // This handles bare parameters like "X" in "G28 X"
            if (cmd->numParams < MAX_GCODE_PARAMS)
            {
                cmd->params[cmd->numParams].letter = letter;
                cmd->params[cmd->numParams].value = 0;
                cmd->numParams++;
            }
            continue;
        }

        float value = atof(buffer);

        // Store as a parameter for all letters
        if (cmd->numParams < MAX_GCODE_PARAMS)
        {
            cmd->params[cmd->numParams].letter = letter;
            cmd->params[cmd->numParams].value = value;
            cmd->numParams++;
        }

        // Process the letter-number pair
        switch (letter)
        {
        case 'G':
            cmd->g_code = (int)value;
            break;
        case 'X':
            cmd->x = value;
            cmd->has_x = true;
            break;
        case 'Y':
            cmd->y = value;
            cmd->has_y = true;
            break;
        case 'Z':
            cmd->z = value;
            cmd->has_z = true;
            break;
        case 'F':
            cmd->feedrate = value;
            cmd->has_f = true;
            break;
            // Other parameters are already stored in the params array
        }
    }
}

/**
 * Parse a G-code command string and fill a GCodeCommand structure
 * @param cmd_str The G-code command string to parse
 * @param cmd Pointer to a GCodeCommand structure to fill
 * @return true if parsing was successful, false otherwise
 */
bool parse_gcode(const char *cmd_str, GCodeCommand *cmd)
{
    // Initialize the command structure
    memset(cmd, 0, sizeof(GCodeCommand));
    cmd->g_code = -1; // Invalid G-code by default

    // Skip leading whitespace
    while (*cmd_str && isspace(*cmd_str))
    {
        cmd_str++;
    }

    // Check if the string is empty or only whitespace
    if (!*cmd_str)
    {
        return false;
    }

    // Parse the command line
    parse_gcode_line(cmd_str, cmd);

    // Return true if a valid G-code was found
    return (cmd->g_code >= 0);
}

bool parse_gcode_file(const char *filename, void (*callback)(GCodeCommand *cmd))
{
    FILE *file = fopen(filename, "r");
    if (!file)
    {
        printf("Failed to open G-code file: %s\n", filename);
        return false;
    }

    char line[256];
    GCodeCommand cmd;

    while (fgets(line, sizeof(line), file))
    {
        // Remove trailing newline
        size_t len = strlen(line);
        if (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r'))
        {
            line[len - 1] = '\0';
        }

        // Parse the line
        parse_gcode_line(line, &cmd);

        // Only process valid G-codes
        if (cmd.g_code >= 0)
        {
            callback(&cmd);
        }
    }

    fclose(file);
    return true;
}

void process_gcode_string(const char *gcode, void (*callback)(GCodeCommand *cmd))
{
    char line[256];
    size_t i = 0;

    while (*gcode)
    {
        if (*gcode == '\n' || *gcode == '\r' || *gcode == '\0')
        {
            line[i] = '\0';
            if (i > 0)
            { // Non-empty line
                GCodeCommand cmd;
                parse_gcode_line(line, &cmd);
                if (cmd.g_code >= 0)
                {
                    callback(&cmd);
                }
            }
            i = 0;
        }
        else
        {
            if (i < sizeof(line) - 1)
            {
                line[i++] = *gcode;
            }
        }
        gcode++;
    }

    // Process the last line if there's no trailing newline
    if (i > 0)
    {
        line[i] = '\0';
        GCodeCommand cmd;
        parse_gcode_line(line, &cmd);
        if (cmd.g_code >= 0)
        {
            callback(&cmd);
        }
    }
}