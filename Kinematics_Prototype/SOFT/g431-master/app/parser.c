/**
 *******************************************************************************
 * @file 	parser.c
 * @author 	naej, ol, your name
 * @date 	Current Date
 * @brief	G-code parser implementation
 *******************************************************************************
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include "parser.h"
#include <stdio.h>

void parse_gcode_line(const char *line, GCodeCommand *cmd) {
    memset(cmd, 0, sizeof(GCodeCommand)); // Zero everything
    cmd->g_code = -1; // Invalid G-code by default

    const char *ptr = line;
    while (*ptr) {
        // Skip whitespace and comments
        if (isspace(*ptr)) {
            ptr++;
            continue;
        }
        
        // Skip comments that start with '(' or ';'
        if (*ptr == '(' || *ptr == ';') {
            break;
        }
        
        // Parse letter-number pairs
        char letter = toupper(*ptr++);
        
        // Skip if not a valid letter code
        if (!isalpha(letter)) {
            continue;
        }
        
        // Read the number after the letter
        char buffer[32] = {0};
        int i = 0;
        
        while (*ptr && (isdigit(*ptr) || *ptr == '.' || *ptr == '-')) {
            buffer[i++] = *ptr++;
        }
        
        if (i == 0) {
            continue; // No number after letter, skip
        }
        
        float value = atof(buffer);
        
        // Process the letter-number pair
        switch (letter) {
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
            // Add other G-code parameters as needed
        }
    }
}

bool parse_gcode_file(const char *filename, void (*callback)(GCodeCommand *cmd)) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        printf("Failed to open G-code file: %s\n", filename);
        return false;
    }
    
    char line[256];
    GCodeCommand cmd;
    
    while (fgets(line, sizeof(line), file)) {
        // Remove trailing newline
        size_t len = strlen(line);
        if (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r')) {
            line[len-1] = '\0';
        }
        
        // Parse the line
        parse_gcode_line(line, &cmd);
        
        // Only process valid G-codes
        if (cmd.g_code >= 0) {
            callback(&cmd);
        }
    }
    
    fclose(file);
    return true;
}

void process_gcode_string(const char *gcode, void (*callback)(GCodeCommand *cmd)) {
    char line[256];
    size_t i = 0;
    
    while (*gcode) {
        if (*gcode == '\n' || *gcode == '\r' || *gcode == '\0') {
            line[i] = '\0';
            if (i > 0) { // Non-empty line
                GCodeCommand cmd;
                parse_gcode_line(line, &cmd);
                if (cmd.g_code >= 0) {
                    callback(&cmd);
                }
            }
            i = 0;
        } else {
            if (i < sizeof(line) - 1) {
                line[i++] = *gcode;
            }
        }
        gcode++;
    }
    
    // Process the last line if there's no trailing newline
    if (i > 0) {
        line[i] = '\0';
        GCodeCommand cmd;
        parse_gcode_line(line, &cmd);
        if (cmd.g_code >= 0) {
            callback(&cmd);
        }
    }
}