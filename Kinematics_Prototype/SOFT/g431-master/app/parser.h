/**
 *******************************************************************************
 * @file 	parser.h
 * @author 	naej, ol, your name
 * @date 	Current Date
 * @brief	G-code parser header
 *******************************************************************************
 */

#ifndef PARSER_H
#define PARSER_H

#include <stdbool.h>

#define MAX_GCODE_PARAMS 10

// Structure to represent a G-code parameter (e.g., X100, Y200)
typedef struct
{
    char letter; // Parameter letter (X, Y, Z, etc.)
    float value; // Parameter value
} GCodeParam;

typedef struct
{
    int g_code;                      // e.g. 0 (rapid) or 1 (linear move)
    float x, y, z;                   // target coordinates
    float feedrate;                  // F code
    bool has_x, has_y, has_z, has_f; // flags for optional fields

    // Parameters collection for more complex G-codes like G28
    GCodeParam params[MAX_GCODE_PARAMS];
    int numParams;
} GCodeCommand;

// Parse a single G-code line
void parse_gcode_line(const char *line, GCodeCommand *cmd);

// Parse a G-code file and call the callback function for each command
bool parse_gcode_file(const char *filename, void (*callback)(GCodeCommand *cmd));

// Process a string containing G-code commands
void process_gcode_string(const char *gcode, void (*callback)(GCodeCommand *cmd));

/**
 * Parse a G-code command string and fill a GCodeCommand structure
 * @param cmd_str The G-code command string to parse
 * @param cmd Pointer to a GCodeCommand structure to fill
 * @return true if parsing was successful, false otherwise
 */
bool parse_gcode(const char *cmd_str, GCodeCommand *cmd);

#endif /* PARSER_H */