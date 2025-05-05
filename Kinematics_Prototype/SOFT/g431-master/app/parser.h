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

typedef struct {
    int g_code;         // e.g. 0 (rapid) or 1 (linear move)
    float x, y, z;      // target coordinates
    float feedrate;     // F code
    bool has_x, has_y, has_z, has_f;  // flags for optional fields
} GCodeCommand;

// Parse a single G-code line
void parse_gcode_line(const char *line, GCodeCommand *cmd);

// Parse a G-code file and call the callback function for each command
bool parse_gcode_file(const char *filename, void (*callback)(GCodeCommand *cmd));

// Process a string containing G-code commands
void process_gcode_string(const char *gcode, void (*callback)(GCodeCommand *cmd));

#endif /* PARSER_H */