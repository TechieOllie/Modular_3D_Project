/**
 *******************************************************************************
 * @file    parser.h
 * @author  GitHub Copilot
 * @date    May 27, 2025
 * @brief   G-code parser header for 3D printer/CNC control system
 *******************************************************************************
 */

#ifndef PARSER_H
#define PARSER_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "kinematics.h"

/* Exported constants --------------------------------------------------------*/
#define MAX_LINE_LENGTH 128

/* Exported types ------------------------------------------------------------*/

/**
 * @brief G-code command types
 */
typedef enum
{
    GCODE_G = 0, // Motion and machine control
    GCODE_M = 1, // Miscellaneous functions
    GCODE_T = 2, // Tool selection
} gcode_type_t;

/**
 * @brief G-code command structure
 */
typedef struct
{
    gcode_type_t type; // Command type (G, M, T)
    uint16_t code;     // Command code number

    // Parameter flags
    bool has_x;
    bool has_y;
    bool has_z;
    bool has_e;
    bool has_f;
    bool has_s;
    bool has_p;

    // Parameter values
    float x; // X coordinate
    float y; // Y coordinate
    float z; // Z coordinate (not used in 2D)
    float e; // Extruder position (not used)
    float f; // Feedrate
    float s; // Spindle speed / parameter S
    float p; // Parameter P (pause time, etc.)

} gcode_command_t;

/**
 * @brief Parser state structure
 */
typedef struct
{
    position_t current_position; // Current machine position
    float feedrate;              // Current feedrate (mm/min)
    bool absolute_mode;          // True for absolute, false for relative
    bool units_mm;               // True for mm, false for inches
    uint32_t line_number;        // Current line number
    uint8_t tool_number;         // Current tool number
    uint16_t extruder_temp;      // Target extruder temperature
    uint16_t bed_temp;           // Target bed temperature
    uint8_t fan_speed;           // Fan speed (0-255)
} parser_state_t;

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief Initialize the G-code parser
 * @return true if successful, false otherwise
 */
bool parser_init(void);

/**
 * @brief Process incoming character from serial communication
 * @param c Character to process
 * @return true if character processed successfully
 */
bool parser_process_char(char c);

/**
 * @brief Process a complete G-code line
 * @param line Null-terminated G-code line
 * @return true if line processed successfully
 */
bool parser_process_line(const char *line);

/**
 * @brief Execute G-code commands from string buffer
 * @param gcode G-code string (can contain multiple lines)
 * @return true if all commands executed successfully
 */
bool parser_execute_gcode(const char *gcode);

/**
 * @brief Get current parser state
 * @return Pointer to parser state structure
 */
parser_state_t *parser_get_state(void);

/**
 * @brief Update parser with current machine position
 * @param pos Current position from kinematics system
 */
void parser_update_position(position_t pos);

/* Common G-code commands for reference:
 *
 * Motion Commands:
 * G0 - Rapid positioning (linear move at maximum speed)
 * G1 - Linear interpolation (linear move at specified feedrate)
 * G4 - Dwell (pause for specified time)
 * G28 - Home axes
 * G90 - Absolute positioning mode
 * G91 - Relative positioning mode
 * G92 - Set position (coordinate system offset)
 *
 * Machine Commands:
 * M0/M1 - Program stop
 * M17 - Enable steppers
 * M18/M84 - Disable steppers
 * M104 - Set extruder temperature
 * M105 - Get temperatures
 * M106 - Fan on
 * M107 - Fan off
 * M109 - Wait for extruder temperature
 * M114 - Get current position
 * M115 - Get firmware version
 * M140 - Set bed temperature
 * M190 - Wait for bed temperature
 * M400 - Wait for moves to complete
 *
 * Tool Commands:
 * T0, T1, etc. - Select tool
 *
 * Example usage:
 * G28          ; Home all axes
 * G90          ; Absolute positioning
 * G1 X10 Y10 F1200  ; Move to (10,10) at 1200 mm/min
 * G1 X20 F600  ; Move to X=20 at 600 mm/min
 * M400         ; Wait for moves to complete
 */

#endif /* PARSER_H */
