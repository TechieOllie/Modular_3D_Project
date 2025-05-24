#ifndef PARSER_H_
#define PARSER_H_

#include <stdint.h>
#include <stdbool.h>

// Command types
typedef enum
{
    COMMAND_TYPE_NONE = 0,
    COMMAND_TYPE_G,
    COMMAND_TYPE_M,
} command_type_t;

// Command parameters bitfield
typedef struct
{
    uint16_t params; // Bitfield of which parameters are present
    float x;         // X coordinate
    float y;         // Y coordinate
    float z;         // Z coordinate
    float feed_rate; // F parameter - feedrate
    float i;         // I parameter - arc center X offset
    float j;         // J parameter - arc center Y offset
    float p;         // P parameter - dwell time
} command_params_t;

// Command structure
typedef struct
{
    command_type_t type;     // G or M command
    uint8_t code;            // Command number (e.g. 0 for G0)
    command_params_t params; // Command parameters
    bool valid;              // Whether the command is valid
} parser_command_t;

// Parameter bit definitions
#define PARAM_X (1 << 0)
#define PARAM_Y (1 << 1)
#define PARAM_Z (1 << 2)
#define PARAM_F (1 << 3)
#define PARAM_I (1 << 4)
#define PARAM_J (1 << 5)
#define PARAM_P (1 << 6)

// Function prototypes
void parser_init(void);
bool parser_parse_line(const char *line, parser_command_t *command);
const char *parser_get_error(void);

#endif /* PARSER_H_ */