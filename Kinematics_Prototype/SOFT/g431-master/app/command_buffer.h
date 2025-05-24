#ifndef COMMAND_BUFFER_H_
#define COMMAND_BUFFER_H_

#include "parser.h"
#include <stdint.h>
#include <stdbool.h>

// Define buffer size (adjust based on your memory constraints)
#define CMD_BUFFER_SIZE 16

// Buffer status
typedef enum {
    BUFFER_EMPTY = 0,
    BUFFER_READY,
    BUFFER_FULL,
    BUFFER_ERROR
} buffer_status_t;

// Command buffer structure
typedef struct {
    parser_command_t commands[CMD_BUFFER_SIZE];
    uint16_t head;                   // Index for adding new commands
    uint16_t tail;                   // Index for retrieving commands
    uint16_t count;                  // Number of commands in buffer
    buffer_status_t status;          // Current buffer status
    char error_message[80];          // Error message if status is BUFFER_ERROR
    bool sd_file_open;               // Whether an SD file is currently open
    char current_filename[32];       // Currently open file
    uint32_t file_position;          // Current position in file
    uint32_t line_number;            // Current line number in file
} command_buffer_t;

// Function prototypes
void cmd_buffer_init(command_buffer_t* buffer);
buffer_status_t cmd_buffer_open_file(command_buffer_t* buffer, const char* filename);
void cmd_buffer_close_file(command_buffer_t* buffer);
buffer_status_t cmd_buffer_fill(command_buffer_t* buffer);
bool cmd_buffer_has_command(command_buffer_t* buffer);
parser_command_t* cmd_buffer_get_command(command_buffer_t* buffer);
void cmd_buffer_consume_command(command_buffer_t* buffer);
const char* cmd_buffer_get_error(command_buffer_t* buffer);
uint32_t cmd_buffer_get_line_number(command_buffer_t* buffer);
float cmd_buffer_get_progress(command_buffer_t* buffer);
bool cmd_buffer_is_eof(command_buffer_t* buffer);

#endif /* COMMAND_BUFFER_H_ */