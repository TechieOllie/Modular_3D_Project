#ifndef COMMAND_EXECUTOR_H_
#define COMMAND_EXECUTOR_H_

#include "command_buffer.h"
#include <stdbool.h>

// Initialize the command executor
void cmd_executor_init(void);

// Start executing commands from the buffer
void cmd_executor_start(command_buffer_t *buffer);

// Pause command execution
void cmd_executor_pause(void);

// Resume command execution
void cmd_executor_resume(void);

// Stop command execution
void cmd_executor_stop(void);

// Process next command (called in the main loop)
void cmd_executor_process(command_buffer_t *buffer);

// Check if execution is active
bool cmd_executor_is_active(void);

// Check if execution is paused
bool cmd_executor_is_paused(void);

// Get current status message
const char *cmd_executor_get_status(void);

// Check if a move is in progress
bool cmd_executor_is_moving(void);

#endif /* COMMAND_EXECUTOR_H_ */