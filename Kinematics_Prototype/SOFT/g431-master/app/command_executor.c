#include "command_executor.h"
#include "corexy.h"
#include "parser.h"
#include <stdio.h>
#include <string.h>

// Status variables
static bool is_executing = false;
static bool is_paused = false;
static uint32_t last_executed_line = 0;
static char status_message[64] = {0};

// Initialize the command executor
void cmd_executor_init(void)
{
    is_executing = false;
    is_paused = false;
    last_executed_line = 0;
    sprintf(status_message, "Ready");

    printf("Command executor initialized\n");
}

// G-code execution callback
static void cmd_execution_callback(bool success)
{
    if (success)
    {
        // Command executed successfully, nothing to do here
    }
    else
    {
        // Command execution failed
        sprintf(status_message, "Command execution failed");
        is_executing = false;
    }
}

// Start executing commands from the buffer
void cmd_executor_start(command_buffer_t *buffer)
{
    if (buffer == NULL)
    {
        sprintf(status_message, "Invalid buffer");
        return;
    }

    is_executing = true;
    is_paused = false;
    sprintf(status_message, "Executing commands...");

    printf("Starting command execution from buffer\n");
}

// Pause command execution
void cmd_executor_pause(void)
{
    if (is_executing)
    {
        is_paused = true;
        sprintf(status_message, "Paused");
        printf("Command execution paused\n");
    }
}

// Resume command execution
void cmd_executor_resume(void)
{
    if (is_executing && is_paused)
    {
        is_paused = false;
        sprintf(status_message, "Executing commands...");
        printf("Command execution resumed\n");
    }
}

// Stop command execution
void cmd_executor_stop(void)
{
    is_executing = false;
    is_paused = false;
    sprintf(status_message, "Stopped");

    printf("Command execution stopped\n");
}

// Process next command
void cmd_executor_process(command_buffer_t *buffer)
{
    // Check if we should be executing
    if (!is_executing || is_paused || buffer == NULL)
    {
        return;
    }

    // Check if CoreXY is busy
    if (CoreXY_GetState() != COREXY_STATE_IDLE)
    {
        return; // Wait for current movement to complete
    }

    // Get the next command if available
    if (cmd_buffer_has_command(buffer))
    {
        parser_command_t *cmd = cmd_buffer_get_command(buffer);

        if (cmd->valid)
        {
            last_executed_line = cmd_buffer_get_line_number(buffer);

            // Execute the command based on its type
            if (cmd->type == COMMAND_TYPE_G)
            {
                // Use our CoreXY system to process G-codes
                CoreXY_ProcessGCodeCommand(cmd);
            }
            else if (cmd->type == COMMAND_TYPE_M)
            {
                // Process M-codes
                switch (cmd->code)
                {
                case 0: // M0: Unconditional stop
                case 1: // M1: Optional stop
                    cmd_executor_pause();
                    break;

                case 112: // M112: Emergency stop
                    CoreXY_EmergencyStop();
                    cmd_executor_stop();
                    break;

                default:
                    printf("Unsupported M-code: M%d\n", cmd->code);
                    break;
                }
            }
        }

        // Mark command as processed
        cmd_buffer_consume_command(buffer);

        // Update status with progress
        sprintf(status_message, "Line %lu (%.1f%%)",
                (unsigned long)last_executed_line,
                cmd_buffer_get_progress(buffer));
    }
    else if (cmd_buffer_is_eof(buffer))
    {
        // End of file reached
        sprintf(status_message, "Complete");
        is_executing = false;
        printf("G-code execution completed\n");
    }
}

// Check if execution is active
bool cmd_executor_is_active(void)
{
    return is_executing;
}

// Check if execution is paused
bool cmd_executor_is_paused(void)
{
    return is_paused;
}

// Get current status message
const char *cmd_executor_get_status(void)
{
    return status_message;
}

// Check if a move is in progress
bool cmd_executor_is_moving(void)
{
    return (CoreXY_GetState() == COREXY_STATE_MOVING);
}