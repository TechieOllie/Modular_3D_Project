/**
 *******************************************************************************
 * @file    command_buffer.c
 * @author  Ol, naej, Fabs
 * @date    May 27, 2025
 * @brief   Command buffer for G-code and special commands
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "command_buffer.h"
#include "parser.h"
#include "kinematics.h"
#include "stepper_motor.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static command_entry_t command_buffer[CMD_BUFFER_SIZE];
static uint16_t buffer_head = 0;  // Index where next command will be added
static uint16_t buffer_tail = 0;  // Index of next command to execute
static uint16_t buffer_count = 0; // Current number of commands in buffer
static cmd_buffer_state_t current_state = CMD_BUFFER_STATE_IDLE;
static cmd_buffer_stats_t stats = {0};
static uint32_t current_line_number = 0;
static bool processing_command = false;

/* Private function prototypes -----------------------------------------------*/
static bool is_buffer_full(void);
static bool is_buffer_empty(void);
static bool execute_next_command(void);
static bool is_emergency_command(const char *command);
static void clear_command_entry(command_entry_t *entry);

/* Public function implementations -------------------------------------------*/

/**
 * @brief Initialize command buffer
 */
bool command_buffer_init(void)
{
    // Clear buffer
    memset(command_buffer, 0, sizeof(command_buffer));
    buffer_head = 0;
    buffer_tail = 0;
    buffer_count = 0;
    current_state = CMD_BUFFER_STATE_IDLE;
    current_line_number = 0;
    processing_command = false;

    // Clear statistics
    memset(&stats, 0, sizeof(stats));

    printf("Command buffer initialized (size: %d commands)\n", CMD_BUFFER_SIZE);
    return true;
}

/**
 * @brief Add command to buffer
 */
bool command_buffer_add(const char *command, bool is_gcode, bool is_priority)
{
    if (!command || strlen(command) == 0)
    {
        return false;
    }

    if (strlen(command) >= CMD_MAX_LENGTH)
    {
        printf("Command too long: %s\n", command);
        return false;
    }

    // Check for emergency commands
    if (is_emergency_command(command))
    {
        return command_buffer_add_emergency(command);
    }

    // Check if buffer is full
    if (is_buffer_full())
    {
        stats.buffer_overflows++;
        printf("Command buffer full, dropping command: %s\n", command);
        return false;
    }

    // Add command to buffer
    command_entry_t *entry = &command_buffer[buffer_head];
    strncpy(entry->command, command, CMD_MAX_LENGTH - 1);
    entry->command[CMD_MAX_LENGTH - 1] = '\0';
    entry->line_number = ++current_line_number;
    entry->timestamp = HAL_GetTick();
    entry->is_gcode = is_gcode;
    entry->is_priority = is_priority;

    buffer_head = (buffer_head + 1) % CMD_BUFFER_SIZE;
    buffer_count++;

    // Update statistics
    stats.commands_added++;
    if (buffer_count > stats.max_count)
    {
        stats.max_count = buffer_count;
    }

    printf("Added command to buffer [%d/%d]: %s\n",
           buffer_count, CMD_BUFFER_SIZE, command);

    return true;
}

/**
 * @brief Add emergency command (bypasses normal queue)
 */
bool command_buffer_add_emergency(const char *command)
{
    if (!command || strlen(command) == 0)
    {
        return false;
    }

    printf("EMERGENCY COMMAND: %s\n", command);

    // For emergency commands, we execute immediately and clear the buffer
    command_buffer_clear();

    // Stop any current movement
    stepper_motor_emergency_stop_all();
    kinematics_stop();

    // Process the emergency command immediately
    if (strncmp(command, "M112", 4) == 0 || strncmp(command, "stop", 4) == 0)
    {
        current_state = CMD_BUFFER_STATE_PAUSED;
        printf("Emergency stop activated - buffer paused\n");
        return true;
    }

    // For other emergency commands, try to parse them
    bool result = parser_process_line(command);
    stats.commands_executed++;

    if (!result)
    {
        stats.execution_errors++;
    }

    return result;
}

/**
 * @brief Update command buffer
 */
void command_buffer_update(void)
{
    // Don't process if paused or in error state
    if (current_state == CMD_BUFFER_STATE_PAUSED ||
        current_state == CMD_BUFFER_STATE_ERROR)
    {
        return;
    }

    // Don't start new command if already processing one
    if (processing_command)
    {
        // Check if current movement is complete
        if (kinematics_get_state() == MOVE_STATE_IDLE)
        {
            processing_command = false;
            current_state = CMD_BUFFER_STATE_IDLE;
        }
        return;
    }

    // Execute next command if available
    if (!is_buffer_empty())
    {
        execute_next_command();
    }
    else
    {
        current_state = CMD_BUFFER_STATE_IDLE;
    }
}

/**
 * @brief Get number of commands currently in buffer
 */
uint16_t command_buffer_get_count(void)
{
    return buffer_count;
}

/**
 * @brief Check if buffer is empty
 */
bool command_buffer_is_empty(void)
{
    return buffer_count == 0;
}

/**
 * @brief Check if buffer is full
 */
bool command_buffer_is_full(void)
{
    return buffer_count >= CMD_BUFFER_SIZE;
}

/**
 * @brief Clear all commands from buffer
 */
void command_buffer_clear(void)
{
    buffer_head = 0;
    buffer_tail = 0;
    buffer_count = 0;
    processing_command = false;
    current_state = CMD_BUFFER_STATE_IDLE;

    // Clear all entries
    for (int i = 0; i < CMD_BUFFER_SIZE; i++)
    {
        clear_command_entry(&command_buffer[i]);
    }

    printf("Command buffer cleared\n");
}

/**
 * @brief Pause command execution
 */
void command_buffer_pause(void)
{
    current_state = CMD_BUFFER_STATE_PAUSED;
    kinematics_stop();
    printf("Command buffer paused\n");
}

/**
 * @brief Resume command execution
 */
void command_buffer_resume(void)
{
    if (current_state == CMD_BUFFER_STATE_PAUSED)
    {
        current_state = CMD_BUFFER_STATE_IDLE;
        processing_command = false;
        printf("Command buffer resumed\n");
    }
}

/**
 * @brief Get current buffer state
 */
cmd_buffer_state_t command_buffer_get_state(void)
{
    return current_state;
}

/**
 * @brief Get buffer statistics
 */
cmd_buffer_stats_t *command_buffer_get_stats(void)
{
    stats.current_count = buffer_count;
    return &stats;
}

/**
 * @brief Wait for buffer to be empty
 */
bool command_buffer_wait_empty(uint32_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();

    while (!command_buffer_is_empty())
    {
        command_buffer_update();

        if (timeout_ms > 0 && (HAL_GetTick() - start_time) > timeout_ms)
        {
            return false;
        }

        HAL_Delay(10);
    }

    // Also wait for any current movement to complete
    while (kinematics_get_state() != MOVE_STATE_IDLE)
    {
        kinematics_update();
        stepper_motor_update();

        if (timeout_ms > 0 && (HAL_GetTick() - start_time) > timeout_ms)
        {
            return false;
        }

        HAL_Delay(10);
    }

    return true;
}

/**
 * @brief Get free space in buffer
 */
uint16_t command_buffer_get_free_space(void)
{
    return CMD_BUFFER_SIZE - buffer_count;
}

/* Private function implementations ------------------------------------------*/

/**
 * @brief Check if buffer is full (internal)
 */
static bool is_buffer_full(void)
{
    return buffer_count >= CMD_BUFFER_SIZE;
}

/**
 * @brief Check if buffer is empty (internal)
 */
static bool is_buffer_empty(void)
{
    return buffer_count == 0;
}

/**
 * @brief Execute the next command in the buffer
 */
static bool execute_next_command(void)
{
    if (is_buffer_empty())
    {
        return false;
    }

    command_entry_t *entry = &command_buffer[buffer_tail];

    printf("Executing command [%lu]: %s\n", entry->line_number, entry->command);

    current_state = CMD_BUFFER_STATE_EXECUTING;
    processing_command = true;

    bool result = false;

    // Execute the command based on type
    if (entry->is_gcode)
    {
        // For G-code commands, use the parser directly
        result = parser_process_line(entry->command);
    }
    else
    {
        // For special commands, use the parser as well
        result = parser_process_line(entry->command);
    }

    // Update statistics
    stats.commands_executed++;
    if (!result)
    {
        stats.execution_errors++;
        current_state = CMD_BUFFER_STATE_ERROR;
        printf("Command execution failed: %s\n", entry->command);
    }

    // Remove command from buffer
    clear_command_entry(entry);
    buffer_tail = (buffer_tail + 1) % CMD_BUFFER_SIZE;
    buffer_count--;

    // For non-movement commands, we're done immediately
    if (result && kinematics_get_state() == MOVE_STATE_IDLE)
    {
        processing_command = false;
        current_state = CMD_BUFFER_STATE_IDLE;
    }

    return result;
}

/**
 * @brief Check if command is an emergency command
 */
static bool is_emergency_command(const char *command)
{
    return (strncmp(command, "M112", 4) == 0 ||
            strncmp(command, "stop", 4) == 0 ||
            strncmp(command, "STOP", 4) == 0);
}

/**
 * @brief Clear a command entry
 */
static void clear_command_entry(command_entry_t *entry)
{
    memset(entry, 0, sizeof(command_entry_t));
}
