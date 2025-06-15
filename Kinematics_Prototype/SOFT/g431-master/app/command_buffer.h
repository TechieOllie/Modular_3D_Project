/**
 *******************************************************************************
 * @file    command_buffer.h
 * @author  Ol, naej, Fabs
 * @date    May 27, 2025
 * @brief   Command buffer for G-code and special commands
 *******************************************************************************
 */

#ifndef COMMAND_BUFFER_H
#define COMMAND_BUFFER_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "parser.h"

/* Exported constants --------------------------------------------------------*/
#define CMD_BUFFER_SIZE 32 // Number of commands to buffer
#define CMD_MAX_LENGTH 128 // Maximum length of a single command

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Command buffer entry
 */
typedef struct
{
    char command[CMD_MAX_LENGTH]; // Command string
    uint32_t line_number;         // Line number for tracking
    uint32_t timestamp;           // When command was added
    bool is_gcode;                // True if G-code, false if special command
    bool is_priority;             // High priority commands (emergency stop, etc.)
} command_entry_t;

/**
 * @brief Command buffer statistics
 */
typedef struct
{
    uint32_t commands_added;    // Total commands added to buffer
    uint32_t commands_executed; // Total commands executed
    uint32_t buffer_overflows;  // Number of times buffer was full
    uint32_t execution_errors;  // Number of execution errors
    uint16_t current_count;     // Current number of commands in buffer
    uint16_t max_count;         // Maximum commands that were buffered
} cmd_buffer_stats_t;

/**
 * @brief Command buffer state
 */
typedef enum
{
    CMD_BUFFER_STATE_IDLE = 0,
    CMD_BUFFER_STATE_EXECUTING,
    CMD_BUFFER_STATE_PAUSED,
    CMD_BUFFER_STATE_ERROR
} cmd_buffer_state_t;

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief Initialize command buffer
 * @return true if successful
 */
bool command_buffer_init(void);

/**
 * @brief Add command to buffer
 * @param command Command string to add
 * @param is_gcode True if G-code, false if special command
 * @param is_priority True for high priority commands
 * @return true if command was added successfully
 */
bool command_buffer_add(const char *command, bool is_gcode, bool is_priority);

/**
 * @brief Add emergency command (bypasses normal queue)
 * @param command Emergency command string
 * @return true if command was added successfully
 */
bool command_buffer_add_emergency(const char *command);

/**
 * @brief Update command buffer (call in main loop)
 * Process one command per call if available
 */
void command_buffer_update(void);

/**
 * @brief Get number of commands currently in buffer
 * @return Number of buffered commands
 */
uint16_t command_buffer_get_count(void);

/**
 * @brief Check if buffer is empty
 * @return true if buffer is empty
 */
bool command_buffer_is_empty(void);

/**
 * @brief Check if buffer is full
 * @return true if buffer is full
 */
bool command_buffer_is_full(void);

/**
 * @brief Clear all commands from buffer
 */
void command_buffer_clear(void);

/**
 * @brief Pause command execution
 */
void command_buffer_pause(void);

/**
 * @brief Resume command execution
 */
void command_buffer_resume(void);

/**
 * @brief Get current buffer state
 * @return Current state
 */
cmd_buffer_state_t command_buffer_get_state(void);

/**
 * @brief Get buffer statistics
 * @return Pointer to statistics structure
 */
cmd_buffer_stats_t *command_buffer_get_stats(void);

/**
 * @brief Wait for buffer to be empty (all commands executed)
 * @param timeout_ms Timeout in milliseconds (0 = wait forever)
 * @return true if buffer is empty, false if timeout
 */
bool command_buffer_wait_empty(uint32_t timeout_ms);

/**
 * @brief Get free space in buffer
 * @return Number of free slots in buffer
 */
uint16_t command_buffer_get_free_space(void);

#endif /* COMMAND_BUFFER_H */
