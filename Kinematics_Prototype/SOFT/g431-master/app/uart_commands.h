/**
 *******************************************************************************
 * @file    uart_commands.h
 * @author  Ol, naej, Fabs
 * @date    May 27, 2025
 * @brief   UART command handling for G-code and special commands
 *******************************************************************************
 */

#ifndef UART_COMMANDS_H
#define UART_COMMANDS_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "stm32g4_uart.h"

/* Exported constants --------------------------------------------------------*/
#define UART_CMD_BUFFER_SIZE 256
#define UART_CMD_MAX_LINE_LENGTH 128

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART command processing state
 */
typedef enum
{
    UART_CMD_STATE_IDLE = 0,
    UART_CMD_STATE_RECEIVING,
    UART_CMD_STATE_PROCESSING,
    UART_CMD_STATE_ERROR
} uart_cmd_state_t;

/**
 * @brief UART command statistics
 */
typedef struct
{
    uint32_t lines_processed;
    uint32_t commands_executed;
    uint32_t errors_count;
    uint32_t buffer_overflows;
} uart_cmd_stats_t;

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief Initialize UART command handling
 * @param uart_id UART ID to use for communication
 * @return true if successful
 */
bool uart_commands_init(uart_id_t uart_id);

/**
 * @brief Process incoming character from UART
 * @param c Character received
 */
void uart_commands_process_char(char c);

/**
 * @brief Update UART command processing (call in main loop)
 */
void uart_commands_update(void);

/**
 * @brief Get current state
 * @return Current processing state
 */
uart_cmd_state_t uart_commands_get_state(void);

/**
 * @brief Get statistics
 * @return Pointer to statistics structure
 */
uart_cmd_stats_t *uart_commands_get_stats(void);

/**
 * @brief Send response message
 * @param message Message to send
 */
void uart_commands_send_response(const char *message);

/**
 * @brief Send formatted response
 * @param format Printf-style format string
 * @param ... Arguments for format string
 */
void uart_commands_send_response_printf(const char *format, ...);

/**
 * @brief Process emergency stop command
 */
void uart_commands_emergency_stop(void);

/**
 * @brief Clear error state
 */
void uart_commands_clear_error(void);

/**
 * @brief Check if SD card printing is active
 * @return true if currently printing from SD card
 */
bool uart_commands_is_sd_printing(void);

#endif /* UART_COMMANDS_H */
