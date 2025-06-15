/**
 *******************************************************************************
 * @file    gcode_move_buffer.h
 * @author  Ol, naej, Fabs
 * @date    Current Date
 * @brief   G-code move buffer for storing parsed movements
 *******************************************************************************
 */

#ifndef GCODE_MOVE_BUFFER_H
#define GCODE_MOVE_BUFFER_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "kinematics.h"

/* Exported constants --------------------------------------------------------*/
#define GCODE_MOVE_BUFFER_SIZE 64
#define GCODE_MAX_LINE_LENGTH 128

/* Exported types ------------------------------------------------------------*/

/**
 * @brief G-code move types
 */
typedef enum
{
    GCODE_MOVE_LINEAR = 0,
    GCODE_MOVE_RAPID,
    GCODE_MOVE_ARC_CW,
    GCODE_MOVE_ARC_CCW,
    GCODE_MOVE_DWELL,
    GCODE_MOVE_HOME,
    GCODE_MOVE_SET_POSITION
} gcode_move_type_t;

/**
 * @brief G-code move structure
 */
typedef struct
{
    gcode_move_type_t type;
    position_t target; // Target position
    float feedrate;    // Movement speed (mm/min)
    float dwell_time;  // For dwell commands (seconds)

    // Arc parameters (for future use)
    position_t center; // Arc center offset
    float radius;      // Arc radius

    // Flags
    bool has_x;
    bool has_y;
    bool has_z;
    bool has_f;

    // Source information
    uint32_t line_number;
    char original_line[GCODE_MAX_LINE_LENGTH];
} gcode_move_t;

/**
 * @brief G-code move buffer structure
 */
typedef struct
{
    gcode_move_t moves[GCODE_MOVE_BUFFER_SIZE];
    uint32_t head;
    uint32_t tail;
    uint32_t count;
    bool full;

    // Statistics
    uint32_t moves_added;
    uint32_t moves_executed;
    uint32_t buffer_overruns;
    uint32_t max_count;
} gcode_move_buffer_t;

/**
 * @brief Move buffer result codes
 */
typedef enum
{
    GCODE_MOVE_BUFFER_OK = 0,
    GCODE_MOVE_BUFFER_FULL,
    GCODE_MOVE_BUFFER_EMPTY,
    GCODE_MOVE_BUFFER_ERROR
} gcode_move_buffer_result_t;

/**
 * @brief Move buffer state
 */
typedef enum
{
    GCODE_MOVE_BUFFER_STATE_IDLE = 0,
    GCODE_MOVE_BUFFER_STATE_PROCESSING,
    GCODE_MOVE_BUFFER_STATE_PAUSED,
    GCODE_MOVE_BUFFER_STATE_ERROR
} gcode_move_buffer_state_t;

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief Initialize G-code move buffer
 * @return true if successful
 */
bool gcode_move_buffer_init(void);

/**
 * @brief Add move to buffer from G-code line
 * @param gcode_line G-code line to parse and add
 * @param current_pos Current machine position
 * @param current_feedrate Current feedrate
 * @param absolute_mode True for absolute positioning
 * @return GCODE_MOVE_BUFFER_OK if successful
 */
gcode_move_buffer_result_t gcode_move_buffer_add_from_gcode(
    const char *gcode_line,
    position_t current_pos,
    float current_feedrate,
    bool absolute_mode);

/**
 * @brief Add pre-parsed move to buffer
 * @param move Move structure to add
 * @return GCODE_MOVE_BUFFER_OK if successful
 */
gcode_move_buffer_result_t gcode_move_buffer_add_move(const gcode_move_t *move);

/**
 * @brief Get next move from buffer
 * @param move Pointer to store the move
 * @return GCODE_MOVE_BUFFER_OK if move retrieved
 */
gcode_move_buffer_result_t gcode_move_buffer_get_move(gcode_move_t *move);

/**
 * @brief Peek at next move without removing it
 * @param move Pointer to store the move
 * @return GCODE_MOVE_BUFFER_OK if move available
 */
gcode_move_buffer_result_t gcode_move_buffer_peek_move(gcode_move_t *move);

/**
 * @brief Check if buffer is empty
 * @return true if empty
 */
bool gcode_move_buffer_is_empty(void);

/**
 * @brief Check if buffer is full
 * @return true if full
 */
bool gcode_move_buffer_is_full(void);

/**
 * @brief Get number of moves in buffer
 * @return Number of moves
 */
uint32_t gcode_move_buffer_get_count(void);

/**
 * @brief Get free space in buffer
 * @return Number of free slots
 */
uint32_t gcode_move_buffer_get_free_space(void);

/**
 * @brief Clear all moves from buffer
 */
void gcode_move_buffer_clear(void);

/**
 * @brief Update buffer processing
 */
void gcode_move_buffer_update(void);

/**
 * @brief Set buffer state
 * @param state New state
 */
void gcode_move_buffer_set_state(gcode_move_buffer_state_t state);

/**
 * @brief Get buffer state
 * @return Current state
 */
gcode_move_buffer_state_t gcode_move_buffer_get_state(void);

/**
 * @brief Get buffer statistics
 * @return Pointer to buffer structure (read-only)
 */
const gcode_move_buffer_t *gcode_move_buffer_get_stats(void);

#endif /* GCODE_MOVE_BUFFER_H */
