/**
 *******************************************************************************
 * @file    gcode_move_buffer.c
 * @author  Ol, naej, Fabs
 * @date    Current Date
 * @brief   G-code move buffer implementation
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "gcode_move_buffer.h"
#include "kinematics.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/* Private variables ---------------------------------------------------------*/
static gcode_move_buffer_t move_buffer = {0};
static gcode_move_buffer_state_t buffer_state = GCODE_MOVE_BUFFER_STATE_IDLE;

/* Private function prototypes -----------------------------------------------*/
static bool parse_gcode_to_move(const char *gcode_line, gcode_move_t *move,
                                position_t current_pos, float current_feedrate, bool absolute_mode);
static float parse_float_parameter(const char *line, char param);
static bool has_parameter(const char *line, char param);
static void clear_move(gcode_move_t *move);

/* Public function implementations -------------------------------------------*/

/**
 * @brief Initialize G-code move buffer
 */
bool gcode_move_buffer_init(void)
{
    printf("Initializing G-code move buffer...\n");

    // Clear buffer
    memset(&move_buffer, 0, sizeof(move_buffer));
    move_buffer.head = 0;
    move_buffer.tail = 0;
    move_buffer.count = 0;
    move_buffer.full = false;

    buffer_state = GCODE_MOVE_BUFFER_STATE_IDLE;

    printf("G-code move buffer initialized (size: %d moves)\n", GCODE_MOVE_BUFFER_SIZE);
    return true;
}

/**
 * @brief Add move to buffer from G-code line
 */
gcode_move_buffer_result_t gcode_move_buffer_add_from_gcode(
    const char *gcode_line,
    position_t current_pos,
    float current_feedrate,
    bool absolute_mode)
{
    if (!gcode_line)
    {
        return GCODE_MOVE_BUFFER_ERROR;
    }

    // Check if buffer is full
    if (move_buffer.full)
    {
        move_buffer.buffer_overruns++;
        return GCODE_MOVE_BUFFER_FULL;
    }

    gcode_move_t move;
    if (!parse_gcode_to_move(gcode_line, &move, current_pos, current_feedrate, absolute_mode))
    {
        return GCODE_MOVE_BUFFER_ERROR;
    }

    return gcode_move_buffer_add_move(&move);
}

/**
 * @brief Add pre-parsed move to buffer
 */
gcode_move_buffer_result_t gcode_move_buffer_add_move(const gcode_move_t *move)
{
    if (!move || move_buffer.full)
    {
        if (move_buffer.full)
        {
            move_buffer.buffer_overruns++;
        }
        return GCODE_MOVE_BUFFER_FULL;
    }

    // Copy move to buffer
    memcpy(&move_buffer.moves[move_buffer.head], move, sizeof(gcode_move_t));

    // Update head pointer
    move_buffer.head = (move_buffer.head + 1) % GCODE_MOVE_BUFFER_SIZE;
    move_buffer.count++;
    move_buffer.moves_added++;

    // Update max count statistic
    if (move_buffer.count > move_buffer.max_count)
    {
        move_buffer.max_count = move_buffer.count;
    }

    // Check if buffer is now full
    if (move_buffer.head == move_buffer.tail)
    {
        move_buffer.full = true;
    }

    return GCODE_MOVE_BUFFER_OK;
}

/**
 * @brief Get next move from buffer
 */
gcode_move_buffer_result_t gcode_move_buffer_get_move(gcode_move_t *move)
{
    if (!move || gcode_move_buffer_is_empty())
    {
        return GCODE_MOVE_BUFFER_EMPTY;
    }

    // Copy move from buffer
    memcpy(move, &move_buffer.moves[move_buffer.tail], sizeof(gcode_move_t));

    // Update tail pointer
    move_buffer.tail = (move_buffer.tail + 1) % GCODE_MOVE_BUFFER_SIZE;
    move_buffer.count--;
    move_buffer.moves_executed++;
    move_buffer.full = false;

    return GCODE_MOVE_BUFFER_OK;
}

/**
 * @brief Peek at next move without removing it
 */
gcode_move_buffer_result_t gcode_move_buffer_peek_move(gcode_move_t *move)
{
    if (!move || gcode_move_buffer_is_empty())
    {
        return GCODE_MOVE_BUFFER_EMPTY;
    }

    // Copy move from buffer without advancing tail
    memcpy(move, &move_buffer.moves[move_buffer.tail], sizeof(gcode_move_t));

    return GCODE_MOVE_BUFFER_OK;
}

/**
 * @brief Check if buffer is empty
 */
bool gcode_move_buffer_is_empty(void)
{
    return (!move_buffer.full && (move_buffer.head == move_buffer.tail));
}

/**
 * @brief Check if buffer is full
 */
bool gcode_move_buffer_is_full(void)
{
    return move_buffer.full;
}

/**
 * @brief Get number of moves in buffer
 */
uint32_t gcode_move_buffer_get_count(void)
{
    return move_buffer.count;
}

/**
 * @brief Get free space in buffer
 */
uint32_t gcode_move_buffer_get_free_space(void)
{
    return GCODE_MOVE_BUFFER_SIZE - move_buffer.count;
}

/**
 * @brief Clear all moves from buffer
 */
void gcode_move_buffer_clear(void)
{
    move_buffer.head = 0;
    move_buffer.tail = 0;
    move_buffer.count = 0;
    move_buffer.full = false;

    printf("G-code move buffer cleared\n");
}

/**
 * @brief Update buffer processing
 */
void gcode_move_buffer_update(void)
{
    if (buffer_state != GCODE_MOVE_BUFFER_STATE_PROCESSING)
    {
        return;
    }

    // Process moves if kinematics system is idle and we have moves
    if (kinematics_get_state() == MOVE_STATE_IDLE && !gcode_move_buffer_is_empty())
    {
        gcode_move_t move;
        if (gcode_move_buffer_get_move(&move) == GCODE_MOVE_BUFFER_OK)
        {
            // Execute the move
            switch (move.type)
            {
            case GCODE_MOVE_LINEAR:
            case GCODE_MOVE_RAPID:
            {
                float feedrate = (move.type == GCODE_MOVE_RAPID) ? 6000.0f : move.feedrate;
                if (!kinematics_move_to(&move.target, feedrate))
                {
                    printf("ERROR: Failed to execute move to X=%.2f Y=%.2f\n",
                           move.target.x, move.target.y);
                }
                break;
            }

            case GCODE_MOVE_HOME:
            {
                kinematics_home_all();
                break;
            }

            case GCODE_MOVE_DWELL:
            {
                // TODO: Implement dwell functionality
                printf("Dwell for %.3f seconds\n", move.dwell_time);
                break;
            }

            case GCODE_MOVE_SET_POSITION:
            {
                kinematics_set_position(&move.target);
                break;
            }

            default:
                printf("WARNING: Unsupported move type %d\n", move.type);
                break;
            }
        }
    }
}

/**
 * @brief Set buffer state
 */
void gcode_move_buffer_set_state(gcode_move_buffer_state_t state)
{
    buffer_state = state;
}

/**
 * @brief Get buffer state
 */
gcode_move_buffer_state_t gcode_move_buffer_get_state(void)
{
    return buffer_state;
}

/**
 * @brief Get buffer statistics
 */
const gcode_move_buffer_t *gcode_move_buffer_get_stats(void)
{
    return &move_buffer;
}

/* Private function implementations ------------------------------------------*/

/**
 * @brief Parse G-code line to move structure
 */
static bool parse_gcode_to_move(const char *gcode_line, gcode_move_t *move,
                                position_t current_pos, float current_feedrate, bool absolute_mode)
{
    if (!gcode_line || !move)
    {
        return false;
    }

    clear_move(move);

    // Skip whitespace and comments
    while (*gcode_line && isspace(*gcode_line))
        gcode_line++;
    if (*gcode_line == ';' || *gcode_line == '\0')
        return false;

    // Store original line
    strncpy(move->original_line, gcode_line, GCODE_MAX_LINE_LENGTH - 1);
    move->original_line[GCODE_MAX_LINE_LENGTH - 1] = '\0';

    // Parse command type
    if (*gcode_line == 'G' || *gcode_line == 'g')
    {
        int g_code = (int)parse_float_parameter(gcode_line, 'G');

        switch (g_code)
        {
        case 0:
            move->type = GCODE_MOVE_RAPID;
            break;
        case 1:
            move->type = GCODE_MOVE_LINEAR;
            break;
        case 4:
            move->type = GCODE_MOVE_DWELL;
            move->dwell_time = parse_float_parameter(gcode_line, 'P');
            return true; // No position parsing needed
        case 28:
            move->type = GCODE_MOVE_HOME;
            return true; // No position parsing needed
        case 92:
            move->type = GCODE_MOVE_SET_POSITION;
            break;
        default:
            return false; // Unsupported G-code
        }
    }
    else
    {
        return false; // Only G-codes supported for moves
    }

    // Parse parameters
    move->has_x = has_parameter(gcode_line, 'X');
    move->has_y = has_parameter(gcode_line, 'Y');
    move->has_f = has_parameter(gcode_line, 'F');

    // Calculate target position
    if (absolute_mode)
    {
        move->target.x = move->has_x ? parse_float_parameter(gcode_line, 'X') : current_pos.x;
        move->target.y = move->has_y ? parse_float_parameter(gcode_line, 'Y') : current_pos.y;
    }
    else
    {
        move->target.x = current_pos.x + (move->has_x ? parse_float_parameter(gcode_line, 'X') : 0.0f);
        move->target.y = current_pos.y + (move->has_y ? parse_float_parameter(gcode_line, 'Y') : 0.0f);
    }

    // Set feedrate
    move->feedrate = move->has_f ? parse_float_parameter(gcode_line, 'F') : current_feedrate;

    return true;
}

/**
 * @brief Parse float parameter from G-code line
 */
static float parse_float_parameter(const char *line, char param)
{
    const char *ptr = strchr(line, param);
    if (!ptr)
        return 0.0f;

    ptr++; // Skip parameter character
    return strtof(ptr, NULL);
}

/**
 * @brief Check if parameter exists in G-code line
 */
static bool has_parameter(const char *line, char param)
{
    return strchr(line, param) != NULL;
}

/**
 * @brief Clear move structure
 */
static void clear_move(gcode_move_t *move)
{
    memset(move, 0, sizeof(gcode_move_t));
}
