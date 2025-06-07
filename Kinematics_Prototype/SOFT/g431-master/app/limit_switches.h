/**
 *******************************************************************************
 * @file    limit_switches.h
 * @author  GitHub Copilot
 * @date    May 27, 2025
 * @brief   Limit switches management for 3D printer axes
 *******************************************************************************
 */

#ifndef LIMIT_SWITCHES_H
#define LIMIT_SWITCHES_H

/* Includes ------------------------------------------------------------------*/
#include "stm32g4_utils.h"
#include "stm32g4_gpio.h"
#include "stm32g4_extit.h"

/* Defines -------------------------------------------------------------------*/
#define LIMIT_SWITCH_MAX_AXES       3   // X, Y, Z axes
#define LIMIT_SWITCH_DEBOUNCE_MS    50  // Debounce time in milliseconds

/* Public types --------------------------------------------------------------*/

/**
 * @brief Axis enumeration
 */
typedef enum {
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2
} axis_t;

/**
 * @brief Limit switch position
 */
typedef enum {
    LIMIT_MIN = 0,  // Home/minimum position limit switch
    LIMIT_MAX = 1   // Maximum position limit switch
} limit_position_t;

/**
 * @brief Limit switch state
 */
typedef enum {
    LIMIT_NOT_TRIGGERED = 0,
    LIMIT_TRIGGERED = 1
} limit_switch_state_t;

/**
 * @brief Limit switch configuration structure
 */
typedef struct {
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;
    uint8_t pin_number;         // For EXTI configuration
    bool active_low;            // true if switch is active low
    bool initialized;
    bool enabled;
    uint32_t last_trigger_time; // For debouncing
} limit_switch_config_t;

/**
 * @brief Callback function type for limit switch events
 * @param axis The axis that triggered
 * @param position MIN or MAX limit
 * @param state Current state of the switch
 */
typedef void(*limit_switch_callback_t)(axis_t axis, limit_position_t position, limit_switch_state_t state);

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief Initialize limit switches system
 */
bool limit_switches_init(void);

/**
 * @brief Configure a limit switch
 * @param axis Axis (X, Y, Z)
 * @param position MIN or MAX limit
 * @param gpio_port GPIO port
 * @param gpio_pin GPIO pin
 * @param active_low true if switch is active low (pullup), false if active high
 * @return true if successful
 */
bool limit_switch_configure(axis_t axis, limit_position_t position, 
                           GPIO_TypeDef *gpio_port, uint16_t gpio_pin, 
                           bool active_low);

/**
 * @brief Enable/disable a specific limit switch
 * @param axis Axis (X, Y, Z)
 * @param position MIN or MAX limit
 * @param enable true to enable, false to disable
 * @return true if successful
 */
bool limit_switch_enable(axis_t axis, limit_position_t position, bool enable);

/**
 * @brief Read current state of a limit switch
 * @param axis Axis (X, Y, Z)
 * @param position MIN or MAX limit
 * @return Current switch state
 */
limit_switch_state_t limit_switch_read(axis_t axis, limit_position_t position);

/**
 * @brief Check if any limit switch is triggered
 * @return true if any switch is triggered
 */
bool limit_switches_any_triggered(void);

/**
 * @brief Check if specific axis has any limit triggered
 * @param axis Axis to check
 * @return true if any limit on this axis is triggered
 */
bool limit_switch_axis_triggered(axis_t axis);

/**
 * @brief Set callback function for limit switch events
 * @param callback Function to call when limit switch state changes
 */
void limit_switches_set_callback(limit_switch_callback_t callback);

/**
 * @brief Emergency stop - called when any limit switch is triggered during movement
 */
void limit_switches_emergency_stop(void);

/**
 * @brief Update limit switches state (call periodically for debouncing)
 */
void limit_switches_update(void);

/**
 * @brief Get string representation of axis
 * @param axis Axis enum
 * @return String representation
 */
const char* limit_switch_axis_to_string(axis_t axis);

/**
 * @brief Get string representation of position
 * @param position Position enum
 * @return String representation
 */
const char* limit_switch_position_to_string(limit_position_t position);

#endif /* LIMIT_SWITCHES_H */
