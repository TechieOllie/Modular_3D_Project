/**
 *******************************************************************************
 * @file    limit_switches.c
 * @author  Ol, naej, Fabs
 * @date    May 27, 2025
 * @brief   Limit switches management for 3D printer axes
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "limit_switches.h"
#include "stepper_motor.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static limit_switch_config_t limit_switches[LIMIT_SWITCH_MAX_AXES][2]; // [axis][position]
static limit_switch_callback_t user_callback = NULL;
static bool system_initialized = false;

/* Private function declarations ---------------------------------------------*/
static void limit_switch_interrupt_handler(uint8_t pin_number);
static uint8_t gpio_pin_to_pin_number(uint16_t gpio_pin);
static void find_switch_by_pin(uint8_t pin_number, axis_t *axis, limit_position_t *position);

/* Public functions implementations ------------------------------------------*/

/**
 * @brief Initialize limit switches system
 */
bool limit_switches_init(void)
{
    printf("Initializing limit switches system...\n");

    // Clear all configurations
    memset(limit_switches, 0, sizeof(limit_switches));

    // Initialize EXTI system if not already done
#if USE_BSP_EXTIT
    // EXTI system should be initialized by the BSP
    printf("EXTI system enabled\n");
#else
    printf("Warning: EXTI system not enabled in config.h\n");
    // Continue anyway for basic functionality without interrupts
#endif

    system_initialized = true;
    printf("Limit switches system initialized\n");

    return true;
}

/**
 * @brief Configure a limit switch
 */
bool limit_switch_configure(axis_t axis, limit_position_t position,
                            GPIO_TypeDef *gpio_port, uint16_t gpio_pin,
                            bool active_low)
{
    if (!system_initialized)
    {
        printf("Error: Limit switches system not initialized\n");
        return false;
    }

    if (axis >= LIMIT_SWITCH_MAX_AXES)
    {
        printf("Error: Invalid axis %d\n", axis);
        return false;
    }

    limit_switch_config_t *config = &limit_switches[axis][position];

    // Configure GPIO
    uint32_t gpio_mode = GPIO_MODE_INPUT;
    uint32_t gpio_pull = active_low ? GPIO_PULLUP : GPIO_PULLDOWN;

    BSP_GPIO_pin_config(gpio_port, gpio_pin, gpio_mode, gpio_pull,
                        GPIO_SPEED_FREQ_LOW, GPIO_NO_AF);

    // Store configuration
    config->gpio_port = gpio_port;
    config->gpio_pin = gpio_pin;
    config->pin_number = gpio_pin_to_pin_number(gpio_pin);
    config->active_low = active_low;
    config->initialized = true;
    config->enabled = false; // Start disabled
    config->last_trigger_time = 0;

    printf("Configured limit switch: %s_%s on %s Pin_%d (active_%s)\n",
           limit_switch_axis_to_string(axis),
           limit_switch_position_to_string(position),
           (gpio_port == GPIOA) ? "GPIOA" : (gpio_port == GPIOB) ? "GPIOB"
                                                                 : "GPIOC",
           config->pin_number,
           active_low ? "low" : "high");

    return true;
}

/**
 * @brief Enable/disable a specific limit switch
 */
bool limit_switch_enable(axis_t axis, limit_position_t position, bool enable)
{
    if (!system_initialized || axis >= LIMIT_SWITCH_MAX_AXES)
    {
        return false;
    }

    limit_switch_config_t *config = &limit_switches[axis][position];

    if (!config->initialized)
    {
        printf("Error: Limit switch %s_%s not configured\n",
               limit_switch_axis_to_string(axis),
               limit_switch_position_to_string(position));
        return false;
    }

    if (enable && !config->enabled)
    {
#if USE_BSP_EXTIT
        // Enable interrupts for this pin
        BSP_EXTIT_set_callback(limit_switch_interrupt_handler, config->pin_number, true);
        BSP_EXTIT_enable(config->pin_number);
#endif
        config->enabled = true;

        printf("Enabled limit switch: %s_%s\n",
               limit_switch_axis_to_string(axis),
               limit_switch_position_to_string(position));
    }
    else if (!enable && config->enabled)
    {
#if USE_BSP_EXTIT
        // Disable interrupts for this pin
        BSP_EXTIT_disable(config->pin_number);
        BSP_EXTIT_set_callback(NULL, config->pin_number, false);
#endif
        config->enabled = false;

        printf("Disabled limit switch: %s_%s\n",
               limit_switch_axis_to_string(axis),
               limit_switch_position_to_string(position));
    }

    return true;
}

/**
 * @brief Read current state of a limit switch
 */
limit_switch_state_t limit_switch_read(axis_t axis, limit_position_t position)
{
    if (!system_initialized || axis >= LIMIT_SWITCH_MAX_AXES)
    {
        return LIMIT_NOT_TRIGGERED;
    }

    limit_switch_config_t *config = &limit_switches[axis][position];

    if (!config->initialized)
    {
        return LIMIT_NOT_TRIGGERED;
    }

    // Read GPIO state
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(config->gpio_port, config->gpio_pin);

    // Determine if triggered based on active level
    bool triggered;
    if (config->active_low)
    {
        triggered = (pin_state == GPIO_PIN_RESET);
    }
    else
    {
        triggered = (pin_state == GPIO_PIN_SET);
    }

    return triggered ? LIMIT_TRIGGERED : LIMIT_NOT_TRIGGERED;
}

/**
 * @brief Check if any limit switch is triggered
 */
bool limit_switches_any_triggered(void)
{
    for (uint8_t axis = 0; axis < LIMIT_SWITCH_MAX_AXES; axis++)
    {
        for (uint8_t pos = 0; pos < 2; pos++)
        {
            if (limit_switches[axis][pos].initialized &&
                limit_switch_read((axis_t)axis, (limit_position_t)pos) == LIMIT_TRIGGERED)
            {
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief Check if specific axis has any limit triggered
 */
bool limit_switch_axis_triggered(axis_t axis)
{
    if (axis >= LIMIT_SWITCH_MAX_AXES)
    {
        return false;
    }

    for (uint8_t pos = 0; pos < 2; pos++)
    {
        if (limit_switches[axis][pos].initialized &&
            limit_switch_read(axis, (limit_position_t)pos) == LIMIT_TRIGGERED)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Set callback function for limit switch events
 */
void limit_switches_set_callback(limit_switch_callback_t callback)
{
    user_callback = callback;
    printf("Limit switch callback %s\n", callback ? "set" : "cleared");
}

/**
 * @brief Emergency stop - called when any limit switch is triggered during movement
 */
void limit_switches_emergency_stop(void)
{
    printf("EMERGENCY STOP: Limit switch triggered!\n");

    // Stop all stepper motors immediately
    for (uint8_t motor_id = 0; motor_id < STEPPER_MOTOR_MAX_COUNT; motor_id++)
    {
        if (STEPPER_MOTOR_get_state(motor_id) == MOTOR_STATE_ERROR)
        {
            STEPPER_MOTOR_stop(motor_id);
            printf("Stopped motor %d\n", motor_id);
        }
    }
}

/**
 * @brief Update limit switches state (call periodically for debouncing)
 */
void limit_switches_update(void)
{
    // This function can be used for additional debouncing logic if needed
    // The main debouncing is handled in the interrupt handler
}

/**
 * @brief Get string representation of axis
 */
const char *limit_switch_axis_to_string(axis_t axis)
{
    switch (axis)
    {
    case AXIS_X:
        return "X";
    case AXIS_Y:
        return "Y";
    case AXIS_Z:
        return "Z";
    default:
        return "UNKNOWN";
    }
}

/**
 * @brief Get string representation of position
 */
const char *limit_switch_position_to_string(limit_position_t position)
{
    switch (position)
    {
    case LIMIT_MIN:
        return "MIN";
    case LIMIT_MAX:
        return "MAX";
    default:
        return "UNKNOWN";
    }
}

/* Private functions implementations ------------------------------------------*/

/**
 * @brief Convert GPIO pin to pin number for EXTI
 */
static uint8_t gpio_pin_to_pin_number(uint16_t gpio_pin)
{
    // Use built-in function to count trailing zeros (which gives us the pin number)
    return __builtin_ctz(gpio_pin);
}

/**
 * @brief Find which switch corresponds to a pin number
 */
static void find_switch_by_pin(uint8_t pin_number, axis_t *axis, limit_position_t *position)
{
    for (uint8_t a = 0; a < LIMIT_SWITCH_MAX_AXES; a++)
    {
        for (uint8_t p = 0; p < 2; p++)
        {
            if (limit_switches[a][p].initialized &&
                limit_switches[a][p].pin_number == pin_number)
            {
                *axis = (axis_t)a;
                *position = (limit_position_t)p;
                return;
            }
        }
    }
    *axis = AXIS_X; // Default values
    *position = LIMIT_MIN;
}

/**
 * @brief Interrupt handler for limit switches
 */
static void limit_switch_interrupt_handler(uint8_t pin_number)
{
    uint32_t current_time = HAL_GetTick();
    axis_t axis;
    limit_position_t position;

    // Find which switch triggered
    find_switch_by_pin(pin_number, &axis, &position);

    limit_switch_config_t *config = &limit_switches[axis][position];

    if (!config->initialized || !config->enabled)
    {
        return;
    }

    // Debouncing check
    if ((current_time - config->last_trigger_time) < LIMIT_SWITCH_DEBOUNCE_MS)
    {
        return; // Too soon, ignore
    }

    config->last_trigger_time = current_time;

    // Read current state
    limit_switch_state_t current_state = limit_switch_read(axis, position);

    printf("Limit switch triggered: %s_%s = %s\n",
           limit_switch_axis_to_string(axis),
           limit_switch_position_to_string(position),
           current_state == LIMIT_TRIGGERED ? "TRIGGERED" : "RELEASED");

    // If switch is triggered, perform emergency stop
    if (current_state == LIMIT_TRIGGERED)
    {
        limit_switches_emergency_stop();
    }

    // Call user callback if set
    if (user_callback)
    {
        user_callback(axis, position, current_state);
    }

#if USE_BSP_EXTIT
    // Acknowledge the interrupt
    BSP_EXTIT_ack_it(pin_number);
#endif
}
