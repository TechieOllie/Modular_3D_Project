/**
 *******************************************************************************
 * @file    stepper_motor.c
 * @author  Your Name
 * @date    Current Date
 * @brief   Stepper motor control implementation using timers for step pulsing
 *******************************************************************************
 */

#include "stepper_motor.h"
#include "stm32g4_systick.h"
#include "stm32g4_timer.h"
#include "stm32g4_gpio.h"
#include <stdio.h>

// Define the stepper motor array
static stepper_motor_t stepper_motors[STEPPER_COUNT];
static uint8_t stepper_count = 0;

/**
 * Initialize the stepper motor module
 */
void STEPPER_init(void)
{
    // Reset the stepper count
    stepper_count = 0;

    // Initialize the stepper motor array
    for (uint8_t i = 0; i < STEPPER_COUNT; i++)
    {
        stepper_motors[i].current_position = 0;
        stepper_motors[i].enabled = false;

#if ENDSTOP_ENABLED
        stepper_motors[i].min_endstop_enabled = false;
        stepper_motors[i].max_endstop_enabled = false;
#endif
    }

    printf("Stepper motor module initialized\n");
}

/**
 * Add a stepper motor to the system
 */
stepper_id_t STEPPER_add(GPIO_TypeDef *step_port, uint16_t step_pin,
                         GPIO_TypeDef *dir_port, uint16_t dir_pin,
                         timer_id_t timer_id, uint32_t timer_channel,
                         float steps_per_mm, uint8_t microstepping, float current_limit_a)
{
    if (stepper_count >= STEPPER_COUNT)
    {
        printf("Error: Maximum number of stepper motors reached\n");
        return -1;
    }

    stepper_id_t id = stepper_count++;

    // Store the motor configuration
    stepper_motors[id].step_port = step_port;
    stepper_motors[id].step_pin = step_pin;
    stepper_motors[id].dir_port = dir_port;
    stepper_motors[id].dir_pin = dir_pin;
    stepper_motors[id].timer_id = timer_id;
    stepper_motors[id].timer_channel = timer_channel;
    stepper_motors[id].steps_per_mm = steps_per_mm;
    stepper_motors[id].current_position = 0;
    stepper_motors[id].enabled = true;
    stepper_motors[id].microstepping = microstepping;
    stepper_motors[id].current_limit_a = current_limit_a;

#if ENDSTOP_ENABLED
    stepper_motors[id].min_endstop_enabled = false;
    stepper_motors[id].max_endstop_enabled = false;
#endif

    // Configure GPIO for step and direction pins
    BSP_GPIO_pin_config(step_port, step_pin, GPIO_MODE_OUTPUT_PP,
                        GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);

    BSP_GPIO_pin_config(dir_port, dir_pin, GPIO_MODE_OUTPUT_PP,
                        GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);

    // Initialize pin states
    HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);

    printf("Stepper motor %d added with %.1f steps/mm (1/%d microstepping)\n",
           id, steps_per_mm, microstepping);

    return id;
}

/**
 * Move a stepper motor a given number of steps
 */
void STEPPER_move_steps(stepper_id_t id, int32_t steps, float speed)
{
    if (id < 0 || id >= stepper_count || !stepper_motors[id].enabled)
    {
        printf("Error: Invalid stepper motor ID\n");
        return;
    }

    // Set direction based on step sign
    bool direction = (steps >= 0);
    steps = (steps >= 0) ? steps : -steps; // Absolute value

    // Set direction pin
    HAL_GPIO_WritePin(stepper_motors[id].dir_port, stepper_motors[id].dir_pin,
                      direction ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Calculate the pulse frequency based on speed (steps per second)
    // Use a more reasonable minimum frequency (5kHz instead of 20kHz)
    uint32_t pulse_freq_hz = (uint32_t)(speed < 5000.0f ? 5000.0f : speed);

    // Calculate delay between steps in microseconds
    uint32_t step_delay_us = 1000000 / pulse_freq_hz;

    // Step pulse width in microseconds (typically 1-5μs is enough for most drivers)
    uint32_t pulse_width_us = 5;

    // Make sure pulse width is reasonable compared to step delay
    if (pulse_width_us > (step_delay_us / 4))
        pulse_width_us = step_delay_us / 4;

    // Generate individual step pulses manually for precise control
    for (int32_t i = 0; i < steps; i++)
    {
        // Generate STEP pulse - rising edge
        HAL_GPIO_WritePin(stepper_motors[id].step_port, stepper_motors[id].step_pin, GPIO_PIN_SET);

        // Hold pulse high for pulse width duration
        Delay_us(pulse_width_us);

        // End of pulse - falling edge
        HAL_GPIO_WritePin(stepper_motors[id].step_port, stepper_motors[id].step_pin, GPIO_PIN_RESET);

        // Delay until next step
        if (i < steps - 1)
        { // No need to delay after the last step
            Delay_us(step_delay_us - pulse_width_us);
        }
    }

    // Update the current position
    if (direction)
        stepper_motors[id].current_position += steps;
    else
        stepper_motors[id].current_position -= steps;
}

/**
 * Move stepper motor to a position in millimeters
 */
void STEPPER_move_to_mm(stepper_id_t id, float position_mm, float speed_mm_s)
{
    if (id < 0 || id >= stepper_count || !stepper_motors[id].enabled)
    {
        printf("Error: Invalid stepper motor ID\n");
        return;
    }

    // Calculate the current position in millimeters
    float current_mm = (float)stepper_motors[id].current_position / stepper_motors[id].steps_per_mm;

    // Calculate the distance to move
    float distance_mm = position_mm - current_mm;

    // Calculate steps to move
    int32_t steps = (int32_t)(distance_mm * stepper_motors[id].steps_per_mm);

    // Calculate speed in steps per second - adjust speed to maintain 20kHz minimum
    float target_freq = 20000.0f; // 20kHz target frequency
    float min_speed_mm_s = target_freq / stepper_motors[id].steps_per_mm;

    // Use adjusted speed if less than minimum required for 20kHz
    float adjusted_speed_mm_s = speed_mm_s < min_speed_mm_s ? min_speed_mm_s : speed_mm_s;
    float steps_per_second = adjusted_speed_mm_s * stepper_motors[id].steps_per_mm;

    // Check if we need to move
    if (steps != 0)
    {
        // Check endstops if enabled
#if ENDSTOP_ENABLED
        if ((steps > 0 && stepper_motors[id].max_endstop_enabled &&
             STEPPER_check_endstop(id, ENDSTOP_MAX)) ||
            (steps < 0 && stepper_motors[id].min_endstop_enabled &&
             STEPPER_check_endstop(id, ENDSTOP_MIN)))
        {
            printf("Endstop triggered - move canceled\n");
            return;
        }
#endif

        // Move the motor
        STEPPER_move_steps(id, steps, steps_per_second);
    }
}

/**
 * Move stepper motor a given distance in millimeters
 */
void STEPPER_move_mm(stepper_id_t id, float distance_mm, float speed_mm_s)
{
    if (id < 0 || id >= stepper_count || !stepper_motors[id].enabled)
    {
        printf("Error: Invalid stepper motor ID\n");
        return;
    }

    // Calculate steps to move
    int32_t steps = (int32_t)(distance_mm * stepper_motors[id].steps_per_mm);

    // Calculate speed in steps per second - adjust speed to maintain 20kHz minimum
    float target_freq = 20000.0f; // 20kHz target frequency
    float min_speed_mm_s = target_freq / stepper_motors[id].steps_per_mm;

    // Use adjusted speed if less than minimum required for 20kHz
    float adjusted_speed_mm_s = speed_mm_s < min_speed_mm_s ? min_speed_mm_s : speed_mm_s;
    float steps_per_second = adjusted_speed_mm_s * stepper_motors[id].steps_per_mm;

    // Check if we need to move
    if (steps != 0)
    {
        // Check endstops if enabled
#if ENDSTOP_ENABLED
        if ((steps > 0 && stepper_motors[id].max_endstop_enabled &&
             STEPPER_check_endstop(id, ENDSTOP_MAX)) ||
            (steps < 0 && stepper_motors[id].min_endstop_enabled &&
             STEPPER_check_endstop(id, ENDSTOP_MIN)))
        {
            printf("Endstop triggered - move canceled\n");
            return;
        }
#endif

        // Move the motor
        STEPPER_move_steps(id, steps, steps_per_second);
    }
}

/**
 * Stop a stepper motor
 */
void STEPPER_stop(stepper_id_t id)
{
    if (id < 0 || id >= stepper_count)
    {
        printf("Error: Invalid stepper motor ID\n");
        return;
    }

    // Stop the timer
    BSP_TIMER_stop(stepper_motors[id].timer_id);
}

/**
 * Get the current position of a stepper motor in millimeters
 */
float STEPPER_get_position_mm(stepper_id_t id)
{
    if (id < 0 || id >= stepper_count)
    {
        printf("Error: Invalid stepper motor ID\n");
        return 0.0f;
    }

    return (float)stepper_motors[id].current_position / stepper_motors[id].steps_per_mm;
}

/**
 * Get the steps per mm value for the specified motor
 */
float STEPPER_get_steps_per_mm(stepper_id_t id)
{
    if (id < 0 || id >= stepper_count)
    {
        printf("Error: Invalid stepper motor ID\n");
        return 0.0f;
    }

    return stepper_motors[id].steps_per_mm;
}

/**
 * Home a stepper motor (move to zero position)
 */
void STEPPER_home(stepper_id_t id)
{
    if (id < 0 || id >= stepper_count)
    {
        printf("Error: Invalid stepper motor ID\n");
        return;
    }

#if ENDSTOP_ENABLED
    if (stepper_motors[id].min_endstop_enabled)
    {
        // Home using the endstop
        STEPPER_home_with_endstop(id, 5.0f, -1000.0f); // Move at 5mm/s, max 1000mm
    }
    else
#endif
    {
        // Simple homing just resets the position counter
        stepper_motors[id].current_position = 0;
        printf("Motor %d homed (position reset to zero)\n", id);
    }
}

#if ENDSTOP_ENABLED
/**
 * Configure endstop for a stepper motor
 */
HAL_StatusTypeDef STEPPER_config_endstop(stepper_id_t id, endstop_type_t endstop_type,
                                         GPIO_TypeDef *port, uint16_t pin,
                                         uint8_t trigger_level)
{
    if (id < 0 || id >= stepper_count)
    {
        printf("Error: Invalid stepper motor ID\n");
        return HAL_ERROR;
    }

    if (port == NULL)
    {
        // Disable the endstop
        if (endstop_type == ENDSTOP_MIN)
            stepper_motors[id].min_endstop_enabled = false;
        else
            stepper_motors[id].max_endstop_enabled = false;

        return HAL_OK;
    }

    // Configure GPIO for endstop pin
    BSP_GPIO_pin_config(port, pin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, GPIO_NO_AF);

    // Store the endstop configuration
    if (endstop_type == ENDSTOP_MIN)
    {
        stepper_motors[id].min_endstop_port = port;
        stepper_motors[id].min_endstop_pin = pin;
        stepper_motors[id].endstop_trigger_level = trigger_level;
        stepper_motors[id].min_endstop_enabled = true;
    }
    else
    {
        stepper_motors[id].max_endstop_port = port;
        stepper_motors[id].max_endstop_pin = pin;
        stepper_motors[id].endstop_trigger_level = trigger_level;
        stepper_motors[id].max_endstop_enabled = true;
    }

    printf("Endstop configured for motor %d, type %d\n", id, endstop_type);

    return HAL_OK;
}

/**
 * Check if an endstop is triggered
 */
bool STEPPER_check_endstop(stepper_id_t id, endstop_type_t endstop_type)
{
    if (id < 0 || id >= stepper_count)
    {
        printf("Error: Invalid stepper motor ID\n");
        return false;
    }

    GPIO_TypeDef *port;
    uint16_t pin;
    bool enabled;

    if (endstop_type == ENDSTOP_MIN)
    {
        port = stepper_motors[id].min_endstop_port;
        pin = stepper_motors[id].min_endstop_pin;
        enabled = stepper_motors[id].min_endstop_enabled;
    }
    else
    {
        port = stepper_motors[id].max_endstop_port;
        pin = stepper_motors[id].max_endstop_pin;
        enabled = stepper_motors[id].max_endstop_enabled;
    }

    if (!enabled || port == NULL)
        return false;

    // Read the pin state and compare with the trigger level
    GPIO_PinState state = HAL_GPIO_ReadPin(port, pin);
    return (state == GPIO_PIN_SET && stepper_motors[id].endstop_trigger_level == ENDSTOP_TRIGGER_HIGH) ||
           (state == GPIO_PIN_RESET && stepper_motors[id].endstop_trigger_level == ENDSTOP_TRIGGER_LOW);
}

/**
 * Home the motor using an endstop
 */
void STEPPER_home_with_endstop(stepper_id_t id, float speed_mm_s, float max_travel_mm)
{
    if (id < 0 || id >= stepper_count)
    {
        printf("Error: Invalid stepper motor ID\n");
        return;
    }

    if (!stepper_motors[id].min_endstop_enabled)
    {
        printf("Error: MIN endstop not configured for motor %d\n", id);
        return;
    }

    printf("Homing motor %d...\n", id);

    // Calculate base frequency requirement - targeting 20kHz for normal movement
    float target_freq = 20000.0f; // 20kHz target
    float min_speed_mm_s = target_freq / stepper_motors[id].steps_per_mm;

    // Use adjusted speed if less than minimum required for 20kHz
    float homing_speed_mm_s = speed_mm_s < min_speed_mm_s ? min_speed_mm_s : speed_mm_s;

    // First phase: Move towards endstop at normal speed
    float steps_per_second = homing_speed_mm_s * stepper_motors[id].steps_per_mm;
    int32_t max_steps = (int32_t)(max_travel_mm * stepper_motors[id].steps_per_mm);
    int32_t steps_moved = 0;
    const int32_t steps_per_check = 50; // Check endstop every 50 steps

    // Set direction towards endstop (negative direction for MIN endstop)
    HAL_GPIO_WritePin(stepper_motors[id].dir_port, stepper_motors[id].dir_pin, GPIO_PIN_RESET);

    // Configure timer for step pulses
    uint32_t timer_period_us = 1000000 / (uint32_t)steps_per_second / 2;
    BSP_TIMER_run_us(stepper_motors[id].timer_id, timer_period_us, false);
    BSP_TIMER_enable_PWM(stepper_motors[id].timer_id, stepper_motors[id].timer_channel, 500, false, false);

    // Move until endstop is triggered or max travel is reached
    while (!STEPPER_check_endstop(id, ENDSTOP_MIN) && steps_moved < max_steps)
    {
        // Move a few steps
        HAL_Delay((1000 * steps_per_check) / (uint32_t)steps_per_second);
        steps_moved += steps_per_check;

        // Update position
        stepper_motors[id].current_position -= steps_per_check;
    }

    // Stop the timer
    BSP_TIMER_stop(stepper_motors[id].timer_id);

    if (steps_moved >= max_steps)
    {
        printf("Error: Endstop not reached within maximum travel distance\n");
        return;
    }

    // Second phase: Back off from endstop - half the base speed (10kHz)
    HAL_Delay(100); // Short delay

    // Move away from endstop
    HAL_GPIO_WritePin(stepper_motors[id].dir_port, stepper_motors[id].dir_pin, GPIO_PIN_SET);

    // Slower speed for precision (10kHz)
    steps_per_second = (homing_speed_mm_s / 2) * stepper_motors[id].steps_per_mm;
    timer_period_us = 1000000 / (uint32_t)steps_per_second / 2;
    BSP_TIMER_run_us(stepper_motors[id].timer_id, timer_period_us, false);
    BSP_TIMER_enable_PWM(stepper_motors[id].timer_id, stepper_motors[id].timer_channel, 500, false, false);

    // Back off a small amount (2mm)
    int32_t backoff_steps = (int32_t)(2.0f * stepper_motors[id].steps_per_mm);
    HAL_Delay((1000 * backoff_steps) / (uint32_t)steps_per_second);
    stepper_motors[id].current_position += backoff_steps;

    // Stop the timer
    BSP_TIMER_stop(stepper_motors[id].timer_id);

    // Third phase: Approach endstop again at slower speed (5kHz)
    HAL_Delay(100);

    // Move towards endstop again
    HAL_GPIO_WritePin(stepper_motors[id].dir_port, stepper_motors[id].dir_pin, GPIO_PIN_RESET);

    // Very slow speed for precision (5kHz - quarter normal speed)
    steps_per_second = (homing_speed_mm_s / 4) * stepper_motors[id].steps_per_mm;
    timer_period_us = 1000000 / (uint32_t)steps_per_second / 2;
    BSP_TIMER_run_us(stepper_motors[id].timer_id, timer_period_us, false);
    BSP_TIMER_enable_PWM(stepper_motors[id].timer_id, stepper_motors[id].timer_channel, 500, false, false);

    // Move until endstop is triggered
    steps_moved = 0;
    while (!STEPPER_check_endstop(id, ENDSTOP_MIN) && steps_moved < (backoff_steps * 2))
    {
        // Move a few steps
        HAL_Delay((1000 * steps_per_check) / (uint32_t)steps_per_second);
        steps_moved += steps_per_check;

        // Update position
        stepper_motors[id].current_position -= steps_per_check;
    }

    // Stop the timer
    BSP_TIMER_stop(stepper_motors[id].timer_id);

    // Set current position to zero
    stepper_motors[id].current_position = 0;

    printf("Motor %d homed successfully\n", id);
}
#endif
