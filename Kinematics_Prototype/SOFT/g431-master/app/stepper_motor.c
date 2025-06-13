/**
 *******************************************************************************
 * @file    stepper_motor.c
 * @author  GitHub Copilot
 * @date    May 27, 2025
 * @brief   Stepper motor control with timer interrupts and acceleration profiles
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stepper_motor.h"
#include "stm32g4xx_hal.h" // Fixed: Added missing include for HAL_GetTick()
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/
static stepper_motor_t motors[STEPPER_MAX_MOTORS];
static bool system_initialized = false;
static uint32_t accel_update_counter = 0;

/* Private function declarations ---------------------------------------------*/
static bool is_motor_id_valid(uint8_t motor_id);
static void calculate_acceleration_profile(stepper_motor_t *motor, uint32_t steps, uint32_t target_freq);
static void update_acceleration(stepper_motor_t *motor);
static void set_motor_frequency(stepper_motor_t *motor, uint32_t frequency);
static void stop_motor_timer(stepper_motor_t *motor);
static bool check_limit_switches(stepper_motor_t *motor);

/* Public functions implementations ------------------------------------------*/

/**
 * @brief Initialize stepper motor system
 */
bool stepper_motor_system_init(void)
{
    printf("Initializing stepper motor system...\n");

    // Clear all motor configurations
    memset(motors, 0, sizeof(motors));

    // Initialize all motors as not initialized
    for (uint8_t i = 0; i < STEPPER_MAX_MOTORS; i++)
    {
        motors[i].initialized = false;
        motors[i].state = MOTOR_STATE_IDLE;
        motors[i].limit_check_enabled = false;

        // Set default acceleration profile
        motors[i].accel_profile.start_frequency = STEPPER_MIN_FREQUENCY;
        motors[i].accel_profile.target_frequency = STEPPER_DEFAULT_FREQUENCY;
        motors[i].accel_profile.max_frequency = STEPPER_MAX_FREQUENCY;
        motors[i].accel_profile.acceleration = 5000; // 5000 steps/sec²
        motors[i].accel_profile.deceleration = 5000; // 5000 steps/sec²
        motors[i].accel_profile.use_acceleration = true;
    }

    system_initialized = true;
    printf("Stepper motor system initialized\n");
    return true;
}

/**
 * @brief Initialize a stepper motor
 */
bool stepper_motor_init(uint8_t motor_id,
                        GPIO_TypeDef *pulse_port, uint16_t pulse_pin,
                        GPIO_TypeDef *dir_port, uint16_t dir_pin,
                        timer_id_t timer_id, uint16_t timer_channel,
                        uint16_t microsteps)
{
    if (!system_initialized)
    {
        printf("Error: Stepper motor system not initialized\n");
        return false;
    }

    if (!is_motor_id_valid(motor_id))
    {
        printf("Error: Invalid motor ID %d\n", motor_id);
        return false;
    }

    stepper_motor_t *motor = &motors[motor_id];

    // Configure direction pin as normal GPIO output
    BSP_GPIO_pin_config(dir_port, dir_pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                        GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);

    // Configure pulse pin for timer PWM - use BSP timer PWM setup
    // The BSP_TIMER_enable_PWM will configure the GPIO automatically

    // Store configuration
    motor->pulse_port = pulse_port;
    motor->pulse_pin = pulse_pin;
    motor->dir_port = dir_port;
    motor->dir_pin = dir_pin;
    motor->timer_id = timer_id;
    motor->timer_channel = timer_channel;
    motor->microsteps = microsteps;
    motor->state = MOTOR_STATE_IDLE;
    motor->direction = MOTOR_DIR_CLOCKWISE;
    motor->current_frequency = 0;
    motor->target_steps = 0;
    motor->completed_steps = 0;
    motor->initialized = true;

    // Set direction pin to known state
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);

    printf("Motor %d initialized: Timer%d CH%d, microsteps=%d\n",
           motor_id, timer_id + 1, (timer_channel >> 2) + 1, microsteps);

    return true;
}

/**
 * @brief Set acceleration profile for a motor
 */
bool stepper_motor_set_acceleration(uint8_t motor_id, uint32_t acceleration, uint32_t max_frequency)
{
    if (!is_motor_id_valid(motor_id) || !motors[motor_id].initialized)
    {
        return false;
    }

    stepper_motor_t *motor = &motors[motor_id];

    // Clamp values to safe ranges
    if (max_frequency < STEPPER_MIN_FREQUENCY)
        max_frequency = STEPPER_MIN_FREQUENCY;
    if (max_frequency > STEPPER_MAX_FREQUENCY)
        max_frequency = STEPPER_MAX_FREQUENCY;
    if (acceleration < 1000)
        acceleration = 1000; // Minimum 1000 steps/sec²
    if (acceleration > 50000)
        acceleration = 50000; // Maximum 50000 steps/sec²

    motor->accel_profile.acceleration = acceleration;
    motor->accel_profile.deceleration = acceleration; // Same as acceleration
    motor->accel_profile.max_frequency = max_frequency;
    motor->accel_profile.use_acceleration = true;

    printf("Motor %d: Acceleration set to %lu steps/sec², max freq %lu Hz\n",
           motor_id, acceleration, max_frequency);

    return true;
}

/**
 * @brief Enable/disable acceleration for a motor
 */
bool stepper_motor_enable_acceleration(uint8_t motor_id, bool enable)
{
    if (!is_motor_id_valid(motor_id) || !motors[motor_id].initialized)
    {
        return false;
    }

    motors[motor_id].accel_profile.use_acceleration = enable;
    printf("Motor %d: Acceleration %s\n", motor_id, enable ? "enabled" : "disabled");

    return true;
}

/**
 * @brief Move motor with acceleration profile
 */
bool stepper_motor_move_accel(uint8_t motor_id, uint32_t steps, uint32_t target_frequency, stepper_motor_dir_t direction)
{
    if (!is_motor_id_valid(motor_id) || !motors[motor_id].initialized)
    {
        printf("Error: Invalid motor ID or motor not initialized\n");
        return false;
    }

    stepper_motor_t *motor = &motors[motor_id];

    if (motor->state != MOTOR_STATE_IDLE)
    {
        printf("Error: Motor %d is busy (state: %d)\n", motor_id, motor->state);
        return false;
    }

    if (steps == 0)
    {
        printf("Warning: Zero steps requested for motor %d\n", motor_id);
        return true;
    }

    // Check limit switches before starting movement
    if (motor->limit_check_enabled && check_limit_switches(motor))
    {
        printf("Error: Limit switch triggered, cannot start movement\n");
        return false;
    }

    // Clamp target frequency to valid range
    if (target_frequency < STEPPER_MIN_FREQUENCY)
        target_frequency = STEPPER_MIN_FREQUENCY;
    if (target_frequency > motor->accel_profile.max_frequency)
        target_frequency = motor->accel_profile.max_frequency;

    // Set direction
    motor->direction = direction;
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin,
                      (direction == MOTOR_DIR_CLOCKWISE) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Setup movement parameters
    motor->target_steps = steps;
    motor->completed_steps = 0;
    motor->accel_profile.target_frequency = target_frequency;

    // Calculate acceleration profile
    if (motor->accel_profile.use_acceleration)
    {
        calculate_acceleration_profile(motor, steps, target_frequency);
        motor->current_frequency = motor->accel_profile.start_frequency;
        motor->state = MOTOR_STATE_ACCELERATING;
    }
    else
    {
        motor->current_frequency = target_frequency;
        motor->accel_steps = 0;
        motor->decel_steps = 0;
        motor->const_steps = steps;
        motor->state = MOTOR_STATE_MOVING;
    }

    // Start timer with PWM - calculate period for frequency
    uint32_t period_us = 1000000 / motor->current_frequency; // Period in microseconds

    // Initialize timer with calculated period and enable interrupts
    BSP_TIMER_run_us(motor->timer_id, period_us, true);

    // Enable PWM on the motor's channel with 50% duty cycle
    BSP_TIMER_enable_PWM(motor->timer_id, motor->timer_channel, 500, false, false);

    printf("Motor %d: Started %lu steps at %lu Hz (%lu us period), direction %d, state %d\n",
           motor_id, steps, motor->current_frequency, period_us, direction, motor->state);

    return true;
}

/**
 * @brief Move motor at constant speed (legacy compatibility)
 */
bool stepper_motor_move(uint8_t motor_id, uint32_t steps, uint32_t frequency, stepper_motor_dir_t direction)
{
    // Temporarily disable acceleration for constant speed move
    if (!is_motor_id_valid(motor_id) || !motors[motor_id].initialized)
    {
        return false;
    }

    bool old_accel_setting = motors[motor_id].accel_profile.use_acceleration;
    motors[motor_id].accel_profile.use_acceleration = false;

    bool result = stepper_motor_move_accel(motor_id, steps, frequency, direction);

    motors[motor_id].accel_profile.use_acceleration = old_accel_setting;

    return result;
}

/**
 * @brief Stop motor immediately
 */
bool stepper_motor_stop(uint8_t motor_id)
{
    if (!is_motor_id_valid(motor_id) || !motors[motor_id].initialized)
    {
        return false;
    }

    stepper_motor_t *motor = &motors[motor_id];

    // Stop timer
    stop_motor_timer(motor);

    // Reset state
    motor->state = MOTOR_STATE_IDLE;
    motor->current_frequency = 0;
    motor->target_steps = 0;
    motor->completed_steps = 0;

    printf("Motor %d stopped\n", motor_id);

    return true;
}

/**
 * @brief Emergency stop all motors
 */
void stepper_motor_emergency_stop_all(void)
{
    printf("EMERGENCY STOP: Stopping all motors\n");

    for (uint8_t i = 0; i < STEPPER_MAX_MOTORS; i++)
    {
        if (motors[i].initialized && motors[i].state != MOTOR_STATE_IDLE)
        {
            stepper_motor_stop(i);
        }
    }
}

/**
 * @brief Get motor state
 */
stepper_motor_state_t stepper_motor_get_state(uint8_t motor_id)
{
    if (!is_motor_id_valid(motor_id) || !motors[motor_id].initialized)
    {
        return MOTOR_STATE_ERROR;
    }

    return motors[motor_id].state;
}

/**
 * @brief Get completed steps
 */
uint32_t stepper_motor_get_completed_steps(uint8_t motor_id)
{
    if (!is_motor_id_valid(motor_id) || !motors[motor_id].initialized)
    {
        return 0;
    }

    return motors[motor_id].completed_steps;
}

/**
 * @brief Get current frequency
 */
uint32_t stepper_motor_get_current_frequency(uint8_t motor_id)
{
    if (!is_motor_id_valid(motor_id) || !motors[motor_id].initialized)
    {
        return 0;
    }

    return motors[motor_id].current_frequency;
}

/**
 * @brief Update stepper motor system - call in main loop
 */
void stepper_motor_update(void)
{
    if (!system_initialized)
    {
        return;
    }

    // Check each motor for completion and state updates
    for (uint8_t i = 0; i < STEPPER_MAX_MOTORS; i++)
    {
        stepper_motor_t *motor = &motors[i];

        if (motor->initialized && motor->state != MOTOR_STATE_IDLE)
        {
            // Check if movement is complete
            if (motor->completed_steps >= motor->target_steps && motor->state != MOTOR_STATE_HOMING)
            {
                printf("Motor %d movement complete: %lu/%lu steps - STOPPING\n",
                       i, motor->completed_steps, motor->target_steps);

                // Stop the timer and reset state
                stop_motor_timer(motor);
                motor->state = MOTOR_STATE_IDLE;
                motor->current_frequency = 0;

                continue;
            }

            // Check limit switches during movement
            if (motor->limit_check_enabled && check_limit_switches(motor))
            {
                printf("Motor %d stopped by limit switch\n", i);
                stop_motor_timer(motor);
                motor->state = MOTOR_STATE_IDLE;
                motor->current_frequency = 0;
                continue;
            }

            // Update acceleration profile
            update_acceleration(motor);
        }
    }
}

/**
 * @brief Associate motor with axis for limit switch checking
 */
void stepper_motor_set_axis(uint8_t motor_id, axis_t axis)
{
    if (is_motor_id_valid(motor_id) && motors[motor_id].initialized)
    {
        motors[motor_id].axis = axis;
        printf("Motor %d associated with %s axis\n", motor_id, limit_switch_axis_to_string(axis));
    }
}

/**
 * @brief Enable/disable limit switch checking for motor
 */
void stepper_motor_enable_limit_check(uint8_t motor_id, bool enable)
{
    if (is_motor_id_valid(motor_id) && motors[motor_id].initialized)
    {
        motors[motor_id].limit_check_enabled = enable;
        printf("Motor %d: Limit checking %s\n", motor_id, enable ? "enabled" : "disabled");
    }
}

/**
 * @brief Home motor (move until limit switch triggered)
 */
bool stepper_motor_home(uint8_t motor_id, uint32_t frequency)
{
    if (!is_motor_id_valid(motor_id) || !motors[motor_id].initialized)
    {
        return false;
    }

    stepper_motor_t *motor = &motors[motor_id];

    if (motor->state != MOTOR_STATE_IDLE)
    {
        printf("Error: Motor %d is busy, cannot home\n", motor_id);
        return false;
    }

    // Enable limit checking for homing
    bool old_limit_setting = motor->limit_check_enabled;
    motor->limit_check_enabled = true;

    // Set homing state and direction (assume homing moves toward MIN limit)
    motor->state = MOTOR_STATE_HOMING;
    motor->direction = MOTOR_DIR_COUNTERCLOCKWISE; // Toward home/MIN

    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_SET); // CCW

    // Start continuous movement at homing speed
    motor->target_steps = 0xFFFFFFFF; // Large number for continuous movement
    motor->completed_steps = 0;
    motor->current_frequency = frequency;

    set_motor_frequency(motor, frequency);

    printf("Motor %d: Homing started at %lu Hz\n", motor_id, frequency);

    return true;
}

/**
 * @brief Move motor with limit checking
 */
bool stepper_motor_move_with_limits(uint8_t motor_id, uint32_t steps, uint32_t frequency, stepper_motor_dir_t direction)
{
    if (!is_motor_id_valid(motor_id) || !motors[motor_id].initialized)
    {
        return false;
    }

    // Temporarily enable limit checking
    bool old_limit_setting = motors[motor_id].limit_check_enabled;
    motors[motor_id].limit_check_enabled = true;

    bool result = stepper_motor_move(motor_id, steps, frequency, direction);

    // Note: Don't restore old setting immediately, let the movement complete with limits enabled

    return result;
}

/**
 * @brief Manual step for testing
 */
void stepper_motor_manual_step(uint8_t motor_id, uint32_t steps, uint32_t delay_ms)
{
    if (!is_motor_id_valid(motor_id) || !motors[motor_id].initialized)
    {
        return;
    }

    stepper_motor_t *motor = &motors[motor_id];

    printf("Manual stepping motor %d: %lu steps with %lu ms delay\n", motor_id, steps, delay_ms);

    // Set direction
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);
    HAL_Delay(1);

    // Generate steps manually
    for (uint32_t i = 0; i < steps; i++)
    {
        HAL_GPIO_WritePin(motor->pulse_port, motor->pulse_pin, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(motor->pulse_port, motor->pulse_pin, GPIO_PIN_RESET);
        HAL_Delay(delay_ms);

        printf("Step %lu/%lu\n", i + 1, steps);
    }

    printf("Manual stepping complete\n");
}

/**
 * @brief Debug motor configuration
 */
void stepper_motor_debug_config(uint8_t motor_id)
{
    if (!is_motor_id_valid(motor_id))
    {
        printf("Invalid motor ID: %d\n", motor_id);
        return;
    }

    stepper_motor_t *motor = &motors[motor_id];

    printf("=== Motor %d Configuration ===\n", motor_id);
    printf("Initialized: %s\n", motor->initialized ? "Yes" : "No");

    if (motor->initialized)
    {
        printf("Timer: %d, Channel: 0x%04X\n", motor->timer_id, motor->timer_channel);
        printf("Microsteps: %d\n", motor->microsteps);
        printf("State: %d\n", motor->state);
        printf("Current Freq: %lu Hz\n", motor->current_frequency);
        printf("Completed Steps: %lu/%lu\n", motor->completed_steps, motor->target_steps);
        printf("Acceleration: %s\n", motor->accel_profile.use_acceleration ? "Enabled" : "Disabled");
        printf("Limit Check: %s\n", motor->limit_check_enabled ? "Enabled" : "Disabled");
    }
    printf("===============================\n");
}

/**
 * @brief Timer interrupt handler for motor pulse generation
 */
void stepper_motor_timer_interrupt(timer_id_t timer_id)
{
    // Find motor using this timer
    for (uint8_t i = 0; i < STEPPER_MAX_MOTORS; i++)
    {
        stepper_motor_t *motor = &motors[i];

        if (motor->initialized && motor->timer_id == timer_id && motor->state != MOTOR_STATE_IDLE)
        {
            // Increment step counter
            motor->completed_steps++;

            // Debug every 500 steps to reduce output
            if (motor->completed_steps % 500 == 0)
            {
                printf("Motor %d: Step %lu/%lu (%.1f%%) at %lu Hz\n",
                       i, motor->completed_steps, motor->target_steps,
                       (float)motor->completed_steps * 100.0f / (float)motor->target_steps,
                       motor->current_frequency);
            }

            // Check for completion directly in interrupt
            if (motor->completed_steps >= motor->target_steps && motor->state != MOTOR_STATE_HOMING)
            {
                // Stop immediately
                stop_motor_timer(motor);
                motor->state = MOTOR_STATE_IDLE;
                motor->current_frequency = 0;

                printf("Motor %d COMPLETED: %lu steps\n", i, motor->completed_steps);
                return;
            }

            return; // Only handle one motor per interrupt
        }
    }
}

/* Private functions implementations ------------------------------------------*/

/**
 * @brief Check if motor ID is valid
 */
static bool is_motor_id_valid(uint8_t motor_id)
{
    return motor_id < STEPPER_MAX_MOTORS;
}

/**
 * @brief Calculate acceleration profile for a move
 */
static void calculate_acceleration_profile(stepper_motor_t *motor, uint32_t steps, uint32_t target_freq)
{
    uint32_t start_freq = motor->accel_profile.start_frequency;
    uint32_t accel = motor->accel_profile.acceleration;

    // Calculate steps needed for acceleration and deceleration
    uint32_t accel_steps = ((target_freq * target_freq) - (start_freq * start_freq)) / (2 * accel);
    uint32_t decel_steps = accel_steps; // Symmetric profile

    // Ensure we don't exceed total steps
    if ((accel_steps + decel_steps) > steps)
    {
        // Triangular profile - no constant speed phase
        accel_steps = steps / 2;
        decel_steps = steps - accel_steps;
        motor->const_steps = 0;
    }
    else
    {
        // Trapezoidal profile
        motor->const_steps = steps - accel_steps - decel_steps;
    }

    motor->accel_steps = accel_steps;
    motor->decel_steps = decel_steps;

    printf("Acceleration profile: accel=%lu, const=%lu, decel=%lu steps\n",
           accel_steps, motor->const_steps, decel_steps);
}

/**
 * @brief Update acceleration during movement
 */
static void update_acceleration(stepper_motor_t *motor)
{
    if (!motor->accel_profile.use_acceleration)
    {
        return;
    }

    uint32_t new_frequency = motor->current_frequency;

    if (motor->state == MOTOR_STATE_ACCELERATING)
    {
        if (motor->completed_steps >= motor->accel_steps)
        {
            // Transition to constant speed or deceleration
            motor->state = (motor->const_steps > 0) ? MOTOR_STATE_MOVING : MOTOR_STATE_DECELERATING;
            new_frequency = motor->accel_profile.target_frequency;
        }
        else
        {
            // Continue accelerating
            float progress = (float)motor->completed_steps / motor->accel_steps;
            new_frequency = motor->accel_profile.start_frequency +
                            (uint32_t)(progress * (motor->accel_profile.target_frequency - motor->accel_profile.start_frequency));
        }
    }
    else if (motor->state == MOTOR_STATE_MOVING)
    {
        if (motor->completed_steps >= (motor->accel_steps + motor->const_steps))
        {
            // Transition to deceleration
            motor->state = MOTOR_STATE_DECELERATING;
            new_frequency = motor->accel_profile.target_frequency;
        }
    }
    else if (motor->state == MOTOR_STATE_DECELERATING)
    {
        uint32_t decel_start = motor->accel_steps + motor->const_steps;
        uint32_t decel_progress = motor->completed_steps - decel_start;

        if (decel_progress < motor->decel_steps)
        {
            float progress = (float)decel_progress / motor->decel_steps;
            new_frequency = motor->accel_profile.target_frequency -
                            (uint32_t)(progress * (motor->accel_profile.target_frequency - motor->accel_profile.start_frequency));
        }
    }

    // Update frequency if changed
    if (new_frequency != motor->current_frequency)
    {
        motor->current_frequency = new_frequency;
        set_motor_frequency(motor, new_frequency);
    }
}

/**
 * @brief Set motor frequency by updating timer period
 */
static void set_motor_frequency(stepper_motor_t *motor, uint32_t frequency)
{
    if (frequency == 0)
    {
        stop_motor_timer(motor);
        return;
    }

    // Clamp frequency to safe range
    if (frequency < STEPPER_MIN_FREQUENCY)
        frequency = STEPPER_MIN_FREQUENCY;
    if (frequency > STEPPER_MAX_FREQUENCY)
        frequency = STEPPER_MAX_FREQUENCY;

    // Calculate new period in microseconds
    uint32_t period_us = 1000000 / frequency;

    // Restart with new period
    BSP_TIMER_run_us(motor->timer_id, period_us, true);
    BSP_TIMER_enable_PWM(motor->timer_id, motor->timer_channel, 500, false, false);

    motor->current_frequency = frequency;
}

/**
 * @brief Stop motor timer
 */
static void stop_motor_timer(stepper_motor_t *motor)
{
    if (motor->current_frequency > 0)
    {
        BSP_TIMER_enable_PWM(motor->timer_id, motor->timer_channel, 0, false, false);

        motor->current_frequency = 0;
        printf("Timer %d stopped for motor\n", motor->timer_id + 1);
    }
}

/**
 * @brief Check limit switches for motor
 */
static bool check_limit_switches(stepper_motor_t *motor)
{
    if (!motor->limit_check_enabled)
    {
        return false;
    }

    // Check appropriate limit switch based on direction
    limit_position_t limit_to_check = (motor->direction == MOTOR_DIR_CLOCKWISE) ? LIMIT_MAX : LIMIT_MIN;

    return (limit_switch_read(motor->axis, limit_to_check) == LIMIT_TRIGGERED);
}
