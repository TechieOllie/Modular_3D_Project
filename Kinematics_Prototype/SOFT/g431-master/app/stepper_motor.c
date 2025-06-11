/**
 *******************************************************************************
 * @file    stepper_motor.c
 * @author  GitHub Copilot
 * @date    May 27, 2025
 * @brief   Stepper motor control using TB6600 driver with PWM signals
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stepper_motor.h"
#include "limit_switches.h"
#include "stm32g4_timer.h"
#include "stm32g4_gpio.h"
#include <stdio.h>
#include <stdlib.h>

/* Private variables ---------------------------------------------------------*/
static stepper_motor_t motors[STEPPER_MAX_MOTORS];
static uint8_t motor_count = 0;

/* Private function declarations ---------------------------------------------*/
static void configure_pwm_timer(stepper_motor_t *motor, uint32_t frequency_hz);
static void timer_interrupt_handler(timer_id_t timer_id);
static void set_direction_pin(stepper_motor_t *motor, stepper_motor_dir_t direction);
static bool check_motor_limits_before_move(uint8_t motor_id, stepper_motor_dir_t direction);
bool stepper_motor_reset(uint8_t motor_id);

/* Public functions implementations ------------------------------------------*/

/**
 * @brief Initialize a stepper motor with specified parameters
 */
bool stepper_motor_init(uint8_t motor_id,
                        GPIO_TypeDef *pul_gpio, uint16_t pul_pin,
                        GPIO_TypeDef *dir_gpio, uint16_t dir_pin,
                        timer_id_t timer_id, uint16_t timer_channel,
                        uint16_t microsteps)
{
    // Check if motor_id is valid
    if (motor_id >= STEPPER_MAX_MOTORS)
    {
        return false;
    }

    // Check if the microsteps value is valid (should be 8, 16, or 32)
    if (microsteps != 8 && microsteps != 16 && microsteps != 32)
    {
        microsteps = STEPPER_MICROSTEPS_DEFAULT;
    }

    // Configure the motor structure
    stepper_motor_t *motor = &motors[motor_id];
    motor->pul_gpio = pul_gpio;
    motor->pul_pin = pul_pin;
    motor->dir_gpio = dir_gpio;
    motor->dir_pin = dir_pin;
    motor->timer_id = timer_id;
    motor->timer_channel = timer_channel;
    motor->microsteps = microsteps;
    motor->state = MOTOR_STATE_IDLE;
    motor->step_delay_us = 10000; // Default to 100Hz (10ms)
    motor->steps_to_move = 0;
    motor->steps_moved = 0;
    motor->direction = MOTOR_DIR_CLOCKWISE;

    // Initialize safety features
    motor->limit_check_enabled = false;
    motor->associated_axis = 255; // No axis associated by default

    // Configure PUL GPIO pin with appropriate alternate function based on timer
    uint32_t gpio_af;
    switch (timer_id)
    {
    case TIMER1_ID:
        gpio_af = GPIO_AF4_TIM1;
        break;
    case TIMER2_ID:
        gpio_af = GPIO_AF1_TIM2;
        break;
    case TIMER3_ID:
        gpio_af = GPIO_AF2_TIM3;
        break;
    case TIMER4_ID:
        gpio_af = GPIO_AF2_TIM4;
        break;
    default:
        gpio_af = GPIO_AF4_TIM1; // Default fallback
        break;
    }

    BSP_GPIO_pin_config(pul_gpio, pul_pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, gpio_af);

    // Configure DIR GPIO pin
    BSP_GPIO_pin_config(dir_gpio, dir_pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);

    // Initialize DIR pin to clockwise direction (low)
    set_direction_pin(motor, MOTOR_DIR_CLOCKWISE);

    // Set the motor as initialized
    motor->initialized = true;
    motor_count++;

    printf("Motor %d initialized: PUL=%s Pin_%d, DIR=%s Pin_%d, Timer=%d, Channel=%d\n",
           motor_id,
           (pul_gpio == GPIOA) ? "GPIOA" : (pul_gpio == GPIOB) ? "GPIOB"
                                                               : "GPIOC",
           __builtin_ctz(pul_pin), // Convert GPIO_PIN_x to actual pin number
           (dir_gpio == GPIOA) ? "GPIOA" : (dir_gpio == GPIOB) ? "GPIOB"
                                                               : "GPIOC",
           __builtin_ctz(dir_pin), // Convert GPIO_PIN_x to actual pin number
           timer_id, timer_channel);

    return true;
}

/**
 * @brief Associate a motor with an axis for limit switch checking
 */
bool stepper_motor_set_axis(uint8_t motor_id, uint8_t axis_id)
{
    if (motor_id >= STEPPER_MAX_MOTORS || !motors[motor_id].initialized)
    {
        return false;
    }

    motors[motor_id].associated_axis = axis_id;

    printf("Motor %d associated with axis %s\n", motor_id,
           (axis_id == 0) ? "X" : (axis_id == 1) ? "Y"
                              : (axis_id == 2)   ? "Z"
                                                 : "NONE");

    return true;
}

/**
 * @brief Enable/disable limit switch checking for a motor
 */
bool stepper_motor_enable_limit_check(uint8_t motor_id, bool enable)
{
    if (motor_id >= STEPPER_MAX_MOTORS || !motors[motor_id].initialized)
    {
        return false;
    }

    motors[motor_id].limit_check_enabled = enable;

    printf("Motor %d limit checking %s\n", motor_id, enable ? "enabled" : "disabled");

    return true;
}

/**
 * @brief Start motor movement with specified number of steps, speed, and direction
 */
bool stepper_motor_move(uint8_t motor_id, uint32_t steps, uint32_t speed_steps_per_second,
                        stepper_motor_dir_t direction)
{
    // Check if motor_id is valid
    if (motor_id >= STEPPER_MAX_MOTORS || !motors[motor_id].initialized)
    {
        printf("ERROR: Motor %d is not valid or not initialized!\n", motor_id);
        return false;
    }

    stepper_motor_t *motor = &motors[motor_id];

    printf("Motor %d move requested: %lu steps at %lu Hz, direction=%d\n",
           motor_id, steps, speed_steps_per_second, direction);

    // Explicitly stop any previous movement
    if (motor->state != MOTOR_STATE_IDLE)
    {
        stepper_motor_stop(motor_id);
    }

    // Reset counters explicitly
    motor->steps_moved = 0;
    motor->steps_to_move = steps;

    // Set direction
    set_direction_pin(motor, direction);
    motor->direction = direction;

    // Calculate step delay
    motor->step_delay_us = 1000000 / speed_steps_per_second;

    printf("Starting movement: %lu steps at %lu Hz (delay: %lu us)\n",
           steps, speed_steps_per_second, motor->step_delay_us);

    // Configure and start the timer
    configure_pwm_timer(motor, speed_steps_per_second);

    // Update motor state
    motor->state = MOTOR_STATE_MOVING;

    return true;
}

/**
 * @brief Start motor movement with limit switch checking
 */
bool stepper_motor_move_with_limits(uint8_t motor_id, uint32_t steps,
                                    uint32_t speed_steps_per_second,
                                    stepper_motor_dir_t direction)
{
    if (motor_id >= STEPPER_MAX_MOTORS || !motors[motor_id].initialized)
    {
        printf("ERROR: Motor %d is not valid or not initialized!\n", motor_id);
        return false;
    }

    stepper_motor_t *motor = &motors[motor_id];

    // Check limits before starting movement
    if (motor->limit_check_enabled && !check_motor_limits_before_move(motor_id, direction))
    {
        printf("ERROR: Cannot move motor %d - limit switch already triggered!\n", motor_id);
        return false;
    }

    // If steps is 0, move until limit is hit (used for homing)
    if (steps == 0)
    {
        steps = 0xFFFFFFFF; // Very large number
        printf("Motor %d: Moving until limit switch in direction %d\n", motor_id, direction);
    }

    // Start normal movement
    return stepper_motor_move(motor_id, steps, speed_steps_per_second, direction);
}

/**
 * @brief Home a motor by moving towards minimum limit switch
 */
bool stepper_motor_home(uint8_t motor_id, uint32_t speed_steps_per_second)
{
    if (motor_id >= STEPPER_MAX_MOTORS || !motors[motor_id].initialized)
    {
        return false;
    }

    stepper_motor_t *motor = &motors[motor_id];

    if (!motor->limit_check_enabled || motor->associated_axis == 255)
    {
        printf("ERROR: Motor %d cannot home - no axis or limits configured\n", motor_id);
        return false;
    }

    printf("Homing motor %d (axis %d) at %lu Hz...\n",
           motor_id, motor->associated_axis, speed_steps_per_second);

    // Move towards minimum limit (usually counterclockwise for homing)
    return stepper_motor_move_with_limits(motor_id, 0, speed_steps_per_second,
                                          MOTOR_DIR_COUNTERCLOCKWISE);
}

/**
 * @brief Stop a motor
 */
bool stepper_motor_stop(uint8_t motor_id)
{
    // Check if motor_id is valid
    if (motor_id >= STEPPER_MAX_MOTORS || !motors[motor_id].initialized)
    {
        return false;
    }

    stepper_motor_t *motor = &motors[motor_id];

    // Stop the timer completely
    BSP_TIMER_enable_PWM(motor->timer_id, motor->timer_channel, 0, false, false);

    // Wait a bit to ensure timer is completely stopped
    HAL_Delay(1);

    // Reconfigure pin as output and ensure it's low - this is the key fix
    BSP_GPIO_pin_config(motor->pul_gpio, motor->pul_pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                        GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
    HAL_GPIO_WritePin(motor->pul_gpio, motor->pul_pin, GPIO_PIN_RESET);

    // Also ensure DIR pin is in a stable state
    HAL_GPIO_WritePin(motor->dir_gpio, motor->dir_pin, GPIO_PIN_RESET);

    // Print final step count
    printf("Motor %d stopped: moved %lu of %lu steps\n",
           motor_id, motor->steps_moved, motor->steps_to_move);

    // Update motor state and reset counters
    motor->state = MOTOR_STATE_IDLE;
    motor->target_speed_hz = 0;
    motor->current_speed_hz = 0;

    return true;
}

/**
 * @brief Get motor state
 */
stepper_motor_state_t stepper_motor_get_state(uint8_t motor_id)
{
    // Check if motor_id is valid
    if (motor_id >= STEPPER_MAX_MOTORS || !motors[motor_id].initialized)
    {
        return MOTOR_STATE_IDLE;
    }

    return motors[motor_id].state;
}

/**
 * @brief Update motor state - should be called periodically in main loop
 */
void stepper_motor_update(void)
{
    // Check status of each motor
    for (uint8_t i = 0; i < STEPPER_MAX_MOTORS; i++)
    {
        stepper_motor_t *motor = &motors[i];

        if (!motor->initialized || motor->state != MOTOR_STATE_MOVING)
        {
            continue;
        }

        // Check limit switches if enabled
        if (motor->limit_check_enabled && motor->associated_axis != 255)
        {
            axis_t axis = (axis_t)motor->associated_axis;

            // Check if any limit on this axis is triggered
            if (limit_switch_axis_triggered(axis))
            {
                printf("Limit triggered during movement - stopping motor %d\n", i);
                stepper_motor_stop(i);
                continue;
            }
        }

        // Check if we've completed all steps
        if (motor->steps_moved >= motor->steps_to_move)
        {
            stepper_motor_stop(i);
        }
    }
}

/**
 * @brief Manually step a motor a specific number of times with delay
 */

void stepper_motor_manual_step(uint8_t motor_id, uint32_t steps, uint32_t delay_ms)
{
    if (motor_id >= STEPPER_MAX_MOTORS || !motors[motor_id].initialized)
    {
        return;
    }

    stepper_motor_t *motor = &motors[motor_id];

    printf("Manually stepping motor %d for %lu steps\n", motor_id, steps);

    // Temporarily reconfigure the pin as a standard output for manual stepping
    BSP_GPIO_pin_config(motor->pul_gpio, motor->pul_pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                        GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);

    for (uint32_t i = 0; i < steps; i++)
    {
        // Set pulse pin high
        HAL_GPIO_WritePin(motor->pul_gpio, motor->pul_pin, 1);
        HAL_Delay(delay_ms);

        // Set pulse pin low
        HAL_GPIO_WritePin(motor->pul_gpio, motor->pul_pin, 0);
        HAL_Delay(delay_ms);

        if (i % 10 == 0)
        {
            printf("Step %lu\n", i);
        }
    }
    printf("Manual stepping complete\n");

    // Reconfigure the pin for timer functionality afterward with correct alternate function
    uint32_t gpio_af;
    switch (motor->timer_id)
    {
    case TIMER1_ID:
        gpio_af = GPIO_AF4_TIM1;
        break;
    case TIMER2_ID:
        gpio_af = GPIO_AF1_TIM2;
        break;
    case TIMER3_ID:
        gpio_af = GPIO_AF2_TIM3;
        break;
    case TIMER4_ID:
        gpio_af = GPIO_AF2_TIM4;
        break;
    default:
        gpio_af = GPIO_AF4_TIM1; // Default fallback
        break;
    }

    BSP_GPIO_pin_config(motor->pul_gpio, motor->pul_pin, GPIO_MODE_AF_PP, GPIO_NOPULL,
                        GPIO_SPEED_FREQ_HIGH, gpio_af);
}

/**
 * @brief Test routine for stepping motor at various speeds
 */
void stepper_motor_speed_test(uint8_t motor_id)
{
    uint32_t test_speeds[] = {1000, 2000, 5000, 8000};
    uint8_t num_speeds = sizeof(test_speeds) / sizeof(test_speeds[0]);
    uint32_t last_print = 0; // Move from static to regular variable

    printf("Starting speed test sequence for motor %d...\n", motor_id);

    for (uint8_t i = 0; i < num_speeds; i++)
    {
        uint32_t speed = test_speeds[i];
        printf("Testing motor %d at speed: %lu Hz for 1000 steps\n", motor_id, speed);

        // Reset motor state before starting new test
        stepper_motor_reset(motor_id);
        HAL_Delay(100); // Brief delay after reset

        // Use regular move function
        stepper_motor_move(motor_id, 1000, speed, MOTOR_DIR_CLOCKWISE);

        // Wait until motor stops
        uint32_t start_time = HAL_GetTick();
        uint32_t timeout = 5000 + (1000 * 1000 / speed); // More time for slower speeds        while (stepper_motor_get_state(motor_id) != MOTOR_STATE_IDLE)
        {
            stepper_motor_update();

            // Print progress periodically
            static uint32_t last_print[STEPPER_MAX_MOTORS] = {0}; // Separate for each motor
            uint32_t now = HAL_GetTick();
            if (now - last_print[motor_id] >= 500) // Every 500ms
            {
                last_print[motor_id] = now;
                printf("Motor %d Progress: %lu/%lu steps\n", motor_id,
                       motors[motor_id].steps_moved,
                       motors[motor_id].steps_to_move);
            }

            // Check for timeout
            if (HAL_GetTick() - start_time > timeout)
            {
                printf("Timeout waiting for motor %d to stop! Forcing stop.\n", motor_id);
                stepper_motor_stop(motor_id);
                break;
            }

            HAL_Delay(10);
        }

        // Wait between tests
        printf("Completed %lu Hz test for motor %d. Waiting 2 seconds...\n", speed, motor_id);
        HAL_Delay(2000);
    }

    printf("Speed test complete for motor %d!\n", motor_id);
}

/**
 * @brief Reset a stepper motor's position and state
 */
bool stepper_motor_reset(uint8_t motor_id)
{
    // Check if motor_id is valid
    if (motor_id >= STEPPER_MAX_MOTORS || !motors[motor_id].initialized)
    {
        printf("ERROR: Cannot reset motor %d - not valid or not initialized!\n", motor_id);
        return false;
    }

    stepper_motor_t *motor = &motors[motor_id];

    printf("Resetting motor %d (current state: %d, steps_moved: %lu)\n",
           motor_id, motor->state, motor->steps_moved);

    // Stop the motor if it's moving
    if (motor->state == MOTOR_STATE_MOVING)
    {
        stepper_motor_stop(motor_id);
    }

    // Reset all counters and state
    motor->steps_moved = 0;
    motor->steps_to_move = 0;
    motor->state = MOTOR_STATE_IDLE;

    printf("Motor %d reset\n", motor_id);

    return true;
}

/* Private functions implementations ------------------------------------------*/

/**
 * @brief Set direction pin based on desired direction
 *
 * @param motor The motor structure
 * @param direction Direction to set
 */
static void set_direction_pin(stepper_motor_t *motor, stepper_motor_dir_t direction)
{
    if (direction == MOTOR_DIR_CLOCKWISE)
    {
        // Set DIR pin low for clockwise direction (based on TB6600 documentation)
        HAL_GPIO_WritePin(motor->dir_gpio, motor->dir_pin, 1);
    }
    else
    {
        // Set DIR pin high for counterclockwise direction
        HAL_GPIO_WritePin(motor->dir_gpio, motor->dir_pin, 0);
    }
}

/**
 * @brief Configure timer for PWM generation with improved acceleration
 *
 * @param motor The motor structure to configure
 * @param frequency_hz The desired PWM frequency in Hz
 */
static void configure_pwm_timer(stepper_motor_t *motor, uint32_t frequency_hz)
{
    // Start with much higher frequency to reduce vibration
    uint32_t start_frequency = frequency_hz / 2; // Start at 50% of target speed
    if (start_frequency < 6000)
        start_frequency = 6000; // Minimum 6000 Hz (optimal frequency from tests)
    if (start_frequency > 15000)
        start_frequency = 15000; // Maximum 15000 Hz

    // Limit maximum target frequency
    if (frequency_hz > 15000)
        frequency_hz = 15000;

    // If target is close to start frequency, just use target
    if (frequency_hz - start_frequency < 1000)
    {
        start_frequency = frequency_hz;
    }

    // Calculate period for the timer based on start frequency
    uint32_t period_us = 1000000 / start_frequency;

    printf("Configuring timer %d for %lu Hz (starting at %lu Hz, period = %lu us)\n",
           motor->timer_id, frequency_hz, start_frequency, period_us);

    // Stop the timer if it was running
    BSP_TIMER_stop(motor->timer_id);

    // Ensure PUL pin is configured for timer alternate function
    uint32_t gpio_af;
    switch (motor->timer_id)
    {
    case TIMER1_ID:
        gpio_af = GPIO_AF4_TIM1;
        break;
    case TIMER2_ID:
        gpio_af = GPIO_AF1_TIM2;
        break;
    case TIMER3_ID:
        gpio_af = GPIO_AF2_TIM3;
        break;
    case TIMER4_ID:
        gpio_af = GPIO_AF2_TIM4;
        break;
    default:
        gpio_af = GPIO_AF4_TIM1; // Default fallback
        break;
    }

    BSP_GPIO_pin_config(motor->pul_gpio, motor->pul_pin, GPIO_MODE_AF_PP, GPIO_NOPULL,
                        GPIO_SPEED_FREQ_VERY_HIGH, gpio_af);

    // Configure timer in PWM mode
    BSP_TIMER_run_us(motor->timer_id, period_us, true);

    // Use 50% duty cycle for clean step signal
    uint16_t duty_cycle = 500; // 50% of 1000

    // Enable PWM with interrupt generation
    BSP_TIMER_enable_PWM(motor->timer_id, motor->timer_channel, duty_cycle, false, true); // Enable interrupt

    // Store acceleration parameters
    motor->target_speed_hz = frequency_hz;
    motor->current_speed_hz = start_frequency;
    motor->acceleration_steps_per_sec2 = 8000; // Increased acceleration for faster ramp-up
    motor->accel_steps = 0;

    // Calculate when to start deceleration (last 25% of move)
    motor->decel_start_step = (motor->steps_to_move * 3) / 4;

    printf("PWM enabled on timer %d channel %d at %lu Hz (will ramp to %lu Hz)\n",
           motor->timer_id, motor->timer_channel, start_frequency, frequency_hz);
}

/**
 * @brief Configure timer for step counting (not PWM)
 */
static void configure_step_timer(stepper_motor_t *motor, uint32_t frequency_hz)
{
    // Calculate period for the timer based on frequency
    uint32_t period_us = 1000000 / frequency_hz;

    printf("Configuring timer %d for stepping at %lu Hz\n", motor->timer_id, frequency_hz);

    // Stop the timer if it was running
    BSP_TIMER_stop(motor->timer_id);

    // Configure timer for interrupt-based stepping (not PWM)
    BSP_TIMER_run_us(motor->timer_id, period_us, true);

    // Use PWM with very low duty cycle to generate interrupts
    BSP_TIMER_enable_PWM(motor->timer_id, motor->timer_channel, 10, false, false);
}

/**
 * @brief Improved timer interrupt handler for step generation
 */
static void timer_interrupt_handler(timer_id_t timer_id)
{
    // Find which motor is using this timer
    for (uint8_t i = 0; i < STEPPER_MAX_MOTORS; i++)
    {
        stepper_motor_t *motor = &motors[i];

        if (!motor->initialized || motor->state != MOTOR_STATE_MOVING || motor->timer_id != timer_id)
        {
            continue;
        }

        // Increment steps counter first
        motor->steps_moved++;

        // Check if movement is complete
        if (motor->steps_moved >= motor->steps_to_move)
        {
            // Stop this motor immediately
            BSP_TIMER_stop(motor->timer_id);

            // Reconfigure pin as output and ensure it's low
            BSP_GPIO_pin_config(motor->pul_gpio, motor->pul_pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                                GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
            HAL_GPIO_WritePin(motor->pul_gpio, motor->pul_pin, GPIO_PIN_RESET);

            // Set motor state to idle
            motor->state = MOTOR_STATE_IDLE;
            motor->target_speed_hz = 0;
            motor->current_speed_hz = 0;

            printf("Motor %d completed: %lu steps\n", i, motor->steps_moved);
        }

        break; // Only one motor per timer
    }
}

/**
 * @brief Non-blocking motor move function
 */
bool stepper_motor_move_non_blocking(uint8_t motor_id, uint32_t steps,
                                     uint32_t speed_steps_per_second,
                                     stepper_motor_dir_t direction)
{
    if (motor_id >= STEPPER_MAX_MOTORS || !motors[motor_id].initialized)
    {
        return false;
    }

    stepper_motor_t *motor = &motors[motor_id];

    // Stop any current movement
    // Stop any current movement
    if (motor->state == MOTOR_STATE_MOVING)
    {
        BSP_TIMER_stop(motor->timer_id);
    }
    // Configure step pin as regular output (not PWM)
    BSP_GPIO_pin_config(motor->pul_gpio, motor->pul_pin, GPIO_MODE_OUTPUT_PP,
                        GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NO_AF);

    // Set initial pulse state to LOW
    HAL_GPIO_WritePin(motor->pul_gpio, motor->pul_pin, GPIO_PIN_RESET);

    // Setup movement parameters
    motor->steps_to_move = steps;
    motor->steps_moved = 0;
    motor->direction = direction;
    motor->state = MOTOR_STATE_MOVING;

    // Set direction
    set_direction_pin(motor, direction);

    // Configure and start timer for step generation
    configure_step_timer(motor, speed_steps_per_second);

    printf("Started non-blocking movement: Motor %d, %lu steps at %lu Hz\n",
           motor_id, steps, speed_steps_per_second);

    return true;
}

/**
 * @brief Move multiple motors simultaneously
 */
bool stepper_motor_move_synchronized(uint8_t num_motors, uint8_t *motor_ids,
                                     uint32_t *steps, uint32_t *speeds,
                                     stepper_motor_dir_t *directions)
{
    // Start all motors
    for (uint8_t i = 0; i < num_motors; i++)
    {
        if (!stepper_motor_move_non_blocking(motor_ids[i], steps[i], speeds[i], directions[i]))
        {
            printf("Failed to start motor %d\n", motor_ids[i]);
            return false;
        }
    }

    printf("Started synchronized movement of %d motors\n", num_motors);
    return true;
}

/**
 * @brief Check if all specified motors have completed their movements
 */
bool stepper_motors_all_idle(uint8_t num_motors, uint8_t *motor_ids)
{
    for (uint8_t i = 0; i < num_motors; i++)
    {
        if (stepper_motor_get_state(motor_ids[i]) != MOTOR_STATE_IDLE)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief Handler for timer interrupts - must be implemented for each timer
 * These functions are declared as weak in the BSP, so we can override them
 */
void TIMER1_user_handler_it(void)
{
    timer_interrupt_handler(TIMER1_ID);
}

void TIMER2_user_handler_it(void)
{
    timer_interrupt_handler(TIMER2_ID);
}

void TIMER3_user_handler_it(void)
{
    timer_interrupt_handler(TIMER3_ID);
}

void TIMER4_user_handler_it(void)
{
    timer_interrupt_handler(TIMER4_ID);
}

/**
 * @brief Find the maximum reliable speed for the motor by testing
 */
void stepper_motor_find_max_speed(uint8_t motor_id)
{
    uint32_t test_speeds[] = {1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 12000, 15000};
    uint8_t num_speeds = sizeof(test_speeds) / sizeof(test_speeds[0]);
    uint32_t target_steps = 500; // Use fewer steps for this test

    printf("Finding maximum reliable speed...\n");

    for (uint8_t i = 0; i < num_speeds; i++)
    {
        uint32_t speed = test_speeds[i];
        printf("Testing speed: %lu Hz for %lu steps\n", speed, target_steps);

        // Reset motor and make sure it's stopped
        stepper_motor_reset(motor_id);
        HAL_Delay(500);

        // Mark the starting position
        uint32_t start_pos = HAL_GetTick(); // Just using this as a marker
        printf("Starting position mark: %lu\n", start_pos);

        // Move exactly 500 steps at current test speed
        stepper_motor_move(motor_id, target_steps, speed, MOTOR_DIR_CLOCKWISE);

        // Wait until motor stops (with timeout)
        uint32_t timeout = (speed > 0) ? (3000 + (target_steps * 1000 / speed)) : 5000;
        uint32_t start_wait = HAL_GetTick();

        while (stepper_motor_get_state(motor_id) != MOTOR_STATE_IDLE)
        {
            stepper_motor_update();

            if (HAL_GetTick() - start_wait > timeout)
            {
                printf("Timeout waiting for motor to stop! Forcing stop.\n");
                stepper_motor_stop(motor_id);
                break;
            }

            HAL_Delay(10);
        }

        // Verify steps moved and position
        uint32_t steps_moved = motors[motor_id].steps_moved;
        uint32_t end_pos = HAL_GetTick(); // Just as a marker

        printf("Completed %lu Hz test: moved %lu/%lu steps\n", speed, steps_moved, target_steps);

        // Wait between tests
        HAL_Delay(1000);

        // Move back to starting position at a reliable speed (1000 Hz)
        printf("Returning to starting position...\n");
        stepper_motor_move(motor_id, target_steps, 1000, MOTOR_DIR_COUNTERCLOCKWISE);

        // Wait until motor stops
        while (stepper_motor_get_state(motor_id) != MOTOR_STATE_IDLE)
        {
            stepper_motor_update();
            HAL_Delay(10);
        }

        HAL_Delay(1000); // Pause between tests
    }

    printf("Speed test complete. Check which speeds maintained accurate positioning.\n");
}

/**
 * @brief Simple speed test with constant speed (no acceleration)
 */
void stepper_motor_simple_speed_test(uint8_t motor_id)
{
    // Test just a few speeds with constant movement (no acceleration)
    uint32_t test_speeds[] = {1000, 2000, 3000};
    uint8_t num_speeds = sizeof(test_speeds) / sizeof(test_speeds[0]);

    printf("Starting simple speed test sequence...\n");

    for (uint8_t i = 0; i < num_speeds; i++)
    {
        uint32_t speed = test_speeds[i];
        printf("Testing speed: %lu Hz for 500 steps\n", speed);

        // Reset motor
        stepper_motor_reset(motor_id);
        HAL_Delay(500);

        // Move at constant speed (no acceleration)
        stepper_motor_move(motor_id, 500, speed, MOTOR_DIR_CLOCKWISE);

        // Wait for completion (with timeout)
        uint32_t start_time = HAL_GetTick();
        uint32_t timeout = 3000 + (500 * 1000 / speed);

        while (stepper_motor_get_state(motor_id) != MOTOR_STATE_IDLE)
        {
            stepper_motor_update();

            if (HAL_GetTick() - start_time > timeout)
            {
                printf("Timeout waiting for motor to stop. Forcing stop.\n");
                stepper_motor_stop(motor_id);
                break;
            }

            HAL_Delay(10);
        }

        printf("Completed %lu Hz test. Waiting 2 seconds...\n", speed);
        HAL_Delay(2000);
    }

    printf("Simple speed test complete!\n");
}

/**
 * @brief Check if motor can move in specified direction without hitting limits
 */
static bool check_motor_limits_before_move(uint8_t motor_id, stepper_motor_dir_t direction)
{
    stepper_motor_t *motor = &motors[motor_id];

    if (!motor->limit_check_enabled || motor->associated_axis == 255)
    {
        return true; // No limits to check
    }

    axis_t axis = (axis_t)motor->associated_axis;

    // Check appropriate limit based on direction
    if (direction == MOTOR_DIR_COUNTERCLOCKWISE)
    {
        // Moving towards minimum - check MIN limit
        if (limit_switch_read(axis, LIMIT_MIN) == LIMIT_TRIGGERED)
        {
            printf("Cannot move motor %d: MIN limit already triggered\n", motor_id);
            return false;
        }
    }
    else
    {
        // Moving towards maximum - check MAX limit
        if (limit_switch_read(axis, LIMIT_MAX) == LIMIT_TRIGGERED)
        {
            printf("Cannot move motor %d: MAX limit already triggered\n", motor_id);
            return false;
        }
    }

    return true;
}

/**
 * @brief Set motor acceleration parameters
 */
bool stepper_motor_set_acceleration(uint8_t motor_id, uint32_t acceleration_steps_per_sec2)
{
    if (motor_id >= STEPPER_MAX_MOTORS || !motors[motor_id].initialized)
    {
        return false;
    }

    motors[motor_id].acceleration_steps_per_sec2 = acceleration_steps_per_sec2;
    printf("Motor %d acceleration set to %lu steps/sec²\n", motor_id, acceleration_steps_per_sec2);

    return true;
}

/**
 * @brief High frequency speed test to find optimal range
 */
void stepper_motor_high_frequency_test(uint8_t motor_id)
{
    // Test higher frequencies to reduce vibration
    uint32_t test_speeds[] = {2000, 2500, 3000, 4000, 5000, 6000, 7000, 8000};
    uint8_t num_speeds = sizeof(test_speeds) / sizeof(test_speeds[0]);

    printf("Starting high frequency test sequence...\n");

    for (uint8_t i = 0; i < num_speeds; i++)
    {
        uint32_t speed = test_speeds[i];
        printf("Testing speed: %lu Hz for 200 steps\n", speed);

        // Reset motor
        stepper_motor_reset(motor_id);
        HAL_Delay(500);

        // Move at constant high speed
        stepper_motor_move(motor_id, 200, speed, MOTOR_DIR_CLOCKWISE);

        // Wait for completion
        uint32_t start_time = HAL_GetTick();
        uint32_t timeout = 3000;

        while (stepper_motor_get_state(motor_id) != MOTOR_STATE_IDLE)
        {
            stepper_motor_update();

            if (HAL_GetTick() - start_time > timeout)
            {
                printf("Timeout - forcing stop\n");
                stepper_motor_stop(motor_id);
                break;
            }

            HAL_Delay(10);
        }

        printf("Completed %lu Hz test - check vibration and accuracy\n", speed);
        HAL_Delay(1500); // Brief pause between tests
    }

    printf("High frequency test complete!\n");
}

/**
 * @brief Emergency stop all motors
 */
void stepper_motor_emergency_stop_all(void)
{
    printf("EMERGENCY STOP - Stopping all motors immediately\n");

    for (uint8_t i = 0; i < STEPPER_MAX_MOTORS; i++)
    {
        if (motors[i].initialized)
        {
            // Force stop the timer
            BSP_TIMER_stop(motors[i].timer_id);

            // Reconfigure GPIO as output and set low
            BSP_GPIO_pin_config(motors[i].pul_gpio, motors[i].pul_pin, GPIO_MODE_OUTPUT_PP,
                                GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
            HAL_GPIO_WritePin(motors[i].pul_gpio, motors[i].pul_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motors[i].dir_gpio, motors[i].dir_pin, GPIO_PIN_RESET);

            // Update motor state
            motors[i].state = MOTOR_STATE_IDLE;
            motors[i].target_speed_hz = 0;
            motors[i].current_speed_hz = 0;
        }
    }

    printf("All motors stopped\n");
}