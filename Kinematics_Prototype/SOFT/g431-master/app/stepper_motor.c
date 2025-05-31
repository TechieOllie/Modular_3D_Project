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

    // Configure PUL GPIO pin
    // For TIM1_CH2 on PA9
    BSP_GPIO_pin_config(pul_gpio, pul_pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF4_TIM1);

    // Configure DIR GPIO pin
    BSP_GPIO_pin_config(dir_gpio, dir_pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);

    // Initialize DIR pin to clockwise direction (low)
    set_direction_pin(motor, MOTOR_DIR_CLOCKWISE);

    // Set the motor as initialized
    motor->initialized = true;
    motor_count++;

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
        return false;
    }

    stepper_motor_t *motor = &motors[motor_id];

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
 * @brief Set motor direction
 */
bool stepper_motor_set_direction(uint8_t motor_id, stepper_motor_dir_t direction)
{
    // Check if motor_id is valid
    if (motor_id >= STEPPER_MAX_MOTORS || !motors[motor_id].initialized)
    {
        return false;
    }

    stepper_motor_t *motor = &motors[motor_id];

    // Set the direction pin
    set_direction_pin(motor, direction);
    motor->direction = direction;

    return true;
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

    // Stop the timer
    BSP_TIMER_stop(motor->timer_id);

    // Reconfigure pin as output and ensure it's low
    BSP_GPIO_pin_config(motor->pul_gpio, motor->pul_pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                        GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
    HAL_GPIO_WritePin(motor->pul_gpio, motor->pul_pin, GPIO_PIN_RESET);

    // Print final step count
    printf("Motor %d stopped: moved %lu of %lu steps\n",
           motor_id, motor->steps_moved, motor->steps_to_move);

    // Update motor state and reset counters
    motor->state = MOTOR_STATE_IDLE;

    // Don't reset steps_moved here, so we can see the final count
    // motor->steps_moved = 0;
    // motor->steps_to_move = 0;

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

    // Reconfigure the pin for timer functionality afterward
    BSP_GPIO_pin_config(motor->pul_gpio, motor->pul_pin, GPIO_MODE_AF_PP, GPIO_NOPULL,
                        GPIO_SPEED_FREQ_HIGH, GPIO_AF4_TIM1);
}

/**
 * @brief Test routine for stepping motor at various speeds
 */
void stepper_motor_speed_test(uint8_t motor_id)
{
    // Use fewer high speeds
    uint32_t test_speeds[] = {1000, 2000, 3000, 5000, 8000}; // Removed 10000 Hz
    uint8_t num_speeds = sizeof(test_speeds) / sizeof(test_speeds[0]);

    printf("Starting speed test sequence with acceleration...\n");

    for (uint8_t i = 0; i < num_speeds; i++)
    {
        uint32_t max_speed = test_speeds[i];
        uint32_t start_speed = 500;

        // Use more steps for accel/decel at higher speeds
        uint32_t accel_steps = max_speed / 5; // 20% of max speed as steps
        if (accel_steps < 100)
            accel_steps = 100;
        if (accel_steps > 300)
            accel_steps = 300;

        // Adjust total steps based on max speed
        uint32_t total_steps = 600; // Base steps
        if (max_speed > 3000)
            total_steps = 800;
        if (max_speed > 6000)
            total_steps = 1000;

        printf("Testing speed: %lu Hz (accel: %lu steps, total: %lu steps)\n",
               max_speed, accel_steps, total_steps);

        // Reset motor state before starting new test
        stepper_motor_reset(motor_id);

        // Move with acceleration parameters
        stepper_motor_move_with_accel(motor_id, total_steps, start_speed, max_speed,
                                      accel_steps, MOTOR_DIR_CLOCKWISE);

        // Wait until motor stops, with timeout
        uint32_t start_time = HAL_GetTick();
        // Add extra time for acceleration phases
        uint32_t timeout = 5000 + (2000 * 1000 / start_speed); // More time for slower speeds

        while (stepper_motor_get_state(motor_id) != MOTOR_STATE_IDLE)
        {
            stepper_motor_update();

            // Print progress less frequently
            static uint32_t last_print = 0;
            uint32_t now = HAL_GetTick();
            if (now - last_print >= 500) // Every 500ms instead of 200ms
            {
                last_print = now;
                printf("Progress: %lu/%lu steps (Speed: %lu Hz, Phase: %d)\n",
                       motors[motor_id].steps_moved,
                       motors[motor_id].steps_to_move,
                       motors[motor_id].accel_state.current_speed,
                       motors[motor_id].accel_state.phase);
            }

            // Check for timeout
            if (HAL_GetTick() - start_time > timeout)
            {
                printf("Timeout waiting for motor to stop! Forcing stop.\n");
                stepper_motor_stop(motor_id);
                break;
            }

            HAL_Delay(10);
        }

        // Wait longer between tests
        printf("Completed %lu Hz test. Waiting 2 seconds...\n", max_speed);
        HAL_Delay(2000);
    }
}

/**
 * @brief Reset a stepper motor's position and state
 */
bool stepper_motor_reset(uint8_t motor_id)
{
    // Check if motor_id is valid
    if (motor_id >= STEPPER_MAX_MOTORS || !motors[motor_id].initialized)
    {
        return false;
    }

    stepper_motor_t *motor = &motors[motor_id];

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
        HAL_GPIO_WritePin(motor->dir_gpio, motor->dir_pin, 0);
    }
    else
    {
        // Set DIR pin high for counterclockwise direction
        HAL_GPIO_WritePin(motor->dir_gpio, motor->dir_pin, 1);
    }
}

/**
 * @brief Configure timer for PWM generation
 *
 * @param motor The motor structure to configure
 * @param frequency_hz The desired PWM frequency in Hz
 */
static void configure_pwm_timer(stepper_motor_t *motor, uint32_t frequency_hz)
{
    // Calculate period for the timer based on frequency
    uint32_t period_us = 1000000 / frequency_hz;
    // Only print debug for significant frequency changes to avoid flooding
    static uint32_t last_printed_freq = 0;
    if (labs((int32_t)frequency_hz - (int32_t)last_printed_freq) > 500 ||
        frequency_hz == motor->accel_state.start_speed ||
        frequency_hz == motor->accel_state.max_speed)
    {
        printf("Configuring timer %d for %lu Hz (period = %lu us)\n",
               motor->timer_id, frequency_hz, period_us);
        last_printed_freq = frequency_hz;
    }

    // Stop the timer if it was running
    BSP_TIMER_stop(motor->timer_id);

    // Configure timer in PWM mode
    // First run the timer to set it up with the correct period
    BSP_TIMER_run_us(motor->timer_id, period_us, true);

    // For high frequencies, you might need a different duty cycle
    // 50% duty cycle is usually optimal
    uint16_t duty_cycle = 500; // 50% of 1000

    BSP_TIMER_enable_PWM(motor->timer_id, motor->timer_channel, duty_cycle, false, false);

    printf("PWM enabled on timer %d channel %d at %lu Hz\n",
           motor->timer_id, motor->timer_channel, frequency_hz);
}

/**
 * @brief Timer interrupt handler
 *
 * @param timer_id Timer ID that triggered the interrupt
 */
static void timer_interrupt_handler(timer_id_t timer_id)
{
    static uint32_t counter = 0;
    static uint32_t last_tick = 0;
    static uint32_t interrupt_rate_counter = 0;
    static uint32_t last_rate_check = 0;
    uint32_t current_tick = HAL_GetTick();

    counter++;
    interrupt_rate_counter++;

    // Instead of checking consecutive interrupts, check interrupt rate over time
    if (current_tick != last_rate_check)
    {
        // We've moved to a new millisecond
        if (interrupt_rate_counter > 1000)
        { // More than 1000 interrupts per ms is too high
            printf("WARNING: High interrupt rate detected (%lu/ms)! Reducing motor speed.\n",
                   interrupt_rate_counter);

            // Instead of emergency stop, gradually reduce speed
            for (uint8_t j = 0; j < STEPPER_MAX_MOTORS; j++)
            {
                if (motors[j].initialized && motors[j].state == MOTOR_STATE_MOVING)
                {
                    // Reduce to 25% of current speed but not below 1000Hz
                    uint32_t current_speed = motors[j].accel_state.current_speed;
                    uint32_t reduced_speed = current_speed / 4;
                    if (reduced_speed < 1000)
                        reduced_speed = 1000;

                    printf("Reducing motor %d speed from %lu to %lu Hz\n",
                           j, current_speed, reduced_speed);

                    configure_pwm_timer(&motors[j], reduced_speed);
                    motors[j].accel_state.current_speed = reduced_speed;
                    motors[j].accel_state.max_speed = reduced_speed; // Prevent re-acceleration
                }
            }
        }

        // Reset counter for next millisecond
        interrupt_rate_counter = 0;
        last_rate_check = current_tick;
    }

    // Debug output at reasonable intervals
    if (current_tick - last_tick >= 1000)
    {
        last_tick = current_tick;
        printf("Timer %d: counter=%lu, steps moved for motor 0: %lu\n",
               timer_id, counter, motors[0].steps_moved);
    }
    // Find which motor is using this timer
    for (uint8_t i = 0; i < STEPPER_MAX_MOTORS; i++)
    {
        stepper_motor_t *motor = &motors[i];

        // Make sure we only process initialized motors that are actually moving
        if (!motor->initialized || motor->state != MOTOR_STATE_MOVING)
        {
            continue;
        }

        if (motor->timer_id == timer_id)
        {
            // Increment steps moved counter
            motor->steps_moved++;

            // Handle acceleration/deceleration
            if (motor->accel_state.phase == ACCEL_PHASE_ACCELERATING)
            {
                if (motor->steps_moved < motor->accel_state.accel_steps)
                {
                    // Only update speed every N steps to avoid overwhelming the system
                    if (motor->steps_moved % motor->accel_state.speed_update_interval == 0)
                    {
                        // Acceleration phase - gradually increase speed
                        uint32_t new_speed = motor->accel_state.start_speed +
                                             ((motor->accel_state.max_speed - motor->accel_state.start_speed) *
                                              motor->steps_moved / motor->accel_state.accel_steps);

                        // Limit speed changes per update
                        if (new_speed - motor->accel_state.current_speed > 500)
                        {
                            new_speed = motor->accel_state.current_speed + 500; // Max 500Hz change per update
                        }

                        if (new_speed > motor->accel_state.current_speed)
                        {
                            motor->accel_state.current_speed = new_speed;
                            configure_pwm_timer(motor, new_speed);
                        }
                    }
                }
                else
                {
                    // We've reached max speed zone
                    motor->accel_state.phase = ACCEL_PHASE_CONSTANT;
                    // Ensure we're at target speed
                    configure_pwm_timer(motor, motor->accel_state.max_speed);
                    motor->accel_state.current_speed = motor->accel_state.max_speed;
                }
            }
            else if (motor->accel_state.phase == ACCEL_PHASE_CONSTANT)
            {
                // Check if we need to start decelerating
                if (motor->steps_moved >= motor->steps_to_move - motor->accel_state.decel_steps)
                {
                    motor->accel_state.phase = ACCEL_PHASE_DECELERATING;
                }
            }
            else if (motor->accel_state.phase == ACCEL_PHASE_DECELERATING)
            {
                // Only update speed every N steps to avoid overwhelming the system
                if (motor->steps_moved % motor->accel_state.speed_update_interval == 0)
                {
                    // Use larger interval during deceleration at high speeds
                    if (motor->accel_state.current_speed > 2000)
                    {
                        if (motor->steps_moved % (motor->accel_state.speed_update_interval * 5) != 0)
                        {
                            continue; // Skip more updates at higher speeds
                        }
                    }

                    // Deceleration phase
                    uint32_t steps_left = motor->steps_to_move - motor->steps_moved;
                    uint32_t new_speed = motor->accel_state.start_speed +
                                         ((motor->accel_state.max_speed - motor->accel_state.start_speed) *
                                          steps_left / motor->accel_state.decel_steps);

                    // Ensure we don't go below start speed
                    if (new_speed < motor->accel_state.start_speed)
                        new_speed = motor->accel_state.start_speed;

                    // Limit speed changes per update
                    if (motor->accel_state.current_speed - new_speed > 500)
                    {
                        new_speed = motor->accel_state.current_speed - 500; // Max 500Hz change per update
                    }

                    if (new_speed < motor->accel_state.current_speed)
                    {
                        motor->accel_state.current_speed = new_speed;
                        configure_pwm_timer(motor, new_speed);
                    }
                }
            }

            // If we've reached the target, stop the motor
            if (motor->steps_moved >= motor->steps_to_move)
            {
                printf("Target reached: %lu/%lu steps\n", motor->steps_moved, motor->steps_to_move);
                stepper_motor_stop(i);
            }

            break;
        }
    }

    // Check if handler is taking too long (detect potential crash)
    static uint32_t last_handler_time = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_handler_time < 10)
    { // Handler called too frequently
        // Emergency stop if updates happening too fast
        for (uint8_t j = 0; j < STEPPER_MAX_MOTORS; j++)
        {
            if (motors[j].initialized && motors[j].state == MOTOR_STATE_MOVING)
            {
                printf("Emergency stop - handler overload!\n");
                BSP_TIMER_stop(motors[j].timer_id);
                motors[j].state = MOTOR_STATE_IDLE;
            }
        }
    }
    last_handler_time = now;
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
 * @brief Start motor movement with acceleration control
 */
bool stepper_motor_move_with_accel(uint8_t motor_id, uint32_t steps,
                                   uint32_t start_speed, uint32_t max_speed,
                                   uint32_t accel_steps, stepper_motor_dir_t direction)
{
    // Validate parameters
    if (motor_id >= STEPPER_MAX_MOTORS || !motors[motor_id].initialized)
    {
        return false;
    }

    stepper_motor_t *motor = &motors[motor_id];

    // Explicitly stop any previous movement
    if (motor->state != MOTOR_STATE_IDLE)
    {
        stepper_motor_stop(motor_id);
    }

    // Limit maximum speed to prevent crashes
    if (max_speed > 15000)
    {
        max_speed = 15000; // Cap at 15kHz to avoid issues
    }

    // Ensure we have enough steps for proper acceleration
    if (steps < 2 * accel_steps)
    {
        // If total steps is less than acceleration + deceleration, reduce accel steps
        accel_steps = steps / 3; // Use 1/3 for accel, 1/3 for steady, 1/3 for decel
        if (accel_steps < 10)
            accel_steps = 10; // Minimum accel steps
    }

    // Setup initial parameters
    motor->steps_moved = 0;
    motor->steps_to_move = steps;

    // Set direction
    set_direction_pin(motor, direction);
    motor->direction = direction;

    // Start at the lower speed
    uint32_t current_speed = start_speed;
    configure_pwm_timer(motor, current_speed);

    // Set speed update interval based on max speed - use even larger values
    uint32_t update_interval = 20; // Start higher
    if (max_speed > 3000)
        update_interval = 40;
    if (max_speed > 6000)
        update_interval = 80;
    if (max_speed > 9000)
        update_interval = 120;

    // Track acceleration state
    motor->accel_state.start_speed = start_speed;
    motor->accel_state.max_speed = max_speed;
    motor->accel_state.accel_steps = accel_steps;
    motor->accel_state.decel_steps = accel_steps;
    motor->accel_state.current_speed = start_speed;
    motor->accel_state.phase = ACCEL_PHASE_ACCELERATING;
    motor->accel_state.speed_update_interval = update_interval;

    printf("Starting accel move: %lu steps, %lu to %lu Hz, accel=%lu steps\n",
           steps, start_speed, max_speed, accel_steps);

    // Set state to moving after all parameters are set
    motor->state = MOTOR_STATE_MOVING;

    return true;
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