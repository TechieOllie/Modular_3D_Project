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
    set_direction_pin(motor, MOTOR_DIR_CLOCKWISE); // Set the motor as initialized
    motor->initialized = true;
    motor_count++;

    printf("Motor %d initialized: PUL=%s Pin_%d, DIR=%s Pin_%d, Timer=%d, Channel=%d\n",
           motor_id,
           (pul_gpio == GPIOA) ? "GPIOA" : (pul_gpio == GPIOB) ? "GPIOB"
                                                               : "GPIOC",
           pul_pin,
           (dir_gpio == GPIOA) ? "GPIOA" : (dir_gpio == GPIOB) ? "GPIOB"
                                                               : "GPIOC",
           dir_pin,
           timer_id, timer_channel);

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
 * @brief Configure timer for PWM generation
 *
 * @param motor The motor structure to configure
 * @param frequency_hz The desired PWM frequency in Hz
 */
static void configure_pwm_timer(stepper_motor_t *motor, uint32_t frequency_hz)
{
    // Calculate period for the timer based on frequency
    uint32_t period_us = 1000000 / frequency_hz;

    printf("Configuring timer %d for %lu Hz (period = %lu us)\n",
           motor->timer_id, frequency_hz, period_us);

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
    static uint32_t counter[4] = {0};   // Separate counter for each timer
    static uint32_t last_tick[4] = {0}; // Separate last tick for each timer
    uint32_t current_tick = HAL_GetTick();

    counter++; // Debug output at reasonable intervals
    if (current_tick - last_tick >= 1000)
    {
        last_tick = current_tick;
        // Find which motor is currently moving for debug output
        for (uint8_t j = 0; j < STEPPER_MAX_MOTORS; j++)
        {
            if (motors[j].initialized && motors[j].state == MOTOR_STATE_MOVING && motors[j].timer_id == timer_id)
            {
                printf("Timer %d: counter=%lu, steps moved for motor %d: %lu\n",
                       timer_id, counter, j, motors[j].steps_moved);
                break;
            }
        }
    }

    // Find which motor is using this timer
    for (uint8_t i = 0; i < STEPPER_MAX_MOTORS; i++)
    {
        stepper_motor_t *motor = &motors[i];

        if (!motor->initialized || motor->state != MOTOR_STATE_MOVING)
        {
            continue;
        }

        if (motor->timer_id == timer_id)
        {
            // Increment steps moved counter
            motor->steps_moved++;

            // If we've reached the target, stop the motor
            if (motor->steps_moved >= motor->steps_to_move)
            {
                printf("Target reached: %lu/%lu steps\n", motor->steps_moved, motor->steps_to_move);
                stepper_motor_stop(i);
            }
            break;
        }
    }
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