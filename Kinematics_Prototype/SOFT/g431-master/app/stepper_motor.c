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

/* Private variables ---------------------------------------------------------*/
static stepper_motor_t motors[STEPPER_MAX_MOTORS];
static uint8_t motor_count = 0;

/* Private function declarations ---------------------------------------------*/
static void configure_pwm_timer(stepper_motor_t *motor, uint32_t frequency_hz);
static void timer_interrupt_handler(timer_id_t timer_id);
static void set_direction_pin(stepper_motor_t *motor, stepper_motor_dir_t direction);

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
    BSP_GPIO_pin_config(pul_gpio, pul_pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF4_TIM1);

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

    // Check if the motor is already moving
    if (motor->state != MOTOR_STATE_IDLE)
    {
        return false;
    }

    // Calculate step delay in microseconds from speed
    if (speed_steps_per_second == 0)
    {
        return false; // Can't have zero speed
    }

    // Set direction
    set_direction_pin(motor, direction);
    motor->direction = direction;

    motor->step_delay_us = 1000000 / speed_steps_per_second;
    motor->steps_to_move = steps;
    motor->steps_moved = 0;

    // Configure and start the PWM timer with calculated frequency
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

    // Update motor state
    motor->state = MOTOR_STATE_IDLE;
    motor->steps_moved = 0;
    motor->steps_to_move = 0;

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
 * @brief High-speed test for the motor
 */
void stepper_motor_high_speed_test(uint8_t motor_id, uint32_t steps)
{
    if (motor_id >= STEPPER_MAX_MOTORS || !motors[motor_id].initialized)
    {
        return;
    }

    stepper_motor_t *motor = &motors[motor_id];

    printf("High-speed testing motor %d for %lu steps\n", motor_id, steps);

    // Temporarily reconfigure the pin as a standard output
    BSP_GPIO_pin_config(motor->pul_gpio, motor->pul_pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                        GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);

    // Set direction
    set_direction_pin(motor, MOTOR_DIR_CLOCKWISE);

    // Much faster pulses - 0.5ms HIGH, 0.5ms LOW = 1000 steps per second
    for (uint32_t i = 0; i < steps; i++)
    {
        HAL_GPIO_WritePin(motor->pul_gpio, motor->pul_pin, 1);
        HAL_Delay(1); // 0.5ms delay (1000Hz)

        HAL_GPIO_WritePin(motor->pul_gpio, motor->pul_pin, 0);
        HAL_Delay(1); // 0.5ms delay

        if (i % 100 == 0)
        {
            printf("Step %lu\n", i);
        }
    }

    printf("High-speed test complete\n");

    // Reconfigure for timer function
    BSP_GPIO_pin_config(motor->pul_gpio, motor->pul_pin, GPIO_MODE_AF_PP, GPIO_NOPULL,
                        GPIO_SPEED_FREQ_HIGH, GPIO_AF4_TIM1);
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

    printf("Configuring timer %d for %lu Hz (period = %lu us)\n",
           motor->timer_id, frequency_hz, period_us);

    // Stop the timer if it was running
    BSP_TIMER_stop(motor->timer_id);

    // Configure timer in PWM mode
    // First run the timer to set it up with the correct period
    BSP_TIMER_run_us(motor->timer_id, period_us, true);

    // Now configure the PWM output on the appropriate channel
    // Use 50% duty cycle (500 out of 1000) for maximum reliability
    BSP_TIMER_enable_PWM(motor->timer_id, motor->timer_channel, 500, false, false);

    printf("PWM enabled on timer %d channel %d\n", motor->timer_id, motor->timer_channel);
}

/**
 * @brief Timer interrupt handler
 *
 * @param timer_id Timer ID that triggered the interrupt
 */
static void timer_interrupt_handler(timer_id_t timer_id)
{
    static uint32_t counter = 0;

    // Print status every 100 interrupts to avoid flooding
    if (counter++ % 100 == 0)
    {
        printf("Timer %d interrupt (#%lu)\n", timer_id, counter);
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