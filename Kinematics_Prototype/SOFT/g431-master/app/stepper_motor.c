/**
 *******************************************************************************
 * @file    stm32g4_stepper_motor.c
 * @author  Ol, naej, Fabs (adapted from julia's original)
 * @date    May 27, 2025
 * @brief   Stepper motor control with acceleration profiles for STM32G4
 *******************************************************************************
 */

#include "config.h"
#include "stm32g4_utils.h"
#include "stm32g4xx.h"
#include "stm32g4_timer.h"
#include "stepper_motor.h"
#include "stm32g4_sys.h"
#include "stm32g4_gpio.h"
#include "limit_switches.h"
#include <stdio.h>
#include <math.h>

#if USE_STEPPER_MOTOR

// Timer configuration
#ifndef STEPPER_MOTOR_TIMER
#define STEPPER_MOTOR_TIMER TIMER2_ID
#define STEPPER_MOTOR_timer_irq_handler TIMER2_user_handler_it
#endif

// Driver configuration (using MOSFETs for TB6600 drivers)
#define SLEEPING_STATE 0 // Logic level for inactive state
#define ACTIVE_STATE 1   // Logic level for active state

// Private variables
static stepper_motor_config_t motor_configs[STEPPER_MOTOR_NB];
static stepper_motor_data_t motor_data[STEPPER_MOTOR_NB];
static callback_fun_t callback_at_each_pulse = NULL;
static bool system_initialized = false;

// Private function prototypes
static void STEPPER_MOTOR_pin_set(GPIO_TypeDef *gpio, uint32_t pin, bool state);
static void STEPPER_MOTORS_do_pulse(motor_id_e id);
static void STEPPER_MOTORS_set_dir(motor_id_e id, stepper_motor_dir_t direction);
static void STEPPER_MOTOR_update_acceleration(motor_id_e id);
static uint32_t STEPPER_MOTOR_frequency_to_period(uint32_t frequency);
static bool STEPPER_MOTOR_check_limits(motor_id_e id, stepper_motor_dir_t direction);

// Public function implementations

bool STEPPER_MOTOR_system_init(void)
{
    printf("Initializing stepper motor system...\n");

    // Initialize all motor data structures
    for (motor_id_e m = 0; m < STEPPER_MOTOR_NB; m++)
    {
        // Clear configuration
        motor_configs[m].pulse_gpio = NULL;
        motor_configs[m].pulse_pin = 0;
        motor_configs[m].dir_gpio = NULL;
        motor_configs[m].dir_pin = 0;
        motor_configs[m].enable_gpio = NULL;
        motor_configs[m].enable_pin = 0;
        motor_configs[m].configured = false;

        // Initialize motor data
        motor_data[m].position = 0;
        motor_data[m].goal = 0;
        motor_data[m].steps_to_go = 0;
        motor_data[m].completed_steps = 0;
        motor_data[m].state = MOTOR_STATE_IDLE;
        motor_data[m].direction = MOTOR_DIR_CLOCKWISE;
        motor_data[m].pulse_counter = 0;
        motor_data[m].pulse_period = STEPPER_MOTOR_frequency_to_period(1000); // Default 1kHz
        motor_data[m].enabled = false;
        motor_data[m].microsteps = 16;
        motor_data[m].axis = (axis_t)m; // Default mapping
        motor_data[m].limit_check_enabled = false;

        // Initialize acceleration profile
        motor_data[m].accel.acceleration = 5000;  // Default 5000 steps/sec²
        motor_data[m].accel.max_frequency = 8000; // Default 8kHz max
        motor_data[m].accel.current_frequency = STEPPER_MIN_FREQUENCY;
        motor_data[m].accel.target_frequency = 1000;
        motor_data[m].accel.acceleration_enabled = false;
    }

    // Start base timer for stepper control
    BSP_TIMER_run_us(STEPPER_MOTOR_TIMER, DEFAULT_IT_PERIOD, true);

    system_initialized = true;
    printf("Stepper motor system initialized with %dus base timer\n", DEFAULT_IT_PERIOD);

    return true;
}

bool STEPPER_MOTOR_configure(motor_id_e id, GPIO_TypeDef *pulse_gpio, uint32_t pulse_pin,
                             GPIO_TypeDef *dir_gpio, uint32_t dir_pin,
                             GPIO_TypeDef *enable_gpio, uint32_t enable_pin,
                             uint8_t microsteps)
{
    if (id >= STEPPER_MOTOR_NB || !system_initialized)
    {
        printf("ERROR: Invalid motor ID %d or system not initialized\n", id);
        return false;
    }

    printf("Configuring motor %d: PUL=P%c%lu, DIR=P%c%lu, microsteps=%d\n",
           id,
           (pulse_gpio == GPIOA) ? 'A' : (pulse_gpio == GPIOB) ? 'B'
                                                               : 'X',
           pulse_pin,
           (dir_gpio == GPIOA) ? 'A' : (dir_gpio == GPIOB) ? 'B'
                                                           : 'X',
           dir_pin,
           microsteps);

    // Store configuration
    motor_configs[id].pulse_gpio = pulse_gpio;
    motor_configs[id].pulse_pin = pulse_pin;
    motor_configs[id].dir_gpio = dir_gpio;
    motor_configs[id].dir_pin = dir_pin;
    motor_configs[id].enable_gpio = enable_gpio;
    motor_configs[id].enable_pin = enable_pin;
    motor_configs[id].configured = true;

    motor_data[id].microsteps = microsteps;

    // Configure GPIO pins
    uint32_t gpio_pin_pulse = 1 << pulse_pin;
    uint32_t gpio_pin_dir = 1 << dir_pin;

    BSP_GPIO_pin_config(pulse_gpio, gpio_pin_pulse, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
    BSP_GPIO_pin_config(dir_gpio, gpio_pin_dir, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);

    if (enable_gpio != NULL)
    {
        uint32_t gpio_pin_enable = 1 << enable_pin;
        BSP_GPIO_pin_config(enable_gpio, gpio_pin_enable, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
        STEPPER_MOTOR_pin_set(enable_gpio, enable_pin, SLEEPING_STATE);
    }

    // Set initial states
    STEPPER_MOTOR_pin_set(pulse_gpio, pulse_pin, SLEEPING_STATE);
    STEPPER_MOTOR_pin_set(dir_gpio, dir_pin, SLEEPING_STATE);

    printf("Motor %d configured successfully\n", id);
    return true;
}

void STEPPER_MOTOR_init(void)
{
    if (!system_initialized)
    {
        STEPPER_MOTOR_system_init();
    }

    // Legacy initialization - configure default motors
    // Motor 0: PB14 (DIR), PA15 (PUL)
    STEPPER_MOTOR_configure(STEPPER_MOTOR_0, GPIOA, 15, GPIOB, 14, NULL, 0, 16);

    // Motor 1: PA12 (DIR), PA13 (PUL)
    STEPPER_MOTOR_configure(STEPPER_MOTOR_1, GPIOA, 13, GPIOA, 12, NULL, 0, 16);

    // Enable both motors
    STEPPER_MOTOR_enable(STEPPER_MOTOR_0, true);
    STEPPER_MOTOR_enable(STEPPER_MOTOR_1, true);
}

void STEPPER_MOTOR_demo(void)
{
    STEPPER_MOTOR_init();

    bool toggle0 = false;
    bool toggle1 = false;

    while (1)
    {
        if (STEPPER_MOTOR_is_arrived(STEPPER_MOTOR_0))
        {
            STEPPER_MOTOR_set_goal(STEPPER_MOTOR_0, (toggle0) ? 1000 : -1000);
            toggle0 = !toggle0;
        }

        if (STEPPER_MOTOR_is_arrived(STEPPER_MOTOR_1))
        {
            STEPPER_MOTOR_set_goal(STEPPER_MOTOR_1, (toggle1) ? 900 : -900);
            toggle1 = !toggle1;
        }

        HAL_Delay(100);
    }
}

void STEPPER_MOTOR_set_callback_at_each_pulse(callback_fun_t cb)
{
    callback_at_each_pulse = cb;
}

void STEPPER_MOTOR_enable(motor_id_e id, bool enable)
{
    if (id >= STEPPER_MOTOR_NB || !motor_configs[id].configured)
        return;

    motor_data[id].enabled = enable;

    if (motor_configs[id].enable_gpio != NULL)
    {
        STEPPER_MOTOR_pin_set(motor_configs[id].enable_gpio, motor_configs[id].enable_pin,
                              enable ? ACTIVE_STATE : SLEEPING_STATE);
    }

    printf("Motor %d %s\n", id, enable ? "enabled" : "disabled");
}

void STEPPER_MOTOR_set_goal(motor_id_e id, int32_t newgoal)
{
    if (id >= STEPPER_MOTOR_NB)
        return;

    motor_data[id].goal = newgoal;
}

int32_t STEPPER_MOTOR_get_goal(motor_id_e id)
{
    if (id >= STEPPER_MOTOR_NB)
        return 0;
    return motor_data[id].goal;
}

int32_t STEPPER_MOTOR_get_position(motor_id_e id)
{
    if (id >= STEPPER_MOTOR_NB)
        return 0;
    return motor_data[id].position;
}

void STEPPER_MOTOR_set_position(motor_id_e id, int32_t newposition)
{
    if (id >= STEPPER_MOTOR_NB)
        return;
    motor_data[id].position = newposition;
}

bool STEPPER_MOTOR_is_arrived(motor_id_e id)
{
    if (id >= STEPPER_MOTOR_NB)
        return true;
    return (motor_data[id].goal == motor_data[id].position) && (motor_data[id].state == MOTOR_STATE_IDLE);
}

bool STEPPER_MOTOR_move(motor_id_e id, uint32_t steps, uint32_t frequency, stepper_motor_dir_t direction)
{
    if (id >= STEPPER_MOTOR_NB || !motor_configs[id].configured || !motor_data[id].enabled)
    {
        printf("ERROR: Motor %d not configured or not enabled\n", id);
        return false;
    }

    if (motor_data[id].state != MOTOR_STATE_IDLE)
    {
        printf("WARNING: Motor %d is busy (state %d)\n", id, motor_data[id].state);
        return false;
    }

    // Clamp frequency to safe limits
    if (frequency < STEPPER_MIN_FREQUENCY)
        frequency = STEPPER_MIN_FREQUENCY;
    if (frequency > STEPPER_MAX_FREQUENCY)
        frequency = STEPPER_MAX_FREQUENCY;

    // Check limit switches if enabled
    if (!STEPPER_MOTOR_check_limits(id, direction))
    {
        printf("ERROR: Motor %d movement blocked by limit switch\n", id);
        return false;
    }

    printf("Motor %d: Starting move - %lu steps at %lu Hz, dir %d\n", id, steps, frequency, direction);

    motor_data[id].steps_to_go = steps;
    motor_data[id].completed_steps = 0;
    motor_data[id].direction = direction;
    motor_data[id].pulse_period = STEPPER_MOTOR_frequency_to_period(frequency);
    motor_data[id].pulse_counter = 0;
    motor_data[id].state = MOTOR_STATE_CONSTANT_SPEED;

    // Set direction pin
    STEPPER_MOTORS_set_dir(id, direction);

    return true;
}

bool STEPPER_MOTOR_move_accel(motor_id_e id, uint32_t steps, uint32_t target_freq, stepper_motor_dir_t direction)
{
    if (id >= STEPPER_MOTOR_NB || !motor_configs[id].configured || !motor_data[id].enabled)
    {
        printf("ERROR: Motor %d not configured or not enabled\n", id);
        return false;
    }

    if (motor_data[id].state != MOTOR_STATE_IDLE)
    {
        printf("WARNING: Motor %d is busy (state %d)\n", id, motor_data[id].state);
        return false;
    }

    // Clamp target frequency
    if (target_freq < STEPPER_MIN_FREQUENCY)
        target_freq = STEPPER_MIN_FREQUENCY;
    if (target_freq > motor_data[id].accel.max_frequency)
        target_freq = motor_data[id].accel.max_frequency;

    // Check limit switches if enabled
    if (!STEPPER_MOTOR_check_limits(id, direction))
    {
        printf("ERROR: Motor %d movement blocked by limit switch\n", id);
        return false;
    }

    printf("Motor %d: Starting accelerated move - %lu steps to %lu Hz, dir %d\n",
           id, steps, target_freq, direction);

    motor_data[id].steps_to_go = steps;
    motor_data[id].completed_steps = 0;
    motor_data[id].direction = direction;
    motor_data[id].accel.current_frequency = STEPPER_MIN_FREQUENCY;
    motor_data[id].accel.target_frequency = target_freq;
    motor_data[id].pulse_period = STEPPER_MOTOR_frequency_to_period(STEPPER_MIN_FREQUENCY);
    motor_data[id].pulse_counter = 0;

    if (motor_data[id].accel.acceleration_enabled && steps > 100)
    {
        motor_data[id].state = MOTOR_STATE_ACCELERATING;
    }
    else
    {
        motor_data[id].state = MOTOR_STATE_CONSTANT_SPEED;
        motor_data[id].accel.current_frequency = target_freq;
        motor_data[id].pulse_period = STEPPER_MOTOR_frequency_to_period(target_freq);
    }

    // Set direction pin
    STEPPER_MOTORS_set_dir(id, direction);

    return true;
}

void STEPPER_MOTOR_stop(motor_id_e id)
{
    if (id >= STEPPER_MOTOR_NB)
        return;

    motor_data[id].state = MOTOR_STATE_IDLE;
    motor_data[id].steps_to_go = 0;
    motor_data[id].accel.current_frequency = STEPPER_MIN_FREQUENCY;

    printf("Motor %d stopped\n", id);
}

void STEPPER_MOTOR_emergency_stop_all(void)
{
    printf("EMERGENCY STOP ALL MOTORS\n");
    for (motor_id_e m = 0; m < STEPPER_MOTOR_NB; m++)
    {
        STEPPER_MOTOR_stop(m);
    }
}

void STEPPER_MOTOR_set_acceleration(motor_id_e id, uint32_t acceleration, uint32_t max_frequency)
{
    if (id >= STEPPER_MOTOR_NB)
        return;

    motor_data[id].accel.acceleration = acceleration;
    motor_data[id].accel.max_frequency = max_frequency;

    printf("Motor %d: Acceleration set to %lu steps/sec², max freq %lu Hz\n",
           id, acceleration, max_frequency);
}

void STEPPER_MOTOR_enable_acceleration(motor_id_e id, bool enable)
{
    if (id >= STEPPER_MOTOR_NB)
        return;

    motor_data[id].accel.acceleration_enabled = enable;
    printf("Motor %d: Acceleration %s\n", id, enable ? "enabled" : "disabled");
}

stepper_motor_state_t STEPPER_MOTOR_get_state(motor_id_e id)
{
    if (id >= STEPPER_MOTOR_NB)
        return MOTOR_STATE_ERROR;
    return motor_data[id].state;
}

uint32_t STEPPER_MOTOR_get_completed_steps(motor_id_e id)
{
    if (id >= STEPPER_MOTOR_NB)
        return 0;
    return motor_data[id].completed_steps;
}

uint32_t STEPPER_MOTOR_get_current_frequency(motor_id_e id)
{
    if (id >= STEPPER_MOTOR_NB)
        return 0;
    return motor_data[id].accel.current_frequency;
}

bool STEPPER_MOTOR_home_precision(motor_id_e id, uint32_t fast_frequency, uint32_t slow_frequency)
{
    if (id >= STEPPER_MOTOR_NB || !motor_configs[id].configured)
        return false;

    printf("Motor %d: Starting precision homing (fast=%lu Hz, slow=%lu Hz)\n",
           id, fast_frequency, slow_frequency);

    // Enable limit checking for homing
    motor_data[id].limit_check_enabled = true;

    // Start fast homing move
    motor_data[id].steps_to_go = 10000; // Large number for homing
    motor_data[id].completed_steps = 0;
    motor_data[id].direction = MOTOR_DIR_COUNTERCLOCKWISE; // Home towards minimum
    motor_data[id].pulse_period = STEPPER_MOTOR_frequency_to_period(fast_frequency);
    motor_data[id].pulse_counter = 0;
    motor_data[id].state = MOTOR_STATE_HOMING_FAST;
    motor_data[id].accel.current_frequency = fast_frequency;

    // Set direction pin for homing (towards minimum)
    STEPPER_MOTORS_set_dir(id, MOTOR_DIR_COUNTERCLOCKWISE);

    return true;
}

void STEPPER_MOTOR_set_axis(motor_id_e id, axis_t axis)
{
    if (id >= STEPPER_MOTOR_NB)
        return;
    motor_data[id].axis = axis;
}

void STEPPER_MOTOR_enable_limit_check(motor_id_e id, bool enable)
{
    if (id >= STEPPER_MOTOR_NB)
        return;
    motor_data[id].limit_check_enabled = enable;
}

void STEPPER_MOTOR_update(void)
{
    // This function can be called from main loop for additional processing
    // Most work is done in the timer interrupt
}

void STEPPER_MOTOR_debug_config(motor_id_e id)
{
    if (id >= STEPPER_MOTOR_NB)
    {
        printf("Invalid motor ID %d\n", id);
        return;
    }

    printf("=== Motor %d Debug ===\n", id);
    printf("  Configured: %s\n", motor_configs[id].configured ? "YES" : "NO");
    if (motor_configs[id].configured)
    {
        printf("  PUL: GPIO%c Pin %lu\n",
               (motor_configs[id].pulse_gpio == GPIOA) ? 'A' : 'B',
               motor_configs[id].pulse_pin);
        printf("  DIR: GPIO%c Pin %lu\n",
               (motor_configs[id].dir_gpio == GPIOA) ? 'A' : 'B',
               motor_configs[id].dir_pin);
        printf("  Enabled: %s\n", motor_data[id].enabled ? "YES" : "NO");
        printf("  Position: %ld\n", motor_data[id].position);
        printf("  Goal: %ld\n", motor_data[id].goal);
        printf("  State: %d\n", motor_data[id].state);
        printf("  Microsteps: %d\n", motor_data[id].microsteps);
        printf("  Acceleration: %s (%lu steps/sec², max %lu Hz)\n",
               motor_data[id].accel.acceleration_enabled ? "ON" : "OFF",
               motor_data[id].accel.acceleration,
               motor_data[id].accel.max_frequency);
    }
    printf("==================\n");
}

// Private function implementations

static void STEPPER_MOTOR_pin_set(GPIO_TypeDef *gpio, uint32_t pin, bool state)
{
    if (gpio == NULL)
        return;

    if (state)
        gpio->BSRR = (1UL << pin);
    else
        gpio->BSRR = (1UL << (pin + 16));
}

static void STEPPER_MOTORS_do_pulse(motor_id_e id)
{
    if (!motor_configs[id].configured)
        return;

    // Generate pulse
    STEPPER_MOTOR_pin_set(motor_configs[id].pulse_gpio, motor_configs[id].pulse_pin, ACTIVE_STATE);
    // Small delay for pulse width (hardware dependent)
    for (volatile int i = 0; i < 10; i++)
    {
        __NOP();
    }
    STEPPER_MOTOR_pin_set(motor_configs[id].pulse_gpio, motor_configs[id].pulse_pin, SLEEPING_STATE);

    if (callback_at_each_pulse != NULL)
        callback_at_each_pulse();
}

static void STEPPER_MOTORS_set_dir(motor_id_e id, stepper_motor_dir_t direction)
{
    if (!motor_configs[id].configured)
        return;

    STEPPER_MOTOR_pin_set(motor_configs[id].dir_gpio, motor_configs[id].dir_pin,
                          (direction == MOTOR_DIR_CLOCKWISE) ? SLEEPING_STATE : ACTIVE_STATE);
}

static void STEPPER_MOTOR_update_acceleration(motor_id_e id)
{
    if (!motor_data[id].accel.acceleration_enabled)
        return;

    uint32_t steps_for_decel = motor_data[id].steps_to_go;
    uint32_t current_freq = motor_data[id].accel.current_frequency;
    uint32_t target_freq = motor_data[id].accel.target_frequency;

    // Calculate if we need to start decelerating
    uint32_t decel_distance = (current_freq * current_freq) / (2 * motor_data[id].accel.acceleration);

    switch (motor_data[id].state)
    {
    case MOTOR_STATE_ACCELERATING:
        if (steps_for_decel <= decel_distance || current_freq >= target_freq)
        {
            motor_data[id].state = MOTOR_STATE_CONSTANT_SPEED;
        }
        else
        {
            // Increase frequency
            uint32_t new_freq = current_freq + (motor_data[id].accel.acceleration * DEFAULT_IT_PERIOD) / 1000000;
            if (new_freq > target_freq)
                new_freq = target_freq;
            motor_data[id].accel.current_frequency = new_freq;
            motor_data[id].pulse_period = STEPPER_MOTOR_frequency_to_period(new_freq);
        }
        break;

    case MOTOR_STATE_CONSTANT_SPEED:
        if (steps_for_decel <= decel_distance && current_freq > STEPPER_MIN_FREQUENCY)
        {
            motor_data[id].state = MOTOR_STATE_DECELERATING;
        }
        break;

    case MOTOR_STATE_DECELERATING:
        if (current_freq > STEPPER_MIN_FREQUENCY)
        {
            // Decrease frequency
            uint32_t new_freq = current_freq - (motor_data[id].accel.acceleration * DEFAULT_IT_PERIOD) / 1000000;
            if (new_freq < STEPPER_MIN_FREQUENCY)
                new_freq = STEPPER_MIN_FREQUENCY;
            motor_data[id].accel.current_frequency = new_freq;
            motor_data[id].pulse_period = STEPPER_MOTOR_frequency_to_period(new_freq);
        }
        break;

    default:
        break;
    }
}

static uint32_t STEPPER_MOTOR_frequency_to_period(uint32_t frequency)
{
    if (frequency == 0)
        return 1000000; // Very slow

    // Convert frequency to timer periods
    // Base timer runs at 20kHz (50us), so period = (20000 / frequency)
    uint32_t period = (20000 / frequency);
    if (period < 1)
        period = 1;
    return period;
}

static bool STEPPER_MOTOR_check_limits(motor_id_e id, stepper_motor_dir_t direction)
{
    if (!motor_data[id].limit_check_enabled)
        return true; // No limit checking

    // Check appropriate limit switch based on direction
    limit_position_t limit_pos = (direction == MOTOR_DIR_COUNTERCLOCKWISE) ? LIMIT_MIN : LIMIT_MAX;
    limit_switch_state_t limit_state = limit_switch_read(motor_data[id].axis, limit_pos);

    return (limit_state != LIMIT_TRIGGERED);
}

// Timer interrupt handler
void STEPPER_MOTOR_timer_irq_handler(void)
{
    for (motor_id_e m = 0; m < STEPPER_MOTOR_NB; m++)
    {
        if (!motor_configs[m].configured || !motor_data[m].enabled)
            continue;

        // Handle different motor states
        switch (motor_data[m].state)
        {
        case MOTOR_STATE_IDLE:
            // Handle position-based movement (legacy mode)
            if (motor_data[m].position < motor_data[m].goal)
            {
                motor_data[m].position++;
                STEPPER_MOTORS_set_dir(m, MOTOR_DIR_CLOCKWISE);
                STEPPER_MOTORS_do_pulse(m);
            }
            else if (motor_data[m].position > motor_data[m].goal)
            {
                motor_data[m].position--;
                STEPPER_MOTORS_set_dir(m, MOTOR_DIR_COUNTERCLOCKWISE);
                STEPPER_MOTORS_do_pulse(m);
            }
            break;

        case MOTOR_STATE_ACCELERATING:
        case MOTOR_STATE_CONSTANT_SPEED:
        case MOTOR_STATE_DECELERATING:
            // Handle step-based movement with acceleration
            motor_data[m].pulse_counter++;
            if (motor_data[m].pulse_counter >= motor_data[m].pulse_period)
            {
                motor_data[m].pulse_counter = 0;

                if (motor_data[m].steps_to_go > 0)
                {
                    // Check limits during movement
                    if (motor_data[m].limit_check_enabled)
                    {
                        limit_position_t limit_pos = (motor_data[m].direction == MOTOR_DIR_COUNTERCLOCKWISE) ? LIMIT_MIN : LIMIT_MAX;
                        if (limit_switch_read(motor_data[m].axis, limit_pos) == LIMIT_TRIGGERED)
                        {
                            motor_data[m].state = MOTOR_STATE_IDLE;
                            motor_data[m].steps_to_go = 0;
                            break;
                        }
                    }

                    STEPPER_MOTORS_do_pulse(m);
                    motor_data[m].steps_to_go--;
                    motor_data[m].completed_steps++;

                    // Update position based on direction
                    if (motor_data[m].direction == MOTOR_DIR_CLOCKWISE)
                        motor_data[m].position++;
                    else
                        motor_data[m].position--;

                    // Update acceleration profile
                    STEPPER_MOTOR_update_acceleration(m);

                    // Check if movement complete
                    if (motor_data[m].steps_to_go == 0)
                    {
                        motor_data[m].state = MOTOR_STATE_IDLE;
                        motor_data[m].accel.current_frequency = STEPPER_MIN_FREQUENCY;
                    }
                }
            }
            break;

        case MOTOR_STATE_HOMING_FAST:
        case MOTOR_STATE_HOMING_SLOW:
            // Handle homing
            motor_data[m].pulse_counter++;
            if (motor_data[m].pulse_counter >= motor_data[m].pulse_period)
            {
                motor_data[m].pulse_counter = 0;

                // Check limit switch
                if (limit_switch_read(motor_data[m].axis, LIMIT_MIN) == LIMIT_TRIGGERED)
                {
                    if (motor_data[m].state == MOTOR_STATE_HOMING_FAST)
                    {
                        // Switch to slow homing
                        motor_data[m].state = MOTOR_STATE_HOMING_SLOW;
                        motor_data[m].direction = MOTOR_DIR_CLOCKWISE;                       // Back off
                        motor_data[m].pulse_period = STEPPER_MOTOR_frequency_to_period(400); // Slow speed
                        STEPPER_MOTORS_set_dir(m, MOTOR_DIR_CLOCKWISE);
                    }
                    else
                    {
                        // Homing complete
                        motor_data[m].state = MOTOR_STATE_IDLE;
                        motor_data[m].position = 0;
                        motor_data[m].completed_steps = 0;
                    }
                }
                else if (motor_data[m].state == MOTOR_STATE_HOMING_SLOW &&
                         limit_switch_read(motor_data[m].axis, LIMIT_MIN) != LIMIT_TRIGGERED)
                {
                    // Backed off from limit, now go slow towards it
                    motor_data[m].direction = MOTOR_DIR_COUNTERCLOCKWISE;
                    STEPPER_MOTORS_set_dir(m, MOTOR_DIR_COUNTERCLOCKWISE);
                }

                STEPPER_MOTORS_do_pulse(m);
                motor_data[m].completed_steps++;

                // Update position
                if (motor_data[m].direction == MOTOR_DIR_CLOCKWISE)
                    motor_data[m].position++;
                else
                    motor_data[m].position--;
            }
            break;

        default:
            break;
        }
    }
}

#endif /* USE_STEPPER_MOTOR */
