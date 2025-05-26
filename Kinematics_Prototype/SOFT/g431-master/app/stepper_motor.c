/**
 *******************************************************************************
 * @file    stepper_motor.c
 * @author  Your Name
 * @date    Current Date
 * @brief   Advanced stepper motor control with state machine and timer interrupts
 *******************************************************************************
 */

#include "stepper_motor.h"
#include "stm32g4_utils.h"
#include "stm32g4_gpio.h"
#include "stm32g4_timer.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/**
 * TB6600 Driver Configuration Notes:
 * ----------------------------------
 * 1. Microstepping Configuration:
 *    - SW1-SW3 pins set microstepping mode
 *    - Common settings:
 *      1/1 step: SW1=OFF, SW2=OFF, SW3=OFF
 *      1/2 step: SW1=ON,  SW2=OFF, SW3=OFF
 *      1/4 step: SW1=OFF, SW2=ON,  SW3=OFF
 *      1/8 step: SW1=ON,  SW2=ON,  SW3=OFF
 *      1/16step: SW1=ON,  SW2=ON,  SW3=ON
 *
 * 2. Current Setting:
 *    - Use potentiometer on TB6600 to set current limit
 *    - Recommended: Set to 70-80% of motor's rated current
 *    - NEMA17: typically 1.5-2A, set TB6600 to ~1.2-1.6A
 *
 * 3. Power Supply:
 *    - TB6600 requires 9-42VDC power supply
 *    - For NEMA17, 12-24VDC is typically recommended
 */

// Constants for acceleration profile
#define ACCEL_TIME_MS 300      // Time to accelerate/decelerate (ms)
#define MIN_FREQUENCY 2000.0f  // Minimum frequency for stepping (Hz)
#define MAX_FREQUENCY 20000.0f // Maximum frequency for stepping (Hz)
#define TIMER_PRESCALER 100    // Prescaler for timer counter
#define PULSE_DUTY_CYCLE 60    // PWM duty cycle (percent) - INCREASED TO 50%
#define UPDATE_INTERVAL_MS 10  // How often to update acceleration curve (ms)
#define ACCEL_UPDATES_PER_SEC (1000 / UPDATE_INTERVAL_MS)

// Stepper motor structure with state machine
typedef struct
{
    // Hardware configuration
    GPIO_TypeDef *step_port;
    uint16_t step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    timer_id_t timer_id;
    uint32_t timer_channel;

    // Motor parameters
    float steps_per_mm;
    int32_t current_position;
    bool enabled;
    uint8_t microstepping;
    float current_limit_a;

    // State machine variables
    stepper_state_t state;
    bool direction;
    int32_t target_position;
    int32_t steps_to_move;
    int32_t steps_taken;
    float current_frequency;
    float target_frequency;
    float acceleration_time_ms;
    uint32_t last_update_time;
    stepper_callback_t callback;

// Endstop configuration
#if ENDSTOP_ENABLED
    GPIO_TypeDef *min_endstop_port;
    uint16_t min_endstop_pin;
    GPIO_TypeDef *max_endstop_port;
    uint16_t max_endstop_pin;
    uint8_t endstop_trigger_level;
    bool min_endstop_enabled;
    bool max_endstop_enabled;
#endif
} stepper_motor_t;

// Motor array and helper functions
static stepper_motor_t steppers[STEPPER_COUNT];
static bool is_valid_motor(stepper_id_t id);
static void set_direction(stepper_id_t id, bool forward);
static void update_motor_frequency(stepper_id_t id, float frequency);
static float calculate_acceleration(float progress);
static void motor_step_handler(stepper_id_t id);
static uint32_t get_timer_af(timer_id_t timer_id);

// Function to check if a motor ID is valid
static bool is_valid_motor(stepper_id_t id)
{
    if (id < 0 || id >= STEPPER_COUNT || !steppers[id].enabled)
    {
        return false;
    }
    return true;
}

// Initialize the stepper motor module
void STEPPER_init(void)
{
    for (int i = 0; i < STEPPER_COUNT; i++)
    {
        memset(&steppers[i], 0, sizeof(stepper_motor_t));
        steppers[i].enabled = false;
        steppers[i].state = STEPPER_STATE_IDLE;
    }

    printf("Stepper motor module initialized\n");
}

// Add a stepper motor
stepper_id_t STEPPER_add(GPIO_TypeDef *step_port, uint16_t step_pin,
                         GPIO_TypeDef *dir_port, uint16_t dir_pin,
                         timer_id_t timer_id, uint32_t timer_channel,
                         float steps_per_mm, uint8_t microstepping, float current_limit_a)
{

    // Find an available slot
    for (stepper_id_t id = 0; id < STEPPER_COUNT; id++)
    {
        if (!steppers[id].enabled)
        {
            // Configure step pin as timer output (for PWM)
            uint32_t timer_af = get_timer_af(timer_id);
            BSP_GPIO_pin_config(step_port, step_pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, timer_af);

            // Configure direction pin as output
            BSP_GPIO_pin_config(dir_port, dir_pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0);

            // Initialize the timer
            BSP_TIMER_run_us(timer_id, 100, true); // 10kHz base frequency

            // Store configuration
            steppers[id].step_port = step_port;
            steppers[id].step_pin = step_pin;
            steppers[id].dir_port = dir_port;
            steppers[id].dir_pin = dir_pin;
            steppers[id].timer_id = timer_id;
            steppers[id].timer_channel = timer_channel;
            steppers[id].steps_per_mm = steps_per_mm;
            steppers[id].current_position = 0;
            steppers[id].enabled = true;
            steppers[id].microstepping = microstepping;
            steppers[id].current_limit_a = current_limit_a;
            steppers[id].state = STEPPER_STATE_IDLE;
            steppers[id].current_frequency = 0;
            steppers[id].target_frequency = 0;
            steppers[id].steps_taken = 0;
            steppers[id].callback = NULL;

            // Set default direction
            HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);

#if ENDSTOP_ENABLED
            // Initialize endstop settings
            steppers[id].min_endstop_port = NULL;
            steppers[id].min_endstop_pin = 0;
            steppers[id].max_endstop_port = NULL;
            steppers[id].max_endstop_pin = 0;
            steppers[id].endstop_trigger_level = ENDSTOP_TRIGGER_LOW;
            steppers[id].min_endstop_enabled = false;
            steppers[id].max_endstop_enabled = false;
#endif

            printf("Added stepper motor %d\n", id);
            printf("Motor ID %d initialized successfully on pins: Step=%d, Dir=%d\n",
                   id, step_pin, dir_pin);
            printf("Motor %d config: %d steps/mm, 1/%d microstepping, %.1fA current\n",
                   id, (int)steps_per_mm, microstepping, current_limit_a);

            // Test the direction pin to make sure GPIO is working
            HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_SET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);

            return id;
        }
    }

    printf("Failed to add stepper motor - no slots available\n");
    return -1;
}

// Set the direction pin for a motor
static void set_direction(stepper_id_t id, bool forward)
{
    if (!is_valid_motor(id))
        return;

    if (steppers[id].direction != forward)
    {
        HAL_GPIO_WritePin(steppers[id].dir_port, steppers[id].dir_pin,
                          forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
        steppers[id].direction = forward;

        // Add a small delay after direction change (TB6600 needs this)
        HAL_Delay(1);
    }
}

// Update the timer frequency for a motor
// Update frequency_profile to ensure movement completion
static void update_motor_frequency(stepper_id_t id, float frequency)
{
    if (!is_valid_motor(id))
        return;

    // Only update if change is significant (>1%) or turning off
    if (frequency <= 1.0f ||
        fabsf(steppers[id].current_frequency - frequency) > (steppers[id].current_frequency * 0.01f))
    {
        // If frequency is too low, disable PWM and return
        if (frequency < 1.0f)
        {
            BSP_TIMER_disable_PWM(steppers[id].timer_id, steppers[id].timer_channel);
            steppers[id].current_frequency = 0;
            return;
        }

        // IMPORTANT CHANGE: Convert frequency to period (may need adjustment based on BSP_TIMER implementation)
        // Period in microseconds = 1,000,000 / frequency in Hz
        float period_us = 1000000.0f / frequency;

        // Set timer period in microseconds
        BSP_TIMER_set_period_us(steppers[id].timer_id, (uint32_t)period_us);

        // Enable PWM with higher duty cycle for more reliable stepping
        BSP_TIMER_enable_PWM(steppers[id].timer_id, steppers[id].timer_channel, 80, false, false);

        // Update stored frequency
        steppers[id].current_frequency = frequency;

        // Debug output for troubleshooting
        printf("Motor %d: Set freq=%.1f Hz, period=%lu us\n",
               id, frequency, (uint32_t)period_us);
    }
}

// S-curve acceleration profile
static float calculate_acceleration(float progress)
{
    // Modified sine function for smooth acceleration/deceleration
    if (progress < 0)
        progress = 0;
    if (progress > 1)
        progress = 1;
    return (1.0f - cosf(progress * M_PI)) / 2.0f;
}

// Start moving the motor a specified number of steps
bool STEPPER_move_steps(stepper_id_t id, int32_t steps, float speed, stepper_callback_t callback)
{
    if (!is_valid_motor(id))
        return false;

    // If already moving, stop the current movement
    if (steppers[id].state != STEPPER_STATE_IDLE)
    {
        STEPPER_stop(id);
    }

    // Determine direction
    bool direction = steps >= 0;
    steps = abs(steps);

    // Adjust speed limits based on microstepping
    float max_speed = MAX_FREQUENCY;
    float min_speed = MIN_FREQUENCY;

    if (speed > max_speed)
    {
        printf("Speed limited from %.1f to %.1f Hz\n", speed, max_speed);
        speed = max_speed;
    }

    if (speed < min_speed)
    {
        printf("Speed increased from %.1f to %.1f Hz\n", speed, min_speed);
        speed = min_speed;
    }

    // Set direction
    set_direction(id, direction);

    // Update the motor state variables
    steppers[id].target_position = steppers[id].current_position + (direction ? steps : -steps);
    steppers[id].steps_to_move = steps;
    steppers[id].steps_taken = 0;
    steppers[id].target_frequency = speed;
    steppers[id].acceleration_time_ms = ACCEL_TIME_MS;
    steppers[id].current_frequency = min_speed;
    steppers[id].callback = callback;
    steppers[id].last_update_time = HAL_GetTick();

    printf("Moving motor %d: %d steps at %.1f Hz\n", id, steps, speed);

    // Set initial state to accelerating
    steppers[id].state = STEPPER_STATE_ACCELERATING;

    // Start PWM with initial frequency
    float period_us = 1000000.0f / min_speed;
    BSP_TIMER_set_period_us(steppers[id].timer_id, (uint32_t)period_us);
    BSP_TIMER_enable_PWM(steppers[id].timer_id, steppers[id].timer_channel, PULSE_DUTY_CYCLE, false, false);

    return true;
}

// Move to a specific position in mm
bool STEPPER_move_to_mm(stepper_id_t id, float position_mm, float speed_mm_s, stepper_callback_t callback)
{
    if (!is_valid_motor(id))
        return false;

    // Calculate current position in mm
    float current_mm = (float)steppers[id].current_position / steppers[id].steps_per_mm;

    // Calculate required movement
    float move_mm = position_mm - current_mm;

    // Convert to steps and move
    int32_t steps = (int32_t)(move_mm * steppers[id].steps_per_mm);
    float step_freq = speed_mm_s * steppers[id].steps_per_mm;

    return STEPPER_move_steps(id, steps, step_freq, callback);
}

// Move a specified distance in mm
bool STEPPER_move_mm(stepper_id_t id, float distance_mm, float speed_mm_s, stepper_callback_t callback)
{
    if (!is_valid_motor(id))
        return false;

    // Convert to steps and move
    int32_t steps = (int32_t)(distance_mm * steppers[id].steps_per_mm);
    float step_freq = speed_mm_s * steppers[id].steps_per_mm;

    return STEPPER_move_steps(id, steps, step_freq, callback);
}

// Stop a motor
void STEPPER_stop(stepper_id_t id)
{
    if (!is_valid_motor(id))
        return;

    printf("Stopping motor %d\n", id);

    // Disable PWM
    if (BSP_TIMER_disable_PWM(steppers[id].timer_id, steppers[id].timer_channel) != HAL_OK)
    {
        printf("Error: Failed to disable PWM for motor %d\n", id);

        // Force PWM disable by directly accessing the timer
        TIM_HandleTypeDef *htim = BSP_TIMER_get_handler(steppers[id].timer_id);
        if (htim != NULL)
        {
            // Force disable the timer
            htim->Instance->CR1 &= ~(TIM_CR1_CEN); // Disable counter
            htim->Instance->CCER = 0;              // Disable all outputs
        }
    }

    // Ensure the direction pin is at a safe level
    HAL_GPIO_WritePin(steppers[id].dir_port, steppers[id].dir_pin, GPIO_PIN_RESET);

    // Update state
    steppers[id].state = STEPPER_STATE_IDLE;
    steppers[id].current_frequency = 0;
    steppers[id].steps_taken = steppers[id].steps_to_move; // Mark movement as complete

    // Execute callback if provided
    if (steppers[id].callback != NULL)
    {
        stepper_callback_t callback = steppers[id].callback;
        steppers[id].callback = NULL;
        callback(id, false); // Indicate movement was stopped early
    }

    printf("Motor %d stopped\n", id);
}

// Check if motor is moving
bool STEPPER_is_moving(stepper_id_t id)
{
    if (!is_valid_motor(id))
        return false;
    return (steppers[id].state != STEPPER_STATE_IDLE);
}

// Get position in mm
float STEPPER_get_position_mm(stepper_id_t id)
{
    if (!is_valid_motor(id))
        return 0.0f;
    return (float)steppers[id].current_position / steppers[id].steps_per_mm;
}

// Get steps per mm
float STEPPER_get_steps_per_mm(stepper_id_t id)
{
    if (!is_valid_motor(id))
        return 0.0f;
    return steppers[id].steps_per_mm;
}

// Home a motor
bool STEPPER_home(stepper_id_t id, stepper_callback_t callback)
{
    if (!is_valid_motor(id))
        return false;

#if ENDSTOP_ENABLED
    if (steppers[id].min_endstop_enabled)
    {
        // If endstop is configured, use it for homing
        return STEPPER_home_with_endstop(id, 5.0f, 500.0f, callback);
    }
#endif

    // Otherwise, just reset position
    steppers[id].current_position = 0;
    if (callback != NULL)
    {
        callback(id, true);
    }
    return true;
}

// Process the stepper motor state machines
void STEPPER_process(void)
{
    for (stepper_id_t id = 0; id < STEPPER_COUNT; id++)
    {
        if (!is_valid_motor(id) || steppers[id].state == STEPPER_STATE_IDLE)
            continue;

        uint32_t current_time = HAL_GetTick();

        // Check if the movement is complete
        if (steppers[id].steps_taken >= steppers[id].steps_to_move)
        {
            printf("Motor %d movement complete. Pos: %ld\n", id, steppers[id].current_position);

            // Stop PWM
            BSP_TIMER_disable_PWM(steppers[id].timer_id, steppers[id].timer_channel);

            // Set state to idle
            steppers[id].state = STEPPER_STATE_IDLE;
            steppers[id].current_frequency = 0;

            // Call callback if provided
            if (steppers[id].callback != NULL)
            {
                stepper_callback_t callback = steppers[id].callback;
                steppers[id].callback = NULL;
                callback(id, true);
            }

            continue;
        }

        // Process based on current state
        switch (steppers[id].state)
        {
        case STEPPER_STATE_ACCELERATING:
        {
            // Calculate elapsed time as a percentage of acceleration time
            float elapsed_ms = (float)(current_time - steppers[id].last_update_time);
            float progress = elapsed_ms / steppers[id].acceleration_time_ms;

            // Calculate new frequency using acceleration curve
            float acceleration_factor = calculate_acceleration(progress);
            float new_freq = MIN_FREQUENCY + (steppers[id].target_frequency - MIN_FREQUENCY) * acceleration_factor;

            // Update motor frequency
            update_motor_frequency(id, new_freq);

            // Check if acceleration phase is complete
            if (progress >= 1.0f || steppers[id].steps_taken >= steppers[id].steps_to_move / 2)
            {
                steppers[id].state = STEPPER_STATE_CONSTANT_SPEED;
                printf("Motor %d reached constant speed: %.1f Hz\n", id, new_freq);
            }

            break;
        }

        case STEPPER_STATE_CONSTANT_SPEED:
        {
            // Check if it's time to start decelerating
            int32_t steps_remaining = steppers[id].steps_to_move - steppers[id].steps_taken;
            int32_t steps_needed_to_stop = (int32_t)(steppers[id].current_frequency *
                                                     steppers[id].acceleration_time_ms / 1000.0f);

            // Print debug info occasionally
            if ((steppers[id].steps_taken % 100) == 0)
            {
                printf("Motor %d const - Pos: %ld, Remaining: %ld\n",
                       id, steppers[id].current_position, steps_remaining);
            }

            if (steps_remaining <= steps_needed_to_stop)
            {
                steppers[id].state = STEPPER_STATE_DECELERATING;
                printf("Motor %d starting deceleration, %ld steps remaining\n", id, steps_remaining);
            }
            break;
        }

        case STEPPER_STATE_DECELERATING:
        {
            // Calculate deceleration progress
            int32_t steps_remaining = steppers[id].steps_to_move - steppers[id].steps_taken;
            int32_t steps_needed_to_stop = (int32_t)(steppers[id].current_frequency *
                                                     steppers[id].acceleration_time_ms / 1000.0f);

            // Ensure we don't divide by zero
            if (steps_needed_to_stop <= 0)
                steps_needed_to_stop = 1;

            float progress = 1.0f - (float)steps_remaining / (float)steps_needed_to_stop;
            if (progress > 1.0f)
                progress = 1.0f;

            // Calculate new frequency using deceleration curve
            float deceleration_factor = calculate_acceleration(1.0f - progress);
            float new_freq = MIN_FREQUENCY + (steppers[id].target_frequency - MIN_FREQUENCY) * deceleration_factor;

            // Make sure we don't go below minimum frequency
            if (new_freq < MIN_FREQUENCY)
                new_freq = MIN_FREQUENCY;

            // Update motor frequency
            update_motor_frequency(id, new_freq);

            // Print debug info occasionally
            if ((steppers[id].steps_taken % 100) == 0)
            {
                printf("Motor %d decel - Pos: %ld, Freq: %.1f, Progress: %.2f\n",
                       id, steppers[id].current_position, new_freq, progress);
            }

            break;
        }

#if ENDSTOP_ENABLED
        case STEPPER_STATE_HOMING:
        {
            // Check if endstop is hit
            endstop_type_t endstop = steppers[id].direction ? ENDSTOP_MAX : ENDSTOP_MIN;

            if (STEPPER_is_endstop_hit(id, endstop))
            {
                // Endstop hit - stop the motor
                BSP_TIMER_disable_PWM(steppers[id].timer_id, steppers[id].timer_channel);

                // Update position to zero if it's the min endstop
                if (endstop == ENDSTOP_MIN)
                {
                    steppers[id].current_position = 0;
                }

                // Set state to idle
                steppers[id].state = STEPPER_STATE_IDLE;

                // Call callback if provided
                if (steppers[id].callback != NULL)
                {
                    stepper_callback_t callback = steppers[id].callback;
                    steppers[id].callback = NULL;
                    callback(id, true);
                }
            }
            break;
        }
#endif

        default:
            break;
        }
    }
}

// Timer handler for motor stepping
static void motor_step_handler(stepper_id_t id)
{
    if (!is_valid_motor(id) || steppers[id].state == STEPPER_STATE_IDLE)
    {
        return;
    }

    // Increment step counter
    steppers[id].steps_taken++;

    // Update current position based on direction
    if (steppers[id].direction)
    {
        steppers[id].current_position++;
    }
    else
    {
        steppers[id].current_position--;
    }

// Check endstops if enabled
#if ENDSTOP_ENABLED
    if (steppers[id].state != STEPPER_STATE_HOMING)
    {
        // Check if we hit an endstop
        bool endstop_hit = false;

        if (steppers[id].direction && steppers[id].max_endstop_enabled)
        {
            endstop_hit = STEPPER_is_endstop_hit(id, ENDSTOP_MAX);
        }
        else if (!steppers[id].direction && steppers[id].min_endstop_enabled)
        {
            endstop_hit = STEPPER_is_endstop_hit(id, ENDSTOP_MIN);
        }

        if (endstop_hit)
        {
            // Stop the motor immediately
            BSP_TIMER_disable_PWM(steppers[id].timer_id, steppers[id].timer_channel);
            steppers[id].state = STEPPER_STATE_IDLE;

            printf("Motor %d hit endstop at position %ld\n", id, steppers[id].current_position);

            // Call callback if provided
            if (steppers[id].callback != NULL)
            {
                stepper_callback_t callback = steppers[id].callback;
                steppers[id].callback = NULL;
                callback(id, false); // Indicate unexpected stop
            }
        }
    }
#endif
}

// Timer interrupt handlers
void TIMER1_user_handler_it(void)
{
    for (stepper_id_t id = 0; id < STEPPER_COUNT; id++)
    {
        if (steppers[id].enabled && steppers[id].timer_id == TIMER1_ID)
        {
            motor_step_handler(id);
        }
    }
}

void TIMER2_user_handler_it(void)
{
    for (stepper_id_t id = 0; id < STEPPER_COUNT; id++)
    {
        if (steppers[id].enabled && steppers[id].timer_id == TIMER2_ID)
        {
            motor_step_handler(id);
        }
    }
}

void TIMER3_user_handler_it(void)
{
    for (stepper_id_t id = 0; id < STEPPER_COUNT; id++)
    {
        if (steppers[id].enabled && steppers[id].timer_id == TIMER3_ID)
        {
            motor_step_handler(id);
        }
    }
}

void TIMER4_user_handler_it(void)
{
    for (stepper_id_t id = 0; id < STEPPER_COUNT; id++)
    {
        if (steppers[id].enabled && steppers[id].timer_id == TIMER4_ID)
        {
            motor_step_handler(id);
        }
    }
}

// Get the appropriate alternate function for a timer
static uint32_t get_timer_af(timer_id_t timer_id)
{
    switch (timer_id)
    {
    case TIMER1_ID:
        return GPIO_AF6_TIM1;
    case TIMER2_ID:
        return GPIO_AF1_TIM2;
    case TIMER3_ID:
        return GPIO_AF2_TIM3;
    case TIMER4_ID:
        return GPIO_AF2_TIM4;
    case TIMER6_ID:
        return GPIO_AF2_TIM15;
    default:
        return GPIO_AF2_TIM3;
    }
}

// Endstop functions
#if ENDSTOP_ENABLED

// Configure an endstop
HAL_StatusTypeDef STEPPER_config_endstop(stepper_id_t id, endstop_type_t endstop_type,
                                         GPIO_TypeDef *port, uint16_t pin, uint8_t trigger_level)
{
    if (!is_valid_motor(id))
    {
        return HAL_ERROR;
    }

    // Configure GPIO pin as input with pull-up
    BSP_GPIO_pin_config(port, pin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, 0);

    // Update the stepper motor configuration
    if (endstop_type == ENDSTOP_MIN)
    {
        steppers[id].min_endstop_port = port;
        steppers[id].min_endstop_pin = pin;
        steppers[id].min_endstop_enabled = true;
    }
    else
    { // ENDSTOP_MAX
        steppers[id].max_endstop_port = port;
        steppers[id].max_endstop_pin = pin;
        steppers[id].max_endstop_enabled = true;
    }

    // Set trigger level
    steppers[id].endstop_trigger_level = trigger_level;

    printf("Configured %s endstop for motor %d on pin %d, trigger level: %s\n",
           (endstop_type == ENDSTOP_MIN) ? "MIN" : "MAX", id, pin,
           (trigger_level == ENDSTOP_TRIGGER_HIGH) ? "HIGH" : "LOW");

    return HAL_OK;
}

// Check if an endstop is currently triggered
bool STEPPER_is_endstop_hit(stepper_id_t id, endstop_type_t endstop_type)
{
    if (!is_valid_motor(id))
    {
        return false;
    }

    GPIO_TypeDef *port;
    uint16_t pin;
    bool endstop_enabled;

    if (endstop_type == ENDSTOP_MIN)
    {
        port = steppers[id].min_endstop_port;
        pin = steppers[id].min_endstop_pin;
        endstop_enabled = steppers[id].min_endstop_enabled;
    }
    else
    { // ENDSTOP_MAX
        port = steppers[id].max_endstop_port;
        pin = steppers[id].max_endstop_pin;
        endstop_enabled = steppers[id].max_endstop_enabled;
    }

    if (!endstop_enabled || port == NULL)
    {
        return false;
    }

    // Read the endstop pin
    GPIO_PinState state = HAL_GPIO_ReadPin(port, pin);

    // Check if triggered according to configured logic level
    uint8_t trigger_level = steppers[id].endstop_trigger_level;
    return (state == GPIO_PIN_SET && trigger_level == ENDSTOP_TRIGGER_HIGH) ||
           (state == GPIO_PIN_RESET && trigger_level == ENDSTOP_TRIGGER_LOW);
}

// Home using an endstop
bool STEPPER_home_with_endstop(stepper_id_t id, float homing_speed_mm_s, float max_travel_mm, stepper_callback_t callback)
{
    if (!is_valid_motor(id) || !steppers[id].min_endstop_enabled)
    {
        printf("Error: Cannot home motor %d - no MIN endstop configured\n", id);
        return false;
    }

    printf("Homing motor %d (max travel: %.2f mm)...\n", id, max_travel_mm);

    // First, check if already at home position
    if (STEPPER_is_endstop_hit(id, ENDSTOP_MIN))
    {
        printf("Already at home position\n");
        steppers[id].current_position = 0;

        // Move away from switch a small amount then back for precision
        STEPPER_move_mm(id, 5.0f, homing_speed_mm_s, NULL);
        HAL_Delay(500); // Wait for move to complete

        // Then move back to home position at slow speed
        STEPPER_move_mm(id, -6.0f, homing_speed_mm_s / 2, callback);
        return true;
    }

    // Calculate maximum steps based on max travel distance
    int32_t max_steps = (int32_t)(max_travel_mm * steppers[id].steps_per_mm);

    // Convert speed to steps per second
    float homing_speed_steps = homing_speed_mm_s * steppers[id].steps_per_mm;

    // Update the motor state variables for homing
    steppers[id].direction = false; // Move toward home (negative direction)
    steppers[id].steps_to_move = max_steps;
    steppers[id].steps_taken = 0;
    steppers[id].target_frequency = homing_speed_steps;
    steppers[id].current_frequency = MIN_FREQUENCY;
    steppers[id].acceleration_time_ms = ACCEL_TIME_MS;
    steppers[id].callback = callback;
    steppers[id].last_update_time = HAL_GetTick();

    // Set direction
    set_direction(id, false);

    // Set state to homing
    steppers[id].state = STEPPER_STATE_HOMING;

    // Start PWM with initial frequency
    float period_us = 1000000.0f / MIN_FREQUENCY;
    BSP_TIMER_set_period_us(steppers[id].timer_id, (uint32_t)period_us);
    BSP_TIMER_enable_PWM(steppers[id].timer_id, steppers[id].timer_channel, PULSE_DUTY_CYCLE, false, false);

    return true;
}

#endif // ENDSTOP_ENABLED

bool STEPPER_test_motor(stepper_id_t id)
{
    if (!is_valid_motor(id))
        return false;

    printf("Testing motor %d with simple movement\n", id);

    // Set direction pin directly
    HAL_GPIO_WritePin(steppers[id].dir_port, steppers[id].dir_pin, GPIO_PIN_SET);
    HAL_Delay(10);

    // FIXED: Convert frequency to period (2000 Hz = 500 us period)
    uint32_t period_us = 500; // For 2000 Hz

    // Set the timer period in microseconds
    BSP_TIMER_set_period_us(steppers[id].timer_id, period_us);

    // Enable PWM with higher duty cycle (80%) for more reliable stepping
    BSP_TIMER_enable_PWM(steppers[id].timer_id, steppers[id].timer_channel, 80, false, false);

    // Let it run for 2 seconds
    printf("Motor should be running now\n");
    HAL_Delay(2000);

    // Stop the motor
    BSP_TIMER_disable_PWM(steppers[id].timer_id, steppers[id].timer_channel);
    printf("Motor should now be stopped\n");

    return true;
}