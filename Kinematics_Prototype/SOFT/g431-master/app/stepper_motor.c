/**
 *******************************************************************************
 * @file    stepper_motor.c
 * @author  Your Name
 * @date    Current Date
 * @brief   Stepper motor control implementation
 *******************************************************************************
 */

#include "config.h"
#include "stepper_motor.h"
#include "stm32g4_utils.h"
#include "stm32g4_gpio.h"
#include "stm32g4_timer.h"
#include "stm32g4_uart.h"
/* System includes - correct order is important */
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
 *    - For NEMA17, 24VDC is typically recommended --> we will use 12VDC
 */

#define ACCEL_PROFILE_ENTRIES 100  // Number of steps in acceleration profile

static bool current_direction[STEPPER_COUNT] = {false};
static void set_direction(stepper_id_t id, bool forward);
uint32_t BSP_GPIO_get_AF_from_timer(timer_id_t timer_id);

// Track and update frequency only when it changes significantly
static stepper_motor_t steppers[STEPPER_COUNT];
static float current_frequencies[STEPPER_COUNT] = {0.0f};

void update_frequency(stepper_id_t id, float new_freq) {
    // Only update if change is more than 1%
    if (fabsf(current_frequencies[id] - new_freq) > (current_frequencies[id] * 0.01f)) {
        BSP_TIMER_set_frequency_Hz(steppers[id].timer_id, new_freq);
        current_frequencies[id] = new_freq;
    }
}

// Use PROGMEM or const to keep strings in flash instead of RAM
static const char MSG_MOTOR_INIT[] = "Stepper motor module initialized\n";

/**
 * Validate that a motor ID is valid and the motor is enabled
 * @param id Motor ID to check
 * @return true if valid, false otherwise
 */
static bool is_valid_motor(stepper_id_t id) {
    if (id >= STEPPER_COUNT || !steppers[id].enabled) {
        printf("Error: Motor %d not enabled or invalid ID\n", id);
        return false;
    }
    return true;
}

void STEPPER_init(void) {
    // Initialize the stepper motor array
    for (int i = 0; i < STEPPER_COUNT; i++) {
        steppers[i].enabled = false;
    }
    
    printf(MSG_MOTOR_INIT);
}

stepper_id_t STEPPER_add(GPIO_TypeDef* step_port, uint16_t step_pin,
                         GPIO_TypeDef* dir_port, uint16_t dir_pin,
                         timer_id_t timer_id, uint32_t timer_channel,
                         float steps_per_mm, uint8_t microstepping, float current_limit_a) {
                         
    // Find an available slot
    for (stepper_id_t id = 0; id < STEPPER_COUNT; id++) {
        if (!steppers[id].enabled) {
            // Configure step pin as timer output (for PWM)
            // Use the helper function to get the correct AF for the timer
            uint32_t timer_af = BSP_GPIO_get_AF_from_timer(timer_id);
            
            BSP_GPIO_pin_config(step_port, step_pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, timer_af);
            
            // Configure direction pin as output
            BSP_GPIO_pin_config(dir_port, dir_pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
            
            // Initialize timer for PWM
            BSP_TIMER_run_us(timer_id, 1000, true); // Initialize with 1kHz
            
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
            steppers[id].microstepping = microstepping;  // Store microstepping setting
            steppers[id].current_limit_a = current_limit_a;  // Store current limit
            
            printf("Added stepper motor %d\n", id);
            
            // After setting up the motor configuration
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

static void set_direction(stepper_id_t id, bool forward) {
    if (current_direction[id] != forward) {
        HAL_GPIO_WritePin(steppers[id].dir_port, steppers[id].dir_pin, 
                          forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
        current_direction[id] = forward;
        
        // Add a small delay after direction changes (TB6600 needs this)
        HAL_Delay(1);
    }
}

float acceleration_profile(float progress) {
    // Simple S-curve using sine function (0 to 1)
    return (1.0f - cosf(progress * M_PI)) / 2.0f;
}

void STEPPER_move_steps(stepper_id_t id, int32_t steps, float speed) {
    printf("STEPPER_move_steps called for motor %d, steps=%d, speed=%.1f\n", id, steps, speed);
    
    if (!is_valid_motor(id)) return;
    
    // Set appropriate speed limits based on microstepping setting
    // TB6600 allows higher speeds with higher microstepping
    float max_speed = 800.0f * steppers[id].microstepping;  // Adjust max speed based on microstepping
    float min_speed = 100.0f;  // Lower limit for smooth starting
    
    // With higher microstepping, we can have a proportionally higher max speed
    printf("Using microstepping 1/%d, max speed adjusted to %.1f Hz\n", 
           steppers[id].microstepping, max_speed);
    
    if (speed > max_speed) {
        printf("Speed limited from %.1f to %.1f Hz\n", speed, max_speed);
        speed = max_speed;
    }
    
    if (speed < min_speed) {
        printf("Speed increased from %.1f to %.1f Hz\n", speed, min_speed);
        speed = min_speed;
    }
    
    bool direction = steps >= 0;
    steps = abs(steps);
    
    // Update position tracking
    steppers[id].current_position += (direction ? steps : -steps);
    
    // Set direction pin
    set_direction(id, direction);
    
    printf("Moving NEMA 17 stepper %d: %d steps at %.1f Hz\n", id, steps, speed);
    
    // NEMA 17 motors generally prefer a lower duty cycle for step pulses
    // 10-20% duty cycle often works better than 50%
    BSP_TIMER_enable_PWM(steppers[id].timer_id, steppers[id].timer_channel, 20, false, false);
    
    // Acceleration ramp for NEMA 17 motors
    // Start at a lower speed and gradually increase to target speed
    float current_speed = min_speed;
    update_frequency(id, current_speed);
    
    // Calculate steps for acceleration and deceleration
    float total_accel_time_ms = 300.0f; // Configurable accel time
    float progress;
    for (int i = 0; i < total_accel_time_ms; i += 10) {
        HAL_Delay(10);
        progress = i / total_accel_time_ms;
        current_speed = min_speed + (speed - min_speed) * acceleration_profile(progress);
        update_frequency(id, current_speed);
    }
    
    // Calculate remaining steps at full speed
    int32_t interval_steps = (int32_t)((current_speed * 0.01f) + 0.5f); // exact steps in 10ms
    int32_t const_speed_steps = steps - (2 * (int32_t)(total_accel_time_ms * speed / 1000.0f));
    if (const_speed_steps > 0) {
        // Run at constant speed
        uint32_t const_time_ms = (uint32_t)(const_speed_steps * 1000.0f / speed);
        HAL_Delay(const_time_ms);
    }
    
    // Deceleration phase
    for (int i = total_accel_time_ms; i > 0; i -= 10) {
        HAL_Delay(10);
        progress = i / total_accel_time_ms;
        current_speed = min_speed + (speed - min_speed) * acceleration_profile(progress);
        update_frequency(id, current_speed);
    }
    
    // Run any remaining steps at min_speed
    int32_t remaining_steps = steps - (int32_t)(total_accel_time_ms * speed / 1000.0f) * 2;
    if (remaining_steps > 0) {
        uint32_t remaining_time_ms = (uint32_t)(remaining_steps * 1000.0f / min_speed);
        HAL_Delay(remaining_time_ms);
    }
    
    // Stop PWM
    STEPPER_stop(id);
    printf("NEMA 17 movement complete\n");
}

void STEPPER_move_to_mm(stepper_id_t id, float position_mm, float speed_mm_s) {
    if (!is_valid_motor(id)) return;
    
    // Calculate current position in mm
    float current_mm = (float)steppers[id].current_position / steppers[id].steps_per_mm;
    
    // Calculate required movement
    float move_mm = position_mm - current_mm;
    
    // Convert to steps and move
    int32_t steps = (int32_t)(move_mm * steppers[id].steps_per_mm);
    float step_freq = speed_mm_s * steppers[id].steps_per_mm;
    
    STEPPER_move_steps(id, steps, step_freq);
}

void STEPPER_move_mm(stepper_id_t id, float distance_mm, float speed_mm_s) {
    if (!is_valid_motor(id)) return;
    
    // Convert to steps and move
    int32_t steps = (int32_t)(distance_mm * steppers[id].steps_per_mm);
    float step_freq = speed_mm_s * steppers[id].steps_per_mm;
    
    STEPPER_move_steps(id, steps, step_freq);
}

void STEPPER_stop(stepper_id_t id) {
    if (!is_valid_motor(id)) return;
    
    BSP_TIMER_disable_PWM(steppers[id].timer_id, steppers[id].timer_channel);
}

/**
 * Get position of stepper motor in millimeters
 */
float STEPPER_get_position_mm(stepper_id_t id) {
    if (!is_valid_motor(id)) return 0.0f;
    return (float)steppers[id].current_position / steppers[id].steps_per_mm;
}

/**
 * Get steps per mm value for a motor
 */
float STEPPER_get_steps_per_mm(stepper_id_t id) {
    if (!is_valid_motor(id)) return 0.0f;
    return steppers[id].steps_per_mm;
}

void STEPPER_home(stepper_id_t id) {
    if (!is_valid_motor(id)) return;
    
    // This would require limit switches
    // For now, just reset the position counter
    steppers[id].current_position = 0;
}

uint32_t BSP_GPIO_get_AF_from_timer(timer_id_t timer_id) {
    // STM32G4 alternate function mapping for timers
    switch (timer_id) {
        case TIMER1_ID:
            return GPIO_AF4_TIM1;  // TIM1 on AF4
        case TIMER2_ID:
            return GPIO_AF1_TIM2;  // TIM2 on AF1
        case TIMER3_ID:
            return GPIO_AF2_TIM3;  // TIM3 on AF2
        case TIMER4_ID:
            return GPIO_AF2_TIM4;  // TIM4 on AF2
        case TIMER6_ID:
            return GPIO_AF2_TIM15; // TIM15 on AF2
        // Add other timers as needed
        default:
            return GPIO_AF2_TIM3;  // Default to TIM3
    }
}