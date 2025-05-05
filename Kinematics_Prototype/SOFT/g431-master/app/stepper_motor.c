/**
 *******************************************************************************
 * @file    stepper_motor.c
 * @author  Your Name
 * @date    Current Date
 * @brief   Stepper motor control implementation
 *******************************************************************************
 */

#include "stepper_motor.h"
#include "stm32g4_utils.h"
#include <stdio.h>

static stepper_motor_t steppers[STEPPER_COUNT];

void STEPPER_init(void) {
    // Initialize the stepper motor array
    for (int i = 0; i < STEPPER_COUNT; i++) {
        steppers[i].enabled = false;
    }
    
    printf("Stepper motor module initialized\n");
}

stepper_id_t STEPPER_add(GPIO_TypeDef* step_port, uint16_t step_pin,
                         GPIO_TypeDef* dir_port, uint16_t dir_pin,
                         timer_id_t timer_id, uint32_t timer_channel,
                         float steps_per_mm) {
                         
    // Find an available slot
    for (stepper_id_t id = 0; id < STEPPER_COUNT; id++) {
        if (!steppers[id].enabled) {
            // Configure step pin as timer output (for PWM)
            uint32_t timer_af;
switch (timer_id) {
    case TIMER1_ID:
        timer_af = GPIO_AF4_TIM1;
        break;
    case TIMER2_ID:
        timer_af = GPIO_AF1_TIM2;
        break;
    case TIMER3_ID:
        timer_af = GPIO_AF2_TIM3;
        break;
    // Add other timers as needed
    default:
        timer_af = GPIO_AF2_TIM3;
        break;
}

BSP_GPIO_pin_config(step_port, step_pin, GPIO_MODE_AF_PP, 
                   GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, timer_af);
            
            // Configure direction pin as output
            BSP_GPIO_pin_config(dir_port, dir_pin, GPIO_MODE_OUTPUT_PP,
                               GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
            
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
            
            printf("Added stepper motor %d\n", id);
            
            // After setting up the motor configuration
            printf("Motor ID %d initialized successfully on pins: Step=%d, Dir=%d\n", 
                   id, step_pin, dir_pin);
                   
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
    HAL_GPIO_WritePin(steppers[id].dir_port, steppers[id].dir_pin, 
                     forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void STEPPER_move_steps(stepper_id_t id, int32_t steps, float speed) {
    printf("STEPPER_move_steps called for motor %d, steps=%d, speed=%.1f\n", id, steps, speed);
    
    if (!steppers[id].enabled || id >= STEPPER_COUNT) {
        printf("Error: Motor %d not enabled or invalid ID\n", id);
        return;
    }
    
    // Set appropriate speed limits for NEMA 17
    // Most NEMA 17 steppers operate well between 100-800 Hz without microstepping
    float max_speed = 800.0f; // Upper limit for NEMA 17 speed (adjust based on your specific motors)
    float min_speed = 100.0f; // Lower limit for smooth starting
    
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
    BSP_TIMER_set_frequency_Hz(steppers[id].timer_id, current_speed);
    
    // Calculate steps for acceleration and deceleration
    float accel_rate = 80.0f; // Hz per 10ms (adjust for smoother/faster accel)
    int32_t total_accel_steps = 0;
    
    // Acceleration phase
    while (current_speed < speed && total_accel_steps < steps/2) {
        HAL_Delay(10);
        current_speed += accel_rate;
        if (current_speed > speed) current_speed = speed;
        BSP_TIMER_set_frequency_Hz(steppers[id].timer_id, current_speed);
        
        // Calculate steps taken during this interval
        int32_t interval_steps = current_speed / 100; // steps in 10ms
        total_accel_steps += interval_steps;
    }
    
    // Calculate remaining steps at full speed
    int32_t const_speed_steps = steps - (2 * total_accel_steps);
    if (const_speed_steps > 0) {
        // Run at constant speed
        uint32_t const_time_ms = (uint32_t)(const_speed_steps * 1000.0f / speed);
        HAL_Delay(const_time_ms);
    }
    
    // Deceleration phase
    while (current_speed > min_speed && total_accel_steps > 0) {
        HAL_Delay(10);
        current_speed -= accel_rate;
        if (current_speed < min_speed) current_speed = min_speed;
        BSP_TIMER_set_frequency_Hz(steppers[id].timer_id, current_speed);
        
        // Calculate steps taken during this interval
        int32_t interval_steps = current_speed / 100; // steps in 10ms
        total_accel_steps -= interval_steps;
    }
    
    // Run any remaining steps at min_speed
    if (total_accel_steps > 0) {
        uint32_t remaining_time_ms = (uint32_t)(total_accel_steps * 1000.0f / min_speed);
        HAL_Delay(remaining_time_ms);
    }
    
    // Stop PWM
    STEPPER_stop(id);
    printf("NEMA 17 movement complete\n");
}

void STEPPER_move_to_mm(stepper_id_t id, float position_mm, float speed_mm_s) {
    if (!steppers[id].enabled || id >= STEPPER_COUNT) {
        return;
    }
    
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
    if (!steppers[id].enabled || id >= STEPPER_COUNT) {
        return;
    }
    
    // Convert to steps and move
    int32_t steps = (int32_t)(distance_mm * steppers[id].steps_per_mm);
    float step_freq = speed_mm_s * steppers[id].steps_per_mm;
    
    STEPPER_move_steps(id, steps, step_freq);
}

void STEPPER_stop(stepper_id_t id) {
    if (id >= STEPPER_COUNT) {
        return;
    }
    
    BSP_TIMER_disable_PWM(steppers[id].timer_id, steppers[id].timer_channel);
}

float STEPPER_get_position_mm(stepper_id_t id) {
    if (id >= STEPPER_COUNT) {
        return 0.0f;
    }
    
    return (float)steppers[id].current_position / steppers[id].steps_per_mm;
}

void STEPPER_home(stepper_id_t id) {
    // This would require limit switches
    // For now, just reset the position counter
    if (id < STEPPER_COUNT) {
        steppers[id].current_position = 0;
    }
}

uint32_t BSP_GPIO_get_AF_from_timer(timer_id_t timer_id) {
    // STM32G4 alternate function mapping for timers
    switch (timer_id) {
        case TIMER1_ID:
            return GPIO_AF4_TIM1;  // TIM1 is usually on AF1
        case TIMER2_ID:
            return GPIO_AF1_TIM2;  // TIM2 is usually on AF1
        case TIMER3_ID:
            return GPIO_AF2_TIM3;  // TIM3 is usually on AF2
        case TIMER4_ID:
            return GPIO_AF2_TIM4;  // TIM4 is usually on AF2
        case TIMER6_ID:
            return GPIO_AF2_TIM15;  // TIM5 is usually on AF2
        default:
            return GPIO_AF2_TIM3;  // Default to TIM3
    }
}

void STEPPER_test_nema17(stepper_id_t id) {
    if (!steppers[id].enabled || id >= STEPPER_COUNT) {
        printf("Invalid stepper motor ID\n");
        return;
    }
    
    printf("\nNEMA 17 Test Sequence Starting\n");
    
    // Test different speeds
    float test_speeds[] = {100, 200, 400, 600, 800};
    int steps_per_test = 200; // One full revolution for a typical NEMA 17 (1.8° step)
    
    // Forward tests
    set_direction(id, true);
    printf("Forward direction tests:\n");
    
    for (int i = 0; i < sizeof(test_speeds)/sizeof(test_speeds[0]); i++) {
        float speed = test_speeds[i];
        printf("Testing %d steps at %.1f Hz... ", steps_per_test, speed);
        
        // Use lower duty cycle for NEMA 17 (20% instead of 50%)
        BSP_TIMER_enable_PWM(steppers[id].timer_id, steppers[id].timer_channel, 20, false, false);
        BSP_TIMER_set_frequency_Hz(steppers[id].timer_id, speed);
        
        HAL_Delay((steps_per_test * 1000) / speed);
        STEPPER_stop(id);
        printf("done\n");
        
        // Pause between tests
        HAL_Delay(500);
    }
    
    // Reverse tests
    set_direction(id, false);
    printf("Reverse direction tests:\n");
    
    for (int i = 0; i < sizeof(test_speeds)/sizeof(test_speeds[0]); i++) {
        float speed = test_speeds[i];
        printf("Testing %d steps at %.1f Hz... ", steps_per_test, speed);
        
        BSP_TIMER_enable_PWM(steppers[id].timer_id, steppers[id].timer_channel, 20, false, false);
        BSP_TIMER_set_frequency_Hz(steppers[id].timer_id, speed);
        
        HAL_Delay((steps_per_test * 1000) / speed);
        STEPPER_stop(id);
        printf("done\n");
        
        // Pause between tests
        HAL_Delay(500);
    }
    
    // Test full revolution with acceleration
    printf("Testing full revolution with acceleration...\n");
    STEPPER_move_steps(id, steps_per_test, 400);
    
    printf("NEMA 17 test complete\n");
}
