/**
 *******************************************************************************
 * @file    stepper_motor.h
 * @author  Your Name
 * @date    Current Date
 * @brief   Stepper motor control for CNC applications
 *******************************************************************************
 */

#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "stm32g4xx_hal.h"
#include "stm32g4_timer.h"
#include "stm32g4_gpio.h"
#include <stdbool.h>

typedef enum {
    STEPPER_X,
    STEPPER_COUNT
} stepper_id_t;

// Stepper motor configuration
typedef struct {
    // Step pin
    GPIO_TypeDef* step_port;
    uint16_t step_pin;
    timer_id_t timer_id;
    uint32_t timer_channel;
    
    // Direction pin
    GPIO_TypeDef* dir_port;
    uint16_t dir_pin;
    
    // Motor characteristics
    float steps_per_mm;  // Calibration value (steps needed for 1mm movement)
    float max_speed;     // Maximum speed in mm/s
    
    // Current position in steps
    int32_t current_position;
    
    // Enabled state
    bool enabled;
} stepper_motor_t;

// Initialize the stepper motors
void STEPPER_init(void);

// Add a stepper motor to the system
stepper_id_t STEPPER_add(GPIO_TypeDef* step_port, uint16_t step_pin,
                         GPIO_TypeDef* dir_port, uint16_t dir_pin,
                         timer_id_t timer_id, uint32_t timer_channel,
                         float steps_per_mm);

// Move a stepper motor a specific number of steps
void STEPPER_move_steps(stepper_id_t id, int32_t steps, float speed);

// Move a stepper motor to an absolute position in mm
void STEPPER_move_to_mm(stepper_id_t id, float position_mm, float speed_mm_s);

// Move a stepper motor a relative distance in mm
void STEPPER_move_mm(stepper_id_t id, float distance_mm, float speed_mm_s);

// Stop a stepper motor
void STEPPER_stop(stepper_id_t id);

// Get the current position of a stepper motor in mm
float STEPPER_get_position_mm(stepper_id_t id);

// Home a stepper motor (if limit switches are installed)
void STEPPER_home(stepper_id_t id);

// Add this prototype
void STEPPER_test_nema17(stepper_id_t id);

#endif /* STEPPER_MOTOR_H */
