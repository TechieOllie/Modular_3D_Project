/**
 *******************************************************************************
 * @file    stepper_motor.h
 * @author  Your Name
 * @date    Current Date
 * @brief   Stepper motor control header
 *******************************************************************************
 */

#ifndef STEPPER_MOTOR_H_
#define STEPPER_MOTOR_H_

#include "stm32g4xx_hal.h"
#include "stm32g4_timer.h"

// If using endstops, set to 1
#define ENDSTOP_ENABLED 0

// Endstop configuration when enabled
#if ENDSTOP_ENABLED
// Endstop directions
typedef enum
{
    ENDSTOP_MIN = 0, // Min position endstop (home position)
    ENDSTOP_MAX = 1  // Max position endstop
} endstop_type_t;

// Endstop trigger logic
#define ENDSTOP_TRIGGER_HIGH 1 // Endstop triggers on HIGH signal
#define ENDSTOP_TRIGGER_LOW 0  // Endstop triggers on LOW signal
#endif

// Maximum number of stepper motors that can be controlled
#define STEPPER_COUNT 4

// Stepper motor ID type
typedef int8_t stepper_id_t;

// Stepper motor structure
typedef struct
{
    GPIO_TypeDef *step_port;
    uint16_t step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    timer_id_t timer_id;
    uint32_t timer_channel;
    float steps_per_mm;
    int32_t current_position;
    bool enabled;
    uint8_t microstepping; // Microstepping setting (1, 2, 4, 8, 16, etc)
    float current_limit_a; // Current limit in amperes

#if ENDSTOP_ENABLED
    // Endstop configuration
    GPIO_TypeDef *min_endstop_port;
    uint16_t min_endstop_pin;
    GPIO_TypeDef *max_endstop_port;
    uint16_t max_endstop_pin;
    uint8_t endstop_trigger_level; // ENDSTOP_TRIGGER_HIGH or ENDSTOP_TRIGGER_LOW
    bool min_endstop_enabled;
    bool max_endstop_enabled;
#endif
} stepper_motor_t;

/**
 * Initialize the stepper motor module
 */
void STEPPER_init(void);

/**
 * Add a stepper motor
 * @param step_port GPIO port for the step pin
 * @param step_pin GPIO pin for the step pin
 * @param dir_port GPIO port for the direction pin
 * @param dir_pin GPIO pin for the direction pin
 * @param timer_id Timer ID for PWM generation
 * @param timer_channel Timer channel for PWM generation
 * @param steps_per_mm Steps per millimeter
 * @param microstepping Microstepping setting (1, 2, 4, 8, 16, etc)
 * @param current_limit_a Current limit in amperes
 * @return Motor ID or -1 if error
 */
stepper_id_t STEPPER_add(GPIO_TypeDef *step_port, uint16_t step_pin,
                         GPIO_TypeDef *dir_port, uint16_t dir_pin,
                         timer_id_t timer_id, uint32_t timer_channel,
                         float steps_per_mm, uint8_t microstepping, float current_limit_a);

/**
 * Move stepper motor a given number of steps
 * @param id Motor ID
 * @param steps Number of steps (positive=forward, negative=backward)
 * @param speed Speed in steps per second
 */
void STEPPER_move_steps(stepper_id_t id, int32_t steps, float speed);

/**
 * Move stepper motor to a position in millimeters
 * @param id Motor ID
 * @param position_mm Target position in millimeters
 * @param speed_mm_s Speed in millimeters per second
 */
void STEPPER_move_to_mm(stepper_id_t id, float position_mm, float speed_mm_s);

/**
 * Move stepper motor a given distance in millimeters
 * @param id Motor ID
 * @param distance_mm Distance to move in millimeters
 * @param speed_mm_s Speed in millimeters per second
 */
void STEPPER_move_mm(stepper_id_t id, float distance_mm, float speed_mm_s);

/**
 * Stop a stepper motor
 * @param id Motor ID
 */
void STEPPER_stop(stepper_id_t id);

/**
 * Get the current position of a stepper motor in millimeters
 * @param id Motor ID
 * @return Position in millimeters
 */
float STEPPER_get_position_mm(stepper_id_t id);

/**
 * Get the steps per mm value for the specified motor
 * @param id Motor ID
 * @return Steps per mm value
 */
float STEPPER_get_steps_per_mm(stepper_id_t id);

/**
 * Home a stepper motor (move to zero position)
 * @param id Motor ID
 */
void STEPPER_home(stepper_id_t id);

#if ENDSTOP_ENABLED
/**
 * Configure endstop for a stepper motor
 * @param id Motor ID
 * @param endstop_type Type of endstop (ENDSTOP_MIN or ENDSTOP_MAX)
 * @param port GPIO port for the endstop pin
 * @param pin GPIO pin for the endstop
 * @param trigger_level Logic level that triggers the endstop (ENDSTOP_TRIGGER_HIGH or ENDSTOP_TRIGGER_LOW)
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef STEPPER_config_endstop(stepper_id_t id, endstop_type_t endstop_type,
                                         GPIO_TypeDef *port, uint16_t pin,
                                         uint8_t trigger_level);

/**
 * Check if an endstop is triggered
 * @param id Motor ID
 * @param endstop_type Type of endstop (ENDSTOP_MIN or ENDSTOP_MAX)
 * @return true if endstop is triggered, false otherwise
 */
bool STEPPER_is_endstop_hit(stepper_id_t id, endstop_type_t endstop_type);

/**
 * Home a stepper motor using the min endstop
 * @param id Motor ID
 * @param homing_speed_mm_s Speed for homing in mm/s
 * @param max_travel_mm Maximum travel distance in mm (safety limit)
 * @return true if homing successful, false if failed (no endstop hit or other error)
 */
bool STEPPER_home_with_endstop(stepper_id_t id, float homing_speed_mm_s, float max_travel_mm);
#endif

#endif /* STEPPER_MOTOR_H_ */
