/**
 *******************************************************************************
 * @file    stepper_motor.h
 * @author  Your Name
 * @date    Current Date
 * @brief   Advanced stepper motor control with state machine and timer interrupts
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

// Movement state machine states
typedef enum
{
    STEPPER_STATE_IDLE = 0,       // Motor is idle
    STEPPER_STATE_ACCELERATING,   // Motor is accelerating
    STEPPER_STATE_CONSTANT_SPEED, // Motor is running at constant speed
    STEPPER_STATE_DECELERATING,   // Motor is decelerating
    STEPPER_STATE_HOMING          // Motor is performing homing operation
} stepper_state_t;

// Movement callback function type
typedef void (*stepper_callback_t)(stepper_id_t id, bool success);

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
 * Move stepper motor a given number of steps with callback on completion
 * @param id Motor ID
 * @param steps Number of steps (positive=forward, negative=backward)
 * @param speed Speed in steps per second
 * @param callback Function to call when movement completes (can be NULL)
 * @return true if movement started, false if error
 */
bool STEPPER_move_steps(stepper_id_t id, int32_t steps, float speed, stepper_callback_t callback);

/**
 * Move stepper motor to a position in millimeters with callback on completion
 * @param id Motor ID
 * @param position_mm Target position in millimeters
 * @param speed_mm_s Speed in millimeters per second
 * @param callback Function to call when movement completes (can be NULL)
 * @return true if movement started, false if error
 */
bool STEPPER_move_to_mm(stepper_id_t id, float position_mm, float speed_mm_s, stepper_callback_t callback);

/**
 * Move stepper motor a given distance in millimeters with callback on completion
 * @param id Motor ID
 * @param distance_mm Distance to move in millimeters
 * @param speed_mm_s Speed in millimeters per second
 * @param callback Function to call when movement completes (can be NULL)
 * @return true if movement started, false if error
 */
bool STEPPER_move_mm(stepper_id_t id, float distance_mm, float speed_mm_s, stepper_callback_t callback);

/**
 * Stop a stepper motor
 * @param id Motor ID
 */
void STEPPER_stop(stepper_id_t id);

/**
 * Check if stepper motor is currently moving
 * @param id Motor ID
 * @return true if motor is moving, false otherwise
 */
bool STEPPER_is_moving(stepper_id_t id);

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
 * @param callback Function to call when homing completes (can be NULL)
 * @return true if homing started, false if error
 */
bool STEPPER_home(stepper_id_t id, stepper_callback_t callback);

/**
 * Process stepper motor state machines - call this in main loop
 * This function will update acceleration profiles and state transitions
 * The actual stepping is handled by timer interrupts
 */
void STEPPER_process(void);

/**
 * Timer interrupt handlers for each motor
 * These are called by the timer ISRs
 */
void TIMER1_user_handler_it(void);
void TIMER2_user_handler_it(void);
void TIMER3_user_handler_it(void);
void TIMER4_user_handler_it(void);

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
                                         GPIO_TypeDef *port, uint16_t pin, uint8_t trigger_level);

/**
 * Check if an endstop is currently triggered
 * @param id Motor ID
 * @param endstop_type Type of endstop to check
 * @return true if triggered, false otherwise
 */
bool STEPPER_is_endstop_hit(stepper_id_t id, endstop_type_t endstop_type);

/**
 * Home a stepper motor using endstop
 * @param id Motor ID
 * @param homing_speed_mm_s Speed for homing in mm/s
 * @param max_travel_mm Maximum travel distance for homing
 * @param callback Function to call when homing completes (can be NULL)
 * @return true if homing started, false if error
 */
bool STEPPER_home_with_endstop(stepper_id_t id, float homing_speed_mm_s, float max_travel_mm, stepper_callback_t callback);
#endif

bool STEPPER_test_motor(stepper_id_t id);

#endif /* STEPPER_MOTOR_H_ */