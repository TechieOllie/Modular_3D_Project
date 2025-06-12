/**
 *******************************************************************************
 * @file    stepper_motor.h
 * @author  GitHub Copilot
 * @date    May 27, 2025
 * @brief   Stepper motor control with timer interrupts and acceleration profiles
 *******************************************************************************
 */

#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32g4_utils.h"
#include "stm32g4_gpio.h"
#include "stm32g4_timer.h"
#include "limit_switches.h"

/* Defines -------------------------------------------------------------------*/
#define STEPPER_MAX_MOTORS 4           // Maximum number of motors
#define STEPPER_MIN_FREQUENCY 6000     // Minimum 6kHz
#define STEPPER_MAX_FREQUENCY 12000    // Maximum 12kHz
#define STEPPER_DEFAULT_FREQUENCY 8000 // Default 8kHz
#define STEPPER_ACCEL_UPDATE_FREQ 1000 // Acceleration update frequency (Hz)

/* Public types --------------------------------------------------------------*/

/**
 * @brief Stepper motor direction
 */
typedef enum
{
    MOTOR_DIR_CLOCKWISE = 0,
    MOTOR_DIR_COUNTERCLOCKWISE = 1
} stepper_motor_dir_t;

/**
 * @brief Stepper motor state
 */
typedef enum
{
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_ACCELERATING,
    MOTOR_STATE_MOVING,
    MOTOR_STATE_DECELERATING,
    MOTOR_STATE_ERROR,
    MOTOR_STATE_HOMING
} stepper_motor_state_t;

/**
 * @brief Acceleration profile structure
 */
typedef struct
{
    uint32_t start_frequency;  // Starting frequency (Hz)
    uint32_t target_frequency; // Target frequency (Hz)
    uint32_t max_frequency;    // Maximum allowed frequency (Hz)
    uint32_t acceleration;     // Acceleration in steps/sec²
    uint32_t deceleration;     // Deceleration in steps/sec²
    bool use_acceleration;     // Enable/disable acceleration
} stepper_accel_profile_t;

/**
 * @brief Stepper motor configuration structure
 */
typedef struct
{
    // GPIO configuration
    GPIO_TypeDef *pulse_port;
    uint16_t pulse_pin;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;

    // Timer configuration
    timer_id_t timer_id;
    uint16_t timer_channel;

    // Motor parameters
    uint16_t microsteps;
    stepper_accel_profile_t accel_profile;

    // Limit switch integration
    axis_t axis;
    bool limit_check_enabled;

    // State
    stepper_motor_state_t state;
    stepper_motor_dir_t direction;

    // Movement tracking
    uint32_t target_steps;
    uint32_t completed_steps;
    uint32_t current_frequency;

    // Acceleration tracking
    uint32_t accel_steps; // Steps for acceleration phase
    uint32_t decel_steps; // Steps for deceleration phase
    uint32_t const_steps; // Steps at constant speed

    bool initialized;
} stepper_motor_t;

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief Initialize stepper motor system
 */
bool stepper_motor_system_init(void);

/**
 * @brief Initialize a stepper motor
 * @param motor_id Motor ID (0 to STEPPER_MAX_MOTORS-1)
 * @param pulse_port GPIO port for pulse pin
 * @param pulse_pin GPIO pin for pulse signal
 * @param dir_port GPIO port for direction pin
 * @param dir_pin GPIO pin for direction signal
 * @param timer_id Timer to use for pulse generation
 * @param timer_channel Timer channel for PWM
 * @param microsteps Microstepping setting
 * @return true if successful
 */
bool stepper_motor_init(uint8_t motor_id,
                        GPIO_TypeDef *pulse_port, uint16_t pulse_pin,
                        GPIO_TypeDef *dir_port, uint16_t dir_pin,
                        timer_id_t timer_id, uint16_t timer_channel,
                        uint16_t microsteps);

/**
 * @brief Set acceleration profile for a motor
 * @param motor_id Motor ID
 * @param acceleration Acceleration in steps/sec²
 * @param max_frequency Maximum frequency in Hz
 * @return true if successful
 */
bool stepper_motor_set_acceleration(uint8_t motor_id, uint32_t acceleration, uint32_t max_frequency);

/**
 * @brief Enable/disable acceleration for a motor
 * @param motor_id Motor ID
 * @param enable true to enable acceleration
 * @return true if successful
 */
bool stepper_motor_enable_acceleration(uint8_t motor_id, bool enable);

/**
 * @brief Move motor with acceleration profile
 * @param motor_id Motor ID
 * @param steps Number of steps to move
 * @param target_frequency Target frequency in Hz
 * @param direction Direction to move
 * @return true if movement started successfully
 */
bool stepper_motor_move_accel(uint8_t motor_id, uint32_t steps, uint32_t target_frequency, stepper_motor_dir_t direction);

/**
 * @brief Move motor at constant speed (legacy compatibility)
 * @param motor_id Motor ID
 * @param steps Number of steps to move
 * @param frequency Frequency in Hz
 * @param direction Direction to move
 * @return true if movement started successfully
 */
bool stepper_motor_move(uint8_t motor_id, uint32_t steps, uint32_t frequency, stepper_motor_dir_t direction);

/**
 * @brief Stop motor immediately
 * @param motor_id Motor ID
 * @return true if successful
 */
bool stepper_motor_stop(uint8_t motor_id);

/**
 * @brief Emergency stop all motors
 */
void stepper_motor_emergency_stop_all(void);

/**
 * @brief Get motor state
 * @param motor_id Motor ID
 * @return Current motor state
 */
stepper_motor_state_t stepper_motor_get_state(uint8_t motor_id);

/**
 * @brief Get completed steps
 * @param motor_id Motor ID
 * @return Number of completed steps
 */
uint32_t stepper_motor_get_completed_steps(uint8_t motor_id);

/**
 * @brief Get current frequency
 * @param motor_id Motor ID
 * @return Current frequency in Hz
 */
uint32_t stepper_motor_get_current_frequency(uint8_t motor_id);

/**
 * @brief Update stepper motor system - call in main loop
 */
void stepper_motor_update(void);

/**
 * @brief Associate motor with axis for limit switch checking
 * @param motor_id Motor ID
 * @param axis Axis (X, Y, Z)
 */
void stepper_motor_set_axis(uint8_t motor_id, axis_t axis);

/**
 * @brief Enable/disable limit switch checking for motor
 * @param motor_id Motor ID
 * @param enable true to enable limit checking
 */
void stepper_motor_enable_limit_check(uint8_t motor_id, bool enable);

/**
 * @brief Home motor (move until limit switch triggered)
 * @param motor_id Motor ID
 * @param frequency Homing frequency in Hz
 * @return true if homing started successfully
 */
bool stepper_motor_home(uint8_t motor_id, uint32_t frequency);

/**
 * @brief Move motor with limit checking
 * @param motor_id Motor ID
 * @param steps Number of steps
 * @param frequency Frequency in Hz
 * @param direction Direction
 * @return true if movement started successfully
 */
bool stepper_motor_move_with_limits(uint8_t motor_id, uint32_t steps, uint32_t frequency, stepper_motor_dir_t direction);

/**
 * @brief Manual step for testing
 * @param motor_id Motor ID
 * @param steps Number of steps
 * @param delay_ms Delay between steps in ms
 */
void stepper_motor_manual_step(uint8_t motor_id, uint32_t steps, uint32_t delay_ms);

/**
 * @brief Debug motor configuration
 * @param motor_id Motor ID
 */
void stepper_motor_debug_config(uint8_t motor_id);

/**
 * @brief Timer interrupt handler for motor pulse generation
 * @param timer_id Timer that triggered the interrupt
 */
void stepper_motor_timer_interrupt(timer_id_t timer_id);

#endif /* STEPPER_MOTOR_H */
