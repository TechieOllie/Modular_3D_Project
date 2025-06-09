/**
 *******************************************************************************
 * @file    stepper_motor.h
 * @author  GitHub Copilot
 * @date    May 27, 2025
 * @brief   Stepper motor control using TB6600 driver with PWM signals
 *******************************************************************************
 */

#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32g4_utils.h"
#include "stm32g4_timer.h"

/* Defines -------------------------------------------------------------------*/
#define STEPPER_MAX_MOTORS 4
#define STEPPER_MICROSTEPS_DEFAULT 16

/* Public types --------------------------------------------------------------*/

/**
 * @brief Motor direction enumeration
 */
typedef enum
{
    MOTOR_DIR_CLOCKWISE = 0,
    MOTOR_DIR_COUNTERCLOCKWISE = 1
} stepper_motor_dir_t;

/**
 * @brief Motor state enumeration
 */
typedef enum
{
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_MOVING = 1,
    MOTOR_STATE_ERROR = 2
} stepper_motor_state_t;

/**
 * @brief Stepper motor configuration structure
 */
typedef struct
{
    // GPIO pins
    GPIO_TypeDef *pul_gpio;
    uint16_t pul_pin;
    GPIO_TypeDef *dir_gpio;
    uint16_t dir_pin;

    // Timer configuration
    timer_id_t timer_id;
    uint16_t timer_channel;

    // Motor parameters
    uint16_t microsteps;
    uint32_t step_delay_us;

    // Movement state
    stepper_motor_state_t state;
    stepper_motor_dir_t direction;
    uint32_t steps_to_move;
    uint32_t steps_moved;

    // Initialization flag
    bool initialized;

    // Safety features
    bool limit_check_enabled;
    uint8_t associated_axis; // For limit switch checking (0=X, 1=Y, 2=Z, 255=none)

} stepper_motor_t;

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief Initialize a stepper motor with specified parameters
 * @param motor_id Unique motor identifier (0 to STEPPER_MAX_MOTORS-1)
 * @param pul_gpio GPIO port for PUL pin
 * @param pul_pin GPIO pin number for PUL pin
 * @param dir_gpio GPIO port for DIR pin
 * @param dir_pin GPIO pin number for DIR pin
 * @param timer_id Timer to use for PWM generation
 * @param timer_channel Timer channel to use
 * @param microsteps Microstepping setting (8, 16, or 32)
 * @return true if initialization successful
 */
bool stepper_motor_init(uint8_t motor_id,
                        GPIO_TypeDef *pul_gpio, uint16_t pul_pin,
                        GPIO_TypeDef *dir_gpio, uint16_t dir_pin,
                        timer_id_t timer_id, uint16_t timer_channel,
                        uint16_t microsteps);

/**
 * @brief Associate a motor with an axis for limit switch checking
 * @param motor_id Motor identifier
 * @param axis_id Axis identifier (0=X, 1=Y, 2=Z, 255=none)
 * @return true if successful
 */
bool stepper_motor_set_axis(uint8_t motor_id, uint8_t axis_id);

/**
 * @brief Enable/disable limit switch checking for a motor
 * @param motor_id Motor identifier
 * @param enable true to enable limit checking
 * @return true if successful
 */
bool stepper_motor_enable_limit_check(uint8_t motor_id, bool enable);

/**
 * @brief Start motor movement with specified number of steps, speed, and direction
 * @param motor_id Motor identifier
 * @param steps Number of steps to move
 * @param speed_steps_per_second Speed in steps per second
 * @param direction Direction to move
 * @return true if movement started successfully
 */
bool stepper_motor_move(uint8_t motor_id, uint32_t steps, uint32_t speed_steps_per_second,
                        stepper_motor_dir_t direction);

/**
 * @brief Start motor movement with limit switch checking
 * @param motor_id Motor identifier
 * @param steps Number of steps to move (0 = move until limit hit)
 * @param speed_steps_per_second Speed in steps per second
 * @param direction Direction to move
 * @return true if movement started successfully
 */
bool stepper_motor_move_with_limits(uint8_t motor_id, uint32_t steps,
                                    uint32_t speed_steps_per_second,
                                    stepper_motor_dir_t direction);

/**
 * @brief Home a motor by moving towards minimum limit switch
 * @param motor_id Motor identifier
 * @param speed_steps_per_second Homing speed
 * @return true if homing started successfully
 */
bool stepper_motor_home(uint8_t motor_id, uint32_t speed_steps_per_second);

/**
 * @brief Set motor direction
 *
 * @param motor_id       ID of the motor
 * @param direction      Direction (MOTOR_DIR_CLOCKWISE or MOTOR_DIR_COUNTERCLOCKWISE)
 * @return bool          True if successful, false otherwise
 */
bool stepper_motor_set_direction(uint8_t motor_id, stepper_motor_dir_t direction);

/**
 * @brief Stop a motor
 *
 * @param motor_id       ID of the motor to stop
 * @return bool          True if stop successful, false otherwise
 */
bool stepper_motor_stop(uint8_t motor_id);

/**
 * @brief Get motor state
 *
 * @param motor_id       ID of the motor
 * @return stepper_motor_state_t Current state of the motor
 */
stepper_motor_state_t stepper_motor_get_state(uint8_t motor_id);

/**
 * @brief Update motor state - should be called periodically in main loop
 */
void stepper_motor_update(void);

/**
 * @brief Handler for timer interrupt - must be implemented in application
 *
 * @param timer_id Timer ID that triggered the interrupt
 */
void stepper_motor_timer_handler(timer_id_t timer_id);

void stepper_motor_manual_step(uint8_t motor_id, uint32_t steps, uint32_t delay_ms);

void stepper_motor_speed_test(uint8_t motor_id);

void stepper_motor_find_max_speed(uint8_t motor_id);

/**
 * @brief Reset a stepper motor's position and state
 * @param motor_id Motor identifier
 * @return true if successful
 */
bool stepper_motor_reset(uint8_t motor_id);

/**
 * @brief Start non-blocking motor movement
 * @param motor_id Motor identifier
 * @param steps Number of steps to move
 * @param speed_steps_per_second Speed in steps per second
 * @param direction Direction to move
 * @return true if movement started successfully
 */
bool stepper_motor_move_non_blocking(uint8_t motor_id, uint32_t steps,
                                     uint32_t speed_steps_per_second,
                                     stepper_motor_dir_t direction);

/**
 * @brief Move multiple motors simultaneously
 * @param num_motors Number of motors to move
 * @param motor_ids Array of motor IDs
 * @param steps Array of step counts for each motor
 * @param speeds Array of speeds for each motor
 * @param directions Array of directions for each motor
 * @return true if all motors started successfully
 */
bool stepper_motor_move_synchronized(uint8_t num_motors, uint8_t *motor_ids,
                                     uint32_t *steps, uint32_t *speeds,
                                     stepper_motor_dir_t *directions);

/**
 * @brief Check if all specified motors are idle
 * @param num_motors Number of motors to check
 * @param motor_ids Array of motor IDs to check
 * @return true if all motors are idle
 */
bool stepper_motors_all_idle(uint8_t num_motors, uint8_t *motor_ids);

/**
 * @brief Simple speed test with constant speed
 * @param motor_id Motor identifier
 */
void stepper_motor_simple_speed_test(uint8_t motor_id);

#endif /* STEPPER_MOTOR_H */