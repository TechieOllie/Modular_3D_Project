/**
 *******************************************************************************
 * @file    stepper_motor.h
 * @author  GitHub Copilot
 * @date    May 27, 2025
 * @brief   Stepper motor control using TB6600 driver with PWM signals
 *******************************************************************************
 */

#ifndef STEPPER_MOTOR_H_
#define STEPPER_MOTOR_H_

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "stm32g4_timer.h"
#include "stm32g4_gpio.h"
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/
#define STEPPER_MAX_MOTORS 4
#define STEPPER_MICROSTEPS_DEFAULT 8

// Motor direction
typedef enum
{
    MOTOR_DIR_CLOCKWISE = 0,
    MOTOR_DIR_COUNTERCLOCKWISE = 1
} stepper_motor_dir_t;

// Motor states
typedef enum
{
    MOTOR_STATE_IDLE,
    MOTOR_STATE_MOVING,
    MOTOR_STATE_ACCELERATING,
    MOTOR_STATE_DECELERATING
} stepper_motor_state_t;

// Motor configuration structure
typedef struct
{
    GPIO_TypeDef *pul_gpio; // GPIO port for PUL pin
    uint32_t pul_pin;       // GPIO pin for PUL
    GPIO_TypeDef *dir_gpio; // GPIO port for DIR pin
    uint32_t dir_pin;       // GPIO pin for DIR
    timer_id_t timer_id;    // Timer ID for PWM generation
    uint32_t timer_channel; // Timer channel for PWM

    // Motor parameters
    uint8_t microsteps;            // Number of microsteps (8, 16, 32)
    uint32_t step_delay_us;        // Delay between steps in microseconds
    uint32_t steps_to_move;        // Number of steps to move
    uint32_t steps_moved;          // Number of steps already moved
    stepper_motor_dir_t direction; // Current direction

    // State
    stepper_motor_state_t state; // Current motor state
    bool initialized;            // Flag indicating if the motor is initialized

    // Remove the acceleration state field:
    // accel_state_t accel_state;
} stepper_motor_t;

/* Timer interrupt handlers - required by the BSP */
void TIMER1_user_handler_it(void);
void TIMER2_user_handler_it(void);
void TIMER3_user_handler_it(void);
void TIMER4_user_handler_it(void);

/* Public functions declarations ---------------------------------------------*/
/**
 * @brief Initialize a stepper motor with specified parameters
 *
 * @param motor_id       ID of the motor to initialize
 * @param pul_gpio       GPIO port for PUL pin
 * @param pul_pin        GPIO pin for PUL
 * @param dir_gpio       GPIO port for DIR pin
 * @param dir_pin        GPIO pin for DIR
 * @param timer_id       Timer ID for PWM generation
 * @param timer_channel  Timer channel for PWM
 * @param microsteps     Number of microsteps (8, 16, 32)
 * @return bool          True if initialization successful, false otherwise
 */
bool stepper_motor_init(uint8_t motor_id,
                        GPIO_TypeDef *pul_gpio, uint16_t pul_pin,
                        GPIO_TypeDef *dir_gpio, uint16_t dir_pin,
                        timer_id_t timer_id, uint16_t timer_channel,
                        uint16_t microsteps);

/**
 * @brief Start motor movement with specified number of steps, speed, and direction
 *
 * @param motor_id       ID of the motor to move
 * @param steps          Number of steps to move
 * @param speed_steps_per_second Speed in steps per second
 * @param direction      Direction of movement (MOTOR_DIR_CLOCKWISE or MOTOR_DIR_COUNTERCLOCKWISE)
 * @return bool          True if movement started successfully, false otherwise
 */
bool stepper_motor_move(uint8_t motor_id, uint32_t steps, uint32_t speed_steps_per_second,
                        stepper_motor_dir_t direction);

bool stepper_motor_move_with_accel(uint8_t motor_id, uint32_t steps,
                                   uint32_t start_speed, uint32_t max_speed,
                                   uint32_t accel_steps, stepper_motor_dir_t direction);

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

#endif /* STEPPER_MOTOR_H_ */