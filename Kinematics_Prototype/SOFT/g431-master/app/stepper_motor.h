/**
 *******************************************************************************
 * @file    stm32g4_stepper_motor.h
 * @author  Ol, naej, Fabs (adapted from julia's original)
 * @date    May 27, 2025
 * @brief   Stepper motor control with acceleration profiles for STM32G4
 *******************************************************************************
 */

#ifndef BSP_STM32G4_STEPPER_MOTOR_H_
#define BSP_STM32G4_STEPPER_MOTOR_H_

#include "stm32g4_utils.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "limit_switches.h"

// Motor configuration
#define STEPPER_MOTOR_MAX_COUNT 3
#define STEPPER_MIN_FREQUENCY 400
#define STEPPER_MAX_FREQUENCY 12000
#define DEFAULT_IT_PERIOD 50 // 50us base timer period (20kHz)

// Motor IDs enumeration
typedef enum
{
    STEPPER_MOTOR_0,
    STEPPER_MOTOR_1,
    STEPPER_MOTOR_2,
    STEPPER_MOTOR_3,
    STEPPER_MOTOR_NB
} motor_id_e;

// Motor states
typedef enum
{
    MOTOR_STATE_IDLE,
    MOTOR_STATE_ACCELERATING,
    MOTOR_STATE_CONSTANT_SPEED,
    MOTOR_STATE_DECELERATING,
    MOTOR_STATE_HOMING_FAST,
    MOTOR_STATE_HOMING_SLOW,
    MOTOR_STATE_ERROR
} stepper_motor_state_t;

// Motor direction
typedef enum
{
    MOTOR_DIR_CLOCKWISE = 0,
    MOTOR_DIR_COUNTERCLOCKWISE = 1
} stepper_motor_dir_t;

// Motor configuration structure
typedef struct
{
    GPIO_TypeDef *pulse_gpio;
    uint32_t pulse_pin;
    GPIO_TypeDef *dir_gpio;
    uint32_t dir_pin;
    GPIO_TypeDef *enable_gpio;
    uint32_t enable_pin;
    bool configured;
} stepper_motor_config_t;

// Acceleration profile structure
typedef struct
{
    uint32_t acceleration;      // steps/sec²
    uint32_t max_frequency;     // maximum frequency (Hz)
    uint32_t current_frequency; // current frequency (Hz)
    uint32_t target_frequency;  // target frequency for this move
    bool acceleration_enabled;
} stepper_accel_profile_t;

// Motor runtime data
typedef struct
{
    volatile int32_t position;
    volatile int32_t goal;
    volatile uint32_t steps_to_go;
    volatile uint32_t completed_steps;
    volatile stepper_motor_state_t state;
    volatile stepper_motor_dir_t direction;
    volatile uint32_t pulse_counter;
    volatile uint32_t pulse_period; // Timer ticks between pulses
    volatile bool enabled;
    stepper_accel_profile_t accel;
    uint8_t microsteps;
    axis_t axis; // Associated axis for limit checking
    bool limit_check_enabled;
} stepper_motor_data_t;

// Public function declarations
void STEPPER_MOTOR_init(void);
void STEPPER_MOTOR_demo(void);
void STEPPER_MOTOR_set_callback_at_each_pulse(callback_fun_t cb);

// Motor configuration
bool STEPPER_MOTOR_configure(motor_id_e id, GPIO_TypeDef *pulse_gpio, uint32_t pulse_pin,
                             GPIO_TypeDef *dir_gpio, uint32_t dir_pin,
                             GPIO_TypeDef *enable_gpio, uint32_t enable_pin,
                             uint8_t microsteps);

// Basic control
void STEPPER_MOTOR_enable(motor_id_e id, bool enable);
void STEPPER_MOTOR_set_goal(motor_id_e id, int32_t newgoal);
void STEPPER_MOTOR_set_position(motor_id_e id, int32_t newposition);
int32_t STEPPER_MOTOR_get_position(motor_id_e id);
int32_t STEPPER_MOTOR_get_goal(motor_id_e id);
bool STEPPER_MOTOR_is_arrived(motor_id_e id);

// Movement with speed control
bool STEPPER_MOTOR_move(motor_id_e id, uint32_t steps, uint32_t frequency, stepper_motor_dir_t direction);
bool STEPPER_MOTOR_move_accel(motor_id_e id, uint32_t steps, uint32_t target_freq, stepper_motor_dir_t direction);
void STEPPER_MOTOR_stop(motor_id_e id);
void STEPPER_MOTOR_emergency_stop_all(void);

// Acceleration profile
void STEPPER_MOTOR_set_acceleration(motor_id_e id, uint32_t acceleration, uint32_t max_frequency);
void STEPPER_MOTOR_enable_acceleration(motor_id_e id, bool enable);

// Status functions
stepper_motor_state_t STEPPER_MOTOR_get_state(motor_id_e id);
uint32_t STEPPER_MOTOR_get_completed_steps(motor_id_e id);
uint32_t STEPPER_MOTOR_get_current_frequency(motor_id_e id);

// Homing functions
bool STEPPER_MOTOR_home_precision(motor_id_e id, uint32_t fast_frequency, uint32_t slow_frequency);
bool STEPPER_MOTOR_move_with_limits(motor_id_e id, uint32_t steps, uint32_t frequency, stepper_motor_dir_t direction);

// Limit switch integration
void STEPPER_MOTOR_set_axis(motor_id_e id, axis_t axis);
void STEPPER_MOTOR_enable_limit_check(motor_id_e id, bool enable);

// System functions
void STEPPER_MOTOR_update(void);
bool STEPPER_MOTOR_system_init(void);
void STEPPER_MOTOR_debug_config(motor_id_e id);

// Timer interrupt handler (internal)
void STEPPER_MOTOR_timer_irq_handler(void);

#endif /* BSP_STM32G4_STEPPER_MOTOR_H_ */
