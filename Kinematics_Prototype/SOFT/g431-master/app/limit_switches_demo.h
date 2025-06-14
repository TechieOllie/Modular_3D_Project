/**
 *******************************************************************************
 * @file    limit_switches_demo.h
 * @author  Ol, naej, Fabs
 * @date    May 27, 2025
 * @brief   Demo application for limit switches functionality
 *******************************************************************************
 */

#ifndef LIMIT_SWITCHES_DEMO_H
#define LIMIT_SWITCHES_DEMO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32g4_utils.h"

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief Initialize and configure limit switches for demo
 * @return true if successful
 */
bool limit_switches_demo_init(void);

/**
 * @brief Demo for testing limit switches functionality
 */
void limit_switches_demo_run(void);

/**
 * @brief Demo for stepper motor with limit switches
 */
void stepper_motor_with_limits_demo(void);

/**
 * @brief Stop the demo
 */
void limit_switches_demo_stop(void);

#endif /* LIMIT_SWITCHES_DEMO_H */
