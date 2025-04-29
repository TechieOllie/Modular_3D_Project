/**
 *******************************************************************************
 * @file 	motorcontrol.h
 * @author 	naej
 * @date 	Avr 29, 2025
 * @brief	Fichier principal de configuration du controle des moteurs.
 *******************************************************************************
 */

#include "stm32g4xx_hal.h"
#include "stm32g4_utils.h"
#include "stm32g4_systick.h"
#include "stm32g4_gpio.h"


#ifndef COREXY_H
#define COREXY_H



// Prototypes des fonctions
void Motor_Init(void);
void StepMotorX(int steps, int direction);
void StepMotorY(int steps, int direction);

#endif // COREXY_H
