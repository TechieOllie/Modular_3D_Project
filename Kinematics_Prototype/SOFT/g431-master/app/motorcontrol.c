/**
 *******************************************************************************
 * @file 	motorcontrol.c
 * @author 	naej
 * @date 	Avr 29, 2025
 * @brief	Fichier principal du controle des moteurs.
 *******************************************************************************
 */
#include "stm32g4xx_hal.h"
#include "stm32g4_utils.h"
#include "stm32g4_systick.h"
#include "stm32g4_gpio.h"
#include <motorcontrol.h>

// Définir les broches pour les moteurs pas à pas
#define STEP_PIN_X GPIO_PIN_0
#define DIR_PIN_X GPIO_PIN_1
#define STEP_PIN_Y GPIO_PIN_2
#define DIR_PIN_Y GPIO_PIN_3

// Définir les ports pour les moteurs pas à pas
#define STEP_PORT_X GPIOA
#define DIR_PORT_X GPIOA
#define STEP_PORT_Y GPIOA
#define DIR_PORT_Y GPIOA


void Motor_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = STEP_PIN_X | DIR_PIN_X | STEP_PIN_Y | DIR_PIN_Y;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void StepMotorX(int steps, int direction) {
    HAL_GPIO_WritePin(DIR_PORT_X, DIR_PIN_X, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
    for (int i = 0; i < steps; i++) {
        HAL_GPIO_WritePin(STEP_PORT_X, STEP_PIN_X, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(STEP_PORT_X, STEP_PIN_X, GPIO_PIN_RESET);
        HAL_Delay(1);
    }
}

void StepMotorY(int steps, int direction) {
    HAL_GPIO_WritePin(DIR_PORT_Y, DIR_PIN_Y, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
    for (int i = 0; i < steps; i++) {
        HAL_GPIO_WritePin(STEP_PORT_Y, STEP_PIN_Y, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(STEP_PORT_Y, STEP_PIN_Y, GPIO_PIN_RESET);
        HAL_Delay(1);
    }
}
