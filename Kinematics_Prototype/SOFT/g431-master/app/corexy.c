/**
 *******************************************************************************
 * @file 	corexy.c
 * @author 	naej
 * @date 	Avr 29, 2025
 * @brief	Fichier principal du core x, y.
 *******************************************************************************
 */
#include "stm32g4xx_hal.h"
#include "stm32g4_utils.h"
#include "stm32g4_systick.h"
#include "corexy.h"
#include "stm32g4_gpio.h"
#include "motorcontrol.h"


void Motor_Init_process_main(void){
	Motor_Init();
}


void motorcontrol_process_main(void){


}
