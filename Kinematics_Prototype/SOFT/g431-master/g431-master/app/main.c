/**
 *******************************************************************************
 * @file 	main.c
 * @author 	naej
 * @date 	Avr 29, 2025
 * @brief	Fichier principal de votre projet sur carte Nucléo STM32G431KB
 *******************************************************************************
 */

#include <motorcontrol.h>
#include "config.h"
#include "stm32g4_sys.h"
#include "stm32g4_systick.h"
#include "stm32g4_gpio.h"
#include "stm32g4_uart.h"
#include "stm32g4_utils.h"
#include <stdio.h>
#include "corexy.h"

#define BLINK_DELAY 100 // ms

void write_LED(bool b) {
    HAL_GPIO_WritePin(LED_GREEN_GPIO, LED_GREEN_PIN, b);
}

bool char_received(uart_id_t uart_id) {
    if (BSP_UART_data_ready(uart_id)) { // Si un caractère est reçu sur l'UART 2
        BSP_UART_get_next_byte(uart_id); // On "utilise" le caractère pour vider le buffer de réception
        return true;
    } else {
        return false;
    }
}

void heartbeat(void) {
    while (!char_received(UART2_ID)) {
        write_LED(true);
        HAL_Delay(50);
        write_LED(false);
        HAL_Delay(1500);
    }
}

/**
  * @brief  Point d'entrée de votre application
  */
int main(void) {
    HAL_Init(); // Initialisation des couches basses des drivers

    BSP_GPIO_enable();
    BSP_UART_init(UART2_ID, 115200);

    BSP_SYS_set_std_usart(UART2_ID, UART2_ID, UART2_ID);

    BSP_GPIO_pin_config(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);

    printf("Hi Jean, can you read me?\n");

    Motor_Init_process_main(); // Initialisation des moteurs

    while (1) {
        if (char_received(UART2_ID)) {
            write_LED(true);
            HAL_Delay(BLINK_DELAY);
            write_LED(false);
        }

        // Exemple de contrôle des moteurs
        motorcontrol_process_main();
    }
}

