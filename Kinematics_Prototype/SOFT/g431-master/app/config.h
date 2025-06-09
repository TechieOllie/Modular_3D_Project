/**
 *******************************************************************************
 * @file 	config.h
 * @author 	naej
 * @date 	Avr 29, 2025
 * @brief	Fichier principal de configuration de votre projet sur carte Nucléo STM32G431KB.
 * 			Permet d'activer les différents modules logiciels à votre disposition.
 *******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONFIG_H_
#define CONFIG_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "sys/types.h"

/* Defines -------------------------------------------------------------------*/
#define LED_GREEN_PIN GPIO_PIN_4
#define LED_GREEN_GPIO GPIOA

#define UART2_ON_PA3_PA2
#define UART1_ON_PA10_PA9

#define USE_BSP_TIMER 1
#define USE_BSP_EXTIT 1

#define USE_RTC 0

#define USE_ADC 0
/* Configuration pour activer les entrées analogiques souhaitées */
#define USE_IN1 0  // Broche correspondante: PA0
#define USE_IN2 0  // Broche correspondante: PA1
#define USE_IN3 0  // Broche correspondante: PA6
#define USE_IN4 0  // Broche correspondante: PA7
#define USE_IN10 0 // Broche correspondante: PF1
#define USE_IN13 0 // Broche correspondante: PA5
#define USE_IN17 0 // Broche correspondante: PA4

#define USE_DAC 0

/*------------------Afficheurs------------------*/
#define USE_ILI9341 0 // Écran TFT (disabled for CNC)
#define USE_XPT2046 0
#define USE_FONT7x10 0
#define USE_FONT11x18 0
#define USE_FONT16x26 0

#define USE_EPAPER 0 // e-paper (disabled for CNC)
#define USE_WS2812 0 // Matrice de led (disabled for CNC)

/*------------------Capteurs------------------*/
#define USE_MPU6050 0		  // Accéléromètre, Gyroscope
#define USE_APDS9960 0		  // Capteur de mouvements, présence, couleurs
#define USE_BMP180 0		  // Capteur de pression atmosphérique
#define USE_BH1750FVI 0		  // Capteur de luminosité ambiante
#define USE_DHT11 0			  // Capteur de température et d'humidité
#define USE_DS18B20 0		  // Sonde de température
#define USE_YX6300 0		  // Lecteur MP3
#define USE_MATRIX_KEYBOARD 0 // Clavier matriciel
#define USE_HCSR04 0		  // Télémètre à ultrason
#define USE_GPS 0			  // GPS
#define USE_LD19 0			  // Lidar
#define USE_NFC03A1 0		  // Shield NFC
#define USE_VL53L0 0		  // Télémètre laser

/*------------------Expanders------------------*/
#define USE_MCP23017 0 // GPIO expander (disabled for CNC)
#define USE_MCP23S17 0 // GPIO expander (disabled for CNC)
#define USE_SD_CARD 0  // Carte SD (disabled for now)

/*------------------Actionneurs------------------*/
#define USE_MOTOR_DC 0

/*------------------Périphériques------------------*/
#define USE_I2C 0
#define I2C_TIMEOUT 5 // ms

#define USE_SPI 0

#define USE_TESTBOARD 0

#endif /* CONFIG_H_ */
