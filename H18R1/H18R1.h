/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved
 
 File Name     : H18R1.h
 Description   : Header file for module H18R1.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):

>> USARTs 1,2,3,5,6 for module ports.

>> Timer3 (Ch3) & Timer2 (Ch1) for L298 PWM.
 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef H18R1_H
#define H18R1_H

/* Includes ****************************************************************/
#include "BOS.h"
#include "H18R1_MemoryMap.h"
#include "H18R1_uart.h"
#include "H18R1_gpio.h"
#include "H18R1_dma.h"
#include "H18R1_inputs.h"
#include "H18R1_eeprom.h"

/* Exported Macros *********************************************************/
#define	MODULE_PN		_H18R1

/* Port-related Definitions */
#define	NUM_OF_PORTS	5
#define P_PROG 			P2		/* ST factory bootloader UART */

/* Define Available ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5

/* Define Available USARTs */
#define _USART1
#define _USART2
#define _USART3
#define _USART5
#define _USART6

/* Port-UART Mapping */
#define UART_P1 &huart6
#define UART_P2 &huart2
#define UART_P3 &huart3
#define UART_P4 &huart1
#define UART_P5 &huart5

/* Module-specific Hardware Definitions ************************************/
/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT		GPIOA
#define	USART1_RX_PORT		GPIOA
#define	USART1_AF			GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT		GPIOA
#define	USART2_RX_PORT		GPIOA
#define	USART2_AF			GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_2
#define	USART5_TX_PORT		GPIOD
#define	USART5_RX_PORT		GPIOD
#define	USART5_AF			GPIO_AF3_USART5

#define	USART6_TX_PIN		GPIO_PIN_8
#define	USART6_RX_PIN		GPIO_PIN_9
#define	USART6_TX_PORT		GPIOB
#define	USART6_RX_PORT		GPIOB
#define	USART6_AF			GPIO_AF8_USART6

/* Motor A/B Timer Definitions */
#define MOTOR_A_PWM_PIN        GPIO_PIN_0
#define MOTOR_A_PWM_PORT       GPIOB
#define MOTOR_A_TIM_HANDLE     &htim3
#define MOTOR_A_TIM_CH         TIM_CHANNEL_3
#define MOTOR_A_ARR            htim3.Instance->ARR
#define MOTOR_A_CCR            htim3.Instance->CCR3

#define MOTOR_B_PWM_PIN        GPIO_PIN_5
#define MOTOR_B_PWM_PORT       GPIOA
#define MOTOR_B_TIM_HANDLE     &htim2
#define MOTOR_B_TIM_CH         TIM_CHANNEL_1
#define MOTOR_B_ARR            htim2.Instance->ARR
#define MOTOR_B_CCR            htim2.Instance->CCR1

/* Motor A/B Direction GPIO Pins */
#define MOTOR_A_IN1_PIN        GPIO_PIN_7
#define MOTOR_A_IN1_PORT  	   GPIOA
#define MOTOR_A_IN2_PIN        GPIO_PIN_1
#define MOTOR_A_IN2_PORT       GPIOB

#define MOTOR_B_IN3_PIN        GPIO_PIN_4
#define MOTOR_B_IN3_PORT       GPIOA
#define MOTOR_B_IN4_PIN        GPIO_PIN_6
#define MOTOR_B_IN4_PORT       GPIOA

/* Indicator LED */
#define _IND_LED_PORT		   GPIOB
#define _IND_LED_PIN		   GPIO_PIN_7

/* Module-specific Macro Definitions ***************************************/
#define NUM_MODULE_PARAMS	   1

/* Module_Status Type Definition */
typedef enum {
	H18R1_OK =0,
	H18R1_ERR_UNKNOWNMESSAGE,
	H18R1_ERR_WRONGPARAMS,
	H18R1_ERR_WRONGDIRECTION,
    H18R1_ERR_WRONGMOTOR,
    H18R1_ERR_WRONGDUTYCYCLE,
	H18R1_ERROR =255
} Module_Status;

/* Motor Direction */
typedef enum {
	FORWARD_DIR=1,
	BACKWARD_DIR
} H_BridgeDirection;

/* Motor Selection */
typedef enum {
	MOTOR_A=1,
	MOTOR_B
} Motor;

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status MotorTurnOff(Motor motor);
Module_Status MotorTurnOn(H_BridgeDirection direction,Motor motor);
Module_Status MotorSpeedControl(H_BridgeDirection direction,uint8_t dutyCycle,Motor motor);

#endif /* H18R1_H */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
