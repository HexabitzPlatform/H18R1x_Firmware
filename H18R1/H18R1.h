/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved
 
 File Name     : H18R1.h
 Description   : Header file for module H18R1.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):

>> USARTs 1,2,3,5,6 for module ports.

>> Timer3 (Ch3) & Timer2 (Ch1) for L298 PWM.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H18R1_H
#define H18R1_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H18R1_MemoryMap.h"
#include "H18R1_uart.h"
#include "H18R1_gpio.h"
#include "H18R1_dma.h"
#include "H18R1_inputs.h"
#include "H18R1_eeprom.h"
/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H18R1


/* Port-related definitions */
#define	NumOfPorts			5

#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1 
#define _P2 
#define _P3 
#define _P4 
#define _P5 


/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#define _Usart5 1
#define _Usart6	1


/* Port-UART mapping */

#define P1uart &huart6
#define P2uart &huart2
#define P3uart &huart3
#define P4uart &huart1
#define P5uart &huart5


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

/* Module-specific Definitions */

#define NUM_MODULE_PARAMS						1

/*  Pins For H_Bridge*/
#define IN1_Pin                              GPIO_PIN_7
#define IN1_GPIO_Port  		             	 GPIOA

#define IN2_Pin                              GPIO_PIN_1
#define IN2_GPIO_Port                        GPIOB

#define IN3_Pin                              GPIO_PIN_4
#define IN3_GPIO_Port                        GPIOA

#define IN4_Pin                              GPIO_PIN_6
#define IN4_GPIO_Port                        GPIOA

#define TIM2_CH1_ENB_Pin                     GPIO_PIN_5
#define TIM2_CH1_ENB_GPIO_Port               GPIOA

#define TIM3_CH3_ENA_Pin                     GPIO_PIN_0
#define TIM3_CH3_ENA_GPIO_Port               GPIOB

/* Module EEPROM Variables */

// Module Addressing Space 500 - 599
#define _EE_MODULE							500		

/* Module_Status Type Definition */
typedef enum {
	H18R1_OK =0,
	H18R1_ERR_UnknownMessage,
	H18R1_ERR_WrongParams,
	H18R1_ERR_WrongDirection,
    H18R1_ERR_WrongMotor,
    H18R1_ERR_WrongDutyCycle,
	H18R1_ERROR =255
} Module_Status;

typedef enum {
	forward=1,
	backward
} H_BridgeDirection;

typedef enum {
	MotorA=1,
	MotorB
} Motor;
/* Indicator LED */
#define _IND_LED_PORT			GPIOB
#define _IND_LED_PIN			GPIO_PIN_7

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/*Timer for PWM*/

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);
extern void ExecuteMonitor(void);


/*-----------------Private function------------------*/

extern Module_Status MotorPWM( uint8_t dutycycle,Motor motor);
/* -----------------------------------------------------------------------
 |								  APIs							          |  																 	|
/* -----------------------------------------------------------------------
 */
extern Module_Status Turn_ON(H_BridgeDirection direction,Motor motor);
extern Module_Status Turn_OFF(Motor motor);
extern Module_Status Turn_PWM(H_BridgeDirection direction,uint8_t dutyCycle,Motor motor);

void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);

/* -----------------------------------------------------------------------
 |								Commands							      |															 	|
/* -----------------------------------------------------------------------
 */
extern const CLI_Command_Definition_t CLI_Turn_ONCommandDefinition;
extern const CLI_Command_Definition_t CLI_Turn_OFFCommandDefinition;
extern const CLI_Command_Definition_t CLI_Turn_PWMCommandDefinition;

#endif /* H18R1_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
