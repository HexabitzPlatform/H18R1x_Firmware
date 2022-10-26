/*
 BitzOS (BOS) V0.2.7 - Copyright (C) 2017-2022 Hexabitz
 All rights reserved

 File Name     : H18R1_gpio.c
 Description   : Source code provides code for the configuration of all used GPIO pins .

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/*  */
BOS_Status GetPortGPIOs(uint8_t port,uint32_t *TX_Port,uint16_t *TX_Pin,uint32_t *RX_Port,uint16_t *RX_Pin);

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/

/** Pinout Configuration
 */
void GPIO_Init(void){
	/* GPIO Ports Clock Enable */
	__GPIOC_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	__GPIOD_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__GPIOF_CLK_ENABLE();		// for HSE and Boot0
	
	IND_LED_Init();
}

//-- Configure indicator LED
void IND_LED_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin = _IND_LED_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(_IND_LED_PORT,&GPIO_InitStruct);
}

//H_Bridge GPIO Init:
void H_Bridge_gpio_init(void){
	 GPIO_InitTypeDef GPIO_InitStruct = {0};

	 /* GPIO Ports Clock Enable */
	   __HAL_RCC_GPIOF_CLK_ENABLE();
	   __HAL_RCC_GPIOA_CLK_ENABLE();
	   __HAL_RCC_GPIOB_CLK_ENABLE();

	   /*Configure GPIO pin Output Level */
	    HAL_GPIO_WritePin(GPIOA, ENB_Pin|IN4_Pin, GPIO_PIN_RESET);

	    /*Configure GPIO pin Output Level */
	    HAL_GPIO_WritePin(GPIOB, ENA_Pin|IN2_Pin, GPIO_PIN_RESET);

	    /*Configure GPIO pins : ENB_Pin IN4_Pin */
	    GPIO_InitStruct.Pin = ENB_Pin|IN4_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	    /*Configure GPIO pins : ENA_Pin IN2_Pin */
	    GPIO_InitStruct.Pin = ENA_Pin|IN2_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);






}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PA7     ------> TIM3_CH2
    */
    GPIO_InitStruct.Pin = TIM3_CH2_IN1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(TIM3_CH2_IN1_GPIO_Port, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    /* TIM3 interrupt Init */
       HAL_NVIC_SetPriority(TIM3_TIM4_IRQn, 0, 0);
       HAL_NVIC_EnableIRQ(TIM3_TIM4_IRQn);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }
  else if(htim->Instance==TIM14)
  {
  /* USER CODE BEGIN TIM14_MspPostInit 0 */

  /* USER CODE END TIM14_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM14 GPIO Configuration
    PA4     ------> TIM14_CH1
    */
    GPIO_InitStruct.Pin = TIM14_CH1_IN3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM14;
    HAL_GPIO_Init(TIM14_CH1_IN3_GPIO_Port, &GPIO_InitStruct);

    /* Peripheral interrupt init */
   	HAL_NVIC_SetPriority(TIM14_IRQn, 0, 0);
   	HAL_NVIC_EnableIRQ(TIM14_IRQn);


  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}
/*-----------------------------------------------------------*/

/* --- Check for factory reset condition: 
 - P1 TXD is connected to last port RXD    
 */
uint8_t IsFactoryReset(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	uint32_t P1_TX_Port, P1_RX_Port, P_last_TX_Port, P_last_RX_Port;
	uint16_t P1_TX_Pin, P1_RX_Pin, P_last_TX_Pin, P_last_RX_Pin;
	
	/* -- Setup GPIOs -- */

	/* Enable all GPIO Ports Clocks */
	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();
	__GPIOD_CLK_ENABLE();
	
	/* Get GPIOs */
	GetPortGPIOs(P1,&P1_TX_Port,&P1_TX_Pin,&P1_RX_Port,&P1_RX_Pin);
	GetPortGPIOs(P_LAST,&P_last_TX_Port,&P_last_TX_Pin,&P_last_RX_Port,&P_last_RX_Pin);
	
	/* TXD of first port */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin =P1_TX_Pin;
	HAL_GPIO_Init((GPIO_TypeDef* )P1_TX_Port,&GPIO_InitStruct);
	
	/* RXD of last port */
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Pin =P_last_RX_Pin;
	HAL_GPIO_Init((GPIO_TypeDef* )P_last_RX_Port,&GPIO_InitStruct);
	
	/* Check for factory reset conditions */
	HAL_GPIO_WritePin((GPIO_TypeDef* )P1_TX_Port,P1_TX_Pin,GPIO_PIN_RESET);
	Delay_ms_no_rtos(5);
	if(HAL_GPIO_ReadPin((GPIO_TypeDef* )P_last_RX_Port,P_last_RX_Pin) == RESET){
		HAL_GPIO_WritePin((GPIO_TypeDef* )P1_TX_Port,P1_TX_Pin,GPIO_PIN_SET);
		Delay_ms_no_rtos(5);
		if(HAL_GPIO_ReadPin((GPIO_TypeDef* )P_last_RX_Port,P_last_RX_Pin) == SET){
			return 1;
		}
	}
	
	/* Clear flag for formated EEPROM if it was already set */
	/* Flag address (STM32F09x) - Last 4 words of SRAM */
	*((unsigned long* )0x20007FF0) =0xFFFFFFFF;
	
	return 0;
}

/*-----------------------------------------------------------*/

/* --- Get GPIO pins and ports of this array port
 */
BOS_Status GetPortGPIOs(uint8_t port,uint32_t *TX_Port,uint16_t *TX_Pin,uint32_t *RX_Port,uint16_t *RX_Pin){
	BOS_Status result =BOS_OK;
	
	/* Get port UART */
	UART_HandleTypeDef *huart =GetUart(port);
	
	if(huart == &huart1){
#ifdef _Usart1		
		*TX_Port =(uint32_t ) USART1_TX_PORT;
		*TX_Pin = USART1_TX_PIN;
		*RX_Port =(uint32_t ) USART1_RX_PORT;
		*RX_Pin = USART1_RX_PIN;
#endif
	}
#ifdef _Usart2	
	else if(huart == &huart2){
		*TX_Port =(uint32_t ) USART2_TX_PORT;
		*TX_Pin = USART2_TX_PIN;
		*RX_Port =(uint32_t ) USART2_RX_PORT;
		*RX_Pin = USART2_RX_PIN;
	}
#endif
#ifdef _Usart3	
	else if(huart == &huart3){
		*TX_Port =(uint32_t ) USART3_TX_PORT;
		*TX_Pin = USART3_TX_PIN;
		*RX_Port =(uint32_t ) USART3_RX_PORT;
		*RX_Pin = USART3_RX_PIN;
	}
#endif
#ifdef _Usart4
	else if(huart == &huart4){
		*TX_Port =(uint32_t ) USART4_TX_PORT;
		*TX_Pin = USART4_TX_PIN;
		*RX_Port =(uint32_t ) USART4_RX_PORT;
		*RX_Pin = USART4_RX_PIN;
	}
#endif
#ifdef _Usart5	
	else if(huart == &huart5){
		*TX_Port =(uint32_t ) USART5_TX_PORT;
		*TX_Pin = USART5_TX_PIN;
		*RX_Port =(uint32_t ) USART5_RX_PORT;
		*RX_Pin = USART5_RX_PIN;
	}
#endif
#ifdef _Usart6	
	else if(huart == &huart6){
		*TX_Port =(uint32_t ) USART6_TX_PORT;
		*TX_Pin = USART6_TX_PIN;
		*RX_Port =(uint32_t ) USART6_RX_PORT;
		*RX_Pin = USART6_RX_PIN;
	}
#endif
#ifdef _Usart7
	else if (huart == &huart7) 
	{		
		*TX_Port = (uint32_t)USART7_TX_PORT;
		*TX_Pin = USART7_TX_PIN;
		*RX_Port = (uint32_t)USART7_RX_PORT;
		*RX_Pin = USART7_RX_PIN;
	} 
#endif
#ifdef _Usart8	
	else if (huart == &huart8) 
	{	
		*TX_Port = (uint32_t)USART8_TX_PORT;
		*TX_Pin = USART8_TX_PIN;
		*RX_Port = (uint32_t)USART8_RX_PORT;
		*RX_Pin = USART8_RX_PIN;
	} 
#endif
	else
		result =BOS_ERROR;
	
	return result;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
