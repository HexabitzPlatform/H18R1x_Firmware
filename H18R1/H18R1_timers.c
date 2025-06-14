/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H18R1_timers.c
 Description: Timer configurations for H18R1 module.
 Timers: TIM16 (µs delays), TIM17 (ms delays), IWDG (500ms watchdog).
 PWM: TIM2/TIM3 for motor control (CH1/CH3).
*/

/* Includes ****************************************************************/
#include "BOS.h"

/* Exported Functions ******************************************************/
void TIM_USEC_Init(void);
void TIM_MSEC_Init(void);
void MX_IWDG_Init(void);

extern void MX_TIM2_Init(void);
extern void MX_TIM3_Init(void);

/* Exported Variables ******************************************************/
TIM_HandleTypeDef htim16; /* micro-second delay counter */
TIM_HandleTypeDef htim17; /* milli-second delay counter */
IWDG_HandleTypeDef hiwdg;

extern TIM_HandleTypeDef htim2; /* Timer CH1 for PWM */
extern TIM_HandleTypeDef htim3; /* Timer CH3 for PWM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/***************************************************************************/
/* Configure Timers ********************************************************/
/***************************************************************************/
/* IWDG init function */
void MX_IWDG_Init(void) {

	/* Reload Value = [(Time * 32 KHz) / (4 * 2^(pr) * 1000)] - 1
	 * RL = [(500 mS * 32000) / (4 * 2^1 * 1000)]  - 1 = 2000 - 1 = 1999
	 * timeout time = 500 mS
	 * Pre-scaler = 8
	 * Reload Value = 1999
	 *  */

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
	hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
	hiwdg.Init.Reload = 1999;

	HAL_IWDG_Init(&hiwdg);

}

/***************************************************************************/
/* Timer CH1 for PWM */
void MX_TIM2_Init(void){
	TIM_ClockConfigTypeDef sClockSourceConfig ={0};
	TIM_MasterConfigTypeDef sMasterConfig ={0};
	TIM_OC_InitTypeDef sConfigOC ={0};

	__HAL_RCC_TIM2_CLK_ENABLE();

	htim2.Instance = TIM2;
	htim2.Init.Prescaler =(uint32_t )(HAL_RCC_GetSysClockFreq() / 10000) - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period =0;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim2);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim2,&sClockSourceConfig);

	HAL_TIM_PWM_Init(&htim2);
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim2,&sMasterConfig);
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse =0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim2,&sConfigOC,TIM_CHANNEL_1);

	HAL_TIM_MspPostInit(&htim2);

}

/***************************************************************************/
/* Timer CH3 for PWM */
void MX_TIM3_Init(void){
	TIM_ClockConfigTypeDef sClockSourceConfig ={0};
	TIM_MasterConfigTypeDef sMasterConfig ={0};
	TIM_OC_InitTypeDef sConfigOC ={0};

	__HAL_RCC_TIM3_CLK_ENABLE();

	htim3.Instance = TIM3;
	htim3.Init.Prescaler =(uint32_t )(HAL_RCC_GetSysClockFreq() / 10000) - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period =0;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim3);
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim3,&sClockSourceConfig);

	HAL_TIM_PWM_Init(&htim3);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim3,&sMasterConfig);
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse =0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3,&sConfigOC,TIM_CHANNEL_3);

	HAL_TIM_MspPostInit(&htim3);

}

/***************************************************************************/
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim){
	GPIO_InitTypeDef GPIO_InitStruct ={0};
	if(htim->Instance == TIM2){
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**TIM2 GPIO Configuration
		 PA5     ------> TIM2_CH1
		 */
		GPIO_InitStruct.Pin = MOTOR_B_PWM_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
		HAL_GPIO_Init(MOTOR_B_PWM_PORT,&GPIO_InitStruct);

	}
	else if(htim->Instance == TIM3){
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**TIM3 GPIO Configuration
		 PB0     ------> TIM3_CH3
		 */
		GPIO_InitStruct.Pin = MOTOR_A_PWM_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
		HAL_GPIO_Init(MOTOR_A_PWM_PORT,&GPIO_InitStruct);

	}
}

/***************************************************************************/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base){
	if(htim_base->Instance == TIM2){
		/* Peripheral clock disable */
		__HAL_RCC_TIM2_CLK_DISABLE();
	}
	else if(htim_base->Instance == TIM3){
		/* Peripheral clock disable */
		__HAL_RCC_TIM3_CLK_DISABLE();
	}
}

/***************************************************************************/
/* Micro-seconds timebase init function - TIM16 (16-bit) */
void TIM_USEC_Init(void){

	__TIM16_CLK_ENABLE();

	htim16.Instance = TIM16;
	htim16.Init.Prescaler =47;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period =0XFFFF;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter =0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim16);

	HAL_TIM_Base_Start(&htim16);

}

/***************************************************************************/
/* Milli-seconds timebase init function - TIM17 (16-bit) */
void TIM_MSEC_Init(void){
	
	__TIM17_CLK_ENABLE();

	htim17.Instance = TIM17;
	htim17.Init.Prescaler =47999;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period =0xFFFF;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter =0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim17);

	HAL_TIM_Base_Start(&htim17);
}

/***************************************************************************/
/* Load and start micro-second delay counter */
void StartMicroDelay(uint16_t Delay) {
	uint32_t t0 = 0;

	portENTER_CRITICAL();

	if (Delay) {
		t0 = htim16.Instance->CNT;

		while (htim16.Instance->CNT - t0 <= Delay) {
		};
	}

	portEXIT_CRITICAL();
}

/***************************************************************************/
/* Load and start milli-second delay counter */
void StartMilliDelay(uint16_t Delay) {
	uint32_t t0 = 0;

	portENTER_CRITICAL();

	if (Delay) {
		t0 = htim17.Instance->CNT;

		while (htim17.Instance->CNT - t0 <= Delay) {
		};
	}

	portEXIT_CRITICAL();
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
