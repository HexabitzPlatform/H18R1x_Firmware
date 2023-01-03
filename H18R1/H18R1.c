/*
 BitzOS (BOS) V0.2.7 - Copyright (C) 2017-2022 Hexabitz
 All rights reserved

 File Name     : H18R1.c
 Description   : Source code for module H18R1.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):

>> USARTs 1,2,3,5,6 for module ports.
>> Timer3 (Ch2) & Timer14 (Ch1) for L298 PWM.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H18R1_inputs.h"
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;

/* Module exported parameters ------------------------------------------------*/
module_param_t modParam[NUM_MODULE_PARAMS] ={{.paramPtr = NULL, .paramFormat =FMT_FLOAT, .paramName =""}};

/* Private variables ---------------------------------------------------------*/

/*Timer for PWM*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;



/* Private function prototypes -----------------------------------------------*/
void MX_TIM3_Init(void);
void MX_TIM14_Init(void);
void ExecuteMonitor(void);

/* Create CLI commands --------------------------------------------------------*/
portBASE_TYPE CLI_Turn_ONCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_Turn_OFFCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_Turn_PWMCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure : Turn_ON */
const CLI_Command_Definition_t CLI_Turn_ONCommandDefinition =
{
	( const int8_t * ) "turn_on", /* The command string to type. */
	( const int8_t * ) "turn_on :\r\n Parameters required to execute a Turn_ON: Direction forward or backward and Motor on: MotorA or MotorB \r\n\r\n",
	CLI_Turn_ONCommand, /* The function to run. */
	2 /* two parameters are expected. */
};

/* CLI command structure : Turn_OFF */
const CLI_Command_Definition_t CLI_Turn_OFFCommandDefinition =
{
	( const int8_t * ) "turn_off", /* The command string to type. */
	( const int8_t * ) "turn_off :\r\n Parameters required to execute a Turn_OFF: Motor off: MotorA or MotorB  \r\n\r\n",
	CLI_Turn_OFFCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/* CLI command structure : Turn_PWM */
const CLI_Command_Definition_t CLI_Turn_PWMCommandDefinition =
{
	( const int8_t * ) "turn_pwm", /* The command string to type. */
	( const int8_t * ) "turn_pwm :\r\n Parameters required to execute a Turn_PWM: Direction forward or backward and"
		" dutyCycle: PWM duty cycle in precentage (0 to 100) Motor on: MotorA or MotorB \r\n\r\n",
		CLI_Turn_PWMCommand, /* The function to run. */
	3 /* three parameters are expected. */
};
/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |												 Private Functions	 														|
 ----------------------------------------------------------------------- 
 */

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow : 
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            PREDIV                         = 1
 *            PLLMUL                         = 6
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */
void SystemClock_Config(void){
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	  /** Configure the main internal regulator output voltage
	  */
	  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	  RCC_OscInitStruct.PLL.PLLN = 12;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	 HAL_RCC_OscConfig(&RCC_OscInitStruct);

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	 HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

	  /** Initializes the peripherals clocks
	  */
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
	  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	  HAL_NVIC_SetPriority(SysTick_IRQn,0,0);
	
}

/*-----------------------------------------------------------*/


/* --- Save array topology and Command Snippets in Flash RO --- 
 */
uint8_t SaveToRO(void){
	BOS_Status result =BOS_OK;
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint16_t add =2, temp =0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};
	
	HAL_FLASH_Unlock();
	
	/* Erase RO area */
	FLASH_PageErase(FLASH_BANK_1,RO_START_ADDRESS);
	//TOBECHECKED
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	if(FlashStatus != HAL_OK){
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}
	
	/* Save number of modules and myID */
	if(myID){
		temp =(uint16_t )(N << 8) + myID;
		//HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,RO_START_ADDRESS,temp);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS,temp);
		//TOBECHECKED
		FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
		if(FlashStatus != HAL_OK){
			return pFlash.ErrorCode;
		}
		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}
		
		/* Save topology */
		for(uint8_t i =1; i <= N; i++){
			for(uint8_t j =0; j <= MaxNumOfPorts; j++){
				if(array[i - 1][0]){
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,              //HALFWORD
						//TOBECHECKED
					RO_START_ADDRESS + add,array[i - 1][j]);
					add +=2;
					FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(FlashStatus != HAL_OK){
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					}
				}
			}
		}
	}
	
	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for(uint8_t s =0; s < numOfRecordedSnippets; s++){
		if(snippets[s].cond.conditionType){
			snipBuffer[0] =0xFE;		// A marker to separate Snippets
			memcpy((uint8_t* )&snipBuffer[1],(uint8_t* )&snippets[s],sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j =0; j < (sizeof(snippet_t) / 2); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint16_t* )&snipBuffer[j * 2]);
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=2;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j =0; j < ((strlen(snippets[s].cmd) + 1) / 2); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint16_t* )(snippets[s].cmd + j * 2));
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=2;
				}
			}
		}
	}
	
	HAL_FLASH_Lock();
	
	return result;
}

/* --- Clear array topology in SRAM and Flash RO --- 
 */
uint8_t ClearROtopology(void){
	// Clear the array 
	memset(array,0,sizeof(array));
	N =1;
	myID =0;
	
	return SaveToRO();
}
/*-----------------------------------------------------------*/

/* --- Trigger ST factory bootloader update for a remote module.
 */
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = route[NumberOfHops(dst)-1]; /* previous module = route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 exclusion on this buffer as it is assumed only one command console
		 interface will be used at any one time. */
		pcOutputString =FreeRTOS_CLIGetOutputBuffer();

		if(outport == 0)		// This is a remote module update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateMessage,dst);
		else
			// This is a 'via port' remote update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateViaPortMessage,dst,outport);

		strcat((char* )pcOutputString,pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport,(char* )pcOutputString,strlen((char* )pcOutputString),cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);


	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport,myID,myOutport,myID,BIDIRECTIONAL,0xFFFFFFFF,0xFFFFFFFF,false);
}

/*-----------------------------------------------------------*/

/* --- Setup a port for remote ST factory bootloader update:
 - Set baudrate to 57600
 - Enable even parity
 - Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){
	UART_HandleTypeDef *huart =GetUart(port);

	huart->Init.BaudRate =57600;
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);
}

/* --- H18R1 module initialization.
 */
void Module_Peripheral_Init(void){

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();



	/*H_Bridge GPIO Init: */
	H_Bridge_gpio_init();
	MX_TIM3_Init();
	MX_TIM14_Init();



	/* Create module special task (if needed) */
}

/*-----------------------------------------------------------*/
/* --- H18R1 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){
	Module_Status result =H18R1_OK;

	uint8_t direction;
	uint8_t Motor;
	uint8_t dutyCycle=0;


	switch(code){
		case CODE_H18R1_Turn_ON:
			direction=(uint8_t)cMessage[port - 1][shift];
			Motor=(uint8_t)cMessage[port - 1][1+shift];
			Turn_ON(direction,Motor);
			break;

		case CODE_H18R1_Turn_OFF:
			Motor=(uint8_t)cMessage[port - 1][shift];
			Turn_OFF(Motor);
			break;

		case CODE_H18R1_Turn_PWM:
			direction=(uint8_t)cMessage[port - 1][shift];
			dutyCycle=(uint8_t)cMessage[port - 1][1 + shift];
			Motor=(uint8_t)cMessage[port - 1][2+shift];
			Turn_PWM(direction, dutyCycle,Motor);
			break;

		default:
			result =H18R1_ERR_UnknownMessage;
			break;
	}
	
	return result;
}
/* --- Get the port for a given UART. 
 */
uint8_t GetPort(UART_HandleTypeDef *huart){

	if(huart->Instance == USART6)
		return P1;
	else if(huart->Instance == USART2)
		return P2;
	else if(huart->Instance == USART3)
		return P3;
	else if(huart->Instance == USART1)
		return P4;
	else if(huart->Instance == USART5)
		return P5;
	

	return 0;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void){
    FreeRTOS_CLIRegisterCommand(&CLI_Turn_ONCommandDefinition);
    FreeRTOS_CLIRegisterCommand(&CLI_Turn_OFFCommandDefinition);
    FreeRTOS_CLIRegisterCommand(&CLI_Turn_PWMCommandDefinition);



}

/*-----------------------------------------------------------*/


/* Module special task function (if needed) */
//void Module_Special_Task(void *argument){
//
//	/* Infinite loop */
//	uint8_t cases; // Test variable.
//	for(;;){
//		/*  */
//		switch(cases){
//
//
//			default:
//				osDelay(10);
//				break;
//		}
//
//		taskYIELD();
//	}
//
//}


/*-----------------------------------------------------------*/
Module_Status MotorON(){

	Module_Status status=H18R1_OK;

		HAL_GPIO_WritePin(ENA_GPIO_Port ,ENA_Pin ,GPIO_PIN_SET);

		HAL_GPIO_WritePin(ENB_GPIO_Port ,ENB_Pin ,GPIO_PIN_SET);


	return status;

}
/*-----------------------------------------------------------*/
Module_Status SetupMotor(H_BridgeMode MovementDirection, Motor motor){

	Module_Status status=H18R1_OK;


	if(MovementDirection==forward && motor==MotorA){
		HAL_GPIO_WritePin(TIM3_CH2_IN1_GPIO_Port ,TIM3_CH2_IN1_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN2_GPIO_Port ,IN2_Pin ,GPIO_PIN_RESET);

	}
	else if(MovementDirection==forward && motor==MotorB){
		HAL_GPIO_WritePin(TIM14_CH1_IN3_GPIO_Port ,TIM14_CH1_IN3_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN4_GPIO_Port ,IN4_Pin ,GPIO_PIN_RESET);
	}
	else if(MovementDirection==backward && motor==MotorA){
		HAL_GPIO_WritePin(TIM3_CH2_IN1_GPIO_Port ,TIM3_CH2_IN1_Pin ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN2_GPIO_Port ,IN2_Pin ,GPIO_PIN_SET);
	}
	else if(MovementDirection==backward && motor==MotorB){
		HAL_GPIO_WritePin(TIM14_CH1_IN3_GPIO_Port ,TIM14_CH1_IN3_Pin  ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN4_GPIO_Port ,IN4_Pin ,GPIO_PIN_SET);
	}
	return status;
}
/*-----------------------------------------------------------*/
Module_Status MotorOFF(Motor motor){

	Module_Status status=H18R1_OK;
	H_Bridge_gpio_init();

	if(motor==MotorA){
		HAL_GPIO_WritePin(ENA_GPIO_Port ,ENA_Pin ,GPIO_PIN_RESET);
	}

	else if(motor==MotorB){
	HAL_GPIO_WritePin(ENB_GPIO_Port ,ENB_Pin ,GPIO_PIN_RESET);
	}

	return status;

}
/*-----------------------------------------------------------*/
Module_Status PWM_stop(){

	Module_Status status=H18R1_OK;


	HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);

	return status;


}
/*-----------------------------------------------------------*/

/* --- Set Motor PWM frequency and dutycycle ---*/
Module_Status MotorPWM(uint32_t freq, uint8_t dutycycle,Motor motor) {

	Module_Status status=H18R1_OK;

	uint32_t period = PWM_TIMER_CLOCK / freq;



	switch(motor){
		case MotorA:
			htim3.Instance->ARR = period - 1;
			htim3.Instance->CCR2 = ((float) dutycycle / 100.0f) * period;
			break;
		case MotorB:
			htim14.Instance->ARR = period - 1;
			htim14.Instance->CCR1 = ((float) dutycycle / 100.0f) * period;
			break;

		default: break;



	}


	if (HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2 ) != HAL_OK)
		return H18R1_ERROR;
	if (HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1 ) != HAL_OK)
		return H18R1_ERROR;

	return status;
}
/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |								  APIs							          | 																 	|
/* -----------------------------------------------------------------------
 */

/*--------------Run the motor at full speed------------*/
Module_Status Turn_ON(H_BridgeMode direction,Motor motor){

     Module_Status status=H18R1_OK;
     H_Bridge_gpio_init();


     if( direction!= forward &&  direction!= backward)
     {
     	status= H18R1_ERR_WrongParams;
     	return status;

     }

     if( motor!= MotorA && motor!= MotorB)
	  {
          	status= H18R1_ERR_WrongParams;
          	return status;

	  }

 	 MotorON();
	 SetupMotor(direction,motor);

	 return status;

}

/*-----------------------------------------------------------------------------*/

/*------------------Off the motor---------------*/
Module_Status Turn_OFF(Motor motor){

    Module_Status status=H18R1_OK;


    MotorOFF(motor);

	return status;
}
/*-----------------------------------------------------------------------------*/
/* --- Turn-on H_Bridge with pulse-width modulation (PWM) ---
 dutyCycle: PWM duty cycle in precentage (0 to 100)
 */
Module_Status Turn_PWM(H_BridgeMode direction,uint8_t dutyCycle,Motor motor){

    Module_Status status=H18R1_OK;

    if( direction!= forward &&  direction!= backward)
         {
         	status= H18R1_ERR_WrongParams;
         	return status;
         }

	 if( motor!= MotorA && motor!= MotorB)
		  {
			status= H18R1_ERR_WrongParams;
			return status;

		  }


		if (dutyCycle < 0 || dutyCycle > 100)
		{
			status= H18R1_ERR_WrongParams;
			return status;
		}



	MotorON();
	SetupMotor(direction,motor);
	MotorPWM(H_Bridge_PWM_FREQ, dutyCycle,motor);


	return status;
}

/*-----------------------------------------------------------*/


/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */
portBASE_TYPE CLI_Turn_ONCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H18R1_OK;

	H_BridgeMode direction;
	Motor motor;

	static int8_t *pcParameterString1;
	static int8_t *pcParameterString2;

	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;


	static const int8_t *pcOKMessage=(int8_t* )"H_Bridge is on \r\n  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";


	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	direction =(H_BridgeMode )atol((char* )pcParameterString1);

	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength2 );
	motor =(Motor )atol((char* )pcParameterString2);



	status=Turn_ON(direction,motor);
	if(status == H18R1_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,direction);

	}

	else if(status == H18R1_ERR_WrongParams)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);



	return pdFALSE;
}

/* ----------------------------------------------------------------------------*/
portBASE_TYPE CLI_Turn_OFFCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H18R1_OK;

	Motor motor;

	static int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;

	static const int8_t *pcOKMessage=(int8_t* )"H_Bridge is off \n\r";
	static const int8_t *pcErrorsMessage =(int8_t* )"Error Params!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

		pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
		motor =(Motor )atol((char* )pcParameterString1);

	 	status=Turn_OFF(motor);

	 if(status == H18R1_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage);

	 }

	 else if(status == H18R1_ERROR)
			strcpy((char* )pcWriteBuffer,(char* )pcErrorsMessage);


	return pdFALSE;

}
/* ----------------------------------------------------------------------------*/
portBASE_TYPE CLI_Turn_PWMCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H18R1_OK;

	H_BridgeMode direction;
	uint8_t dutyCycle;
	Motor motor;

	static int8_t *pcParameterString1;
	static int8_t *pcParameterString2;
	static int8_t *pcParameterString3;

	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;
	portBASE_TYPE xParameterStringLength3 =0;

	static const int8_t *pcOKMessage=(int8_t* )"H_Bridge is on in mode PWM in duty cycle   %d percent \r\n";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"WrongParams!\n\r";
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);


	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	direction =(H_BridgeMode )atol((char* )pcParameterString1);

	 pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength2 );
	 dutyCycle =(uint8_t )atol((char* )pcParameterString2);

	 pcParameterString3 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 3, &xParameterStringLength3 );
	 motor =(uint8_t )atol((char* )pcParameterString3);

	 status=Turn_PWM(direction, dutyCycle, motor);

	 if(status == H18R1_OK)
	 {
		 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,direction,dutyCycle);

	 }

	 else if(status == H18R1_ERR_WrongParams)
	 {
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);
	 }


	return pdFALSE;

}

/* ----------------------------------------------------------------------------*/


/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
