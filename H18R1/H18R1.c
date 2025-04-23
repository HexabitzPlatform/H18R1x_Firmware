/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H18R1.c
 Description   : Source code for module H18R1.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):

>> USARTs 1,2,3,5,6 for module ports.
>> Timer3 (Ch3) & Timer2 (Ch1) for L298 PWM.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H18R1_inputs.h"
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;

/* Module exported parameters ------------------------------------------------*/
module_param_t modParam[NUM_MODULE_PARAMS] ={0};

/* Private variables ---------------------------------------------------------*/

/*Timer for PWM*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;


/* Private function prototypes -----------------------------------------------*/

 void MX_TIM3_Init(void);
 void MX_TIM2_Init(void);
 void GPIO_MotorA_Init(void);
 void GPIO_MotorB_Init(void);
 Module_Status Turn_PWM(H_BridgeDirection direction,uint8_t dutyCycle,Motor motor);
 void ExecuteMonitor(void);



/* Create CLI commands --------------------------------------------------------*/
portBASE_TYPE CLI_Turn_ONCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_Turn_OFFCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_Turn_PWMCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure : Turn_ON */
const CLI_Command_Definition_t CLI_Turn_ONCommandDefinition =
{
	( const int8_t * ) "turn_on", /* The command string to type. */
	( const int8_t * ) "turn_on : Parameters required to execute a Turn_ON:\n\r 1)Direction: forward or backward\n\r 2)Motor on: MotorA or MotorB \n\r\n",
	CLI_Turn_ONCommand, /* The function to run. */
	2 /* two parameters are expected. */
};

/* CLI command structure : Turn_OFF */
const CLI_Command_Definition_t CLI_Turn_OFFCommandDefinition =
{
	( const int8_t * ) "turn_off", /* The command string to type. */
	( const int8_t * ) "turn_off :Parameters required to execute a Turn_OFF:\n\r 1)Motor off: MotorA or MotorB \n\r\n",
	CLI_Turn_OFFCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/* CLI command structure : Turn_PWM */
const CLI_Command_Definition_t CLI_Turn_PWMCommandDefinition =
{
	( const int8_t * ) "turn_pwm", /* The command string to type. */
	( const int8_t * ) "turn_pwm :Parameters required to execute a Turn_PWM:\n\r 1)Direction: forward or backward \n\r"
		" 2)dutyCycle: PWM duty cycle in precentage (0 to 100)% \n\r"
	    " 3)Motor on: MotorA or MotorB \n\r\n",
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
 *         This function configures the system clock as follows:
 *            - System Clock source            = PLL (HSE)
 *            - SYSCLK(Hz)                     = 64000000
 *            - HCLK(Hz)                       = 64000000
 *            - AHB Prescaler                  = 1
 *            - APB1 Prescaler                 = 1
 *            - HSE Frequency(Hz)              = 8000000
 *            - PLLM                           = 1
 *            - PLLN                           = 16
 *            - PLLP                           = 2
 *            - Flash Latency(WS)              = 2
 *            - Clock Source for UART1,UART2,UART3 = 16MHz (HSI)
 * @param  None
 * @retval None
 */
 void SystemClock_Config(void){
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE; // Enable both HSI and HSE oscillators
    RCC_OscInitStruct.HSEState = RCC_HSE_ON; // Enable HSE (External High-Speed Oscillator)
    RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Enable HSI (Internal High-Speed Oscillator)
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1; // No division on HSI
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration value for HSI
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // Enable PLL
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // Set PLL source to HSE
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1; // Prescaler for PLL input
    RCC_OscInitStruct.PLL.PLLN = 16; // Multiplication factor for PLL
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}
/*-----------------------------------------------------------*/

/* --- Save Command Topology in Flash RO --- */

uint8_t SaveTopologyToRO(void)
{
	HAL_StatusTypeDef flashStatus =HAL_OK;
	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd = 8;
    uint16_t temp =0;

    /* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();

	/* Erase Topology page */
	FLASH_PageErase(FLASH_BANK_2,TOPOLOGY_PAGE_NUM);

	/* Wait for an Erase operation to complete */
	flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(flashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}

	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save module's ID and topology */
	if(myID){

		/* Save module's ID */
		temp =(uint16_t )(N << 8) + myID;

		/* Save module's ID in Flash memory */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS,temp);

		/* Wait for a Write operation to complete */
		flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

		if(flashStatus != HAL_OK){
			/* return FLASH error code */
			return pFlash.ErrorCode;
		}

		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}

		/* Save topology */
		for(uint8_t row =1; row <= N; row++){
			for(uint8_t column =0; column <= MaxNumOfPorts; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(array[row - 1][0]){
					/* Save each element in topology array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,array[row - 1][column]);
					/* Wait for a Write operation to complete */
					flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(flashStatus != HAL_OK){
						/* return FLASH error code */
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						/* update new flash memory address */
						flashAdd += 8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/*-----------------------------------------------------------*/

/* --- Save Command Snippets in Flash RO --- */

uint8_t SaveSnippetsToRO(void)
{
	HAL_StatusTypeDef FlashStatus =HAL_OK;
    uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};

    /* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();
    /* Erase Snippets page */
	FLASH_PageErase(FLASH_BANK_2,SNIPPETS_PAGE_NUM);
	/* Wait for an Erase operation to complete */
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(FlashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save Command Snippets */
	int currentAdd = SNIPPETS_START_ADDRESS;
	for(uint8_t index = 0; index < numOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(snippets[index].cond.conditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&snippets[index],sizeof(snippet_t));
			/* Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(snippet_t)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j*8]);
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
			/* Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped */
			for(uint8_t j = 0; j < ((strlen(snippets[index].cmd) + 1)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(snippets[index].cmd + j*4 ));
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd += 8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/*-----------------------------------------------------------*/

/* --- Clear array topology in SRAM and Flash RO --- */

uint8_t ClearROtopology(void){
	// Clear the array 
	memset(array,0,sizeof(array));
	N =1;
	myID =0;
	
	return SaveTopologyToRO();
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
	HAL_UART_DeInit(huart);
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);
	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);

}

/* H18R1 module initialization */
void Module_Peripheral_Init(void){

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	/* H_Bridge GPIO Init */
	GPIO_MotorA_Init();
	GPIO_MotorB_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();

	/* Circulating DMA Channels ON All Module */
	for(int i =1; i <= NUM_OF_PORTS; i++){
		if(GetUart(i) == &huart1){
			dmaIndex[i - 1] =&(DMA1_Channel1->CNDTR);
		}
		else if(GetUart(i) == &huart2){
			dmaIndex[i - 1] =&(DMA1_Channel2->CNDTR);
		}
		else if(GetUart(i) == &huart3){
			dmaIndex[i - 1] =&(DMA1_Channel3->CNDTR);
		}
		else if(GetUart(i) == &huart4){
			dmaIndex[i - 1] =&(DMA1_Channel4->CNDTR);
		}
		else if(GetUart(i) == &huart5){
			dmaIndex[i - 1] =&(DMA1_Channel5->CNDTR);
		}
		else if(GetUart(i) == &huart6){
			dmaIndex[i - 1] =&(DMA1_Channel6->CNDTR);
		}
	}
}

/***************************************************************************/
/* This functions is useful only for input (sensors) modules.
 * @brief: Samples a module parameter value based on parameter index.
 * @param paramIndex: Index of the parameter (1-based index).
 * @param value: Pointer to store the sampled float value.
 * @retval: Module_Status indicating success or failure.
 */
Module_Status GetModuleParameter(uint8_t paramIndex,float *value){
	Module_Status status =BOS_OK;

	switch(paramIndex){

		/* Invalid parameter index */
		default:
			status =BOS_ERR_WrongParam;
			break;
	}

	return status;
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

/* --- Set Motor PWM dutycycle ---*/
Module_Status MotorPWM(uint8_t dutycycle,Motor motor)
{
	Module_Status  status=H18R1_OK;
	uint32_t period =100;
	switch(motor)
	{
			case MotorA:
				status=H18R1_OK;
				htim3.Instance->ARR = period - 1;
				htim3.Instance->CCR3 = ((float) dutycycle / 100.0f) * period;
				break;
			case MotorB:
				status=H18R1_OK;
				htim2.Instance->ARR = period - 1;
				htim2.Instance->CCR1 = ((float) dutycycle / 100.0f) * period;
				break;
			default:
				status=H18R1_ERR_WrongMotor;
				break;
	}
		return status;
}

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |								  APIs							          | 																 	|
/* -----------------------------------------------------------------------


/*--------------Run the motor at full speed------------*/
Module_Status Turn_ON(H_BridgeDirection direction,Motor motor)
{
	Module_Status status=H18R1_OK;
	Turn_PWM(direction,100, motor);
	return status;
}
/*-----------------------------------------------------------------------------*/

/*------------------Off the motor---------------*/
Module_Status Turn_OFF(Motor motor)
{

	Module_Status  status=H18R1_OK;
	if(motor==MotorA)
	    {
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		htim3.Instance->CCR3=0;
			status=H18R1_OK;
	    }
	else if(motor==MotorB)
	    {
	   	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	  	htim2.Instance->CCR1=0;
		status=H18R1_OK;
		}
	else{
		status=H18R1_ERR_WrongMotor;
		}
	return status;
}


/*-----------------------------------------------------------------------------*/
/* --- Turn-on H_Bridge with pulse-width modulation (PWM) ---
 dutyCycle: PWM duty cycle in precentage (0 to 100)
 */
Module_Status Turn_PWM(H_BridgeDirection direction,uint8_t dutyCycle,Motor motor)
{
	Module_Status  status=H18R1_OK;
	if (dutyCycle < 0 || dutyCycle > 100)
		{
		    status=  H18R1_ERR_WrongDutyCycle;
			return status;
		}
	else if(motor== MotorA )
	    {
//		   GPIO_MotorA_Init();
	       MX_TIM3_Init();
	       HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
			if( direction== forward)
			{   HAL_GPIO_WritePin(IN1_GPIO_Port ,IN1_Pin ,GPIO_PIN_SET);
			    HAL_GPIO_WritePin(IN2_GPIO_Port ,IN2_Pin ,GPIO_PIN_RESET);
			    status=H18R1_OK;
			}
			else if ( direction== backward)
			{
			    HAL_GPIO_WritePin(IN1_GPIO_Port ,IN1_Pin ,GPIO_PIN_RESET);
			    HAL_GPIO_WritePin(IN2_GPIO_Port ,IN2_Pin ,GPIO_PIN_SET);
			    status=H18R1_OK;
			}
			else
			{  status= H18R1_ERR_WrongDirection;
			return status;
			}
	    }
	 else if(motor== MotorB )
		{
//		    GPIO_MotorB_Init();
		    MX_TIM2_Init();
		    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			if(direction== forward)
			{
			    HAL_GPIO_WritePin(IN3_GPIO_Port ,IN3_Pin ,GPIO_PIN_SET);
			    HAL_GPIO_WritePin(IN4_GPIO_Port ,IN4_Pin ,GPIO_PIN_RESET);
			    status=H18R1_OK;
			}
		    else if (direction== backward)
			{
		     HAL_GPIO_WritePin(IN3_GPIO_Port ,IN3_Pin ,GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(IN4_GPIO_Port ,IN4_Pin ,GPIO_PIN_SET);
			 status=H18R1_OK;
			}
			else
			{   status= H18R1_ERR_WrongDirection;
			return status;
			}
		}
	   else
	   {
			status= H18R1_ERR_WrongMotor;
			return status;
	   }

	     MotorPWM(dutyCycle,motor);
	     return status;
}


/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */
portBASE_TYPE CLI_Turn_ONCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H18R1_OK;

	H_BridgeDirection direction;
	Motor motor;

	static int8_t *pcParameterString1;
	static int8_t *pcParameterString2;

	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;


	static const int8_t *pcOKMessage=(int8_t* )"The Motor %d is running in the direction %d \r\n  \n\r";
	static const int8_t *pcWrongDirectionMessage =(int8_t* )"WrongDirection!\n\r";
	static const int8_t *pcWrongMotorMessage =(int8_t* )"WrongMotor!\n\r";

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	direction =(H_BridgeDirection )atol((char* )pcParameterString1);

	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength2 );
	motor =(Motor )atol((char* )pcParameterString2);



	status=Turn_ON(direction,motor);
	if(status == H18R1_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,motor,direction);

	}

	else if(status == H18R1_ERR_WrongMotor)
			strcpy((char* )pcWriteBuffer,(char* )pcWrongMotorMessage);

	else if(status == H18R1_ERR_WrongDirection)
			strcpy((char* )pcWriteBuffer,(char* )pcWrongDirectionMessage);

	return pdFALSE;
}

/* ----------------------------------------------------------------------------*/
portBASE_TYPE CLI_Turn_OFFCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H18R1_OK;

	Motor motor;

	static int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;

	static const int8_t *pcOKMessage=(int8_t* )"The Motor %d is off \n\r";
	static const int8_t *pcWrongMotorMessage =(int8_t* )"WrongMotor!\n\r";

		(void )xWriteBufferLen;
		configASSERT(pcWriteBuffer);

		pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
		motor =(Motor )atol((char* )pcParameterString1);

	 	status=Turn_OFF(motor);

	 if(status == H18R1_OK)
	 {
			 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,motor);

	 }

	 else if(status == H18R1_ERR_WrongMotor)
			strcpy((char* )pcWriteBuffer,(char* )pcWrongMotorMessage);


	return pdFALSE;

}
/* ----------------------------------------------------------------------------*/
portBASE_TYPE CLI_Turn_PWMCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H18R1_OK;

	H_BridgeDirection direction;
	uint8_t dutyCycle;
	Motor motor;

	static int8_t *pcParameterString1;
	static int8_t *pcParameterString2;
	static int8_t *pcParameterString3;

	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;
	portBASE_TYPE xParameterStringLength3 =0;

	static const int8_t *pcOKMessage=(int8_t* )"The Motor %d is running  PWM in duty cycle %d percent and the direction %d  \r\n";
	static const int8_t *pcWrongDutyCycleMessage =(int8_t* )"WrongDutyCycle!\n\r";
	static const int8_t *pcWrongMotorMessage =(int8_t* )"WrongMotor!\n\r";
	static const int8_t *pcWrongDirectionMessage =(int8_t* )"WrongDirection!\n\r";


	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);


	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	direction =(H_BridgeDirection )atol((char* )pcParameterString1);

	 pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength2 );
	 dutyCycle =(uint8_t )atol((char* )pcParameterString2);

	 pcParameterString3 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 3, &xParameterStringLength3 );
	 motor =(uint8_t )atol((char* )pcParameterString3);

	 status=Turn_PWM(direction, dutyCycle, motor);

	 if(status == H18R1_OK)
	 {
		 sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,motor,dutyCycle,direction);

	 }

	 else if(status == H18R1_ERR_WrongDutyCycle)
	 		 strcpy((char* )pcWriteBuffer,(char* )pcWrongDutyCycleMessage);

	 else if(status == H18R1_ERR_WrongMotor)
		 strcpy((char* )pcWriteBuffer,(char* )pcWrongMotorMessage);


	 else if(status == H18R1_ERR_WrongDirection)
	 		strcpy((char* )pcWriteBuffer,(char* )pcWrongDirectionMessage);



	return pdFALSE;

}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
