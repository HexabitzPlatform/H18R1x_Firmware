/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H18R1.c
 Description   : Source code for module H18R1.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):

>> USARTs 1,2,3,5,6 for module ports.
>> Timer3 (Ch3) & Timer2 (Ch1) for L298 PWM.

 */

/* Includes ****************************************************************/
#include "BOS.h"
#include "H18R1_inputs.h"

/* Exported Typedef ********************************************************/
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* Private Variables *******************************************************/


/* Module Parameters */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] = { 0 };


/* Private Function Prototypes *********************************************/
void MX_TIM3_Init(void);
void MX_TIM2_Init(void);
void Module_Peripheral_Init(void);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
uint8_t ClearROtopology(void);
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift);

/* Local Function Prototypes ***********************************************/
Module_Status MotorPWM( uint8_t dutycycle,Motor motor);

/* Create CLI commands *****************************************************/
portBASE_TYPE CLI_Turn_ONCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_Turn_OFFCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_Turn_PWMCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure ***************************************************/
/* CLI command structure : MotorTurnOn */
const CLI_Command_Definition_t CLI_Turn_ONCommandDefinition = {
	( const int8_t * ) "turn_on", /* The command string to type. */
	( const int8_t * ) "turn_on : Parameters required to execute a MotorTurnOn:\n\r 1)Direction: FORWARD_DIR or BACKWARD_DIR\n\r 2)Motor on: MOTOR_A or MOTOR_B \n\r\n",
	CLI_Turn_ONCommand, /* The function to run. */
	2 /* two parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : MotorTurnOff */
const CLI_Command_Definition_t CLI_Turn_OFFCommandDefinition = {
	( const int8_t * ) "turn_off", /* The command string to type. */
	( const int8_t * ) "turn_off :Parameters required to execute a MotorTurnOff:\n\r 1)Motor off: MOTOR_A or MOTOR_B \n\r\n",
	CLI_Turn_OFFCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : MotorSpeedControl */
const CLI_Command_Definition_t CLI_Turn_PWMCommandDefinition = {
	( const int8_t * ) "turn_pwm", /* The command string to type. */
	( const int8_t * ) "turn_pwm :Parameters required to execute a MotorSpeedControl:\n\r 1)Direction: FORWARD_DIR or BACKWARD_DIR \n\r"
		" 2)dutyCycle: PWM duty cycle in precentage (0 to 100)% \n\r"
	    " 3)Motor on: MOTOR_A or MOTOR_B \n\r\n",
	CLI_Turn_PWMCommand, /* The function to run. */
	3 /* three parameters are expected. */
};

/***************************************************************************/
/************************ Private function Definitions *********************/
/***************************************************************************/
/* @brief  System Clock Configuration
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
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Initializes the RCC Oscillators according to the specified parameters
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

/***************************************************************************/
/* enable stop mode regarding only UART1 , UART2 , and UART3 */
BOS_Status EnableStopModebyUARTx(uint8_t port) {

	UART_WakeUpTypeDef WakeUpSelection;
	UART_HandleTypeDef *huart = GetUart(port);

	if ((huart->Instance == USART1) || (huart->Instance == USART2) || (huart->Instance == USART3)) {

		/* make sure that no UART transfer is on-going */
		while (__HAL_UART_GET_FLAG(huart, USART_ISR_BUSY) == SET);

		/* make sure that UART is ready to receive */
		while (__HAL_UART_GET_FLAG(huart, USART_ISR_REACK) == RESET);

		/* set the wake-up event:
		 * specify wake-up on start-bit detection */
		WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
		HAL_UARTEx_StopModeWakeUpSourceConfig(huart, WakeUpSelection);

		/* Enable the UART Wake UP from stop mode Interrupt */
		__HAL_UART_ENABLE_IT(huart, UART_IT_WUF);

		/* enable MCU wake-up by LPUART */
		HAL_UARTEx_EnableStopMode(huart);

		/* enter STOP mode */
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	} else
		return BOS_ERROR;

}

/***************************************************************************/
/* Enable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status EnableStandbyModebyWakeupPinx(WakeupPins_t wakeupPins) {

	/* Clear the WUF FLAG */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

	/* Enable the WAKEUP PIN */
	switch (wakeupPins) {

	case PA0_PIN:
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
		break;

	case PA2_PIN:
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
		break;

	case PB5_PIN:
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
		break;

	case PC13_PIN:
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
		break;

	case NRST_PIN:
		/* do no thing*/
		break;
	}

	/* Enable SRAM content retention in Standby mode */
	HAL_PWREx_EnableSRAMRetention();

	/* Finally enter the standby mode */
	HAL_PWR_EnterSTANDBYMode();

	return BOS_OK;
}

/***************************************************************************/
/* Disable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status DisableStandbyModeWakeupPinx(WakeupPins_t wakeupPins) {

	/* The standby wake-up is same as a system RESET:
	 * The entire code runs from the beginning just as if it was a RESET.
	 * The only difference between a reset and a STANDBY wake-up is that, when the MCU wakes-up,
	 * The SBF status flag in the PWR power control/status register (PWR_CSR) is set */
	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) {
		/* clear the flag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		/* Disable  Wake-up Pinx */
		switch (wakeupPins) {

		case PA0_PIN:
			HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
			break;

		case PA2_PIN:
			HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
			break;

		case PB5_PIN:
			HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
			break;

		case PC13_PIN:
			HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
			break;

		case NRST_PIN:
			/* do no thing*/
			break;
		}

		IND_blink(1000);

	} else
		return BOS_OK;

}

/***************************************************************************/
/* Save Command Topology in Flash RO */
uint8_t SaveTopologyToRO(void) {

	HAL_StatusTypeDef flashStatus = HAL_OK;

	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd = 8;
	uint16_t temp = 0;

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();

	/* Erase Topology page */
	FLASH_PageErase(FLASH_BANK_2, TOPOLOGY_PAGE_NUM);

	/* Wait for an Erase operation to complete */
	flashStatus = FLASH_WaitForLastOperation((uint32_t) HAL_FLASH_TIMEOUT_VALUE);

	if (flashStatus != HAL_OK) {
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}

	else {
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}

	/* Save module's ID and topology */
	if (myID) {

		/* Save module's ID */
		temp = (uint16_t) (N << 8) + myID;

		/* Save module's ID in Flash memory */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, TOPOLOGY_START_ADDRESS, temp);

		/* Wait for a Write operation to complete */
		flashStatus = FLASH_WaitForLastOperation((uint32_t) HAL_FLASH_TIMEOUT_VALUE);

		if (flashStatus != HAL_OK) {
			/* return FLASH error code */
			return pFlash.ErrorCode;
		}

		else {
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
		}

		/* Save topology */
		for (uint8_t row = 1; row <= N; row++) {
			for (uint8_t column = 0; column <= MAX_NUM_OF_PORTS; column++) {
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if (Array[row - 1][0]) {
					/* Save each element in topology Array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, TOPOLOGY_START_ADDRESS + flashAdd,
							Array[row - 1][column]);
					/* Wait for a Write operation to complete */
					flashStatus = FLASH_WaitForLastOperation((uint32_t) HAL_FLASH_TIMEOUT_VALUE);
					if (flashStatus != HAL_OK) {
						/* return FLASH error code */
						return pFlash.ErrorCode;
					} else {
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
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

/***************************************************************************/
/* Save Command Snippets in Flash RO */
uint8_t SaveSnippetsToRO(void) {
	HAL_StatusTypeDef FlashStatus = HAL_OK;
	uint8_t snipBuffer[sizeof(Snippet_t) + 1] = { 0 };

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();
	/* Erase Snippets page */
	FLASH_PageErase(FLASH_BANK_2, SNIPPETS_PAGE_NUM);
	/* Wait for an Erase operation to complete */
	FlashStatus = FLASH_WaitForLastOperation((uint32_t) HAL_FLASH_TIMEOUT_VALUE);

	if (FlashStatus != HAL_OK) {
		/* return FLASH error code */
		return pFlash.ErrorCode;
	} else {
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}

	/* Save Command Snippets */
	int currentAdd = SNIPPETS_START_ADDRESS;
	for (uint8_t index = 0; index < NumOfRecordedSnippets; index++) {
		/* Check if Snippet condition is true or false */
		if (Snippets[index].Condition.ConditionType) {
			/* A marker to separate Snippets */
			snipBuffer[0] = 0xFE;
			memcpy((uint32_t*) &snipBuffer[1], (uint8_t*) &Snippets[index], sizeof(Snippet_t));
			/* Copy the snippet struct buffer (20 x NumOfRecordedSnippets). Note this is assuming sizeof(Snippet_t) is even */
			for (uint8_t j = 0; j < (sizeof(Snippet_t) / 4); j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAdd, *(uint64_t*) &snipBuffer[j * 8]);
				FlashStatus = FLASH_WaitForLastOperation((uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 8;
				}
			}
			/* Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped */
			for (uint8_t j = 0; j < ((strlen(Snippets[index].CMD) + 1) / 4); j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAdd, *(uint64_t*) (Snippets[index].CMD + j * 4));
				FlashStatus = FLASH_WaitForLastOperation((uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Clear Array topology in SRAM and Flash RO */
uint8_t ClearROtopology(void) {
	/* Clear the Array */
	memset(Array, 0, sizeof(Array));
	N = 1;
	myID = 0;

	return SaveTopologyToRO();
}

/***************************************************************************/
/* Trigger ST factory bootloader update for a remote module */
void RemoteBootloaderUpdate(uint8_t src, uint8_t dst, uint8_t inport, uint8_t outport) {

	uint8_t myOutport = 0, lastModule = 0;
	int8_t *pcOutputString;

	/* 1. Get Route to destination module */
	myOutport = FindRoute(myID, dst);
	if (outport && dst == myID) { /* This is a 'via port' update and I'm the last module */
		myOutport = outport;
		lastModule = myID;
	} else if (outport == 0) { /* This is a remote update */
		if (NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = Route[NumberOfHops(dst)-1]; /* previous module = Route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if (src == myID) {
		/* Obtain the address of the output buffer.  Note there is no mutual
		 * exclusion on this buffer as it is assumed only one command console
		 * interface will be used at any one time. */
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		if (outport == 0)		// This is a remote module update
			sprintf((char*) pcOutputString, pcRemoteBootloaderUpdateMessage, dst);
		else
			// This is a 'via port' remote update
			sprintf((char*) pcOutputString, pcRemoteBootloaderUpdateViaPortMessage, dst, outport);

		strcat((char*) pcOutputString, pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport, (char*) pcOutputString, strlen((char*) pcOutputString), cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);

	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport, myID, myOutport, myID, BIDIRECTIONAL, 0xFFFFFFFF, 0xFFFFFFFF, false);
}

/***************************************************************************/
/* Setup a port for remote ST factory bootloader update:
 * Set baudrate to 57600
 * Enable even parity
 * Set datasize to 9 bits
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

/***************************************************************************/
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

	uint8_t Motor =0;
	uint8_t dutyCycle =0;
	uint8_t direction =0;

	switch(code){
		case CODE_H18R1_TURN_ON:
			direction =(uint8_t )cMessage[port - 1][shift];
			Motor =(uint8_t )cMessage[port - 1][1 + shift];
			MotorTurnOn(direction,Motor);
			break;

		case CODE_H18R1_TURN_OFF:
			Motor =(uint8_t )cMessage[port - 1][shift];
			MotorTurnOff(Motor);
			break;

		case CODE_H18R1_TURN_PWM:
			direction =(uint8_t )cMessage[port - 1][shift];
			dutyCycle =(uint8_t )cMessage[port - 1][1 + shift];
			Motor =(uint8_t )cMessage[port - 1][2 + shift];
			MotorSpeedControl(direction,dutyCycle,Motor);
			break;

		default:
			result =H18R1_ERR_UNKNOWNMESSAGE;
			break;
	}
	
	return result;
}

/***************************************************************************/
/* Get the port for a given UART */
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

/***************************************************************************/
/* --- Register this module CLI Commands */
void RegisterModuleCLICommands(void){
	FreeRTOS_CLIRegisterCommand(&CLI_Turn_ONCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_Turn_OFFCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_Turn_PWMCommandDefinition);

}

/***************************************************************************/
/****************************** Local Functions ****************************/
/***************************************************************************/
/* Set Motor PWM duty cycle */
Module_Status MotorPWM(uint8_t dutycycle,Motor motor){
	Module_Status status =H18R1_OK;
	uint32_t period =100;

	switch(motor){
		case MOTOR_A:
			MOTOR_A_ARR =period - 1;
			MOTOR_A_CCR =((float )dutycycle / 100.0f) * period;
			break;

		case MOTOR_B:
			MOTOR_B_ARR =period - 1;
			MOTOR_B_CCR =((float )dutycycle / 100.0f) * period;
			break;

		default:
			status =H18R1_ERR_WRONGMOTOR;
			break;
	}
	return status;
}

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
/* Run the motor at full speed */
Module_Status MotorTurnOn(H_BridgeDirection direction,Motor motor){
	Module_Status status =H18R1_OK;

	MotorSpeedControl(direction,100,motor);

	return status;
}

/***************************************************************************/
/* Off the motor */
Module_Status MotorTurnOff(Motor motor){
	Module_Status status =H18R1_OK;

	if(motor == MOTOR_A){
		HAL_TIM_PWM_Stop(MOTOR_A_TIM_HANDLE,MOTOR_A_TIM_CH);
		MOTOR_A_CCR =0;
	}
	else if(motor == MOTOR_B){
		HAL_TIM_PWM_Stop(MOTOR_B_TIM_HANDLE,MOTOR_B_TIM_CH);
		MOTOR_B_CCR =0;
	}
	else
		status =H18R1_ERR_WRONGMOTOR;

	return status;
}

/***************************************************************************/
/* Turn-on H_Bridge with pulse-width modulation (PWM)
 * dutyCycle: PWM duty cycle in precentage (0 to 100)
 */
Module_Status MotorSpeedControl(H_BridgeDirection direction,uint8_t dutyCycle,Motor motor){
	Module_Status status =H18R1_OK;

	if(dutyCycle < 0 || dutyCycle > 100){
		status =H18R1_ERR_WRONGDUTYCYCLE;
		return status;
	}

	/* Motor A Selected ****************************************************/
	else if(motor == MOTOR_A){
		HAL_TIM_PWM_Start(MOTOR_A_TIM_HANDLE,MOTOR_A_TIM_CH);

		if(direction == FORWARD_DIR){
			HAL_GPIO_WritePin(MOTOR_A_IN1_PORT,MOTOR_A_IN1_PIN,GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR_A_IN2_PORT,MOTOR_A_IN2_PIN,GPIO_PIN_RESET);
		}
		else if(direction == BACKWARD_DIR){
			HAL_GPIO_WritePin(MOTOR_A_IN1_PORT,MOTOR_A_IN1_PIN,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_A_IN2_PORT,MOTOR_A_IN2_PIN,GPIO_PIN_SET);
		}
		else{
			status =H18R1_ERR_WRONGDIRECTION;
			return status;
		}
	}

	/* Motor B Selected ****************************************************/
	else if(motor == MOTOR_B){
		HAL_TIM_PWM_Start(MOTOR_B_TIM_HANDLE,MOTOR_B_TIM_CH);

		if(direction == FORWARD_DIR){
			HAL_GPIO_WritePin(MOTOR_B_IN3_PORT,MOTOR_B_IN3_PIN,GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR_B_IN4_PORT,MOTOR_B_IN4_PIN,GPIO_PIN_RESET);
		}
		else if(direction == BACKWARD_DIR){
			HAL_GPIO_WritePin(MOTOR_B_IN3_PORT,MOTOR_B_IN3_PIN,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_B_IN4_PORT,MOTOR_B_IN4_PIN,GPIO_PIN_SET);
		}
		else{
			status =H18R1_ERR_WRONGDIRECTION;
			return status;
		}
	}

	else{
		status =H18R1_ERR_WRONGMOTOR;
		return status;
	}

	/* Start Motor */
	MotorPWM(dutyCycle,motor);

	return status;
}

/***************************************************************************/
/********************************* Commands ********************************/
/***************************************************************************/
portBASE_TYPE CLI_Turn_ONCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H18R1_OK;

	H_BridgeDirection direction;
	Motor motor;

	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;

	static int8_t *pcParameterString1;
	static int8_t *pcParameterString2;
	static const int8_t *pcOKMessage=(int8_t* )"The Motor %d is running in the direction %d \r\n  \n\r";
	static const int8_t *pcWrongDirectionMessage =(int8_t* )"WrongDirection!\n\r";
	static const int8_t *pcWrongMotorMessage =(int8_t* )"WrongMotor!\n\r";

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	direction =(H_BridgeDirection )atol((char* )pcParameterString1);

	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength2 );
	motor =(Motor )atol((char* )pcParameterString2);

	status=MotorTurnOn(direction,motor);
	if(status == H18R1_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,motor,direction);

	else if(status == H18R1_ERR_WRONGMOTOR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongMotorMessage);

	else if(status == H18R1_ERR_WRONGDIRECTION)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongDirectionMessage);

	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_Turn_OFFCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H18R1_OK;

	Motor motor;

	portBASE_TYPE xParameterStringLength1 =0;

	static int8_t *pcParameterString1;
	static const int8_t *pcOKMessage =(int8_t* )"The Motor %d is off \n\r";
	static const int8_t *pcWrongMotorMessage =(int8_t* )"WrongMotor!\n\r";

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	motor =(Motor )atol((char* )pcParameterString1);

	status =MotorTurnOff(motor);

	if(status == H18R1_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,motor);

	else if(status == H18R1_ERR_WRONGMOTOR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongMotorMessage);

	return pdFALSE;

}

/***************************************************************************/
portBASE_TYPE CLI_Turn_PWMCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H18R1_OK;

	H_BridgeDirection direction;
	uint8_t dutyCycle;
	Motor motor;

	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;
	portBASE_TYPE xParameterStringLength3 =0;

	static int8_t *pcParameterString1;
	static int8_t *pcParameterString2;
	static int8_t *pcParameterString3;
	static const int8_t *pcOKMessage=(int8_t* )"The Motor %d is running  PWM in duty cycle %d percent and the direction %d  \r\n";
	static const int8_t *pcWrongDutyCycleMessage =(int8_t* )"WrongDutyCycle!\n\r";
	static const int8_t *pcWrongMotorMessage =(int8_t* )"WrongMotor!\n\r";
	static const int8_t *pcWrongDirectionMessage =(int8_t* )"WrongDirection!\n\r";

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	direction =(H_BridgeDirection )atol((char* )pcParameterString1);

	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,2,&xParameterStringLength2);
	dutyCycle =(uint8_t )atol((char* )pcParameterString2);

	pcParameterString3 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,3,&xParameterStringLength3);
	motor =(uint8_t )atol((char* )pcParameterString3);

	status =MotorSpeedControl(direction,dutyCycle,motor);

	if(status == H18R1_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,motor,dutyCycle,direction);

	else if(status == H18R1_ERR_WRONGDUTYCYCLE)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongDutyCycleMessage);

	else if(status == H18R1_ERR_WRONGMOTOR)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongMotorMessage);

	else if(status == H18R1_ERR_WRONGDIRECTION)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongDirectionMessage);

	return pdFALSE;

}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
