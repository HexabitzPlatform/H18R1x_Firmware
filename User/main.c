/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int main(void){

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for(;;){}
}

/*-----------------------------------------------------------*/
/* Global variables */
Motor motor;
Module_Status motorTurnOff_Status;
H_BridgeDirection direction;
Module_Status motorTurnOn_Status;
uint8_t dutyCycle;
Module_Status motorSpeedControl_Status;

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument){

  // put your code here, to run repeatedly.


    while (1) {
//    	motorTurnOff_Status = MotorTurnOff(MOTOR_B);
//    	HAL_Delay(2222);
//    	motorTurnOn_Status = MotorTurnOn(2, MOTOR_B);
    	HAL_Delay(2222);
    	motorSpeedControl_Status = MotorSpeedControl(2, 20, MOTOR_B);
    	    	HAL_Delay(2222);
    	    	motorSpeedControl_Status = MotorSpeedControl(2, 70, MOTOR_B);

    	    	    	HAL_Delay(2222);
    	    	    	motorSpeedControl_Status = MotorSpeedControl(2, 99, MOTOR_B);

  }
}
/*-----------------------------------------------------------*/
