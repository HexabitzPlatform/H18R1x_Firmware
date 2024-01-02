/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
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

/* User Task */
void UserTask(void *argument){


	// put your code here, to run repeatedly.
	while(1){
		Turn_PWM(1,100, MotorA);
				Turn_PWM(1,100, MotorB);
				Delay_ms(2000);
				Turn_PWM(1,0, MotorA);
				Turn_PWM(1,0, MotorB);
				Delay_ms(2000);
				Turn_PWM(1,50, MotorA);
				Turn_PWM(1,50, MotorB);
				Delay_ms(2000);
				Turn_PWM(1,80, MotorA);
				Turn_PWM(1,80, MotorB);
				Delay_ms(2000);
				Turn_PWM(1,20, MotorA);
				Turn_PWM(1,20, MotorB);
				Delay_ms(2000);
				Turn_PWM(1,60, MotorA);
				Turn_PWM(1,60, MotorB);
				Delay_ms(2000);

	}
}

/*-----------------------------------------------------------*/
