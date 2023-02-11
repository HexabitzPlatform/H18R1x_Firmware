/*
 BitzOS (BOS) V0.2.7 - Copyright (C) 2017-2022 Hexabitz
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

//Turn_ON(forward, MotorA);
//Delay_ms(2000);
//Turn_ON(backward, MotorB);
////Delay_ms(2000);
	Turn_PWM(backward, 80, MotorA);
	Turn_PWM(backward, 80, MotorB);
	// put your code here, to run repeatedly.
	while(1){
//		Turn_ON(forward, MotorB);
//		Delay_ms(1000);
//		Turn_OFF(MotorA);
//		Delay_ms(10);
//		Turn_ON(backward, MotorA);
//		Turn_PWM(backward, 100, MotorA);
//				Delay_ms(1000);
//				Turn_OFF(MotorB);
//				Delay_ms(10);
//		Turn_ON(forward, MotorA);
//		Delay_ms(1000);

	}
}

/*-----------------------------------------------------------*/
