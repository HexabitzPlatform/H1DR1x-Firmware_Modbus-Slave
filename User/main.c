/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */

/* Includes ****************************************************************/
#include "BOS.h"
#include "mb.h"
#include "mb_API.h"
#include "port.h"
#include "mb_slave_API.h"
/* Private variables *******************************************************/

/* Private Function Prototypes *********************************************/

/* Main Function ***********************************************************/
int main(void) {

	/* Initialize Module &  BitzOS */
	Module_Init();

	/* Don't place your code here */
	for (;;) {
	}
}
/***************************************************************************/

uint16_t qq[10]={0x44,0x55,0x55,0x55 ,0x00 ,0xff ,0x55,0x22} ;
uint16_t ss[10];
Module_Status s ;
void UserTask(void const *argument) {

	 /* USER CODE BEGIN 2 */
	ModbusSlaveInit(MB_SLAVE_ADDRESS);
	  /* USER CODE END 2 */

	/* Infinite loop */
	for (;;) {

		ReadFromModbusBuffer(ss, 0, 10);
	}
	/* USER CODE END StartDefaultTask */
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
