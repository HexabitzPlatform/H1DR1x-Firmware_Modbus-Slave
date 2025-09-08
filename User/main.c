/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */

/* Includes ****************************************************************/
#include "BOS.h"

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
#define STARTING_ADDRESS     0
#define NUMBER_OF_REGISTERS  10
uint16_t Slave_Receive_buffer[10];
uint16_t Slave_Transmit_buffer[10] = { 4, 3, 2, 1, 0, 4, 3, 2, 1, 0 };
Module_Status Slave_Rstatus, Slave_wstatus;
void UserTask(void const *argument) {

	/* USER CODE BEGIN 2 */
	ModbusSlaveInit(2);
	/* USER CODE END 2 */

	/* Infinite loop */
	for (;;) {
		Delay_ms(500);
		Slave_Rstatus = ReadFromModbusBuffer(Slave_Receive_buffer, STARTING_ADDRESS, NUMBER_OF_REGISTERS);
	}
	/* USER CODE END StartDefaultTask */
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
