/**
  ******************************************************************************
  * @file    portserial.c
  * @author  Motor Control Team, ST Microelectronics
  * @brief   FreeModbus Libary interface porting for STM32.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "port.h"
#include "mb.h"
#include "mbport.h"
#include "stm32g0xx_hal_uart.h"
#include "H1DR1_uart.h"
#include "stm32g0xx_ll_usart.h"
/* External variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
  if (xRxEnable) {
	    LL_USART_EnableIT_RXNE(SERIAL_PORT);
	  } else {
	    LL_USART_DisableIT_RXNE(SERIAL_PORT);
	  }


  if (xTxEnable) {
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
    LL_USART_EnableIT_TXE(SERIAL_PORT);
  }
  else {
//    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
    LL_USART_DisableIT_TXE(SERIAL_PORT);
  }
}
 
BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity)
{
  /* Performed by CubeMX */
	MB_UART_INIT_FN();
//HAL_UART_Init(SERIAL_PORT);
SET_BIT(SERIAL_PORT->CR1, USART_CR1_TCIE);
//  HAL_UART_Enable(SERIAL_PORT);
//  HAL_USART_EnableIT_TC(SERIAL_PORT);
  return TRUE;
}

HAL_StatusTypeDef result;
int ii ;
BOOL xMBPortSerialPutByte(CHAR ucByte)
{
	ii++;
	  LL_USART_TransmitData8(SERIAL_PORT, ucByte);
//  HAL_UART_Transmit(SERIAL_PORT, &ucByte, 1, HAL_MAX_DELAY);
  return TRUE;
}
 
BOOL xMBPortSerialGetByte(CHAR * pucByte)
{
	  *pucByte = (uint8_t)(LL_USART_ReceiveData8(SERIAL_PORT) & (uint8_t)0x00FF);
//  HAL_UART_Receive(SERIAL_PORT, (uint8_t *)pucByte, 1, HAL_MAX_DELAY);
  return TRUE;
}
/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
