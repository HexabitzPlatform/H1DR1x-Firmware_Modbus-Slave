/*
* mb_slave_API.h
* Description   : Header file contains all the functions prototypes for the Modbus slave API.
*  Created on   : Sep 2, 2024
*      Author   : MAHMOOD_REDA @ Hexabitz
******************************************************************************
* @attention
*
* Copyright (c) 2024 Hexabitz.
* All rights reserved.
*
******************************************************************************
*/

#ifndef MB_SLAVE_API_H_
#define MB_SLAVE_API_H_

#include "mb.h"




eMBErrorCode MbSlaveMode(eMBMode eMode, UCHAR Slave_Address, UCHAR Port, ULONG Baud_Rate, eMBParity Parity);
void WriteModbusBuffer(USHORT *Buffer, uint16_t RegAdd, uint16_t NofReg);
void ReadModbusBuffer(USHORT *Buffer, uint16_t RegAdd, uint16_t NofReg);

#endif /* MB_SLAVE_API_H_ */

/************************ (C) COPYRIGHT Hexabitz *****END OF FILE****/
