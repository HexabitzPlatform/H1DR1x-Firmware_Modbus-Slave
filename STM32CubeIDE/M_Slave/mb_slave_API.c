/*
* mb_slave_API.c
* Description   : Source file contains all the functions for the Modbus slave API.
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

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbtask.h"

extern USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

void MX_NVIC_Init(void);

/********************************************************************/
/*
 * @brief: Initializes the Modbus slave mode.
 * @param1: eMode - the mode of operation (e.g., RTU, ASCII).
 * @param2: Slave_Address - the address of the slave device.
 * @param3: Port - the communication port to be used.
 * @param4: Baud_Rate - the baud rate for communication.
 * @param5: Parity - the parity setting (e.g., none, even, odd).
 * @retval: eMBErrorCode - status of the initialization process.
 */
eMBErrorCode MbSlaveMode(eMBMode eMode, UCHAR Slave_Address, UCHAR Port, ULONG Baud_Rate, eMBParity Parity) {
    MX_NVIC_Init();
    eMBErrorCode eStatus = eMBInit(eMode, Slave_Address, Port, Baud_Rate, Parity);
    eStatus = eMBEnable();
    return eStatus;
}

/********************************************************************/
/*
 * @brief: Writes data to the Modbus holding register buffer.
 * @param1: Buffer - pointer to a buffer containing the data to be written.
 * @param2: RegAdd - starting address of the register to write to.
 * @param3: NofReg - number of registers to write.
 * @retval: None
 */
void WriteModbusBuffer(USHORT *Buffer, uint16_t RegAdd, uint16_t NofReg) {
    int Index = RegAdd;
    while (RegAdd < RegAdd+NofReg) {
        usRegHoldingBuf[Index] = *Buffer++;
        NofReg--;
        Index++;
    }
}

/********************************************************************/
/*
 * @brief: Reads data from the Modbus holding register buffer.
 * @param1: Buffer - pointer to a buffer where the read data will be stored.
 * @param2: RegAdd - starting address of the register to read from.
 * @param3: NofReg - number of registers to read.
 * @retval: None
 */
void ReadModbusBuffer(USHORT *Buffer, uint16_t RegAdd, uint16_t NofReg) {
    int Index = RegAdd;
    while (RegAdd < RegAdd+NofReg) {
        *Buffer++ = usRegHoldingBuf[Index];
        NofReg--;
        Index++;
    }
}

/********************************************************************/
/*
 * @brief: Initializes the NVIC (Nested Vectored Interrupt Controller) with specific priorities and enables the interrupts.
 * @param: None
 * @retval: None
 */
void MX_NVIC_Init(void) {

    /* DMA1_Channel1_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* TIM14_IRQn interrupt configuration */
    NVIC_SetPriority(TIM14_IRQn, 2);
    NVIC_EnableIRQ(TIM14_IRQn);

    /* USART1_IRQn interrupt configuration */
    NVIC_SetPriority(USART1_IRQn, 3);
    NVIC_EnableIRQ(USART1_IRQn);

//    HAL_NVIC_SetPriority(TIM16_FDCAN_IT0_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(TIM16_FDCAN_IT0_IRQn);
    /* TIM16_IRQn interrupt configuration */
    NVIC_SetPriority(TIM16_FDCAN_IT0_IRQn, 2);
    NVIC_EnableIRQ(TIM16_FDCAN_IT0_IRQn);
}

/************************ (C) COPYRIGHT Hexabitz *****END OF FILE****/
