/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "port.h"
#include "stm32g4xx_hal.h"
/* ----------------------- Modbus includes ----------------------------------*/
//#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
extern	UART_HandleTypeDef * adr_huart_MB;
/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
    if(xRxEnable)
  {
    __HAL_UART_ENABLE_IT(adr_huart_MB, UART_IT_RXNE);
  }
  else
  {
    __HAL_UART_DISABLE_IT(adr_huart_MB, UART_IT_RXNE);
  }

  if(xTxEnable)
  {
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
   // __HAL_UART_ENABLE_IT(adr_huart_MB, UART_IT_TXE);
  }
  else
  {
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    //__HAL_UART_DISABLE_IT(adr_huart_MB, UART_IT_TXE);
  }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	  // надо сделать проверку
	  return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Поместите байт в буфер передачи Uart.
     * Эта функция вызывается стеком протоколов, если был вызван pxMBFrameCBTransmitterEmpty( ).*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////	hlpuart1.Instance->TDR =ucByte;
      return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
    if(adr_huart_MB->Init.Parity == UART_PARITY_NONE)
    {
        *pucByte = (uint8_t)(adr_huart_MB->Instance->RDR & (uint8_t)0x00FF);
    }
    else
    {
        *pucByte = (uint8_t)(adr_huart_MB->Instance->RDR & (uint8_t)0x007F);
    }
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
//static


/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
//static
//void prvvUARTRxISR( void )
//{
//	xMBRTUReceiveFSM(  );
//}



