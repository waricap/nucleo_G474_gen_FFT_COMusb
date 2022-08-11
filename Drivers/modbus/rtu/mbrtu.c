/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mbrtu.c,v 1.18 2007/09/12 10:15:56 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"
#include "stdio.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "usart.h"

/* ----------------------- Modbus includes ----------------------------------*/
//#include "mb.h"
#include "mbrtu.h"
#include "mbframe.h"

#include "mbcrc.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_SER_PDU_SIZE_MIN     4       /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX     4096     /*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */


/* ----------------------- Static variables ---------------------------------*/

volatile UCHAR  ucRTUBuf[MB_SER_PDU_SIZE_MAX];

// static volatile UCHAR *pucSndBufferCur;
static  UCHAR *pucSndBufferCur;
static volatile USHORT usSndBufferCount;

static volatile USHORT usRcvBufferPos;

extern	UART_HandleTypeDef * adr_huart_MB;
extern volatile eMBRcvState eRcvState;
extern volatile eMBSndState eSndState;

#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
extern volatile uint32_t count_tic_start;
extern volatile uint32_t count_tic_finish;
#include "arm_math.h"
extern float32_t count_tic_float_mks;

uint32_t usCRC16_HAL;
USHORT          usCRC16;
extern CRC_HandleTypeDef hcrc;

/* ----------------------- Start implementation -----------------------------*/

void
eMBRTUStop( void )
{
    ENTER_CRITICAL_SECTION(  );
    vMBPortSerialEnable( FALSE, FALSE );
    vMBPortTimersDisable(  );
    EXIT_CRITICAL_SECTION(  );
}

eMBErrorCode
eMBRTUReceive( UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength )
{

    eMBErrorCode    eStatus = MB_ENOERR;

    ENTER_CRITICAL_SECTION(  );
    assert( usRcvBufferPos < MB_SER_PDU_SIZE_MAX );

    /* Length and CRC check */
    //printf("eMBRTUReceive ucRTUBuf %d %d %d %d %d %d \n", ucRTUBuf[0], ucRTUBuf[1], ucRTUBuf[2], ucRTUBuf[3], ucRTUBuf[4], ucRTUBuf[5]);
    if( ( usRcvBufferPos >= MB_SER_PDU_SIZE_MIN ) && ( usMBCRC16( ( UCHAR * ) ucRTUBuf, usRcvBufferPos ) == 0 ) )
    {
        /* Save the address field. All frames are passed to the upper layed
         * and the decision if a frame is used is done there.
         */
        *pucRcvAddress = ucRTUBuf[MB_SER_PDU_ADDR_OFF];

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.
         */
        *pusLength = ( USHORT )( usRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC );

        /* Return the start of the Modbus PDU to the caller. */
        *pucFrame = ( UCHAR * ) & ucRTUBuf[MB_SER_PDU_PDU_OFF];
    }
    else
    {
        eStatus = MB_EIO;
    }

    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBRTUSend( UCHAR ucSlaveAddress, const UCHAR * pucFrame, USHORT usLength )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    //USHORT          usCRC16;

 //   ENTER_CRITICAL_SECTION(  );

    /* Check if the receiver is still in idle state. If not we where to
     * slow with processing the received frame and the master sent another
     * frame on the network. We have to abort sending the frame.
     * Проверьте, находится ли приемник все еще в режиме ожидания. В противном случае мы должны были
		замедлить обработку полученного кадра, и мастер отправил другой кадр по сети.
		Мы должны прервать отправку фрейма.
     */
    //printf("eMBRTUSend_eRcvState = %d \n", eRcvState);
    if( eRcvState == STATE_RX_IDLE )
    {
        /* Первый байт перед Modbus-PDU - это адрес SLAVE. */
        pucSndBufferCur = ( UCHAR * ) pucFrame - 1;
        usSndBufferCount = 1;

        /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
        pucSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
        usSndBufferCount += usLength;

        /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
		//смотрим сколько натикало -цикл HAL_CRC_Calculate(85char) длится 11.30мкс (1921 тика)
		usCRC16 = HAL_CRC_Calculate(&hcrc, ( UCHAR * ) pucSndBufferCur, usSndBufferCount);
				//смотрим сколько натикало -цикл usMBCRC16(85char) длится 28.629мкс (4867 тика)
				//usCRC16 = usMBCRC16( ( UCHAR * ) pucSndBufferCur, usSndBufferCount );

        ucRTUBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 & 0xFF );
        ucRTUBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 >> 8 );

        /* Activate the transmitter. */
        if( usSndBufferCount != 0 )
        {
        	eSndState = STATE_TX_XMIT;
        	vMBPortSerialEnable( FALSE, TRUE );
        	HAL_UART_Transmit_DMA(adr_huart_MB, pucSndBufferCur, usSndBufferCount);
        	//printf("_transmit_DMA %d_ \n", usSndBufferCount);
        }
    }
    else
    {
        eStatus = MB_EIO;
    }
//    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

BOOL
xMBRTUReceiveFSM( void )
{
    BOOL            xTaskNeedSwitch = FALSE;
    UCHAR           ucByte;

    assert( eSndState == STATE_TX_IDLE );

    /* Всегда читайте  character. */
    ( void )xMBPortSerialGetByte( ( CHAR * ) & ucByte );

    switch ( eRcvState )
    {
        /* If we have received a character in the init state we have to
         * wait until the frame is finished.
         */
    case STATE_RX_INIT:
        vMBPortTimersEnable(  );
        //printf("xMBRTUReceiveFSM eRcvState =STATE_RX_INIT\n");
        break;

        /* In the error state we wait until all characters in the
         * damaged frame are transmitted.
         */
    case STATE_RX_ERROR:
        vMBPortTimersEnable(  );
        //printf("xMBRTUReceiveFSM eRcvState =STATE_RX_ERROR\n");
        break;

        /* In the idle state we wait for a new character. If a character
         * is received the t1.5 and t3.5 timers are started and the
         * receiver is in the state STATE_RX_RECEIVCE.
         */
    case STATE_RX_IDLE:
    	//printf("xMBRTUReceiveFSM eRcvState =STATE_RX_IDLE\n");
        usRcvBufferPos = 0;
        ucRTUBuf[usRcvBufferPos++] = ucByte;
        eRcvState = STATE_RX_RCV;

        /* Enable t3.5 timers. */
        vMBPortTimersEnable(  );
        break;

        /* We are currently receiving a frame. Reset the timer after
         * every character received. If more than the maximum possible
         * number of bytes in a modbus frame is received the frame is
         * ignored.
         */
    case STATE_RX_RCV:
        if( usRcvBufferPos < MB_SER_PDU_SIZE_MAX )
        {
            ucRTUBuf[usRcvBufferPos++] = ucByte;
            //printf("xMBRTUReceiveFSM eRcvState =STATE_RX_RCV\n");
        }
        else
        {
            eRcvState = STATE_RX_ERROR;
            //printf("xMBRTUReceiveFSM eRcvState =STATE_RX_ERROR\n");
        }
        vMBPortTimersEnable(  );
        break;
    }
    // printf("xMBRTUReceiveFSM eRcvState_OUT %d \n", eRcvState);
    return xTaskNeedSwitch;
}

BOOL
xMBRTUTransmitFSM( void )
{
    BOOL            xNeedPoll = FALSE;

    // assert( eRcvState == STATE_RX_IDLE );

    switch ( eSndState )
    {
        /* We should not get a transmitter event if the transmitter is in
         * idle state.  */
    case STATE_TX_IDLE:
        /* enable receiver/disable transmitter. */
        vMBPortSerialEnable( TRUE, FALSE );
        //printf("xMBRTUTransmitFSM _eSndState =STATE_TX_IDLE");
        break;

    case STATE_TX_XMIT:
        /* проверьте, закончили ли мы. */
            xNeedPoll = xMBPortEventPost( EV_FRAME_SENT );
            /* Отключите передатчик.
             * Это предотвращает еще одно прерывание пустого буфера передачи.*/
            vMBPortSerialEnable( TRUE, FALSE );
            eSndState = STATE_TX_IDLE;
            //printf("xMBRTUTransmitFSM _eSndState =STATE_TX_IDLE");
        break;
    }

    return xNeedPoll;
}

BOOL
xMBRTUTimerT35Expired( void )  // вызывается только по 35 событию преполнения TIM6
{
    BOOL            xNeedPoll = FALSE;

    switch ( eRcvState )
    {
        /* Timer t35 expired. Startup phase is finished. Таймер t35 истек. Этап запуска завершен.*/
    case STATE_RX_INIT:   // приемник в режиме инициализации
        xNeedPoll = xMBPortEventPost( EV_READY );
        //printf("xMBRTUTimerT35Expired eRcvState=_RX_INIT \n");
        break;

        /* A frame was received and t35 expired. Notify the listener that a new frame was received.
         *  Был получен кадр, и срок действия t35 истек. Уведомите слушателя о том, что был получен новый кадр.*/
    case STATE_RX_RCV:    // приемник в режиме приема
        xNeedPoll = xMBPortEventPost( EV_FRAME_RECEIVED );
        //printf("xMBRTUTimerT35Expired eRcvState=_RX_RCV, _xNeedPoll=%d \n", xNeedPoll);
        break;

        /* An error occured while receiving the frame. */
    case STATE_RX_ERROR:    // приемник в режиме ошибки
    	//printf("xMBRTUTimerT35Expired eRcvState=_RX_ERROR \n");
        break;

    case STATE_RX_IDLE:      // приемник в режиме ожидания, ничего нет на приеме
    	//printf("xMBRTUTimerT35Expired eRcvState=_RX_IDLE \n");
        break;

        /* Function called in an illegal state. Функция вызывается в illegal состоянии. */
    default:
    	if ( ( eRcvState == STATE_RX_INIT ) ||  ( eRcvState == STATE_RX_RCV ) || ( eRcvState == STATE_RX_ERROR ) )
    	{
    		printf("xMBRTUTimerT35Expired  Function called in an illegal state \n");
    	}
    }

    vMBPortTimersDisable(  );
    eRcvState = STATE_RX_IDLE; // всегда, при каждом срабатывании TIM6 статус приемника переводится в это состояние

    return xNeedPoll;
}
