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
 * File: $Id: mb.c,v 1.28 2010/06/06 13:54:40 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"
#include "stdio.h"

#include "main.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbconfig.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbfunc.h"

#include "mbport.h"
// #include "mbrtu.h"

#ifndef MB_PORT_HAS_CLOSE
#define MB_PORT_HAS_CLOSE 0
#endif

/* ----------------------- Static variables ---------------------------------*/

static UCHAR    ucMBAddress;
static uint8_t count_eMBPool;

static enum
{
    STATE_ENABLED,
    STATE_DISABLED,
    STATE_NOT_INITIALIZED
} eMBState = STATE_NOT_INITIALIZED;


extern uint16_t timeout_Tim6_50us;
extern volatile eMBRcvState eRcvState;
extern volatile eMBSndState eSndState;

/* Functions pointer which are initialized in eMBInit( ). Depending on the
 * mode (RTU or ASCII) the are set to the correct implementations.
 */

/* Callback functions required by the porting layer. They are called when
 * an external event has happend which includes a timeout or the reception
 * or transmission of a character.
 * Функции обратного вызова, требуемые уровнем переноса.
 * Они вызываются, когда произошло внешнее событие,
 * которое включает в себя тайм-аут или прием или передачу символа.
 */

/* An array of Modbus functions handlers which associates Modbus function
 * codes with implementing functions.
 * Массив обработчиков функций Modbus,
 * который связывает коды функций Modbus с реализующими функциями.
 */
static xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX] = {
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    {MB_FUNC_OTHER_REPORT_SLAVEID, eMBFuncReportSlaveID}, //  17
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
    {MB_FUNC_READ_INPUT_REGISTER, eMBFuncReadInputRegister}, // 4 Read_Input_Register
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
    {MB_FUNC_READ_HOLDING_REGISTER, eMBFuncReadHoldingRegister}, // 3 Read_Holding_Register
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_REGISTERS, eMBFuncWriteMultipleHoldingRegister}, // 16  Write_Multiple_Holding_Register
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_REGISTER, eMBFuncWriteHoldingRegister}, // 6  Write_Holding_Register
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, eMBFuncReadWriteMultipleHoldingRegister}, // 23 Read_Write_Multiple_Holding_Register
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    {MB_FUNC_READ_COILS, eMBFuncReadCoils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    {MB_FUNC_WRITE_SINGLE_COIL, eMBFuncWriteCoil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_COILS, eMBFuncWriteMultipleCoils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    {MB_FUNC_READ_DISCRETE_INPUTS, eMBFuncReadDiscreteInputs},
#endif
};

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBInit( eMBMode eMode, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
    /* проверка предварительных условий */
    if( ( ucSlaveAddress == MB_ADDRESS_BROADCAST ) ||  ( ucSlaveAddress < MB_ADDRESS_MIN ) || ( ucSlaveAddress > MB_ADDRESS_MAX ) )
    	{ ucSlaveAddress = 7;  } // по умолчанию такой адрес будет, а потому что

    ucMBAddress = ucSlaveAddress; // раскинули адрес по этому файлу
    timeout_Tim6_50us = 35; // на нашей скорости здесь будет только такой таймаут //////////eStatus = eMBRTUInit( ucMBAddress, ucPort, ulBaudRate, eParity ); // здесь остался только инит времени тайаута TIM6

    xMBPortEventInit(  ); // сброс очереди событий  ==> в файле portevent.c
    eMBState = STATE_DISABLED;

    return MB_ENOERR;
}


/*eMBErrorCode
eMBRegisterCB( UCHAR ucFunctionCode, pxMBFunctionHandler pxHandler )
{
    int             i;
    eMBErrorCode    eStatus;

    if( ( 0 < ucFunctionCode ) && ( ucFunctionCode <= 127 ) )
    {
        ENTER_CRITICAL_SECTION(  );
        if( pxHandler != NULL )
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( ( xFuncHandlers[i].pxHandler == NULL ) ||
                    ( xFuncHandlers[i].pxHandler == pxHandler ) )
                {
                    xFuncHandlers[i].ucFunctionCode = ucFunctionCode;
                    xFuncHandlers[i].pxHandler = pxHandler;
                    break;
                }
            }
            eStatus = ( i != MB_FUNC_HANDLERS_MAX ) ? MB_ENOERR : MB_ENORES;
        }
        else
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                {
                    xFuncHandlers[i].ucFunctionCode = 0;
                    xFuncHandlers[i].pxHandler = NULL;
                    break;
                }
            }
            // Remove can't fail.
            eStatus = MB_ENOERR;
        }
        EXIT_CRITICAL_SECTION(  );
    }
    else
    {
        eStatus = MB_EINVAL;
    }
    return eStatus;
}
*/



eMBErrorCode
eMBEnable( void )
{
        /* Activate the protocol stack. */
    ENTER_CRITICAL_SECTION(  );
    /* Initially the receiver is in the state STATE_RX_INIT. we start
     * the timer and if no character is received within t3.5 we change
     * to STATE_RX_IDLE. This makes sure that we delay startup of the
     * modbus protocol stack until the bus is free.
     */
    eRcvState = STATE_RX_INIT;
    vMBPortSerialEnable( TRUE, FALSE ); // перевод порта в режим приема
    vMBPortTimersEnable(  ); // старт таймера TIM6 и сброс счетчика в ноль

    EXIT_CRITICAL_SECTION(  );

    eMBState = STATE_ENABLED;
    return MB_ENOERR;
}


eMBErrorCode
eMBPoll( void )
{
    static UCHAR   *ucMBFrame;
    static UCHAR    ucRcvAddress;
    static UCHAR    ucFunctionCode;
    static USHORT   usLength;
    static eMBException eException;

    int             i;
    eMBErrorCode    eStatus = MB_ENOERR;
    eMBEventType    eEvent;

    BOOL  flag_event = xMBPortEventGet( &eEvent );



    /* Проверьте, есть ли доступное событие.
     * Если нет, верните управление вызывающему абоненту.
     * В противном случае мы будем обрабатывать это событие. */

    if( flag_event == TRUE ) // если там в файле portevent.c  кемто чемто сформировано событие, тО сейчас мы его обработаем
    {
    	count_eMBPool=0;
    	//printf("eMBPoll-STARTevent _eMBState=%d, _eEvent=%d \n", eMBState, eEvent);
        switch ( eEvent )
        {
        case EV_READY:
        	//printf( "eMBPoll eEvent <= EV_READY \n");
            break;

        case EV_FRAME_RECEIVED:
            eStatus = eMBRTUReceive( &ucRcvAddress, &ucMBFrame, &usLength );
            if( eStatus == MB_ENOERR )
            {
                /* Check if the frame is for us. If not ignore the frame. */
                if( ( ucRcvAddress == ucMBAddress ) || ( ucRcvAddress == MB_ADDRESS_BROADCAST ) )
                {
                    ( void )xMBPortEventPost( EV_EXECUTE );  // значит фрейм принят, адрес норм, формируем событие - начало обработки
                }
            }
            break;

        case EV_EXECUTE:
            ucFunctionCode = ucMBFrame[MB_PDU_FUNC_OFF];
            eException = MB_EX_ILLEGAL_FUNCTION;
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                /* No more function handlers registered. Abort. */
                if( xFuncHandlers[i].ucFunctionCode == 0 )
                {
                    break;
                }
                else if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                {
                    eException = xFuncHandlers[i].pxHandler( ucMBFrame, &usLength );
                    break;
                }
            }
            //printf("eMBPoll eEvent==EV_EXECUTE  ucFunctionCode=%d \n", ucFunctionCode);

            /* Если запрос не был отправлен на широковещательный адрес, мы возвращаем ответ. !!!!!!!! ВОТ ЗДЕСЬ передача !!!!!!!!!!  */
            if( ucRcvAddress != MB_ADDRESS_BROADCAST )
            {
                if( eException != MB_EX_NONE )
                {
                    /* An exception occured. Build an error frame. */
                    usLength = 0;
                    ucMBFrame[usLength++] = ( UCHAR )( ucFunctionCode | MB_FUNC_ERROR );
                    ucMBFrame[usLength++] = eException;
                }

                // МОЁ изменение ///////////////// МОЁ изменение ///////////////// МОЁ изменение ///////////////// МОЁ изменение ///////////////
                if (ucMBFrame[MB_PDU_FUNC_OFF] == 4) { usLength =82; }// МОЁ изменение ///////////////// МОЁ изменение ///////////////
                eStatus = eMBRTUSend( ucMBAddress, ucMBFrame, usLength );
            }
            break;

        case EV_FRAME_SENT:
            break;
        }
    }
    return MB_ENOERR;
}
