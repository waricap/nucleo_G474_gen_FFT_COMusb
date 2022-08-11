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
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "stm32g4xx_hal.h"

/* ----------------------- Modbus includes ----------------------------------*/
//#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
extern TIM_HandleTypeDef htim6;
extern uint16_t counter_Tim6_MB ;
/* ----------------------- Start implementation -----------------------------*/

inline void
vMBPortTimersEnable(  )
{
	/*  Включите таймер с таймаутом, переданным xMBPortTimersInit( )  */
	/*  Если baudrate > 19200, то мы должны использовать фиксированные значения таймера usTim1Timerout50us = 1750 us.
	 *  В противном случае usTim1Timerout50us должно быть в 3,5 раза больше времени символа. */
	counter_Tim6_MB=0;
	HAL_TIM_Base_Start_IT(&htim6);
}

inline void
vMBPortTimersDisable(  )
{
    /* Disable any pending timers. */
	HAL_TIM_Base_Stop_IT(&htim6);
	counter_Tim6_MB =0;
}


//void HAL_TIM_PeriodElapsedCallback_for_MB(TIM_HandleTypeDef *htim)
//{
//  if((++counter)>=timeout_usTim1Timerout50us)
//  {
//    prvvTIMERExpiredISR();
//  }
//}
