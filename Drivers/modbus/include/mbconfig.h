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
 * File: $Id: mbconfig.h,v 1.15 2010/06/06 13:54:40 wolti Exp $
 */

#ifndef _MB_CONFIG_H
#define _MB_CONFIG_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif



/* ----------------------- Defines ------------------------------------------*/
/*! \defgroup modbus_cfg Modbus Configuration
 *	Большинство модулей в стеке протоколов являются полностью необязательными и могут быть исключены.
 *	Это особенно важно, если целевые ресурсы очень малы и необходимо сэкономить место в памяти программы.
 *  Все эти настройки доступны в файле mbconfig.h
 */

/*! \brief If Modbus ASCII support is enabled. */
#define MB_ASCII_ENABLED                        (  0 )

/*! \brief If Modbus RTU support is enabled. */
#define MB_RTU_ENABLED                          (  1 )

/*! \brief If Modbus TCP support is enabled. */
#define MB_TCP_ENABLED                          (  0 )

/*! \brief The character timeout value for Modbus ASCII.
 *
 * The character timeout value is not fixed for Modbus ASCII and is therefore
 * a configuration option. It should be set to the maximum expected delay
 * time of the network.
 */
#define MB_ASCII_TIMEOUT_SEC                    (  0 )

/*! \brief Timeout to wait in ASCII prior to enabling transmitter.
 *
 * If defined the function calls vMBPortSerialDelay with the argument
 * MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS to allow for a delay before
 * the serial transmitter is enabled. This is required because some
 * targets are so fast that there is no time between receiving and
 * transmitting the frame. If the master is to slow with enabling its 
 * receiver then he will not receive the response correctly.
 */
#ifndef MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS
#define MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS    ( 0 )
#endif

/* Максимальное количество кодов функций Modbus, которые должен поддерживать стек протоколов.
 * Максимальное количество поддерживаемых функций Modbus
 * должно быть больше суммы всех включенных функций в этом файле и пользовательских обработчиков функций.
 * Если установлено значение small, добавление дополнительных функций завершится неудачей.
 */
#define MB_FUNC_HANDLERS_MAX                    ( 16 )

/* Количество байтов, которые должны быть выделены для строки-идентификатора подчиненного
 *
 * Это число ограничивает максимальный размер строки-идентификатора подчиненного устройства.
 * Дополнительные сведения о том, как установить это значение, см. в разделе eMBSetSlaveID( ).
 * Он используется только в том случае, если значение MB_FUNC_OTHER_REP_SLAVEID_ENABLED равно (1).
 */
#define MB_FUNC_OTHER_REP_SLAVEID_BUF           ( 32 )

/* Если функция "Сообщить идентификатор ведомого устройства" должна быть включена. */
#define MB_FUNC_OTHER_REP_SLAVEID_ENABLED       (  1 )

/*! \brief If the <em>Read Input Registers</em> function should be enabled. */
#define MB_FUNC_READ_INPUT_ENABLED              (  1 )

/*! \brief If the <em>Read Holding Registers</em> function should be enabled. */
#define MB_FUNC_READ_HOLDING_ENABLED            (  1 )

/*! \brief If the <em>Write Single Register</em> function should be enabled. */
#define MB_FUNC_WRITE_HOLDING_ENABLED           (  1 )

/*! \brief If the <em>Write Multiple registers</em> function should be enabled. */
#define MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED  (  1 )

/*! \brief If the <em>Read Coils</em> function should be enabled. */
#define MB_FUNC_READ_COILS_ENABLED              (  0 )

/*! \brief If the <em>Write Coils</em> function should be enabled. */
#define MB_FUNC_WRITE_COIL_ENABLED              (  0 )

/*! \brief If the <em>Write Multiple Coils</em> function should be enabled. */
#define MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED    (  0 )

/*! \brief If the <em>Read Discrete Inputs</em> function should be enabled. */
#define MB_FUNC_READ_DISCRETE_INPUTS_ENABLED    (  0 )

/*! \brief If the <em>Read/Write Multiple Registers</em> function should be enabled. */
#define MB_FUNC_READWRITE_HOLDING_ENABLED       (  1 )

/*! @} */
#ifdef __cplusplus
    PR_END_EXTERN_C
#endif
#endif
