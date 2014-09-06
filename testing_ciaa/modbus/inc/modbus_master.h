/* Copyright 2014, Fernando Beunza
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _MODBUS_MASTER_H_
#define _MODBUS_MASTER_H_
/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20141006 v0.0.1 initials initial
 */

/*==================[inclusions]=============================================*/
#include <termios.h>

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/
/* MODBUS master handler */
typedef struct
{
   int fd;                 /* \brief file descriptor of serial port */
   struct termios oldtio;  /* \brief serial port configuration */
   unsigned char *p_buf;  /* \brief poiter to received data */
   int r_size;             /* \brief length in bytes of received data */
   unsigned char r_slave; /* \brief address slave response */
   unsigned char r_func;  /* \brief function code response */
   unsigned char r_lrc;   /* \brief LRC response */
} modbus_conn;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
/** \brief Initialize a new modbus master connection.
 **
 ** \param[in] device serial port device
 ** \return connection handler
 **/
modbus_conn *modbus_init(const char *device);

/** \brief Release a modbus master connection.
 **
 ** \param[in] conn connection handler
 ** \return 1 is OK, or -1 is error
 **/
int modbus_deinit(modbus_conn *conn);

/** \brief Send modbus commands.
 **
 ** \param[in] conn connection handler
 ** \param[in] slave address slave
 ** \param[in] func function code
 ** \param[in] size length of input data in bytes
 ** \param[in] data_in data of request message
 ** \param[out] data_out data response
 ** \return the length of the received response in bytes
 **/
int modbus_sendcmd(modbus_conn *conn, unsigned char slave, 
                     unsigned char func, int size, void *data_in, 
                     void *data_out);

/*==================[end of file]============================================*/
#endif /* #ifndef _MODBUS_MASTER_H_ */

