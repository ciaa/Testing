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

#ifndef _TESTER_H_
#define _TESTER_H_
/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20141006 v0.0.1 initials initial
 */

/*==================[inclusions]=============================================*/
#include <termios.h>

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/
/* Tester connection handler */
typedef struct
{
   int fd;                 /* \brief file descriptor of serial port */
   struct termios oldtio;  /* \brief serial port configuration */
   unsigned char *p_buf;  /* \brief poiter to received data */
   int recv;               /* \brief number of bytes to receive */
   int r_size;             /* \brief length in bytes of received data */
   unsigned char ack;     /* \brief ACK flag */
} tester_conn;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
/** \brief Initialize a new tester connection.
 **
 ** \param[in] device serial port device
 ** \return connection handler
 **/
tester_conn *tester_init(const char *device);

/** \brief Release a tester connection.
 **
 ** \param[in] conn connection handler
 ** \return 1 is OK, or -1 is error
 **/
int tester_deinit(tester_conn *conn);

/** \brief Send tester commands.
 **
 ** \param[in] conn connection handler
 ** \param[in] cmd command
 ** \param[in] addr address
 ** \param[in] s_size length of input data in bytes
 ** \param[in] data_in data of request message
 ** \param[in] r_size number of bytes to receive
 ** \param[out] data_out data response
 ** \return the length of the received response in bytes
 **/
int tester_sendcmd(tester_conn *conn, unsigned char cmd, unsigned char addr, 
                     int s_size, void *data_in, int r_size, void *data_out);

/** \brief Read tester inputs.
 **
 ** \param[in] conn connection handler
 ** \param[in] addr address
 ** \param[out] val input bit value
 ** \return 1 is OK, or -1 is error
 **/
int tester_readinput(tester_conn *conn, unsigned char addr, 
                        unsigned char *val);

/** \brief Write tester outputs.
 **
 ** \param[in] conn connection handler
 ** \param[in] addr address
 ** \param[out] val output bit value
 ** \return 1 is OK, or -1 is error
 **/
int tester_writeoutput(tester_conn *conn, unsigned char addr, 
                        unsigned char val);

/*==================[end of file]============================================*/
#endif /* #ifndef _TESTER_H_ */

