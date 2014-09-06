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

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20141006 v0.0.1 initials initial
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include "tester.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/
static void *__tester_receive__(void *);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
static void *__tester_receive__(void *ptr)
{
   int res;
   int rcv;
   tester_conn *c;

   c = (tester_conn *) ptr;
   rcv = 0;
   if (c->p_buf) c->p_buf[rcv] = 0;
   do
   {
      sleep(2);
      res = read(c->fd, &(c->ack), 1);
      if (res < 1) return NULL;
   } while((c->ack != 0x06) && (c->ack != 0x15));
   while((c->recv) && (c->p_buf))
   {
      res = read(c->fd, c->p_buf + rcv, c->recv - rcv);
      if (res < 1) break;
      rcv += res;
   }
   if (c->p_buf) c->p_buf[rcv] = 0;
   return NULL;
}

/*==================[external functions definition]==========================*/
/** \brief tester_init function
 **
 ** \brief Initialize a new tester connection.
 **
 ** \return connection handler
 **/
tester_conn *tester_init(const char *device)
{
   tester_conn *conn;
   struct termios newtio;

   /* Reserve memory for handler */
   conn = (tester_conn *) malloc(sizeof(tester_conn));
   if (!conn) return NULL;

   /* Open serial port. */
   conn->fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
   if (conn->fd < 0)
   {
      free(conn);
      return NULL;
   }

   /* Set new configuration for serial port */
   tcgetattr(conn->fd, &(conn->oldtio));
   bzero(&newtio, sizeof(newtio));
   newtio.c_cflag = B9600 | CS8 | CSTOPB | CLOCAL | CREAD;
   newtio.c_iflag = IGNPAR | ICRNL;
   newtio.c_oflag = 0;
   newtio.c_lflag = 0;
   newtio.c_cc[VTIME] = 0;
   newtio.c_cc[VMIN] = 1;
   tcflush(conn->fd, TCIFLUSH);
   tcsetattr(conn->fd, TCSANOW, &newtio);

   /* Return new handler */
   return conn;
}

/** \brief tester_deinit function
 **
 ** \brief Release a tester connection.
 **
 ** \return 1 is OK, or -1 is error
 **/
int tester_deinit(tester_conn *conn)
{
   if (!conn) return -1;
   tcsetattr(conn->fd, TCSANOW, &(conn->oldtio));
   close(conn->fd);
   free(conn);
   return 1;
}

/** \brief tester_sendcmd function
 **
 ** \brief Send tester commands.
 **
 ** \return the length of the received response in bytes
 **/
int tester_sendcmd(tester_conn *conn, unsigned char cmd, unsigned char addr, 
                     int s_size, void *data_in, int r_size, void *data_out)
{
   int i;
   int res;
   char *msg;
   pthread_t thread_reader;

   /* Check handler */
   if (!conn) return -1;

   /* Reserve memory for message request */
   msg = (char *) malloc(3 + s_size + 1);
   if (!msg) return -1;

   /* Create message request */
   msg[0] = 0x02;
   msg[1] = 0x41 + cmd;
   msg[2] = 0x31 + addr;
   for(i = 0; i < s_size; i++)
   {
      msg[i + 3] =  *((unsigned char *) (data_in + i));
   }
   msg[s_size + 3] = 0x03;

   /* Create read thread */
   conn->p_buf = data_out;
   conn->recv = r_size;
   pthread_create(&(thread_reader), NULL, __tester_receive__, (void *) conn);
      
   /* Send request */
   res = write(conn->fd, msg, s_size + 4);
   if (res != (s_size + 4))
   {
      pthread_kill(thread_reader, SIGKILL);
      free(msg);
      return -1;
   }
   free(msg);

   /* Wait request */
   pthread_join(thread_reader, NULL);

   /* Return number of bytes received */
   return conn->r_size;
}

/** \brief tester_readinput function
 **
 ** \brief Read tester inputs.
 **
 ** \return 1 is OK, or -1 is error
 **/
int tester_readinput(tester_conn *conn, unsigned char addr, 
                        unsigned char *val)
{
   int res;
   char resp[255];

   res = tester_sendcmd(conn, 0, addr, 0, NULL, 1, &resp);
   if (res == -1) return -1;
   *val = *resp - 0x30;
   return 1;
}

/** \brief tester_writeoutput function
 **
 ** \brief Write tester outputs.
 **
 ** \return 1 is OK, or -1 is error
 **/
int tester_writeoutput(tester_conn *conn, unsigned char addr, 
                        unsigned char val)
{
   int res;

   val += 0x30;
   res = tester_sendcmd(conn, 1, addr, 1, &val, 0, NULL);
   if (res == -1) return -1;
   return 1;
}

