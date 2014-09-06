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
#include "modbus_master.h"

/*==================[macros and definitions]=================================*/
/* \brief Buffer size */
#define MODBUS_BUFFER_SIZE    255
/* \brief Read retries */
#define MODBUS_READ_RETRY     10000

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/
static void *__modbus_receive__(void *);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
static void *__modbus_receive__(void *ptr)
{
   int i;
   int rcv;
   int res;
   int fin;
   char byte[3];
   unsigned char *p;
   unsigned char *r_ptr;
   unsigned char buf[MODBUS_BUFFER_SIZE];
   modbus_conn *conn;


   /* MODBUS connection handler */
   conn = (modbus_conn *) ptr;
   p = buf;

   /* Initialize reception buffer */
   conn->r_slave = 0;
   conn->r_func = 0;
   conn->r_lrc = 0;

   /* Wait start character */
   i = 0;
   do
   {
      res = read(conn->fd, p, 1);
      if (res < 1)
      {
         i++;
         if (i > MODBUS_READ_RETRY) return NULL;
      }
   } while(*p != ':');

   /* Wait MODBUS frame */
   rcv = 0;
   do
   {
      res = read(conn->fd, p, MODBUS_BUFFER_SIZE - rcv);
      if (res < 1) continue;
      fin = 0;
      for(i = 0; i < res; i++, p++, rcv++)
      {
         if ((*p != '\r') && (*p != '\n')) continue;
         fin = 1;
         break;
      }
      *p = 0;
   } while(!fin);

   /* Get slave address */
   byte[0] = buf[0];
   byte[1] = buf[1];
   byte[2] = 0;
   sscanf(byte, "%x", (unsigned int *) &(conn->r_slave));

   /* Get function code */
   byte[0] = buf[2];
   byte[1] = buf[3];
   byte[2] = 0;
   sscanf(byte, "%x", (unsigned int *) &(conn->r_func));

   /* Get payload */
   conn->r_size = 0;
   if (conn->p_buf)
   {
      i = 4;
      r_ptr = conn->p_buf;
      while(i < (rcv - 2))
      {
         byte[0] = buf[i++];
         byte[1] = buf[i++];
         byte[2] = 0;
         sscanf(byte, "%x", (unsigned int *) r_ptr);
         r_ptr ++;
         conn->r_size ++;
      }
   }

   /* Get LRC */
   byte[0] = buf[i++];
   byte[1] = buf[i++];
   byte[2] = 0;
   sscanf(byte, "%x", (unsigned int *) &(conn->r_lrc));

   return NULL;
}

/*==================[external functions definition]==========================*/
/** \brief modbus_init function
 *
 * Initialize a new modbus master connection.
 *
 * \returns connection handler
 *
 */
modbus_conn *modbus_init(const char *device)
{
   modbus_conn *conn;
   struct termios newtio;

   /* Reserve memory for handler */
   conn = (modbus_conn *) malloc(sizeof(modbus_conn));
   if (!conn) return NULL;

   /* Open serial port connection */
   conn->fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
   if (conn->fd < 0)
   {
      free(conn);
      return NULL;
   }

   /* Set configuration for serial port */
   tcgetattr(conn->fd, &(conn->oldtio));
   bzero(&newtio, sizeof(newtio));
   newtio.c_cflag = B115200 | CS8 | CSTOPB | CLOCAL | CREAD;
   newtio.c_iflag = IGNPAR | ICRNL;
   newtio.c_oflag = 0;
   newtio.c_lflag = 0;
   newtio.c_cc[VTIME] = 0;
   newtio.c_cc[VMIN] = 0;
   tcflush(conn->fd, TCIFLUSH);
   tcsetattr(conn->fd, TCSANOW, &newtio);

   /* Return handler */
   return conn;
}

/** \brief modbus_deinit function
 *
 * Release a modbus master connection.
 *
 * \returns 1 is OK, or -1 is error
 *
 */
int modbus_deinit(modbus_conn *conn)
{
   if (!conn) return -1;
   tcsetattr(conn->fd, TCSANOW, &(conn->oldtio));
   close(conn->fd);
   free(conn);
   return 1;
}

/** \brief modbus_sendcmd function
 *
 * Send modbus commands.
 *
 * \returns the length of the received response in bytes
 *
 */
int modbus_sendcmd(modbus_conn *conn, unsigned char slave, 
                     unsigned char func, int size, void *data_in, 
                     void *data_out)
{
   int i;
   int res;
   char byte[3];
   unsigned char lrc;
   unsigned char r_lrc;
   char *msg;
   pthread_t thread_reader;


   /* Handler exist? */
   if (!conn) return -1;

   /* Memory for request message */
   msg = (char *) malloc(1 + 2 + 2 + size * 2 + 2);
   if (!msg) return -1;

   /* Start message */
   sprintf(msg, ":");
   lrc = 0;

   /* Slave address */
   sprintf(byte, "%02X", slave);
   strcat(msg, byte);
   lrc += slave;

   /* Function code */
   sprintf(byte, "%02X", func);
   strcat(msg, byte);
   lrc += func;

   /* Data */
   for(i = 0; i < size; i++)
   {
      sprintf(byte, "%02X", *((unsigned char *) (data_in + i)));
      strcat(msg, byte);
      lrc += *((unsigned char *) (data_in + i));
   }

   /* LRC */
   lrc = (lrc ^ 0xff) + 1;
   
   sprintf(byte, "%02X", (unsigned char) lrc);
   strcat(msg, byte);

   /* End of message. */
   strcat(msg, "\r\n");

   /* Create thread to wait response */
   conn->p_buf = data_out;
   pthread_create(&(thread_reader), NULL, __modbus_receive__, (void *) conn);

   /* Send request message */
   res = write(conn->fd, msg, strlen(msg));
   if (res != strlen(msg))
   {
      pthread_kill(thread_reader, SIGKILL);
      free(msg);
      return -1;
   }
   free(msg);

   /* Wait thread */
   pthread_join(thread_reader, NULL);

   /* Check slave address */
   if (conn->r_slave != slave) return -1;

   /* Check function code */
   if (conn->r_func != func) return -1;

   /* Check LRC */
   r_lrc = conn->r_slave + conn->r_func;
   for(i = 0; i < conn->r_size; i++) r_lrc += conn->p_buf[i];
   r_lrc = (r_lrc ^ 0xff) + 1;
   if (conn->r_lrc != r_lrc) return -1;
 
   /* Return byte of message response */
   return conn->r_size;
}


