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
#include "modbus_master.h"
#include "tester.h"

/*==================[macros and definitions]=================================*/
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief main function
 **
 ** \brief CIAA test application.
 **
 **/
int main(int argc, char *argv[])
{
   int i;
   int res;
   unsigned char val;
   unsigned char data[4];
   unsigned char resp[255];
   modbus_conn *modbus_c;
   tester_conn *tester_c;


   /* Initialize connections */
   modbus_c = modbus_init("/dev/ttyS0");
   if (!modbus_c) return -1;
   tester_c = tester_init("/dev/ttyUSB0");
   if (!tester_c)
   {
      modbus_deinit(modbus_c);
      return -1;
   }

   /* Test CIAA outputs */
   printf("Salida CIAA ---> Entrada Tester\n");
   data[0] = 0x00;
   data[1] = 0x01;
   data[2] = 0x00;   
   data[3] = 0x00;
   res = modbus_sendcmd(modbus_c, 0x02, 0x06, 4, data, resp);
   data[3] = 0x01;
   for(i = 0; i < 8; i++)
   {
      printf(":"); getchar();
      printf("bit %i : ", i);

      res = modbus_sendcmd(modbus_c, 0x02, 0x06, 4, data, resp);

      if (res > 0)
         printf("OK ");
      else
         printf("ERR_COM ");

      if (tester_readinput(tester_c, i, &val) < 1)
         printf("ERR_COM ");
      else
      {
         if (data[3] == (val << i))
            printf("OK ");
         else
            printf("ERR_VAL ");
      }
      
      printf("\n");
      data[3] <<= 1;
   }
   printf("\n");

   /* Test CIAA inputs */
   printf("Salida Tester ---> Entrada CIAA\n");
   data[0] = 0x00;
   data[1] = 0x00;
   data[2] = 0x00;
   data[3] = 0x01;
   val =  0;
   tester_writeoutput(tester_c, 0, val);
   tester_writeoutput(tester_c, 1, val);
   tester_writeoutput(tester_c, 2, val);
   tester_writeoutput(tester_c, 3, val);
   tester_writeoutput(tester_c, 4, val);
   tester_writeoutput(tester_c, 5, val);
   tester_writeoutput(tester_c, 6, val);
   tester_writeoutput(tester_c, 7, val);
   for(i = 0; i < 8; i++)
   {
      printf(":"); getchar();
      printf("bit %i : ", i);

      val = 1;
      if (tester_writeoutput(tester_c, i, val) < 1)
         printf("ERR_COM ");
      else
         printf("OK ");

      res = modbus_sendcmd(modbus_c, 0x02, 0x03, 4, data, resp);
      if (res > 0)
      {
         val = 1;
         val <<= i;
         if (resp[2] & val)
            printf("OK ");
         else
            printf("ERR_VAL ");
      }
      else
         printf("ERR_COM ");

      printf("\n");
   }
   printf("\n");

   /* Close connections */
   tester_deinit(tester_c);
   modbus_deinit(modbus_c);
   return 0;
}

