/*
 * Copyright (c) 2002, Nicholas Veys, veys.com   All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, 
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice, 
 *   this list of conditions and the following disclaimer in the documentation 
 *   and/or other materials provided with the distribution.
 * * Neither the name of the VEYS.COM nor the names of its contributors may 
 *   be used to endorse or promote products derived from this software without 
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* delay.h  --  Delay functions for the PIC
**
** Functions available:
**    DELAY_US(x)
**      - Delay specified number of microseconds.
**    DELAY_MS(x)
**      - Delay specified number of milliseconds.
**
**  Note that there are range limits: x must not exceed 255 - for xtal
**  frequencies > 12MHz the range for DELAY_US is even smaller.
**
**  DELAY_US is implemented as an 'inline' style macro function.
**
**  This library requires PIC_XTAL to be #define'd before inclusion.
**   the MHz and KHz suffixes may be used for a x1000 or x1.
**
**  Updates: 
**    04/29/2002 - Nick Veys
**      - Inital Revision.
**      - Set up rough timing.
**/

#ifndef __DELAY_H
#define __DELAY_H

#ifndef MHz
#define	MHz *1000L
#endif
#ifndef KHz
#define	KHz *1
#endif

#ifndef	PIC_XTAL
#error PIC_XTAL not defined.
#endif

#if PIC_XTAL >= 12MHz
  #define DELAY_US(x)\
          {\
            unsigned char _dcnt;\
            _dcnt = (x)*((PIC_XTAL)/(12MHz));\
            while(--_dcnt != 0) continue;\
          }
#else
  #define DELAY_US(x)\
          {\
            unsigned char _dcnt;\
            _dcnt = (x)/((12MHz)/(PIC_XTAL))|1;\
            while(--_dcnt != 0) continue;\
          }
#endif

void DELAY_MS(unsigned char cnt)
{
  #if PIC_XTAL <= 2MHz
    do
    {
      DELAY_US(996);
    } while(--cnt);
  #endif

  #if PIC_XTAL > 2MHz
    unsigned char i;
    do
    {
      i = 4;
      do
      {
        DELAY_US(250);
      } while(--i);
    } while(--cnt);
  #endif
}

#endif
