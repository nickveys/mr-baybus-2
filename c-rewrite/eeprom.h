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

/* eeprom.h  --  Functions for using EEPROM storage.
**
** Functions available:
**    readEEPROM(addr)
**      - Returns the byte stored in EEPROM addr.
**    writeEEPROM(addr, data)
**      - Writes data to EEPROM addr.
**
**  Updates: 
**    11/01/2002 - Nick Veys
**      - Inital Revision.
**    11/02/2002 - Nick Veys
**      - Working great.  Read/Write OK.
*/

#ifndef __EEPROM_H
#define __EEPROM_H

#include <pic.h>

unsigned char readEEPROM(unsigned char addr)
{
  EEADR = addr;						// load EEPROM address
  EEPGD = 0;						// we're reading DATA memory
  RD = 1;						// initiate the read (1 cycle)
  return(EEDATA);					// return the result
}

void writeEEPROM(unsigned char addr, unsigned char data)
{
  static bit _GIE;					// GIE holder

  while(WR);						// wait for previous write to complete

  EEADR = addr;						// load EEPROM address
  EEDATA = data;					// load EEPROM data
  EEPGD = 0;						// we're writing to DATA memory
  WREN = 1;						// enable EEPROM writes

  _GIE = GIE;
  GIE = 0;						// disable interrupts

  EECON2 = 0x55;					// required for
  EECON2 = 0xAA;					// EEPROM write

  WR = 1;						// initiate the write
  WREN = 0;						// disable writes

  GIE = _GIE;						// set to previous value
}

#endif
