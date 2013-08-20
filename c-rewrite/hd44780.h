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

/* hd44780.h  --  Functions for an HD44780-compatible LCD display.
**
** Pin Assignments (for reference):
**  1  - Vss - Power supply (GND)
**  2  - Vcc - Power supply (+5V)
**  3  - Vee - Contrast adjust (0-5V)
**  4  - RS  - 0 = Instruction input, 1 = Data input
**  5  - R/W - 0 = Write to LCD module, 1 = Read from LCD module
**  6  - E   - Enable signal
**  7  - DB0 - Data bus line 0 (LSB)
**  8  - DB1 - Data bus line 1
**  9  - DB2 - Data bus line 2
**  10 - DB3 - Data bus line 3
**  11 - DB4 - Data bus line 4
**  12 - DB5 - Data bus line 5
**  13 - DB6 - Data bus line 6
**  14 - DB7 - Data bus line 7 (MSB)
**
** Functions available:
**    HD44780_OUT_COMMAND(cmd)
**      - Sends cmd to LCD as a command.
**    HD44780_OUT_UPPER_NYBBLE(nybble)
**      - Sends upper nybble of nybble, for init routine.
**    HD44780_OUT_DATA(data)
**      - Sends data to LCD as data.
**    HD44780_CLEAR()
**      - Clears and homes the cursor.
**    HD44780_INITIALIZE()
**      - Initializes the HD44780 display on startup.
**
**  Variables port and mask are the data port and bitmask to find
**   the MAX3110 chip-select line.  i.e. if the MAX3110 is on PORTB's
**   bit #2, port = (unsigned char*) &PORTC, mask = 0b00000100 = 0x04.
**   Recommend this be defined in the main program code, not stated every use.
**
**  This library requires the following defined before inclusion:
**    * LCD_MODE - Determine LCD Mode. 4 -> 4-bit. 8 -> 8-bit.
**    * LCD_E    - LCD Enable Line
**    * LCD_RW   - LCD Read/Write Line
**    * LCD_RS   - LCD Command/Data Line
**    * LCD_DB7  - LCD DB7 Line (4 or 8-bit mode)
**    * LCD_DB6  - LCD DB6 Line (4 or 8-bit mode)
**    * LCD_DB5  - LCD DB5 Line (4 or 8-bit mode)
**    * LCD_DB4  - LCD DB4 Line (4 or 8-bit mode)
**    * LCD_DB3  - LCD DB3 Line (8-bit mode only)
**    * LCD_DB2  - LCD DB2 Line (8-bit mode only)
**    * LCD_DB1  - LCD DB1 Line (8-bit mode only)
**    * LCD_DB0  - LCD DB0 Line (8-bit mode only)
**
**     LCD_MODE is either 4 or 8.  All others are port bits, TRIS'd to Output.
**
**  Updates: 
**    10/30/2002 - Nick Veys
**      - Inital Revision.  Everything compiles!
**    11/02/2002 - Nick Veys
**      - Complete, everything works! :D
*/

#ifndef __HD44780_H
#define __HD44780_H

#ifndef LCD_MODE
#error LCD_MODE not defined!  Need either 4 or 8.
#endif

#ifndef LCD_E
#error LCD_E not defined!
#endif

#ifndef LCD_RW
#error LCD_RW not defined!
#endif

#ifndef LCD_RS
#error LCD_RS not defined!
#endif

#ifndef LCD_DB7
#error LCD_DB7 not defined!
#endif

#ifndef LCD_DB6
#error LCD_DB6 not defined!
#endif

#ifndef LCD_DB5
#error LCD_DB5 not defined!
#endif

#ifndef LCD_DB4
#error LCD_DB4 not defined!
#endif

#if LCD_MODE == 8
#ifndef LCD_DB3
#error LCD_DB3 not defined!
#endif
#endif

#if LCD_MODE == 4
#ifdef LCD_DB3
#error LCD_MODE is 4-bit.  Bits 3-0 are not used!
#endif
#endif

#if LCD_MODE == 8
#ifndef LCD_DB2
#error LCD_DB2 not defined!
#endif
#endif

#if LCD_MODE == 4
#ifdef LCD_DB2
#error LCD_MODE is 4-bit.  Bits 3-0 are not used!
#endif
#endif

#if LCD_MODE == 8
#ifndef LCD_DB1
#error LCD_DB1 not defined!
#endif
#endif

#if LCD_MODE == 4
#ifdef LCD_DB1
#error LCD_MODE is 4-bit.  Bits 3-0 are not used!
#endif
#endif

#if LCD_MODE == 8
#ifndef LCD_DB0
#error LCD_DB0 not defined!
#endif
#endif

#if LCD_MODE == 4
#ifdef LCD_DB0
#error LCD_MODE is 4-bit.  Bits 3-0 are not used!
#endif
#endif

#include <pic.h>
#include "delay.h"

void HD44780_OUT_COMMAND(unsigned char cmd)		// Output a command byte
{
  LCD_RS = 0;						// Command mode
  LCD_RW = 0;						// Write mode

  LCD_DB7 = (cmd & 0x80) ? 1:0;				// Bit 7 -> DB7
  LCD_DB6 = (cmd & 0x40) ? 1:0;				// Bit 6 -> DB6
  LCD_DB5 = (cmd & 0x20) ? 1:0;				// Bit 5 -> DB5
  LCD_DB4 = (cmd & 0x10) ? 1:0;				// Bit 4 -> DB4

#if LCD_MODE == 4
  LCD_E = 1;						// Strobe E high
  DELAY_US(1);						// Wait a wee bit
  LCD_E = 0;						// Strobe E low

  LCD_DB7 = (cmd & 0x08) ? 1:0;				// Bit 3 -> DB7
  LCD_DB6 = (cmd & 0x04) ? 1:0;				// Bit 2 -> DB6
  LCD_DB5 = (cmd & 0x02) ? 1:0;				// Bit 1 -> DB5
  LCD_DB4 = (cmd & 0x01) ? 1:0;				// Bit 0 -> DB4
#else
  LCD_DB3 = (cmd & 0x08) ? 1:0;				// Bit 3 -> DB3
  LCD_DB2 = (cmd & 0x04) ? 1:0;				// Bit 2 -> DB2
  LCD_DB1 = (cmd & 0x02) ? 1:0;				// Bit 1 -> DB1
  LCD_DB0 = (cmd & 0x01) ? 1:0;				// Bit 0 -> DB0
#endif

  LCD_E = 1;						// Strobe E high
  DELAY_US(1);						// Wait a wee bit
  LCD_E = 0;						// Strobe E low
  DELAY_MS(5);						// Wait for instruction to complete
}

void HD44780_OUT_UPPER_NYBBLE(unsigned char nybble)	// Output upper nybble (command)
{
  LCD_RS = 0;
  LCD_RW = 0;

  LCD_DB7 = (nybble & 0x80) ? 1:0;			// Bit 7 -> DB7
  LCD_DB6 = (nybble & 0x40) ? 1:0;			// Bit 6 -> DB6
  LCD_DB5 = (nybble & 0x20) ? 1:0;			// Bit 5 -> DB5
  LCD_DB4 = (nybble & 0x10) ? 1:0;			// Bit 4 -> DB4

  LCD_E = 1;						// Strobe E high
  DELAY_US(1);						// Wait a wee bit
  LCD_E = 0;						// Strobe E low
  DELAY_MS(5);						// Wait for instruction to complete
}

void HD44780_OUT_DATA(unsigned char data)		// Output a data byte
{
  LCD_RS = 1;						// Data mode
  LCD_RW = 0;						// Write mode

  LCD_DB7 = (data & 0x80) ? 1:0;			// Bit 7 -> DB7
  LCD_DB6 = (data & 0x40) ? 1:0;			// Bit 6 -> DB6
  LCD_DB5 = (data & 0x20) ? 1:0;			// Bit 5 -> DB5
  LCD_DB4 = (data & 0x10) ? 1:0;			// Bit 4 -> DB4

#if LCD_MODE == 4
  LCD_E = 1;						// Strobe E high
  DELAY_US(1);						// Wait a wee bit
  LCD_E = 0;						// Strobe E low

  LCD_DB7 = (data & 0x08) ? 1:0;			// Bit 3 -> DB7
  LCD_DB6 = (data & 0x04) ? 1:0;			// Bit 2 -> DB6
  LCD_DB5 = (data & 0x02) ? 1:0;			// Bit 1 -> DB5
  LCD_DB4 = (data & 0x01) ? 1:0;			// Bit 0 -> DB4
#else
  LCD_DB3 = (data & 0x08) ? 1:0;			// Bit 3 -> DB3
  LCD_DB2 = (data & 0x04) ? 1:0;			// Bit 2 -> DB2
  LCD_DB1 = (data & 0x02) ? 1:0;			// Bit 1 -> DB1
  LCD_DB0 = (data & 0x01) ? 1:0;			// Bit 0 -> DB0
#endif

  LCD_E = 1;						// Strobe E high
  DELAY_US(1);						// Wait a wee bit
  LCD_E = 0;						// Strobe E low
  DELAY_MS(1);						// Wait for instruction to complete
}

#define HD44780_CLEAR() HD44780_OUT_COMMAND(0x01)
#define HD44780_HOME() HD44780_OUT_COMMAND(0x02)
#define HD44780_SET_CGRAM(x) HD44780_OUT_COMMAND(0x40 | (x & 0x3F))
#define HD44780_SET_DDRAM(x) HD44780_OUT_COMMAND(0x80 | (x & 0x7F))

void HD44780_INITIALIZE(unsigned char lines)		// Initialize the display
{
  LCD_E = 0;						// Initialize all control signals
  LCD_RW = 0;
  LCD_RS = 0;

  DELAY_MS(20);						// Wait >15ms for Vcc to rise.

#if LCD_MODE == 4
  HD44780_OUT_UPPER_NYBBLE(0x30);			// Function Set (8-bit)
  HD44780_OUT_UPPER_NYBBLE(0x30);			// Function Set (8-bit)
  HD44780_OUT_UPPER_NYBBLE(0x30);			// Function Set (8-bit)
  HD44780_OUT_UPPER_NYBBLE(0x20);			// Function Set (4-bit)

  if(lines == 1)
    HD44780_OUT_COMMAND(0x20);				// 4-bit interface, 1 line, 5x7 font
  else if(lines == 2)
    HD44780_OUT_COMMAND(0x28);				// 4-bit interface, 2 lines, 5x7 font
#else
  HD44780_OUT_COMMAND(0x30);				// Function Set (8-bit)
  HD44780_OUT_COMMAND(0x30);				// Function Set (8-bit)
  HD44780_OUT_COMMAND(0x30);				// Function Set (8-bit)

  if(lines == 1)
    HD44780_OUT_COMMAND(0x30);				// 8-bit interface, 1 line, 5x7 font
  else if(lines == 2)
    HD44780_OUT_COMMAND(0x38);				// 8-bit interface, 2 lines, 5x7 font
#endif

  HD44780_OUT_COMMAND(0x0C);				// Display on, no cursor, no blink
  HD44780_CLEAR();					// Display clear
  HD44780_OUT_COMMAND(0x06);				// Auto-increment cursor
}

void HD44780_PUTS(const char* s, unsigned char len)
{
  unsigned char i;

  for(i = 0; i < len; i++)
  {
    HD44780_OUT_DATA(s[i]);
  }
}

#endif
