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

/* mbb2.c - Mr. Baybus 2 in C!
**
**  Updates: 
**    10/30/2002 - Nick Veys
**      - Barely started...  Testing LCD routine compilation.
**    11/01/2002 - Nick Veys
**      - Actually did stuff, all output complete
**    11/02/2002 - Nick Veys
**      - Cake.  Done.
*/

#define PIC_XTAL 	20MHz

#define LCD_E		RB0				// LCD Enable line
#define LCD_RS		RB2				// LCD Data/Command line
#define LCD_RW		RB1				// LCD Read/Write line
#define LCD_DB7		RC7				// LCD Data bit 7 (MSB)
#define LCD_DB6		RC6				// LCD Data bit 6
#define LCD_DB5		RC5				// LCD Data bit 5
#define LCD_DB4		RC4				// LCD Data bit 4
#define LCD_DB3		RC3				// LCD Data bit 3
#define LCD_DB2		RC2				// LCD Data bit 2
#define LCD_DB1		RC1				// LCD Data bit 1
#define LCD_DB0		RC0				// LCD Data bit 0 (LSB)

#define LCD_MODE 	8				// 8-bit mode

#include <pic.h>
#include "delay.h"
#include "eeprom.h"
#include "hd44780.h"

/* HS oscillator
** Power-up timer enabled
** Watchdog disabled
** Low-voltage programming disabled */
__CONFIG(HS & PWRTEN & BOREN & WDTDIS & LVPDIS);

#define EE_FAN1		0x00				// Fan 1 EEPROM address
#define EE_FAN2		0x01				// Fan 2 EEPROM address
#define EE_FAN3		0x02				// Fan 3 EEPROM address
#define EE_LIGHT	0x03				// Light EEPROM address
#define EE_TEMP		0x04				// Temperature EEPROM address

#define FAN1		RB3				// Fan 1 Control Line
#define FAN2		RB4				// Fan 2 Control Line
#define FAN3		RB5				// Fan 3 Control Line
#define LIGHT		RB7				// Light Control Line

#define B_MODE		RA4				// Mode Button
#define B_GO		RA5				// Go Button
#define TEMP1		RA0				// Temperature Line 1
#define TEMP2		RA1				// Temperature Line 2

#define HD44780_LINE2() HD44780_SET_DDRAM(40)

#define debounce() DELAY_MS(10)

static bit _FAN1, _FAN2, _FAN3, _TEMP;			// Shadow bits
unsigned char _LIGHT;					// Light
static bit _STROBE;					// Strobe holder

unsigned char mode;					// Current mode

unsigned int temp1, temp2;				// Temperature holders

unsigned char fanHeader[]   = "     Fan Status";
unsigned char lightHeader[] = "  Lighting Status";
unsigned char tempHeader[]  = "    Temperatures";

void getTemperatures()
{
  ADCON0 = 0x80;					// Channel 0
  ADON = 1;						// Turn ADC on
  DELAY_US(10);						// Wait for it to start up
  ADGO = 1;						// Start the conversion
  while(ADGO);						// Wait for it to complete
  ADON = 0;						// The the ADC off
  ADIF = 0;						// Clear interrupt flag
  temp1 = (ADRESH << 8) + ADRESL;

  ADCON0 = 0x88;					// Channel 1
  ADON = 1;						// Turn ADC on
  DELAY_US(10);						// Wait for it to start up
  ADGO = 1;						// Start the conversion
  while(ADGO);						// Wait for it to complete
  ADON = 0;						// The the ADC off
  ADIF = 0;						// Clear interrupt flag
  temp2 = (ADRESH << 8) + ADRESL;

  temp1 = (500*temp1) / 1024;				// Scaling
  temp2 = (500*temp2) / 1024;

  if(_TEMP)						// Go to Fahrenheit
  {
    temp1 = ((9*temp1)/5) + 32;
    temp2 = ((9*temp2)/5) + 32;
  }
}

void separateDigits(unsigned int num, unsigned char* retval)
{
  retval[2] = num % 10;					// Get ones digit
  num -= retval[2];

  retval[1] = num % 100;				// Get tens digit
  num -= retval[1];
  retval[1] /= 10;

  retval[0] = num % 1000;				// Get hundreds digit
  num -= retval[0];
  retval[0] /= 100;

  retval[0] += '0';					// Make them ASCII
  retval[1] += '0';
  retval[2] += '0';

  if(retval[0] == '0')					// check for leading 0's
  {
    if(retval[1] == '0')
    {
      retval[0] = retval[1] = ' ';
    }
    else
      retval[0] = ' ';
  }
}

void updateDisplay()
{
  unsigned char t1[3], t2[3];				// Temperature ASCII holders

  HD44780_CLEAR();

  switch(mode)
  {
    case 0:						// Fans, none selected
      HD44780_PUTS(fanHeader, 15);			// Fan Status header
      HD44780_LINE2();					// Move to line 2
      HD44780_PUTS(" 1:", 3);				// Fan 1 header
      HD44780_PUTS((_FAN1) ? "On  " : "Off ", 4);	// Fan 1 status
      HD44780_PUTS("2:", 2);				// Fan 2 header
      HD44780_PUTS((_FAN2) ? "On  " : "Off ", 4);	// Fan 2 status
      HD44780_PUTS("3:", 2);				// Fan 3 header
      HD44780_PUTS((_FAN3) ? "On " : "Off", 3);		// Fan 3 status
      break;						// done with case 0
    case 1:						// Fans, #1 selected
      HD44780_PUTS(fanHeader, 15);			// Fan Status header
      HD44780_LINE2();					// Move to line 2
      HD44780_PUTS(">1:", 3);				// Fan 1 header
      HD44780_PUTS((_FAN1) ? "On <" : "Off<", 4);	// Fan 1 status
      HD44780_PUTS("2:", 2);				// Fan 2 header
      HD44780_PUTS((_FAN2) ? "On  " : "Off ", 4);	// Fan 2 status
      HD44780_PUTS("3:", 2);				// Fan 3 header
      HD44780_PUTS((_FAN3) ? "On " : "Off", 3);		// Fan 3 status
      break;						// done with case 1
    case 2:						// Fans, #2 selected
      HD44780_PUTS(fanHeader, 15);			// Fan Status header
      HD44780_LINE2();					// Move to line 2
      HD44780_PUTS(" 1:", 3);				// Fan 1 header
      HD44780_PUTS((_FAN1) ? "On >" : "Off>", 4);	// Fan 1 status
      HD44780_PUTS("2:", 2);				// Fan 2 header
      HD44780_PUTS((_FAN2) ? "On <" : "Off<", 4);	// Fan 2 status
      HD44780_PUTS("3:", 2);				// Fan 3 header
      HD44780_PUTS((_FAN3) ? "On " : "Off", 3);		// Fan 3 status
      break;						// done with case 2
    case 3:						// Fans, #3 selected
      HD44780_PUTS(fanHeader, 15);			// Fan Status header
      HD44780_LINE2();					// Move to line 2
      HD44780_PUTS(" 1:", 3);				// Fan 1 header
      HD44780_PUTS((_FAN1) ? "On  " : "Off ", 4);	// Fan 1 status
      HD44780_PUTS("2:", 2);				// Fan 2 header
      HD44780_PUTS((_FAN2) ? "On >" : "Off>", 4);	// Fan 2 status
      HD44780_PUTS("3:", 2);				// Fan 3 header
      HD44780_PUTS((_FAN3) ? "On <" : "Off<", 4);	// Fan 3 status
      break;						// done with case 3
    case 4:						// Lighting
      HD44780_PUTS(lightHeader, 17);			// Lighting Status header
      HD44780_LINE2();					// Move to line 2
      switch(_LIGHT)
      {
        case 0:
          HD44780_PUTS("    Deactivated", 15);		// Light off
          break;
        case 1:
          HD44780_PUTS("     Activated", 14);		// Light on
          break;
        default:
          HD44780_PUTS("      Strobe", 12);		// Light strobing
          break;
      }
      break;						// done with case 4
    case 5:						// Temperatures
      HD44780_PUTS(tempHeader, 16);			// Temperature header
      HD44780_LINE2();					// Move to line 2
      getTemperatures();				// Read ADC values
      separateDigits(temp1, t1);			// Get ASCII temperature
      separateDigits(temp2, t2);			// Get ASCII temperature
      HD44780_PUTS("  1:", 4);				// Temperature 1 header
      HD44780_PUTS(t1, 3);				// Temperature 1
      HD44780_OUT_DATA(0xDF);				// Degree symbol
      HD44780_OUT_DATA((_TEMP) ? 'F' : 'C');		// F or C
      HD44780_PUTS("  2:", 4);				// Temperature 2 header
      HD44780_PUTS(t2, 3);				// Temperature 2
      HD44780_OUT_DATA(0xDF);				// Degree symbol
      HD44780_OUT_DATA((_TEMP) ? 'F' : 'C');		// F or C
      break;						// done with case 5
  }
}

void updateHardware()					// Make it so...
{
  FAN1  = _FAN1;					// Set Fans
  FAN2  = _FAN2;
  FAN3  = _FAN3;
  if(!_LIGHT)						// If light off
    LIGHT = 0;						// kill it
  else if(_LIGHT == 1)					// of if it's on
    LIGHT = 1;						// light it up!
  else
  {
    LIGHT = _STROBE;					// Otherwise strobe!
    _STROBE = !_STROBE;
  }
}

void pressMode()
{
  if(mode >= 5)
    mode = 0;						// Start over
  else
    mode++;						// Advance to next mode
}

void pressGo()
{
  switch(mode)
  {
    case 1:
      _FAN1 = !_FAN1;					// Toggle FAN1
      writeEEPROM(EE_FAN1, _FAN1);			// Write FAN1 to EEPROM
      break;						// Done
    case 2:
      _FAN2 = !_FAN2;					// Toggle FAN1
      writeEEPROM(EE_FAN2, _FAN2);			// Write FAN2 to EEPROM
      break;						// Done
    case 3:
      _FAN3 = !_FAN3;					// Toggle FAN1
      writeEEPROM(EE_FAN3, _FAN3);			// Write FAN3 to EEPROM
      break;						// Done
    case 4:
      switch(_LIGHT)
      {
        case 0:						// Off -> On
          _LIGHT = 1;
          break;
        case 1:						// On -> Strobe
          _LIGHT = 2;
          break;
        default:					// Else -> Off
          _LIGHT = 0;
          break;
      }
      writeEEPROM(EE_LIGHT, _LIGHT);			// Write LIGHT to EEPROM
      break;						// Done
    case 5:
      _TEMP = !_TEMP;					// Toggle TEMP
      writeEEPROM(EE_TEMP, _TEMP);			// Write TEMP to EEPROM
      break;						// Done
    default: break;
  }
}

main()
{
  unsigned int refreshCounter = 0;
  unsigned char strobeCounter = 0;

  TRISA = 0b11111111;
  TRISB = 0b01000000;
  TRISC = 0b00000000;

  ADCON1 = 0x84;

  mode = 0;						// Start from mode 0

  _FAN1  = readEEPROM(EE_FAN1)  ? 1 : 0;		// Load FAN1 shadow
  _FAN2  = readEEPROM(EE_FAN2)  ? 1 : 0;		// Load FAN2 shadow
  _FAN3  = readEEPROM(EE_FAN3)  ? 1 : 0;		// Load FAN3 shadow
  _LIGHT = readEEPROM(EE_LIGHT);			// Load LIGHT shadow
  _TEMP  = readEEPROM(EE_TEMP)  ? 1 : 0;		// Load TEMP shadow

  updateHardware();					// Enact those settings

  HD44780_INITIALIZE(2);				// 2-line LCD display

  HD44780_PUTS(" Mr. Baybus Digital", 19);
  HD44780_LINE2();
  HD44780_PUTS("Control System v2.10", 20);
  DELAY_MS(250);
  DELAY_MS(250);
  DELAY_MS(250);
  DELAY_MS(250);
  DELAY_MS(250);
  DELAY_MS(250);
  DELAY_MS(250);
  DELAY_MS(250);
  DELAY_MS(250);
  DELAY_MS(250);
  DELAY_MS(250);
  DELAY_MS(250);

  updateDisplay();

  while(1)
  {
    if(!B_GO)
    {
      debounce();					// De-bounce wait
      if(!B_GO)						// Button still pressed
      {
        pressGo();					// Pressed GO
        updateHardware();				// Update the hardware
        updateDisplay();				// Update the display
      }
      while(!B_GO);					// Wait for release
      debounce();					// De-bounce wait
    }
    if(!B_MODE)
    {
      debounce();					// De-bounce wait
      if(!B_MODE)					// Button still pressed
      {
        pressMode();					// Pressed MODE
        updateDisplay();				// Update the display
      }
      while(!B_MODE);					// Wait for release
      debounce();					// De-bounce wait
    }
    if(!refreshCounter)					// When refresh counter rolls
    {
      updateDisplay();					// Refresh the display
      refreshCounter = 1000;
    }
    else
      refreshCounter--;
    if(!strobeCounter)					// When strobe counter rolls
    {
      updateHardware();					// Refresh the hardware
      strobeCounter = 50;
    }
    else
      strobeCounter--;
    DELAY_MS(5);
  }
}
