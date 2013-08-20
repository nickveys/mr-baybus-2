;----------------------------------------------------------------------------------------
; Copyright (c) 2000, Nicholas Veys, veys.com  All rights reserved.
; 
; Redistribution and use in source and binary forms, with or without modification, 
;   are permitted provided that the following conditions are met:
;
; * Redistributions of source code must retain the above copyright notice, this 
;     list of conditions and the following disclaimer.
; * Redistributions in binary form must reproduce the above copyright notice, 
;     this list of conditions and the following disclaimer in the documentation 
;     and/or other materials provided with the distribution.
; * Neither the name of the VEYS.COM nor the names of its contributors may be used 
;     to endorse or promote products derived from this software without specific 
;     prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
; EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
; OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
; SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
; INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
; TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
; BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
; ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
; DAMAGE.
;----------------------------------------------------------------------------------------

 title  "Mr. BayBus Digital Control System v2.0"

  LIST R=DEC
  INCLUDE "p16f870.inc"
 __CONFIG _CP_OFF & _WDT_OFF & _HS_OSC & _PWRTE_ON & _LVP_OFF

;----------------------------------------------------------------------------------------
; Variable declarations
;----------------------------------------------------------------------------------------
 CBLOCK 0x20
DELAY,DELAYTMP					; storage used in delay loops
MODE						; current mode
DEVSTATUS					; device status
EEPROM_ADDRESS,EEPROM_DATA			; EEPROM variables
TENS,ONES,BCD_COUNTER,BCD_TEMP			; temperature conversion variables
REFRESH_TIMER1					; LCD refresh timer delay
 ENDC

;----------------------------------------------------------------------------------------
; #define's
;----------------------------------------------------------------------------------------
#define LCD_RS 		PORTB,2			; LCD RS Line
#define LCD_RW 		PORTB,1			; LCD R/_W Line
#define LCD_E		PORTB,0			; LCD E Strobe Line
#define LCD_DATA	PORTC			; LCD Data Port
#define LCD_BUSY	PORTC,7			; LCD Busy Line
#define LCD_BUSY_TRIS	TRISC,7			; LCD Busy Line Direction Bit
#define FAN1		PORTB,3			; FAN1 Control Line
#define FAN2		PORTB,4			; FAN2 Control Line
#define FAN3		PORTB,5			; FAN3 Control Line
#define LIGHT		PORTB,7			; Light Control Line
#define B_MODE		PORTA,4			; Mode Button
#define B_GO		PORTA,5			; GO Button
#define TEMP2		PORTA,1			; Temperature Line 2
#define TEMP1		PORTA,0			; Temperature Line 1

#define F1_S		DEVSTATUS,0		; FAN1 bit
#define F1MASK		0x01			; 0000 0001 toggle bit
#define F2_S		DEVSTATUS,1		; FAN2 bit
#define F2MASK		0x02			; 0000 0010 toggle bit
#define F3_S		DEVSTATUS,2		; FAN3 bit
#define F3MASK		0x04			; 0000 0100 toggle bit
#define LIT_S		DEVSTATUS,6		; LIGHT bit
#define LIGHTMASK	0x40			; 0100 0000 toggle bit
#define CELCIUS		DEVSTATUS,7		; 1 = Celcius, 0 = Fahrenheit
#define TEMPERATUREMASK 0x80			; 1000 0000 toggle bit

#define DEVSTATUS_ADDR	0x00			; DEVSTATUS EEPROM address

;----------------------------------------------------------------------------------------
; Macro declarations
;----------------------------------------------------------------------------------------
BANK0 macro           				; Switch to BANK0
  bcf STATUS,RP1
  bcf STATUS,RP0
  endm

BANK1 macro           				; Switch to BANK1
  bcf STATUS,RP1
  bsf STATUS,RP0
  endm

BANK2 macro           				; Switch to BANK2
  bsf STATUS,RP1
  bcf STATUS,RP0
  endm

BANK3 macro           				; Switch to BANK3
  bsf STATUS,RP1
  bsf STATUS,RP0
  endm

DELAY_MILLI macro TIME				; 0-255 millisecond delay macro
  movlw TIME
  movwf DELAY
  call DELAY_MS
  endm

DELAY_MICRO macro TIME				; 0-255 microsecond delay macro
  movlw TIME
  movwf DELAY
  call DELAY_US
  endm

LCD_CLEAR macro					; sends clear command to LCD
  movlw 0x01
  call LCD_OUT_COMMAND
  DELAY_MILLI 2
  endm

LCD_HOME_LINE1 macro				; homes LCD on line 1
  movlw 0x80
  call LCD_OUT_COMMAND
  DELAY_MICRO 45
  endm

LCD_HOME_LINE2 macro				; homes LCD on line 2
  movlw 0xA8
  call LCD_OUT_COMMAND
  DELAY_MICRO 45
  endm

LCD_COMMAND macro CMD				; LCD command output
  movlw CMD
  call LCD_OUT_COMMAND
  endm

LCD_PRINT macro CHAR				; prints a single ascii to LCD
  movlw CHAR
  call LCD_OUT_DATA
  endm

;----------------------------------------------------------------------------------------
; Program code
;----------------------------------------------------------------------------------------
  PAGE

 org 0
  goto MAIN

;----------------------------------------------------------------------------------------
; Subroutines
;----------------------------------------------------------------------------------------
DELAY_MS					; busy wait of DELAY ms
  movf DELAY,w
  movwf DELAYTMP				; save DELAY time
DELAY_MS_LOOP					; inner loop
  movlw 245					; load 245 (1)
  movwf DELAY					; into DELAY (2)
  call DELAY_US					; wait 245us (3-249)
  movlw 245					; load 245 (250)
  movwf DELAY					; into DELAY (251)
  call DELAY_US					; wait 245us (252-498)
  movlw 245					; load 245 (499)
  movwf DELAY					; into DELAY (500)
  call DELAY_US					; wait 245us (501-747)
  movlw 246					; load 246 (748)
  movwf DELAY					; into DELAY (749)
  call DELAY_US					; wait 246us (750-997)
  decfsz DELAYTMP,f				; test DELAYTMP count (998)
    goto DELAY_MS_LOOP				; loop if not done (999,1000)
  return					; gtfo (999,1000)

DELAY_US					; busy wait of DELAY us
						; .2us instruction period assumed
  nop						; (1)
  nop						; (2)
  decfsz DELAY,f				; test DELAY count (3)
    goto DELAY_US				; loop if not done (4,5)
  return					; gtfo (4,5)

LCD_OUT_COMMAND
  bcf LCD_RS					; LCD command mode
  movwf LCD_DATA				; put data on LCD Data Bus
  bsf LCD_E					; bring E high
  nop						; wait a teensy bit
  bcf LCD_E					; bring E low
  return

LCD_OUT_DATA
  bsf LCD_RS					; LCD command mode
  movwf LCD_DATA				; put data on LCD Data Bus
  bsf LCD_E					; bring E high
  nop						; wait a teensy bit
  bcf LCD_E					; bring E low
  DELAY_MICRO 45				; wait for instruction to complete
  return

LCD_SPLASH
  LCD_PRINT ' '
  LCD_PRINT 'M'
  LCD_PRINT 'r'
  LCD_PRINT '.'
  LCD_PRINT ' '
  LCD_PRINT 'B'
  LCD_PRINT 'a'
  LCD_PRINT 'y'
  LCD_PRINT 'B'
  LCD_PRINT 'u'
  LCD_PRINT 's'
  LCD_PRINT ' '
  LCD_PRINT 'D'
  LCD_PRINT 'i'
  LCD_PRINT 'g'
  LCD_PRINT 'i'
  LCD_PRINT 't'
  LCD_PRINT 'a'
  LCD_PRINT 'l'					; " Mr. BayBus Digital"
  LCD_HOME_LINE2
  LCD_PRINT 'C'
  LCD_PRINT 'o'
  LCD_PRINT 'n'
  LCD_PRINT 't'
  LCD_PRINT 'r'
  LCD_PRINT 'o'
  LCD_PRINT 'l'
  LCD_PRINT ' '
  LCD_PRINT 'S'
  LCD_PRINT 'y'
  LCD_PRINT 's'
  LCD_PRINT 't'
  LCD_PRINT 'e'
  LCD_PRINT 'm'
  LCD_PRINT ' '
  LCD_PRINT 'v'
  LCD_PRINT '2'
  LCD_PRINT '.'
  LCD_PRINT '0'
  LCD_PRINT '0'					; "Control System v2.00"
  DELAY_MILLI 250
  DELAY_MILLI 250
  DELAY_MILLI 250
  DELAY_MILLI 250
  DELAY_MILLI 250
  DELAY_MILLI 250
  DELAY_MILLI 250
  DELAY_MILLI 250
  return

BUTTONPRESS_MODE
  DELAY_MILLI 10				; 10ms debounce time
  btfsc B_MODE					; are we still pressed?
    return					; if not, get out...
  movf MODE,w					; load mode
  addwf PCL,f					; advance the PC
  goto MODE_0					;
  goto MODE_1					;
  goto MODE_2					;
  goto MODE_3					;
  goto MODE_4					;
  goto MODE_5					;
MODE_0						;
MODE_1						;
MODE_2						;
MODE_3						;
MODE_4						;
  incf MODE,f					; increment the mode
  goto MODE_DONE				; done with mode change
MODE_5						;
  clrf MODE					; reset mode to 0
MODE_DONE					;
  btfss B_MODE					; still pressed?
    goto $-1					; wait for release
  call UPDATE_DISPLAY				; update the display...
  return

BUTTONPRESS_GO
  DELAY_MILLI 10				; 10ms debounce time
  btfsc B_GO					; are we still pressed?
    return					; if not, get out...
  movf MODE,w					; load mode
  addwf PCL,f					; advance PC
  goto BPGO_DONE				; do nothing, no active objects
  goto BPGO_1					; fan 1 modified
  goto BPGO_2					; fan 2 modified
  goto BPGO_3					; fan 3 modified
  goto BPGO_4					; light toggled
  goto BPGO_5					; temperature unit toggled
BPGO_1
  movlw F1MASK					; Load bitmask to toggle fan
  xorwf DEVSTATUS,f				; toggle bit
  goto BPGO_DONE  
BPGO_2
  movlw F2MASK					; Load bitmask to toggle fan
  xorwf DEVSTATUS,f				; toggle bit
  goto BPGO_DONE  
BPGO_3
  movlw F3MASK					; Load bitmask to toggle fan
  xorwf DEVSTATUS,f				; toggle bit
  goto BPGO_DONE  
BPGO_4
  movlw LIGHTMASK				; Load bitmask to toggle light
  xorwf DEVSTATUS,f				; toggle bit
  goto BPGO_DONE
BPGO_5

BPGO_DONE
  btfss B_GO					; still pressed?
    goto $-1					; wait for release
  call UPDATE_HARDWARE				; make it so
  call UPDATE_DISPLAY				; update the display
  BANK2						; switch to BANK2 to start EEPROM write
  movlw DEVSTATUS_ADDR				; load DEVSTATUS address
  movwf EEADR ^ 0x100				; put in EEPROM address reg
  movf DEVSTATUS,w				; load DEVSTATUS value
  movwf EEDATA ^ 0x100				; put in EEPRON data red
  BANK3						; switch to BANK3 for EEPROM config
  bcf EECON1 ^ 0x180,EEPGD			; DATA memory
  bsf EECON1 ^ 0x180,WREN			; enable writes
  bcf INTCON,GIE				; disable interrupts
  movlw 0x55					; * required
  movwf EECON2 ^ 0x180				; * eeprom
  movlw 0xAA					; * write
  movwf EECON2 ^ 0x180				; * stuff
  bsf EECON1 ^ 0x180,WR				; initiate the write
  bcf EECON1 ^ 0x180,WREN			; disable writes
  BANK0						; switch back to BANK0
  btfss PIR2,EEIF				; wait til write complete
    goto $-1
  ;bsf INTCON,GIE				; re-enable interrupts
  return

UPDATE_DISPLAY					; update the display after some change
  movf MODE,w					; load display mode
  addwf PCL,f					; advance the PC
  goto DISPLAY0					; fan status display
  goto DISPLAY1					; fan status w/1 selected
  goto DISPLAY2					; fan status w/2 selected
  goto DISPLAY3					; fan status w/3 selected
  goto DISPLAY4					; lighting status
  goto DISPLAY5					; temperatures
DISPLAY0
  call DISPLAY_FAN_HEADER
DISP0_F1
  LCD_PRINT ' '
  LCD_PRINT '1'
  LCD_PRINT ':'					; "1:"
  btfss F1_S					; is it on?
    goto DISP0_F1_OFF
  call DISPLAY_ON				; "On"
  goto DISP0_F2
DISP0_F1_OFF
  call DISPLAY_OFF				; "Off"
DISP0_F2
  LCD_PRINT ' '
  LCD_PRINT '2'
  LCD_PRINT ':'					; "2:"
  btfss F2_S					; is it on?
    goto DISP0_F2_OFF
  call DISPLAY_ON				; "On"
  goto DISP0_F3
DISP0_F2_OFF
  call DISPLAY_OFF				; "Off"
DISP0_F3
  LCD_PRINT ' '
  LCD_PRINT '3'
  LCD_PRINT ':'					; "3:"
  btfss F3_S					; is it on?
    goto DISP0_F3_OFF
  call DISPLAY_ON				; "On"
  return
DISP0_F3_OFF
  call DISPLAY_OFF				; "Off"
  return
DISPLAY1
  call DISPLAY_FAN_HEADER
DISP1_F1
  LCD_PRINT 0x7E
  LCD_PRINT '1'
  LCD_PRINT ':'					; ">1:"
  btfss F1_S					; is it on?
    goto DISP1_F1_OFF
  call DISPLAY_ON				; "On"
  LCD_PRINT 0x7F				; "<"
  goto DISP1_F2
DISP1_F1_OFF
  call DISPLAY_OFF				; "Off"
  LCD_PRINT 0x7F				; "<"
DISP1_F2
  LCD_PRINT '2'
  LCD_PRINT ':'					; "2:"
  btfss F2_S					; is it on?
    goto DISP1_F2_OFF
  call DISPLAY_ON				; "On"
  goto DISP1_F3
DISP1_F2_OFF
  call DISPLAY_OFF				; "Off"
DISP1_F3
  LCD_PRINT ' '
  LCD_PRINT '3'
  LCD_PRINT ':'					; "3:"
  btfss F3_S					; is it on?
    goto DISP1_F3_OFF
  call DISPLAY_ON				; "On"
  return
DISP1_F3_OFF
  call DISPLAY_OFF				; "Off"
  return
DISPLAY2
  call DISPLAY_FAN_HEADER
DISP2_F1
  LCD_PRINT ' '
  LCD_PRINT '1'
  LCD_PRINT ':'					; "1:"
  btfss F1_S					; is it on?
    goto DISP2_F1_OFF
  call DISPLAY_ON				; "On"
  goto DISP2_F2
DISP2_F1_OFF
  call DISPLAY_OFF				; "Off"
DISP2_F2
  LCD_PRINT 0x7E
  LCD_PRINT '2'
  LCD_PRINT ':'					; ">2:"
  btfss F2_S					; is it on?
    goto DISP2_F2_OFF
  call DISPLAY_ON				; "On"
  LCD_PRINT 0x7F				; "<"
  goto DISP2_F3
DISP2_F2_OFF
  call DISPLAY_OFF				; "Off"
  LCD_PRINT 0x7F				; "<"
DISP2_F3
  LCD_PRINT '3'
  LCD_PRINT ':'					; "3:"
  btfss F3_S					; is it on?
    goto DISP2_F3_OFF
  call DISPLAY_ON				; "On"
  return
DISP2_F3_OFF
  call DISPLAY_OFF				; "Off"
  return
DISPLAY3
  call DISPLAY_FAN_HEADER
DISP3_F1
  LCD_PRINT ' '
  LCD_PRINT '1'
  LCD_PRINT ':'					; "1:"
  btfss F1_S					; is it on?
    goto DISP3_F1_OFF
  call DISPLAY_ON				; "On"
  goto DISP3_F2
DISP3_F1_OFF
  call DISPLAY_OFF				; "Off"
DISP3_F2
  LCD_PRINT ' '
  LCD_PRINT '2'
  LCD_PRINT ':'					; "2:"
  btfss F2_S					; is it on?
    goto DISP3_F2_OFF
  call DISPLAY_ON				; "On"
  goto DISP3_F3
DISP3_F2_OFF
  call DISPLAY_OFF				; "Off"
DISP3_F3
  LCD_PRINT 0x7E
  LCD_PRINT '3'
  LCD_PRINT ':'					; ">3:"
  btfss F3_S					; is it on?
    goto DISP3_F3_OFF
  call DISPLAY_ON				; "On"
  LCD_PRINT 0x7F				; "<"
  return
DISP3_F3_OFF
  call DISPLAY_OFF				; "Off"
  LCD_PRINT 0x7F				; "<"
  return
DISPLAY4
  call DISPLAY_LIGHT_HEADER
  LCD_PRINT ' '
  LCD_PRINT ' '
  LCD_PRINT ' '
  LCD_PRINT ' '
  btfss LIT_S					; is the light on?
    goto LIGHT_OFF
LIGHT_ON
  LCD_PRINT ' '
  LCD_PRINT 'A'
  LCD_PRINT 'c'
  LCD_PRINT 't'
  LCD_PRINT 'i'
  LCD_PRINT 'v'
  LCD_PRINT 'a'
  LCD_PRINT 't'
  LCD_PRINT 'e'
  LCD_PRINT 'd'					; "Activated"
  return
LIGHT_OFF
  LCD_PRINT 'D'
  LCD_PRINT 'e'
  LCD_PRINT 'a'
  LCD_PRINT 'c'
  LCD_PRINT 't'
  LCD_PRINT 'i'
  LCD_PRINT 'v'
  LCD_PRINT 'a'
  LCD_PRINT 't'
  LCD_PRINT 'e'
  LCD_PRINT 'd'					; "Deactivated"
  return
DISPLAY5
  call DISPLAY_TEMPERATURE_HEADER
  LCD_PRINT ' '
  LCD_PRINT ' '
  LCD_PRINT '1'
  LCD_PRINT ':'
  movlw 0x80
  movwf ADCON0					; channel 0
  bsf ADCON0,ADON				; turn the ADC on
  DELAY_MICRO 25				; wait for it to start up
  bsf ADCON0,GO_DONE				; start the conversion
  btfsc ADCON0,GO_DONE				; wait for it to complete
    goto $-1  
  bcf ADCON0,ADON				; shut the ADC off
  bcf PIR1,ADIF					; clear interrupt flag (?)
  rlf ADRESH,w					; shift into W
  BANK1
  btfss ADRESL^0x80,7				; gain that extra precision
    goto $+2
  addlw 1
  BANK0
  call BINARY_TO_BCD				; convert A/D value to BCD
  call LCD_OUT_TEMPS				; output 2 digit temperature
  LCD_PRINT 0xDF
  LCD_PRINT 'C'
  LCD_PRINT ' '
  LCD_PRINT ' '
  LCD_PRINT ' '
  LCD_PRINT ' '
  LCD_PRINT '2'
  LCD_PRINT ':'
  movlw 0x88
  movwf ADCON0					; channel 1
  bsf ADCON0,ADON				; turn the ADC on
  DELAY_MICRO 25				; wait for it to start up
  bsf ADCON0,GO_DONE				; start the conversion
  btfsc ADCON0,GO_DONE				; wait for it to complete
    goto $-1  
  bcf ADCON0,ADON				; shut the ADC off
  bcf PIR1,ADIF					; clear interrupt flag (?)
  rlf ADRESH,w					; shift into W
  BANK1
  btfss ADRESL^0x80,7				; gain that extra precision
    goto $+2
  addlw 1
  BANK0
  call BINARY_TO_BCD				; convert A/D value to BCD
  call LCD_OUT_TEMPS				; output 2 digit temperature
  LCD_PRINT 0xDF
  LCD_PRINT 'C'
  return

DISPLAY_FAN_HEADER				; "Fan Status" header line
  LCD_CLEAR
  LCD_PRINT ' '
  LCD_PRINT ' '
  LCD_PRINT ' '
  LCD_PRINT ' '
  LCD_PRINT ' '
  LCD_PRINT 'F'
  LCD_PRINT 'a'
  LCD_PRINT 'n'
  LCD_PRINT ' '
  LCD_PRINT 'S'
  LCD_PRINT 't'
  LCD_PRINT 'a'
  LCD_PRINT 't'
  LCD_PRINT 'u'
  LCD_PRINT 's'
  LCD_HOME_LINE2
  return

DISPLAY_ON					; "On"
  LCD_PRINT 'O'
  LCD_PRINT 'n'
  LCD_PRINT ' '
  return

DISPLAY_OFF					; "Off"
  LCD_PRINT 'O'
  LCD_PRINT 'f'
  LCD_PRINT 'f'
  return

DISPLAY_LIGHT_HEADER				; "Lighting Status" header line
  LCD_CLEAR
  LCD_PRINT ' '
  LCD_PRINT ' '
  LCD_PRINT 'L'
  LCD_PRINT 'i'
  LCD_PRINT 'g'
  LCD_PRINT 'h'
  LCD_PRINT 't'
  LCD_PRINT 'i'
  LCD_PRINT 'n'
  LCD_PRINT 'g'
  LCD_PRINT ' '
  LCD_PRINT 'S'
  LCD_PRINT 't'
  LCD_PRINT 'a'
  LCD_PRINT 't'
  LCD_PRINT 'u'
  LCD_PRINT 's'
  LCD_HOME_LINE2
  return

DISPLAY_TEMPERATURE_HEADER			; "Temperatures" header line
  LCD_CLEAR
  LCD_PRINT ' '
  LCD_PRINT ' '
  LCD_PRINT ' '
  LCD_PRINT ' '
  LCD_PRINT 'T'
  LCD_PRINT 'e'
  LCD_PRINT 'm'
  LCD_PRINT 'p'
  LCD_PRINT 'e'
  LCD_PRINT 'r'
  LCD_PRINT 'a'
  LCD_PRINT 't'
  LCD_PRINT 'u'
  LCD_PRINT 'r'
  LCD_PRINT 'e'
  LCD_PRINT 's'
  LCD_HOME_LINE2
  return

UPDATE_HARDWARE
S_FAN1
  btfss F1_S					; should the fan be on?
    goto S_FAN1_OFF
  bsf FAN1					; if so, turn it on
  goto S_FAN2
S_FAN1_OFF
  bcf FAN1					; otherwise turn it off
S_FAN2
  btfss F2_S					; should the fan be on?
    goto S_FAN2_OFF
  bsf FAN2					; if so, turn it on
  goto S_FAN3
S_FAN2_OFF
  bcf FAN2					; otherwise turn it off
S_FAN3
  btfss F3_S					; should the fan be on?
    goto S_FAN3_OFF
  bsf FAN3					; if so, turn it on
  goto S_LIGHT
S_FAN3_OFF
  bcf FAN3					; otherwise turn it off
S_LIGHT
  btfss LIT_S					; should the light be on?
    goto S_LIGHT_OFF
  bsf LIGHT					; if so, turn it on
  goto S_DONE
S_LIGHT_OFF
  bcf LIGHT					; otherwise turn it off
S_DONE
  return

;----------------------------------------------------------------------------------------
; Mainline of code
;----------------------------------------------------------------------------------------

 org 1024

MAIN  
  BANK1						; set up registers
  movlw 0x3F					; W <- xx11 1111
  movwf TRISA ^ 0x80				; PORTA[5:0] input
  clrf TRISB ^ 0x80				; PORTB[7:0] output
  clrf TRISC ^ 0x80				; PORTC[7:0] output
  movlw 0x04					; W <- 1xxx 0100
  movwf ADCON1 ^ 0x80				; set ADCON1
  BANK0
  clrf MODE					; start at MODE = 0
  movlw 40					; load 40
  movwf REFRESH_TIMER1				; into the refresh delay timer

  ; ** EEPROM DATA RETRIEVAL
  BANK2						; switch to bank2 for EEADR
  movlw 0					; load DEVSTATUS EEPROM address
  movwf EEADR ^ 0x100				; move it to EEADR
  BANK3						; get to EECON1 bank
  bcf EECON1 ^ 0x180,EEPGD			; DATA memory...
  bsf EECON1 ^ 0x180,RD				; initiate the read
  BANK2						; get back to bank2
  movf EEDATA ^ 0x100,w				; get our data out...
  BANK0						; get back to bank0
  movwf DEVSTATUS				; set the devstatus stuff

  call UPDATE_HARDWARE				; make it so number one

LCD_INITIALIZATION				; setup the LCD for us...
  bcf LCD_RS					; LCD command mode
  bcf LCD_RW					; LCD write mode
  bcf LCD_E					; LCD strobe low
  DELAY_MILLI 20				; 20ms wait from startup
  LCD_COMMAND 0x30				; required
  DELAY_MILLI 5					; 5ms delay
  LCD_COMMAND 0x30				; required
  DELAY_MICRO 125				; 125us delay
  LCD_COMMAND 0x30				; required
  DELAY_MICRO 125				; 125us delay
  LCD_COMMAND 0x38				; 8-bit, 2-line, fontset
  DELAY_MILLI 5					; 5ms delay
  LCD_COMMAND 0x0C				; no cursor, no blink
  DELAY_MILLI 5					; 5ms delay
  LCD_COMMAND 0x06				; auto-increment cursor
  DELAY_MILLI 5					; 5ms delay
  LCD_CLEAR					; clear it out
  call LCD_SPLASH				; display the splash screen
  LCD_CLEAR					; done
  call UPDATE_DISPLAY				; update the display to start

MAINLOOP					; button watching loop
  btfss B_MODE					; test for mode
    call BUTTONPRESS_MODE
  btfss B_GO					; test for go
    call BUTTONPRESS_GO
  DELAY_MILLI 50
  decfsz REFRESH_TIMER1,f			; totally funky
    goto MAINLOOP				; screen refreshing
  movlw 40					; delay code
  movwf REFRESH_TIMER1				; because I didn't feel like
  call UPDATE_DISPLAY				; messing w/interrupts for some reason
  goto MAINLOOP

 org 1536

BINARY_TO_BCD
  movwf BCD_TEMP				; save our number
  movlw 8
  movwf BCD_COUNTER				; load counter w/8 (8bit #)
  clrf TENS					; zero out TENS
  clrf ONES					; zero out ONES
BCDLOOP
BCD_TENS_TEST					; if TENS digit is >= 5, add 3
  movlw 5
  subwf TENS,w
  btfss STATUS,C				; if TENS < 5
    goto BCD_ONES_TEST				; goto next test
  movlw 3					; otherwise add 3
  addwf TENS,f
BCD_ONES_TEST					; if ONES digit is >= 5, add 3
  movlw 5
  subwf ONES,w
  btfss STATUS,C				; if ONES < 5
    goto BCD_SHIFT				; goto shift section
  movlw 3					; otherwise add 3
  addwf ONES,f
BCD_SHIFT					; shift left all the way through
  rlf TENS,f					; shift TENS left
  rlf ONES,f					; shift ONES left
  btfss ONES,4					; check for shift-out bit
    goto BCD_TEMP_SHIFT				; if nothing, move on
  movlw 1					; otherwise add 1 to TENS
  addwf TENS,f
BCD_TEMP_SHIFT
  rlf BCD_TEMP,f				; shift the value left
  btfss STATUS,C				; if there's no carryout
    goto BCD_SHIFTDONE				; move on
  movlw 1					; otherwise add 1 to ONES
  addwf ONES,f
BCD_SHIFTDONE
  movlw 0x0F					; we only need lower nybble
  andwf ONES,f					; mask out dead bits
  andwf TENS,f					; mask out dead bits
  decfsz BCD_COUNTER,f				; decrement BCD_COUNTER
    goto BCDLOOP				; if !=0 goto BCDLOOP
  return					; otherwise we should be done

LCD_OUT_TEMPS
  movf TENS,w					; load value
  addlw 48					; shift it to its ASCII value
  call LCD_OUT_DATA				; spit it out
  movf ONES,w					; load value
  addlw 48					; shift it to its ASCII value
  call LCD_OUT_DATA				; spit her out too
  return

 end						; viola, instant baybus.
