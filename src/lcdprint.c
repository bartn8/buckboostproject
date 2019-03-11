/*
This file is part of BuckBoostProject.

BuckBoostProject is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

BuckBoostProject is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with BuckBoostProject.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
* LCD stuff: http://davidegironi.blogspot.com/2013/06/an-avr-atmega-library-for-hd44780-based.html
*/

/*
* lcdprint.c
*
* Created: 04/03/2019 23:24:42
*  Author: Luca Bartolomei
*/

#include <avr/pgmspace.h>
#include "thirdparty/lcdpcf8574/lcdpcf8574.h"
#include "lcdprint.h"

//DISPLAY CONSTANTS
const char finalCharacter[] = "<UDCM";

const char voltsUnit[] PROGMEM = "V";

const char oneSpace[] PROGMEM = " ";
const char twoSpaces[] PROGMEM = "  ";
const char threeSpaces[] PROGMEM = "   ";
const char fourSpaces[] PROGMEM = "    ";
const char sixSpaces[] PROGMEM = "       ";
const char sevenSpaces[] PROGMEM = "       ";

const char on[] PROGMEM = "ON ";
const char off[] PROGMEM = "OFF";

void initDisplay()
{
	lcd_init(LCD_DISP_ON);	//init lcd
	lcd_home();				//lcd go home
	lcd_led(0);				//Retro-illumination on
	
	//First display value set.
	
	//Clear display.
	lcd_clrscr();
}

void printPSULine(PowerState state, int y)
{
	lcd_gotoxy(0,y);
	lcd_puts_p(PSTR("PSU STATE: "));
	lcd_gotoxy(11,y);
	lcd_puts_p(state == PSU_ON ? on : off);
	lcd_gotoxy(14,y);
	lcd_puts_p(oneSpace);
}

void printBoostLine(const char *boostVoltageString, int y)
{
	lcd_gotoxy(0,y);
	lcd_puts_p(PSTR("VH: "));
	lcd_gotoxy(4,y);
	lcd_puts_p(sevenSpaces);
	lcd_gotoxy(4,y);
	lcd_puts(boostVoltageString);
	lcd_gotoxy(11,y);
	lcd_puts_p(voltsUnit);
	lcd_gotoxy(12,y);
	lcd_puts_p(threeSpaces);
}

void printBuckLine(const char *buckVoltageString, int y)
{
	lcd_gotoxy(0,y);
	lcd_puts_p(PSTR("VL: "));
	lcd_gotoxy(4,y);
	lcd_puts_p(sevenSpaces);
	lcd_gotoxy(4,y);
	lcd_puts(buckVoltageString);
	lcd_gotoxy(11,y);
	lcd_puts_p(voltsUnit);
	lcd_gotoxy(12,y);
	lcd_puts_p(threeSpaces);
}

void printRefLine(const char *refVoltageString, int y)
{
	lcd_gotoxy(0,y);
	lcd_puts_p(PSTR("VREF: "));
	lcd_gotoxy(6,y);
	lcd_puts_p(sevenSpaces);
	lcd_gotoxy(6,y);
	lcd_puts(refVoltageString);
	lcd_gotoxy(13,y);
	lcd_puts_p(voltsUnit);
	lcd_gotoxy(14,y);
	lcd_puts_p(oneSpace);
}

void printBoostFactorLine(const char *boostFactorString, int y)
{
	lcd_gotoxy(0,y);
	lcd_puts_p(PSTR("HFACT: "));
	lcd_gotoxy(7,y);
	lcd_puts_p(sixSpaces);
	lcd_gotoxy(7,y);
	lcd_puts(boostFactorString);
	lcd_gotoxy(13,y);
	lcd_puts_p(twoSpaces);
}

void printBuckFactorLine(const char *buckFactorString, int y)
{
	lcd_gotoxy(0,y);
	lcd_puts_p(PSTR("LFACT: "));
	lcd_gotoxy(7,y);
	lcd_puts_p(sixSpaces);
	lcd_gotoxy(7,y);
	lcd_puts(buckFactorString);
	lcd_gotoxy(13,y);
	lcd_puts_p(twoSpaces);
}

void printFinalChar(int state)
{
	lcd_gotoxy(15,0);
	lcd_putc(finalCharacter[state]);
}