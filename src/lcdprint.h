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
 * lcdprint.h
 *
 * Created: 04/03/2019 23:24:31
 *  Author: Luca Bartolomei
 */

#ifndef LCDPRINT_H_
#define LCDPRINT_H_

#include "state.h"

//   00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15
//0  P  S  U     S  T  A  T  E  :     O  F  F     <
//1  V  O  U  T  :     0  0  .  0  0  0     V
//2  A  O  U  T  :     0  0  .  0  0  0     A
//3  V  R  E  F  :     0  0  .  0  0  0     V
//4  V  F  A  C  T  :     0  0  .  0  0  0
//5  S  F  A  C  T  :     0  0  .  0  0  0
//6  S  R  E  S  :     0  0  .  0  0  0     

//Display Methods.

/**
 * Init display, go home(0,0) and enable retro-illumination.
 */
void initDisplay();

/**
 * Print the status of PSU (ON-OFF).
 * Needs the y coordinate on display.
 */
void printPSULine(PowerState state, int y);

/**
 * Print the output voltage.
 * Needs the y coordinate on display.
 */
void printVoltageLine(const char *voltageVoltageString, int y);

/**
 * Print the output ampere.
 * Needs the y coordinate on display.
 */
void printAmpereLine(const char *ampereVoltageString, int y);

/**
 * Print the output voltage reference.
 * Needs the y coordinate on display.
 */
void printVRefLine(const char *refVoltageString, int y);

/**
 * Print the voltage factor.
 * Needs the y coordinate on display.
 */
void printVoltageFactorLine(const char *voltageFactorString, int y);

/**
 * Print the shunt factor.
 * Needs the y coordinate on display.
 */
void printShuntFactorLine(const char *shuntFactorString, int y);

/**
 * Print the shunt resistor.
 * Needs the y coordinate on display.
 */
void printShuntResistorLine(const char *shuntResistorString, int y);

/**
 * Print final char.
 * Configured for 16x2 LCD.
 * Shows the current selection.
 * Needs the y coordinate on display.
 */
void printFinalChar(int state);


#endif /* LCDPRINT_H_ */
