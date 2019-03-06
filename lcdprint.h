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
//0  V  H  :     0  0  .  0  0     V              <
//1  V  L  :     0  0  .  0  0     V
//2  V  R  E  F  :     0  .  0  0  0     V
//3  H  F  A  C  T  :     0  0  .  0  0
//4  L  F  A  C  T  :     0  0  .  0  0

//Display Methods.

void initDisplay();

void printPSULine(enum PowerState state, int y);

void printBoostLine(const char *boostVoltageString, int y);

void printBuckLine(const char *buckVoltageString, int y);

void printRefLine(const char *refVoltageString, int y);

void printBoostFactorLine(const char *boostFactorString, int y);

void printBuckFactorLine(const char *buckFactorString, int y);

void printFinalChar(int state);


#endif /* LCDPRINT_H_ */