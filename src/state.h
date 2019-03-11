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
 * state.h
 *
 * Created: 06/03/2019 21:29:07
 *  Author: Luca Bartolomei
 */ 


#ifndef STATE_H_
#define STATE_H_

//Removed enums because they cause an issue.

#define BTN_STATE_LENGTH 2

#define BTN_LOW 0
#define BTN_HIGH 1

typedef uint8_t ButtonState;

//enum ButtonState {BTN_LOW = 0, BTN_HIGH = 1}; 

//Turn ON/OFF the power supply.
#define PSU_STATE_LENGTH 2

#define PSU_OFF 0
#define PSU_ON 1

typedef uint8_t PowerState;

//enum PowerState {PSU_OFF = 0, PSU_ON = 1};

	
//Finite State Machine:
//Display mode:	show a menu, the encoder is used as cursor. Press button to enter.
//Invert PSU:	after inverting the psu, the state return in DISPLAY.
//Set mode:		with the encoder you can adjust value.
#define FINITE_STATE_LENGTH 7

#define STATE_DISPLAY 0
#define STATE_INVERT_PSU 1
#define STATE_SET_VOLTAGE_BOOST 2
#define STATE_SET_VOLTAGE_BUCK 3
#define STATE_SET_VOLTAGE_REF 4
#define STATE_SET_BOOST_DIV_FACTOR 5
#define STATE_SET_BUCK_DIV_FACTOR 6

typedef uint8_t FiniteState;

//enum FiniteState {DISPLAY = 0, INVERT_PSU = 1, SET_VOLTAGE_BOOST = 2, SET_VOLTAGE_BUCK = 3, SET_VOLTAGE_REF = 4, SET_BOOST_DIV_FACTOR = 5, SET_BUCK_DIV_FACTOR = 6};
	
//Fine value adjustment:
//First adjust unit, then press enter to adjust decimal and so on.
#define UNIT_STATE_LENGTH 4

#define UNIT 0
#define DEC 1
#define CENT 2
#define MILLI 3

typedef uint8_t UnitState;

//enum UnitState {UNIT = 0, DEC = 1, CENT = 2, MILLI = 3};
	
//Display menu possibilities.
#define DISPLAY_STATE_LENGTH 6

#define SELECT_PSU 0
#define SELECT_BOOST 1
#define SELECT_BUCK 2
#define SELECT_REF 3
#define SELECT_BOOST_FACTOR 4
#define SELECT_BUCK_FACTOR 5

typedef uint8_t DisplayState;

//enum DisplayState {SELECT_PSU = 0, SELECT_BOOST = 1, SELECT_BUCK = 2, SELECT_REF = 3, SELECT_BOOST_FACTOR = 4, SELECT_BUCK_FACTOR = 5};


#endif /* STATE_H_ */
