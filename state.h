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

//Turn ON/OFF the power supply.
#define PSU_STATE_LENGTH 2
enum PowerState {PSU_OFF, PSU_ON};
	
//State of a button.
#define BUTTON_STATE_LENGTH 2
enum ButtonState {BTN_LOW, BTN_HIGH};

//Finite State Machine:
//Display mode:	show a menu, the encoder is used as cursor. Press button to enter.
//Set mode:		with the encoder you can adjust value.
#define FINITE_STATE_LENGTH 7
enum FiniteState {DISPLAY, INVERT_PSU, SET_VOLTAGE_BOOST, SET_VOLTAGE_BUCK, SET_VOLTAGE_REF, SET_BOOST_DIV_FACTOR, SET_BUCK_DIV_FACTOR};
	
//Fine value adjustment:
//First adjust unit, then press enter to adjust decimal and so on.
#define UNIT_STATE_LENGTH 4
enum UnitState {UNIT, DEC, CENT, MILLI};
	
//Display menu possibilities.
#define DISPLAY_STATE_LENGTH 6
enum DisplayState {SELECT_PSU, SELECT_BOOST, SELECT_BUCK, SELECT_REF, SELECT_BOOST_FACTOR, SELECT_BUCK_FACTOR};


#endif /* STATE_H_ */