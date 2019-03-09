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
* Rotary Encoder stuff: http://www.giuseppecaccavale.it/arduino/rotary-encoder-arduino/
* Buck - Boost stuff: http://www.electronoobs.com/eng_circuitos_tut10_1.php - http://www.electronoobs.com/eng_circuitos_tut10.php
*/

/*
* BuckBoostConverter.c
*
* Created: 01/03/2019 16:05:53
* Author : Luca Bartolomei
*/

#include <stdlib.h>
#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "state.h"
#include "hwconf.h"
#include "lcdprint.h"

//OPTIMIZATION CONSTANTS
const float factors[] = {1.0, 0.1, 0.01, 0.001};

//Boost Voltage, Buck Voltage, Ref Voltage, Boost Factor, Buck Factor.
#define VOLTAGE_VECTOR_LENGTH 5
volatile float voltageVector[VOLTAGE_VECTOR_LENGTH];

//Boost ADC, Buck ADC.
#define ADC_VECTOR_LENGTH 2
volatile uint16_t adcVector[ADC_VECTOR_LENGTH];

//Things for PSU stuff

//This is not saved in EEPROM for safety (Always OFF on RESET).
volatile enum PowerState psuState;

//Things to save in EEPROM

//Boost Voltage, Buck Voltage, Ref Voltage, Boost Factor, Buck Factor.
float EEMEM voltageVectorEEPROM[VOLTAGE_VECTOR_LENGTH] = {15.0, 5.0, 5.0, 10.0, 3.0};

//Things for Push-Button

volatile enum ButtonState previousBtnState;

//Things for FSM

volatile enum FiniteState state;

volatile enum UnitState unitState;

//Things for Display

volatile enum DisplayState displayState;

char boostVoltageString[8] = "00.00";
char buckVoltageString[8] = "00.00";
char refVoltageString[8] = "0.000";
char boostFactorString[8] = "00.00";
char buckFactorString[8] = "00.00";

//Things for ADC

volatile float *boostFactor = &voltageVector[3];
volatile float *buckFactor = &voltageVector[4];

#define VOLT2ADC(Volt, Vref) ((Volt * 1024) / Vref)
#define ADC2VOLT(ADCValue, Vref) ((ADCValue * Vref * 1.0) / 1024)

#define ACTUAL_BOOST_VOLTAGE(V) (V / *boostFactor)
#define ACTUAL_BUCK_VOLTAGE(V) (V / *buckFactor)

volatile float *boostVoltage = &voltageVector[0];
volatile float *buckVoltage = &voltageVector[1];
volatile float *refVoltage = &voltageVector[2];

volatile uint16_t boostSetADC = 0;
volatile uint16_t buckSetADC = 0;

volatile uint16_t *boostOutputADC = &adcVector[0];
volatile uint16_t *buckOutputADC = &adcVector[1];

//Things for Rotary Encoder

volatile uint8_t lastEncoded = 0;

//Init Methods

void initStates()
{
	//Reset states.
	state = DISPLAY;
	unitState = UNIT;
	displayState = SELECT_PSU;
	previousBtnState = BTN_HIGH;
	psuState = PSU_OFF;
}

void initEEPROM()
{
	//Ignore volatile warning?
	//eeprom_read_block(&voltageVector, &voltageVectorEEPROM, VOLTAGE_VECTOR_LENGTH * sizeof(float));

	//Tmp values
	uint8_t i;
	float voltageVectorTmp[VOLTAGE_VECTOR_LENGTH];

	eeprom_read_block(&voltageVectorTmp, &voltageVectorEEPROM, VOLTAGE_VECTOR_LENGTH * sizeof(float));

	for(i=0;i<VOLTAGE_VECTOR_LENGTH;i++) voltageVector[i] = voltageVectorTmp[i];
}

int main(void)
{
	//Inits
	initStates();
	initEEPROM();
	initDisplay();

	//First disable interrupt.
	cli();

	setIO();
	setEncoder();
	setADC();
	setPWMTimer();
	setPollingTimer();

	sei();

	while (1)
	{
		//Convert float to string.
		sprintf(boostVoltageString, "%5.2f", *boostVoltage);
		sprintf(buckVoltageString, "%5.2f", *buckVoltage);
		sprintf(refVoltageString, "%5.3f", *refVoltage);
		sprintf(boostFactorString, "%5.2f", *boostFactor);
		sprintf(buckFactorString, "%5.2f", *buckFactor);

		//Here we handle the inversion of PSU power.
		if(state == INVERT_PSU)
		{
			psuState = !psuState;	//Invert state.
			INVERT_PSU_STATE;		//Send real signal to PSU.

			//Enable/Disable PWM signals.
			if(psuState == PSU_ON) ENABLE_PWM_SIGNALS;
			if(psuState == PSU_OFF) DISABLE_PWM_SIGNALS;

			state = DISPLAY;
		}

		//Display current view by selection.
		switch(displayState)
		{
			case SELECT_PSU:
				printPSULine(psuState, 0);
				printBoostLine(boostVoltageString, 1);
			break;
			case SELECT_BOOST:
				printBoostLine(boostVoltageString, 0);
				printBuckLine(buckVoltageString, 1);
			break;
			case SELECT_BUCK:
				printBuckLine(buckVoltageString, 0);
				printRefLine(refVoltageString, 1);
			break;
			case SELECT_REF:
				printRefLine(refVoltageString, 0);
				printBoostFactorLine(boostFactorString, 1);
			break;
			case SELECT_BOOST_FACTOR:
				printBoostFactorLine(boostFactorString, 0);
				printBuckFactorLine(buckFactorString, 1);
			break;
			case SELECT_BUCK_FACTOR:
				printBuckFactorLine(buckFactorString, 0);
				printPSULine(psuState, 1);
			break;
		}

		printFinalChar(state == DISPLAY ? 0 : unitState + 1);

		_delay_ms(10);
	}
}

//TIMER COMP A INTERRUPT (BOOST)
//Adjust PWM Duty cycle for boost converter.
ISR(TIMER1_COMPA_vect)
{
	boostSetADC = VOLT2ADC(ACTUAL_BOOST_VOLTAGE(*boostVoltage), *refVoltage);

	if(*boostOutputADC > boostSetADC) OCR1A = (OCR1A - 1) & 0xFF;
	if(*boostOutputADC < boostSetADC) OCR1A = (OCR1A + 1) & 0xFF;
}

//TIMER COMP B INTERRUPT (BUCK)
//Adjust PWM Duty cycle for buck converter.
ISR(TIMER1_COMPB_vect)
{
	buckSetADC = VOLT2ADC(ACTUAL_BUCK_VOLTAGE(*buckVoltage), *refVoltage);

	if(*buckOutputADC > buckSetADC) OCR1B = (OCR1B + 1) & 0xFF;
	if(*buckOutputADC < buckSetADC) OCR1B = (OCR1B - 1) & 0xFF;
}

//ADC INTERRUPT
//Read ADC value and start next conversion.
ISR(ANALOG_COMP_vect)
{
	//Save value.
	adcVector[ADMUX & 0x01] = ADC;

	//Switch mux
	ADMUX ^= 0x01;

	//Start new conversion.
	ADCSRA |= _BV(ADSC);
}

//ENCODER INTERRUPT
ISR(PCINT2_vect)
{
	//PIND: 7 6 5 4 3 2 1 0
	//ENC:  0 0 C D 0 0 0 0
	
	uint8_t sum = (lastEncoded << 2) | ((PIND >> 4) & 0x03);

	//Set state will choose the factor.
	if(state != DISPLAY)
	{
		float factor = factors[unitState];

		if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) voltageVector[state+1] += factor;	//Incremental rotation
		if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) voltageVector[state+1] -= factor;	//Decremental rotation

		//Check ratings
		if(*buckVoltage < MIN_BUCK_VOLTAGE) *buckVoltage = MIN_BUCK_VOLTAGE;
		if(*buckVoltage > MAX_BUCK_VOLTAGE) *buckVoltage = MAX_BUCK_VOLTAGE;

		if(*boostVoltage < MIN_BOOST_VOLTAGE) *boostVoltage = MIN_BOOST_VOLTAGE;
		if(*boostVoltage > MAX_BOOST_VOLTAGE) *boostVoltage = MAX_BOOST_VOLTAGE;
	}
	else
	{
		if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) displayState = (displayState + 1) % DISPLAY_STATE_LENGTH;	//Incremental rotation
		if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) displayState = (displayState - 1) % DISPLAY_STATE_LENGTH;	//Decremental rotation
	}

	lastEncoded = ((PIND >> 4) & 0x03); //store this value for next time
}

//Prototype Push-Button Handler:
//Use INT1 on Falling edge to handle button.

//Final Push-Button Handler:
//Use Timer 2 with a frequency of 100Hz
//Every Timer overflow check button state:
//If current state != previous state
//And current state = LOW
//Then TRIGGER ONLY ONCE
//Else current state = HIGH
//Reset TRIGGER ONCE VARIABLE.

//BUTTON INTERRUPT
ISR(TIMER2_OVF_vect)
{
	//Time to check buttons!
	uint8_t currentBtnState = READ_BTN_PIN;

	if(previousBtnState |= currentBtnState)
	{
		if(currentBtnState == BTN_LOW)
		{
			if(state == DISPLAY)
			{
				state = (displayState + 1) % FINITE_STATE_LENGTH;
			}
			else
			{
				if(unitState + 1 < UNIT_STATE_LENGTH) {
					unitState = UNIT;
					state = DISPLAY;
				}
				else
				{
					unitState++;
				}
			}
		}
	}

	previousBtnState = currentBtnState;
}
