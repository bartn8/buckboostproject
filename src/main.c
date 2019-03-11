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
* LCD stuff: http://davidegironi.blogspot.com/2013/06/an-avr-atmega-library-for-hd44780-based.html
* I2C lib: https://github.com/felias-fogg/SoftI2CMaster
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

#include "bool.h"
#include "state.h"
#include "hwconf.h"
#include "lcdprint.h"

//OPTIMIZATION CONSTANTS
const float factors[] = {1.0f, 0.1f, 0.01f, 0.001f};

//Boost Voltage, Buck Voltage, Ref Voltage, Boost Factor, Buck Factor.
#define VOLTAGE_VECTOR_LENGTH 5
volatile float voltageVector[VOLTAGE_VECTOR_LENGTH];

//Boost ADC, Buck ADC.
#define ADC_VECTOR_LENGTH 2
volatile uint16_t adcVector[ADC_VECTOR_LENGTH];

//Things for PSU stuff

//This is not saved in EEPROM for safety (Always OFF on RESET).
volatile PowerState psuState;

//Things to save in EEPROM

//Boost Voltage, Buck Voltage, Ref Voltage, Boost Factor, Buck Factor.
float EEMEM voltageVectorEEMEM[VOLTAGE_VECTOR_LENGTH] = {DEFAULT_BOOST_VOLTAGE, DEFAULT_BUCK_VOLTAGE, DEFAULT_REF_VOLTAGE, DEFAULT_BOOST_FACTOR, DEFAULT_BUCK_FACTOR};

//It is used to check that EEPROM is ok.
bool EEMEM isEEPROMLoadedEEMEM = TRUE;

//Things for Push-Button
volatile ButtonState previousBtnState = BTN_HIGH;

//Things for FSM

volatile FiniteState state;

volatile UnitState unitState;

//Things for Display

volatile DisplayState displayState;

char boostVoltageString[7] = "00.000";
char buckVoltageString[7] = "00.000";
char refVoltageString[7] = "0.0000";
char boostFactorString[7] = "00.000";
char buckFactorString[7] = "00.000";

//Things for ADC

volatile float *boostFactor = &voltageVector[3];
volatile float *buckFactor = &voltageVector[4];

#define VOLT2ADC(Volt, Vref) ((Volt * 1024) / Vref)
#define ADC2VOLT(ADCValue, Vref) ((ADCValue * Vref * 1.0f) / 1024)

#define ACTUAL_BOOST_VOLTAGE(V) (V * (*boostFactor))
#define ACTUAL_BUCK_VOLTAGE(V) (V * (*buckFactor))

#define VIRTUAL_BOOST_VOLTAGE(V) (V / (*boostFactor))
#define VIRTUAL_BUCK_VOLTAGE(V) (V / (*buckFactor))

volatile float *boostVoltage = &voltageVector[0];
volatile float *buckVoltage = &voltageVector[1];
volatile float *refVoltage = &voltageVector[2];

volatile uint16_t boostSetADC = 0;
volatile uint16_t buckSetADC = 0;

volatile uint16_t *boostOutputADC = &adcVector[0];
volatile uint16_t *buckOutputADC = &adcVector[1];

//Things for Rotary Encoder
#define DIR_LEFT 0
#define DIR_RIGHT 1


volatile uint8_t lastEncoded = 0;
volatile uint8_t debouncingCounter = 0;
volatile uint8_t debouncingDirection = DIR_LEFT;

//Init Methods

void initStates()
{
	//Reset states.
	state = STATE_DISPLAY;
	unitState = UNIT;
	displayState = SELECT_PSU;
	previousBtnState = BTN_HIGH;
	psuState = PSU_OFF;
}

//EEPROM Methods

void checkEEPROM()
{
	bool isEEPROMLoaded = eeprom_read_byte(&isEEPROMLoadedEEMEM);
	
	if(!isEEPROMLoaded)
	{
		float voltageVectorTmp[VOLTAGE_VECTOR_LENGTH] = {DEFAULT_BOOST_VOLTAGE, DEFAULT_BUCK_VOLTAGE, DEFAULT_REF_VOLTAGE, DEFAULT_BOOST_FACTOR, DEFAULT_BUCK_FACTOR};
		
		eeprom_write_block(voltageVectorTmp, voltageVectorEEMEM, VOLTAGE_VECTOR_LENGTH * sizeof(float));
		eeprom_write_byte(&isEEPROMLoadedEEMEM, TRUE);
	}
}

void loadEEPROM()
{
	//Ignore volatile warning?
	//eeprom_read_block(&voltageVector, &voltageVectorEEPROM, VOLTAGE_VECTOR_LENGTH * sizeof(float));

	//Tmp values
	uint8_t i;
	float voltageVectorTmp[VOLTAGE_VECTOR_LENGTH];

	eeprom_read_block(voltageVectorTmp, voltageVectorEEMEM, VOLTAGE_VECTOR_LENGTH * sizeof(float));

	for(i=0;i<VOLTAGE_VECTOR_LENGTH;i++) voltageVector[i] = voltageVectorTmp[i];
}


void saveEEPROM()
{
	//Tmp values
	uint8_t i;
	float voltageVectorTmp[VOLTAGE_VECTOR_LENGTH];
	for(i=0;i<VOLTAGE_VECTOR_LENGTH;i++)  voltageVectorTmp[i] = voltageVector[i];
	
	eeprom_write_block(voltageVectorTmp, voltageVectorEEMEM, VOLTAGE_VECTOR_LENGTH * sizeof(float));
}

//Handlers

void ADCHandler()
{
	if(!(ADCSRA & _BV(ADSC)))
	{
		//Save value.
		adcVector[ADMUX & 0x01] = ADC;

		//Switch mux
		ADMUX ^= 0x01;

		//Start new conversion.
		ADCSRA |= _BV(ADSC);
		
		//Wait until conversion has finished.
		//while(ADCSRA & _BV(ADSC));
	}
}

void buckHandler()
{
	buckSetADC = VOLT2ADC(VIRTUAL_BUCK_VOLTAGE(*buckVoltage), *refVoltage);

	//Mask is 0x01FF (9 bit) because Timer 1 is limited to 256 count in this Fast-PWM mode.
	if(*buckOutputADC > buckSetADC && OCR1A + 1 <= 0x01FF) OCR1A = (OCR1A + 1);
	if(*buckOutputADC < buckSetADC && OCR1A - 1 >= 0x0000) OCR1A = (OCR1A - 1);
}

void boostHandler()
{
	boostSetADC = VOLT2ADC(VIRTUAL_BOOST_VOLTAGE(*boostVoltage), *refVoltage);

	//Mask is 0xFF (8 bit) because Timer 1 is limited to 256 count in this Fast-PWM mode.
	if(*boostOutputADC > boostSetADC && OCR1B - 1 >= 0x00) OCR1B = (OCR1B - 1) & 0xFF;
	if(*boostOutputADC < boostSetADC && OCR1B + 1 <= 0xFF) OCR1B = (OCR1B + 1) & 0xFF;
}

void PSUHandler()
{
	if(state == STATE_INVERT_PSU)
	{
		psuState = psuState == PSU_ON ? PSU_OFF : PSU_ON;	//Invert state.
		INVERT_PSU_STATE;									//Send real signal to PSU.

		//Enable/Disable PWM signals.
		if(psuState == PSU_ON) ENABLE_PWM_SIGNALS;
		if(psuState == PSU_OFF) DISABLE_PWM_SIGNALS;

		state = STATE_DISPLAY;
	}
}

//MAIN METHOD
int main(void)
{
	checkEEPROM();
	
	//Inits
	initStates();
	loadEEPROM();
	initDisplay();

	//First disable interrupt.
	cli();

	setIO();
	setEncoder();
	setADC();
	setPWMTimer();
	setPollingTimer();
	setCheckTimer();

	sei();

	while (TRUE)
	{

		//Handlers.
		PSUHandler();
		
		//Display things.
		
		//Display current view by selection.
		switch(displayState)
		{
			case SELECT_PSU:
			//Convert float to string.
			if(state == STATE_DISPLAY)
			{
				dtostrf(ACTUAL_BOOST_VOLTAGE(ADC2VOLT(*boostOutputADC, *refVoltage)), 5, 3, boostVoltageString);
			}
			else
			{
				dtostrf(*boostVoltage, 5, 3, boostVoltageString);
			}
			
			printPSULine(psuState, 0);
			printBoostLine(boostVoltageString, 1);
			break;
			case SELECT_BOOST:
			//Convert float to string.
			if(state == STATE_DISPLAY)
			{
				dtostrf(ACTUAL_BOOST_VOLTAGE(ADC2VOLT(*boostOutputADC, *refVoltage)), 5, 3, boostVoltageString);
				dtostrf(ACTUAL_BUCK_VOLTAGE(ADC2VOLT(*buckOutputADC, *refVoltage)), 5, 3, buckVoltageString);
			}
			else
			{
				dtostrf(*boostVoltage, 5, 3, boostVoltageString);
				dtostrf(*buckVoltage, 5, 3, buckVoltageString);
			}
			
			
			printBoostLine(boostVoltageString, 0);
			printBuckLine(buckVoltageString, 1);
			break;
			case SELECT_BUCK:
			//Convert float to string.
			if(state == STATE_DISPLAY)
			{
				dtostrf(ACTUAL_BOOST_VOLTAGE(ADC2VOLT(*boostOutputADC, *refVoltage)), 5, 3, boostVoltageString);
				dtostrf(ACTUAL_BUCK_VOLTAGE(ADC2VOLT(*buckOutputADC, *refVoltage)), 5, 3, buckVoltageString);
			}
			else
			{
				dtostrf(*boostVoltage, 5, 3, boostVoltageString);
				dtostrf(*buckVoltage, 5, 3, buckVoltageString);
			}
			dtostrf(*refVoltage, 5, 3, refVoltageString);
			
			printBuckLine(buckVoltageString, 0);
			printRefLine(refVoltageString, 1);
			break;
			case SELECT_REF:
			//Convert float to string.
			dtostrf(*refVoltage, 5, 3, refVoltageString);
			dtostrf(*boostFactor, 5, 3, boostFactorString);
			
			printRefLine(refVoltageString, 0);
			printBoostFactorLine(boostFactorString, 1);
			break;
			case SELECT_BOOST_FACTOR:
			//Convert float to string.
			dtostrf(*boostFactor, 5, 3, boostFactorString);
			dtostrf(*buckFactor, 5, 3, buckFactorString);
			
			printBoostFactorLine(boostFactorString, 0);
			printBuckFactorLine(buckFactorString, 1);
			break;
			case SELECT_BUCK_FACTOR:
			//Convert float to string.
			dtostrf(*buckFactor, 5, 3, buckFactorString);

			printBuckFactorLine(buckFactorString, 0);
			printPSULine(psuState, 1);
			break;
		}

		printFinalChar(state == STATE_DISPLAY ? 0 : unitState + 1);

		//_delay_ms(16);
	}
}

//TIMER COMP A INTERRUPT (BUCK)
//Disabled: slow down LCD.
//Adjust PWM Duty cycle for buck converter.
//ISR(TIMER1_COMPA_vect)

//TIMER COMP B INTERRUPT (BOOST)
//Disabled: slow down LCD.
//Adjust PWM Duty cycle for boost converter.
//ISR(TIMER1_COMPB_vect)

//ADC INTERRUPT
//Read ADC value and start next conversion.
//Disabled: slow down LCD.

//TIMER 0 OVERFLOW INTERRUPT
//Read ADC value and start next conversion.
//Adjust PWM Duty cycle for buck converter.
//Adjust PWM Duty cycle for boost converter.
ISR(TIMER0_OVF_vect)
{
	ADCHandler();
	buckHandler();
	boostHandler();
}


//ENCODER INTERRUPT
ISR(PCINT2_vect)
{
	//PIND: 7 6 5 4 3 2 1 0
	//ENC:  0 0 C D 0 0 0 0
	
	uint8_t sum = (lastEncoded << 2) | ((PIND >> 4) & 0x03);

	//Set state will choose the factor.
	if(state != STATE_DISPLAY)
	{
		float factor = factors[unitState];

		//Incremental rotation
		if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
		{
			if(debouncingDirection != DIR_RIGHT)
			{
				debouncingDirection = DIR_RIGHT;
				debouncingCounter = 0;
			}
			else if(debouncingCounter < ENCODER_STEPS)
			{
				debouncingCounter++;
			}
			else
			{
				voltageVector[state-2] += factor;
				debouncingCounter = 0;
			}
		}
		
		//Decremental rotation
		if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
		{
			if(debouncingDirection != DIR_LEFT)
			{
				debouncingDirection = DIR_LEFT;
				debouncingCounter = 0;
			}
			else if(debouncingCounter < ENCODER_STEPS)
			{
				debouncingCounter++;
			}
			else
			{
				voltageVector[state-2] -= factor;
				debouncingCounter = 0;
			}
		}

		//Check ratings
		if(*buckVoltage < MIN_BUCK_VOLTAGE) *buckVoltage = MIN_BUCK_VOLTAGE;
		if(*buckVoltage > MAX_BUCK_VOLTAGE) *buckVoltage = MAX_BUCK_VOLTAGE;

		if(*boostVoltage < MIN_BOOST_VOLTAGE) *boostVoltage = MIN_BOOST_VOLTAGE;
		if(*boostVoltage > MAX_BOOST_VOLTAGE) *boostVoltage = MAX_BOOST_VOLTAGE;
	}
	else
	{
		//Incremental rotation
		if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
		{
			if(debouncingDirection != DIR_RIGHT)
			{
				debouncingDirection = DIR_RIGHT;
				debouncingCounter = 0;
			}
			else if(debouncingCounter < ENCODER_STEPS)
			{
				debouncingCounter++;
			}
			else
			{
				displayState = (displayState + 1) % DISPLAY_STATE_LENGTH;
				debouncingCounter = 0;
			}
		}
		
		//Decremental rotation
		if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
		{
			if(debouncingDirection != DIR_LEFT)
			{
				debouncingDirection = DIR_LEFT;
				debouncingCounter = 0;
			}
			else if(debouncingCounter < ENCODER_STEPS)
			{
				debouncingCounter++;
			}
			else
			{
				displayState = (displayState - 1) % DISPLAY_STATE_LENGTH;
				debouncingCounter = 0;
			}
		}
		
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
	ButtonState currentBtnState = READ_BTN_PIN;

	if(previousBtnState |= currentBtnState)
	{
		if(currentBtnState == BTN_LOW)
		{
			if(state == STATE_DISPLAY)
			{
				state = (displayState + 1) % FINITE_STATE_LENGTH;
			}
			else
			{
				if(state != STATE_INVERT_PSU)
				{
					if(unitState + 1 < UNIT_STATE_LENGTH) {
						unitState++;
					}
					else
					{
						//Save the EEPROM.
						saveEEPROM();
						
						//Return to main display state.
						unitState = UNIT;
						state = STATE_DISPLAY;
					}
				}
			}
		}
	}

	previousBtnState = currentBtnState;
	
	//Invert LED.
	INVERT_LED;
}

