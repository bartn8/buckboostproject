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

//Voltage, Ampere, Ref Voltage, Voltage Factor, Shunt Factor, Shunt Resistor.
#define VALUES_VECTOR_LENGTH 6
volatile float valuesVector[VALUES_VECTOR_LENGTH];

//Voltage ADC, Shunt ADC.
#define ADC_VECTOR_LENGTH 2
volatile uint16_t adcVector[ADC_VECTOR_LENGTH];

//Things for PSU stuff

//This is not saved in EEPROM for safety (Always OFF on RESET).
volatile PowerState psuState;

//Things to save in EEPROM

//Voltage, Ampere, Ref Voltage, Voltage Factor, Shunt Factor, Shunt Resistor.
float EEMEM valuesVectorEEMEM[VALUES_VECTOR_LENGTH] = {DEFAULT_VOLTAGE, DEFAULT_AMPERE, DEFAULT_REF_VOLTAGE, DEFAULT_VOLTAGE_FACTOR, DEFAULT_SHUNT_FACTOR, DEFAULT_SHUNT_RESISTOR};

//It is used to check that EEPROM is ok.
bool EEMEM isEEPROMLoadedEEMEM = TRUE;

//Things for Push-Button
volatile ButtonState previousBtnState = BTN_HIGH;

//Things for FSM

volatile FiniteState state;

volatile UnitState unitState;

//Things for Display

volatile DisplayState displayState;

char voltageString[7] = "00.000";
char ampereString[7] = "00.000";
char refVoltageString[7] = "0.0000";
char voltageFactorString[7] = "00.000";
char shuntFactorString[7] = "00.000";
char shuntResistorString[7] = "00.000";

//Things for ADC

volatile float *voltage = &valuesVector[0];
volatile float *ampere = &valuesVector[1];
volatile float *refVoltage = &valuesVector[2];
volatile float *voltageFactor = &valuesVector[3];
volatile float *shuntFactor = &valuesVector[4];
volatile float *shuntResistor = &valuesVector[5];


//ADC = (Volts * ADC) / Volts
#define VOLT2ADC(V) ((V * 1024) / (*refVoltage))
//Volts = (ADC * Volts) / ADC
#define ADC2VOLT(ADCValue) ((ADCValue * (*refVoltage)) / 1024)

//Volts = Volts * PURE NUMBER
#define ACTUAL_VOLTAGE(V) (V * (*voltageFactor))

//Ampere = (Volts * PURE NUMBER) / Ohm
#define ACTUAL_AMPERE(V) ((V * (*shuntFactor)) / (*shuntResistor))

//Volts = Volts / PURE NUMBER
#define VIRTUAL_VOLTAGE(V) (V / (*voltageFactor))
//Volts = (Ampere * Ohm) / PURE NUMBER
#define VIRTUAL_AMPERE(A) ((A * (*shuntResistor)) / (*shuntFactor))

volatile uint16_t setVoltageADC;
volatile uint16_t setShuntADC;

volatile uint16_t *voltageOutputADC = &adcVector[0];
volatile uint16_t *shuntOutputADC = &adcVector[1];

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
	
	//INVERTED LOGIC
	if(isEEPROMLoaded)
	{
		float valuesVectorTmp[VALUES_VECTOR_LENGTH] = {DEFAULT_VOLTAGE, DEFAULT_AMPERE, DEFAULT_REF_VOLTAGE, DEFAULT_VOLTAGE_FACTOR, DEFAULT_SHUNT_FACTOR};
		
		eeprom_write_block(valuesVectorTmp, valuesVectorEEMEM, VALUES_VECTOR_LENGTH * sizeof(float));
		eeprom_write_byte(&isEEPROMLoadedEEMEM, FALSE /*TRUE*/);
	}
}

void loadEEPROM()
{
	//Ignore volatile warning?
	//eeprom_read_block(&voltageVector, &voltageVectorEEPROM, VOLTAGE_VECTOR_LENGTH * sizeof(float));

	//Tmp values
	uint8_t i;
	float valuesVectorTmp[VALUES_VECTOR_LENGTH];

	eeprom_read_block(valuesVectorTmp, valuesVectorEEMEM, VALUES_VECTOR_LENGTH * sizeof(float));

	for(i=0;i<VALUES_VECTOR_LENGTH;i++) valuesVector[i] = valuesVectorTmp[i];
}


void saveEEPROM()
{
	//Tmp values
	uint8_t i;
	float valuesVectorTmp[VALUES_VECTOR_LENGTH];
	for(i=0;i<VALUES_VECTOR_LENGTH;i++)  valuesVectorTmp[i] = valuesVector[i];
	
	eeprom_write_block(valuesVectorTmp, valuesVectorEEMEM, VALUES_VECTOR_LENGTH * sizeof(float));
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

//PWM Handler:
//It controls that output parameters match settings parameters, changing PWM duty cycle.
//Ampere setting is more important than voltage setting.
//TODO: Implement a slow start algorithm.
void PWMHandler()
{
	//Convert settings value into an ADC value.
	setVoltageADC = VOLT2ADC(VIRTUAL_VOLTAGE(*voltage));
	setShuntADC = VOLT2ADC(VIRTUAL_AMPERE(*ampere));

	//Output ampere is more than set ampere
	//Need to reduce PWM Duty cycle.
	if(*shuntOutputADC > setShuntADC)
	{
		if(OCR1A > 0x0000)
		{
			OCR1A = OCR1A - 1;
			OCR1B = OCR1A;
			return;
		}
	}
	
	//Output voltage is more than set voltage
	//Need to reduce PWM Duty cycle.
	if(*voltageOutputADC > setVoltageADC)
	{
		if(OCR1A > 0x0000)
		{
			OCR1A = OCR1A - 1;
			OCR1B = OCR1A;
			return;
		}
	}
	
	//Output voltage is less than set voltage
	//Need to increase PWM Duty cycle.
	if(*voltageOutputADC < setVoltageADC)
	{
		//Mask is 0x01FF (9 bit) because Timer 1 is limited to 512 counts in this Fast-PWM mode.
		if(OCR1A < 0x01FF)
		{
			OCR1A = OCR1A + 1;
			OCR1B = OCR1A;
			return;
		}
	}
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
			//--SELECT PSU CASE--
			case SELECT_PSU:
			
			//Convert float to string.
			if(state == STATE_DISPLAY)
			{
				dtostrf(ACTUAL_VOLTAGE(ADC2VOLT(*voltageOutputADC)), 5, 3, voltageString);
			}
			else
			{
				dtostrf(*voltage, 5, 3, voltageString);
			}
			
			printPSULine(psuState, 0);
			printVoltageLine(voltageString, 1);
			break;
			
			//--SELECT VOLTAGE CASE--
			case SELECT_VOLTAGE:
			
			//Convert float to string.
			if(state == STATE_DISPLAY)
			{
				dtostrf(ACTUAL_VOLTAGE(ADC2VOLT(*voltageOutputADC)), 5, 3, voltageString);
				dtostrf(ACTUAL_AMPERE(ADC2VOLT(*shuntOutputADC)), 5, 3, ampereString);
			}
			else
			{
				dtostrf(*voltage, 5, 3, voltageString);
				dtostrf(*ampere, 5, 3, ampereString);
			}
			
			
			printVoltageLine(voltageString, 0);
			printAmpereLine(ampereString, 1);
			break;
			
			//--SELECT AMPERE CASE--
			case SELECT_AMPERE:
			
			//Convert float to string.
			if(state == STATE_DISPLAY)
			{
				dtostrf(ACTUAL_AMPERE(ADC2VOLT(*shuntOutputADC)), 5, 3, ampereString);
			}
			else
			{
				dtostrf(*ampere, 5, 3, ampereString);
			}
			
			dtostrf(*refVoltage, 5, 3, refVoltageString);
			
			printAmpereLine(ampereString, 0);
			printVRefLine(refVoltageString, 1);
			break;
			
			//--SELECT VOTLAGE REFERENCE CASE--
			case SELECT_VOLTAGE_REF:
			
			//Convert float to string.
			dtostrf(*refVoltage, 5, 3, refVoltageString);
			dtostrf(*voltageFactor, 5, 3, voltageFactorString);
			
			printVRefLine(refVoltageString, 0);
			printVoltageFactorLine(voltageFactorString, 1);
			break;
			
			//--SELECT VOTLAGE FACTOR CASE--
			case SELECT_VOLTAGE_FACTOR:
			
			//Convert float to string.
			dtostrf(*voltageFactor, 5, 3, voltageFactorString);
			dtostrf(*shuntFactor, 5, 3, shuntFactorString);
			
			printVoltageFactorLine(voltageFactorString, 0);
			printShuntFactorLine(shuntFactorString, 1);
			break;
			
			//--SELECT SHUNT FACTOR CASE--
			case SELECT_SHUNT_FACTOR:
			
			//Convert float to string.
			dtostrf(*shuntFactor, 5, 3, shuntFactorString);
			dtostrf(*shuntResistor, 5, 3, shuntResistorString);

			printShuntFactorLine(shuntFactorString, 0);
			printShuntResistorLine(shuntResistorString, 1);
			break;
			
			//--SELECT SHUNT RESISTOR CASE--
			case SELECT_SHUNT_RES:
			
			//Convert float to string.
			dtostrf(*shuntResistor, 5, 3, shuntResistorString);

			printShuntResistorLine(shuntResistorString, 0);
			printPSULine(psuState, 1);
			break;
		}

		printFinalChar(state == STATE_DISPLAY ? 0 : unitState + 1);

		//_delay_ms(16);
	}
}

//TIMER 0 OVERFLOW INTERRUPT
//Read ADC value and start next conversion.
//Adjust PWM Duty cycle for buck converter.
//Adjust PWM Duty cycle for boost converter.
ISR(TIMER0_OVF_vect)
{
	ADCHandler();
	PWMHandler();
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
				valuesVector[state-2] += factor;
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
				valuesVector[state-2] -= factor;
				debouncingCounter = 0;
			}
		}

		//Check ratings
		if(*voltage < MIN_VOLTAGE) *voltage = MIN_VOLTAGE;
		if(*voltage > MAX_VOLTAGE) *voltage = MAX_VOLTAGE;

		if(*ampere < MIN_AMPERE) *ampere = MIN_AMPERE;
		if(*ampere > MAX_AMPERE) *ampere = MAX_AMPERE;
		
		//TODO: Add factors ratings???
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
				displayState = displayState < DISPLAY_STATE_LENGTH - 1 ? displayState + 1 : DISPLAY_STATE_LENGTH -1;
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
				displayState = displayState == 0 ? 0 : displayState -1;
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

