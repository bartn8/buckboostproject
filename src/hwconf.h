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
* hwconf.h
*
* Created: 04/03/2019 23:24:09
*  Author: Luca Bartolomei
*/

#ifndef HWCONF_H_
#define HWCONF_H_

#include <avr/io.h>

//PSU RATINGS

#define MIN_BUCK_VOLTAGE 0.0
#define MAX_BUCK_VOLTAGE 12.0

#define MIN_BOOST_VOLTAGE 12.0
#define MAX_BOOST_VOLTAGE 30.0

//DEFAULT VOLTAGE AND FACTORS

#define DEFAULT_BOOST_VOLTAGE 15.0f
#define DEFAULT_BUCK_VOLTAGE 5.0f
#define DEFAULT_REF_VOLTAGE 5.0f
#define DEFAULT_BOOST_FACTOR 7.5456f
#define DEFAULT_BUCK_FACTOR 2.9979f

//ENCODER SETTINGS
#define ENCODER_STEPS 4

//OPTIMIZATIONS:
//Disabled ADC and TIMER 1 Interrupt.

//HARDWARE CALIBRATION
//Vref calibration:
// Use a voltmeter between AVCC and GND to calibrate Vref value.
// ADC need signals with an output impedance <= 10K Ohm.
// Higher values may slow down conversions.

//Boost Factor calibration
//Measure with a multimeter R7 and R8.
//α = R8/(R8+R7)
//Boost Factor = 1/α

//Buck Factor calibration
//Measure with a multimeter R4 and R3.
//α = R4/(R4+R3)
//Buck Factor = 1/α

//HARDWARE CONFIGURATION.
//SET FOR ARDUINO PRO MINI (ATMEGA168P)
//F_CPU = 16000000L

//PSU STARTER:
//PSU:		PD2				(Pro Mini: D2)

//ENCODER:
//Need this order for CLK and DT.
//CLK:		PD5 (PCINT21)	(Pro Mini: D5)
//DT:		PD4 (PCINT20)	(Pro Mini: D4)
//SW:		PD3 (INT1)		(Pro Mini: D3)

//ADC:
//BOOST:	ADC0			(Pro Mini: A0)
//BUCK:		ADC1			(Pro Mini: A1)

//PWM:
//Need this order to better components placement.
//BOOST:	PB2 (OC1B)		(Pro Mini: D10)
//BUCK:		PB1 (OC1A)		(Pro Mini: D9)

//Things for Push-Button
#define READ_BTN_PIN ((PIND >> PD3) & 0x01)

//Things for PSU stuff
#define INVERT_PSU_STATE PORTD^=_BV(PD2)

//Things for PWM signals.
#define ENABLE_PWM_SIGNALS DDRB|=(_BV(PB1) | _BV(PB2))
#define DISABLE_PWM_SIGNALS DDRB&=~(_BV(PB1) | _BV(PB2))

//Things for LED.
#define INVERT_LED PORTB^=_BV(PB5)


/************************************************************************/
/* Input Output Register configuration                                  */
/* Here is enabled pull-up for some pins                                */
/************************************************************************/
inline void setIO()
{
	//Set DDR for LED.
	DDRB = _BV(PB5);
	
	//Set DDR for analog read and rotary encoder
	DDRC = 0x00;
	DDRD = _BV(PD2);							//PSU is an output pin.

	//Enable internal pull-up
	PORTD = _BV(PD3) | _BV(PD4) | _BV(PD5);		//Button Pin & Rotary Pins
}

/************************************************************************/
/* Encoder use two pin change and a digital pin.                        */
/* Pin change are used for decoding rotation signals                    */
/* Digital pin is used with a timer polling to decode button            */
/************************************************************************/
inline void setEncoder()
{
	//Rotary Encoder Settings

	//Need two PCINT for rotary encoder.
	//Use PD5 - PD4 (PCINT21 - PCINT20)

	//Enable PCINT2 Interrupt
	PCICR = _BV(PCIE2);
	//PCINT21 - PCINT20 Trigger
	PCMSK2 = _BV(PCINT21) | _BV(PCINT20);

	//Ex method for button.
	//INT1 Interrupt on Falling Edge
	//EICRA = _BV(ISC11);
	//EIMSK = _BV(INT1);
}

/************************************************************************/
/* ADC is used for read feedback voltages                               */
/* Resolution 10 bit, 5mV (5V AVCC)                                     */
/* Time for a conversion 116 uS                                         */
/************************************************************************/
inline void setADC()
{
	//ADC Settings

	//Reference AVCC, MUX = ADC0
	ADMUX = _BV(REFS0) /*| _BV(MUX2) | _BV(MUX1)*/;

	//Enable ADC and interrupt
	//Prescaler 128 ~ 125Khz
	ADCSRA = _BV(ADEN) /*| _BV(ADIE)*/ | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

	//We start a fist conversion here.
	ADCSRA |= _BV(ADSC);
	while(ADCSRA & _BV(ADSC));
}

/************************************************************************/
/* Timer 1 is used for PWM signals.                                     */
/* PWM Frequency: 31250 Hz                                              */
/* OC1A: Boost, OC1B: Buck                                              */
/************************************************************************/
inline void setPWMTimer()
{
	//Timer 1 Settings

	//Reset timer 1 counter.
	TCNT1 = 0x0000;

	//Shutdown PWM signal
	OCR1A = 0x00;
	OCR1B = 0x00;

	//Fast-PWM 9 bit Port A And Port B NON-INVERTING MODE.
	//Prescaler 1, PWM freq: 31250 Hz
	TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
	TCCR1B = _BV(WGM12) | _BV(CS10);

	//Enable Timer 1 Interrupt to update PWM value.
	//TIMSK1 = /*_BV(OCIE1B) | _BV(OCIE1A)*/ _BV(TOIE1);
}

/************************************************************************/
/* Timer 0 is used to check PWM adjust                                  */
/* Check Frequency: 976.5 Hz                                            */
/************************************************************************/
inline void setCheckTimer()
{
	TCNT0 = 0x00;
	
	//Normal mode, no PWM pin used.
	TCCR0A = 0x00;
	//Prescaler 256 ~ 976.5 Hz
	TCCR0B = _BV(CS01) | _BV(CS00);
	
	//Enable Timer Overflow.
	TIMSK0 = _BV(TOIE0);
}

/************************************************************************/
/* Timer 2 is used for polling button pins                              */
/* Polling Frequency: 61 Hz                                             */
/************************************************************************/
inline void setPollingTimer()
{
	TCNT2 = 0x00;

	//Normal mode, no PWM pin used.
	TCCR2A = 0x00;

	//Prescaler 1024 ~ 15.625 Hz
	TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);

	//Enable Timer Overflow.
	TIMSK2 = _BV(TOIE2);
}

#endif /* HWCONF_H_ */
