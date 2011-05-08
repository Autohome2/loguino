/* Copyright 2011 David Irvine
 *
 * This file is part of Loguino
 *
 * Loguino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Loguino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with Loguino.  If not, see <http://www.gnu.org/licenses/>.
 *
 * $Rev$
 * $Author$
 * $Date$

*/

#ifndef MegaSquirt_h
#define MegaSquirt_h

#define PORTB_INPUT_MAP 128
#define PORTB_INPUT_MAT 64 
#define PORTB_INPUT_CLT 32 
#define PORTB_INPUT_TPS 16 
#define PORTB_INPUT_BAT 8  
#define PORTB_INPUT_EGO 4  
#define PORTB_INPUT_X7  2  
#define PORTB_INPUT_X6  1  

#define PORTC_OUTPUT_COILA 128
#define PORTC_OUTPUT_COILB 64 
#define PORTC_OUTPUT_FAN 32 
#define PORTC_OUTPUT_SHIFT 16 
#define PORTC_OUTPUT_LIGHT 8   

#define PORTD_OUTPUT_COILD 128
#define PORTD_OUTPUT_NOS 64 
#define PORTD_OUTPUT_NO_KNOCK 32 
#define PORTD_OUTPUT_LAUNCH 16 
#define PORTD_OUTPUT_INJ1 8   
#define PORTD_OUTPUT_INJ2 4   


#include "WProgram.h"

class MegaSquirtData
{
	public:
		// clock counter that continuously counts from 0 to 255
		 byte clock;
		
		// inj1 squirt
		bool inj1squirt;
		// inj2 squirt
		bool inj2squirt;
		// scheduled to squirt
		bool schedtosquirt1;
		// squirting
		bool squirting1;
		// injector 2 (sched2)
		bool schedtosquirt2;
		// squirting injector 2
		bool squirting2;
		// boost control Off
		bool boostctrl;
		
		// Engine current status running
		bool running;
		// Engine current status cranking
		bool cranking;
		// After Start Enrichment Active
		bool ase;
		// Warmup Enrichment Active
		bool warmup;
		// TPS based acceleratrion enrichment active
		bool tpsaccel;
		// Deceleration mode active
		bool decel;
		// MAP based acceleration enrichment active
		bool mapaccel;
		// Idle control active
		bool idle;
		
		// MAP value used for baro correction
		 byte baroADC;
		
		// Current MAP value
		 byte	mapADC;
		
		// Manifold air temperature
		 byte	matADC;
		
		// Coolant temperature
			 byte	cltADC;
	
		// Throttle position
		 byte	tpsADC;
		
		// Battery voltage
		 byte	batADC;
		
		// Exhaust gas oxygen sensor
		 byte	egoADC;
		
		// Exhaust gas correction
		 byte	egoCorrection;
		
		// Gair
		 byte	airCorrection;
		
		// Warmup enrichment
		 byte	warmupEnrich;
		
		// rpm divided by 100
		 byte	rpm100;
		
		// Pulse width, divide by 10 to get ms
		 byte	pulsewidth1;
		
		// 
		 byte	accelEnrich;
		
		// Barometer Lookup Correction
		 byte	baroCorrection;
		
		// Total Gamma Enrichments
		 byte	gammaEnrich;
		
		// Current VE value in use (table 1) 
		 byte	veCurr1;
		
		// Pulse width, divide by 10 to get ms
		 byte	pulsewidth2;
		
		// 
		 byte	veCurr2;
		
		// 
		 byte	idleDC;
		
		// The interval Time - i.e. time between decoder triggers. It is used for the "hi-res" rpm calculation.  ( ctimeCommH in msns-extra.h file) This is the middle (H) byte of a 3 byte value X:H:L
		
		 byte	iTimeM;
		// The interval Time - i.e. time between decoder triggers. It is used for the "hi-res" rpm calculation.	( ctimeCommH in msns-extra.h file) This is the lower byte (L) of a 3 byte value X:H:L
		
		 byte	iTimeL;
	
		// raw ignition advance value. Multiply by actual = raw*0.352-10
		 byte	advance;
		
		// Raw ADC target that MS is trying to reach from the target table or switch  point 255 = 5V
		 byte	afrTarget;
		
		// Raw ADC from X7 (second O2 or fuel pressure or VSS sensor)
		 byte	fuelADC;
		
		// Raw ADC from X6 If Exhaust gas temp. then temp in
		// ºF = egtADC*7.15625
		// ºC = egtADC*3.90625 if VSS Volts = egtADC*0.019
		 byte	egtADC;
	
		// Spark Angle added or removed for IAT CLT temp.
		// Angle = raw value*0.352 (Angle < 45 Angle : -90 + Angle)
		 byte	CltIatAngle;
		
		//  Spark Angle removed due to Knock System
		 byte	KnockAngle;
		
		// Same as egocorrection, but this is for second O2 sensor
		 byte	egoCorrection2;
		
		// TPS/MAP last value for MT Accel Wizard, so we have last and current  values to give a gauge of dot (delta)
		 byte TPSLast;
		
		// The interval Time - i.e. time between decoder triggers. It is used for the "hi-res" rpm calculation.  This is the highest byte (X) of a 3 byte value X:H:L
		 byte iTimeH;
	
		bool populate(byte table[39]);
	};
		

#endif

