
#ifndef MegaSquirt_h
#define MegaSquirt_h

#include "WProgram.h"

class MegaSquirtData
{
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

	public:
		int populate(byte table[39]);
	};
		

#endif

