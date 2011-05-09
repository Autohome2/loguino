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

#include "WProgram.h"
#include "message.h"
#include "MegaSquirt.h"
#include "MSPoller.h"

#define SERIALPORT Serial2
#define RETRYTIME 20

bool MSPoller::begin(){
	SERIALPORT.begin(9600);
	active=available();
	}


bool MSPoller::available(){
	int i=0;
	SERIALPORT.flush();
	SERIALPORT.print("C");
	while(SERIALPORT.available() <1 and ++i<100){
		delay(10);
	}
	if (SERIALPORT.available()<1){
		SERIALPORT.flush();
		return true;
	}else{
		return false;
	}
}
	
bool MSPoller::poll(bool (*ptrCallback)(Message)  ){
	int i=0;
	byte table[39];
	MegaSquirtData ms;
	Message m;

	if (!active){
		if (++wait>RETRYTIME){
			wait=0;
			active=available();
		}
		return true;
	}
	SERIALPORT.flush();
	SERIALPORT.print("R");
	while (SERIALPORT.available() <39 and ++i<100){
		delay(10);
	}
	if (SERIALPORT.available()<39){
		active=0;
		return false;
	}
	
	for (i=0;i<39;i++){
		table[i]=SERIALPORT.read();	
	}

	SERIALPORT.flush();
	ms.populate(table);
	
	m.time=millis();

	m.units="Ticks";
	m.nameSpace="MegaSquirt.ClockPulse";
	m.value=ms.clock;
	ptrCallback(m);

	m.units="Boolean";
	m.nameSpace="MegaSquirt.EngineRunning";
	m.value=ms.running ? "True" : "Fales";
	ptrCallback(m);

	m.units="Boolean";
	m.nameSpace="MegaSquirt.Cranking";
	m.value=ms.cranking ? "True" : "Fales";
	ptrCallback(m);

	m.units="Boolean";
	m.nameSpace="MegaSquirt.AfterStartEnrichment";
	m.value=ms.ase ? "True" : "Fales";
	ptrCallback(m);

	m.units="Boolean";
	m.nameSpace="MegaSquirt.TPSAccellerationActive";
	m.value=ms.tpsaccel ? "True" : "Fales";
	ptrCallback(m);

	m.units="Boolean";
	m.nameSpace="MegaSquirt.Decelerating";
	m.value=ms.decel ? "True" : "Fales";
	ptrCallback(m);

	m.units="Boolean";
	m.nameSpace="MegaSquirt.MAPAccellerationActive";
	m.value=ms.mapaccel ? "True" : "Fales";
	ptrCallback(m);

	m.units="Boolean";
	m.nameSpace="MegaSquirt.Idle";
	m.value = ms.idle ? "True" : "False";
	ptrCallback(m);

	m.units="ADC Value";
	m.nameSpace="MegaSquirt.BarometricADC";
	m.value=ms.baroADC;
	ptrCallback(m);

	m.units="ADC Value";
	m.nameSpace="MegaSquirt.MapADC";
	m.value=ms.mapADC;
	ptrCallback(m);

	m.units="ADC Value";
	m.nameSpace="MegaSquirt.MatADC";
	m.value=ms.matADC;
	ptrCallback(m);

	m.units="ADC Value";
	m.nameSpace="MegaSquirt.CoolantADC";
	m.value=ms.cltADC;
	ptrCallback(m);

	m.units="ADC Value";
	m.nameSpace="MegaSquirt.tpsADC";
	m.value=ms.tpsADC;
	ptrCallback(m);

	m.units="ADC Value";
	m.nameSpace="MegaSquirt.BatADC";
	m.value=ms.batADC;
	ptrCallback(m);

	m.units="ADC Value";
	m.nameSpace="MegaSquirt.egoADC";
	m.value=ms.egoADC;
	ptrCallback(m);


	m.units="%";
	m.nameSpace="MegaSquirt.egoCorrection";
	m.value=ms.egoCorrection;
	ptrCallback(m);

	m.units="%";
	m.nameSpace="MegaSquirt.airCorrection";
	m.value=ms.airCorrection;
	ptrCallback(m);

	m.units="%";
	m.nameSpace="MegaSquirt.warmupEnrich";
	m.value=ms.warmupEnrich;
	ptrCallback(m);

	m.units="RPMS";
	m.nameSpace="MegaSquirt.rpm";
	m.value=ms.rpm100*100;
	ptrCallback(m);

	m.units="ms";
	m.nameSpace="MegaSquirt.pulseWidth1";
	m.value=ms.pulsewidth1;
	ptrCallback(m);

	m.units="ms";
	m.nameSpace="MegaSquirt.accelEnrich";
	m.value=ms.accelEnrich;
	ptrCallback(m);

	m.units="%";
	m.nameSpace="MegaSquirt.baroCorrection";
	m.value=ms.baroCorrection;
	ptrCallback(m);

	m.units="%";
	m.nameSpace="MegaSquirt.gammaEnrich";
	m.value=ms.gammaEnrich;
	ptrCallback(m);

	m.units="%";
	m.nameSpace="MegaSquirt.veCurr1";
	m.value=ms.veCurr1;
	ptrCallback(m);

	m.units="ms";
	m.nameSpace="MegaSquirt.pulsewidth2";
	m.value=ms.pulsewidth2;
	ptrCallback(m);

	m.units="%";
	m.nameSpace="MegaSquirt.veCurr2";
	m.value=ms.veCurr2;
	ptrCallback(m);
	
	m.units="%";
	m.nameSpace="MegaSquirt.idleDC";
	m.value=ms.idleDC;
	ptrCallback(m);
	
	m.units="Degrees";
	m.nameSpace="MegaSquirt.advance";
	m.value=ms.advance;
	ptrCallback(m);

	m.units="ADC Value";
	m.nameSpace="MegaSquirt.afrTarget";
	m.value=ms.afrTarget;
	ptrCallback(m);

	m.units="ADC Value";
	m.nameSpace="MegaSquirt.fuelADC";
	m.value=ms.fuelADC;
	ptrCallback(m);

	m.units="ADC Value";
	m.nameSpace="MegaSquirt.egtADC";
	m.value=ms.egtADC;
	ptrCallback(m);

	m.units="Degrees";
	m.nameSpace="MegaSquirt.CltIatAngle";
	m.value=ms.CltIatAngle;
	ptrCallback(m);

	m.units="Degrees";
	m.nameSpace="MegaSquirt.KnockAngle";
	m.value=ms.KnockAngle;
	ptrCallback(m);

	m.units="%";
	m.nameSpace="MegaSquirt.egoCorrection2";
	m.value=ms.egoCorrection2;
	ptrCallback(m);
	
	m.units="ADC Value";
	m.nameSpace="MegaSquirt.TPSLast";
	m.value=ms.TPSLast;
	ptrCallback(m);
}

MSPoller::MSPoller()
{
	wait=0;
	active=false;
}
