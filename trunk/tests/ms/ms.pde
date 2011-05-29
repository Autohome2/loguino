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

#include <MegaSquirt.h>
#define MAX_TRIES 100
#define DELAY 1
#define MS_BLOCK_SIZE 112

void setup(){
	byte table[112];
	MegaSquirtData d;
	Serial.begin(9600);
	Serial1.begin(115200);
	Serial1.flush();
	Serial1.write("a");
	Serial1.write(byte(0));
	Serial1.write(byte(6));
	int tries=0;
	while( (tries++ <= MAX_TRIES ) && ( Serial1.available() < MS_BLOCK_SIZE ) ){
		delay(DELAY);
	}
	if (Serial1.available() < MS_BLOCK_SIZE){
		Serial.println("Error: Cannot read data from Megasquirt");
	}
	else
	{
		Serial.println("Read 112 bytes from megasquirt");
		int i=0;
		for (i=0;i<=MS_BLOCK_SIZE;i++){
			table[i]=Serial1.read();
			Serial.print("Value ");
			Serial.print(i, DEC);
			Serial.print(": ");
			Serial.println(table[i],DEC);
		}
		Serial.print("Read :");
		Serial.println(i, DEC);

		d.loadData(table);


		Serial.print("Firing 1: ");
		Serial.println(d.firing1() ? "Yes" : "No");

		Serial.print("Firing 2: ");
		Serial.println(d.firing2() ? "Yes" : "No");

		Serial.print("Scheduling 1: ");
		Serial.println(d.sched1() ? "Yes" : "No");

		Serial.print("Scheduling 2: ");
		Serial.println(d.sched2() ? "Yes" : "No");

		Serial.print("Injecting 1: ");
		Serial.println(d.inj1() ? "Yes" : "No");

		Serial.print("Injecting 2: ");
		Serial.println(d.inj2() ? "Yes" : "No");

		Serial.print("Ready: ");
		Serial.println(d.ready() ? "Yes" : "No");

		Serial.print("Cranking: ");
		Serial.println(d.crank() ? "Yes" : "No");
		
		Serial.print("After Start Enrichment: ");
		Serial.println(d.startw() ? "Yes" : "No");

		Serial.print("Warmup Enrichment: ");
		Serial.println(d.warmup() ? "Yes" : "No");

		Serial.print("TPS Accelleration Enrichment: ");
		Serial.println(d.tpsaen() ? "Yes" : "No");
		
		Serial.print("MAP Accelleration Enrichment: ");
		Serial.println(d.mapaen() ? "Yes" : "No");

		Serial.print("AFR Target 1: ");
		Serial.println(d.afrtgt1(), DEC);

		Serial.print("AFR Target 2: ");
		Serial.println(d.afrtgt2(), DEC);

		Serial.print("Wideband O2 1: ");
		Serial.println(d.wbo2_en1(), DEC);

		Serial.print("Wideband O2 2: ");
		Serial.println(d.wbo2_en2(), DEC);
		
		Serial.print("Barometric Air Pressure: ");
		Serial.println( (d.barometer() /10), DEC);

		Serial.print("Manifold Air Pressure: ");
		Serial.println( ( d.map() /10), DEC);

		Serial.print("Manifold Air Temperature: ");
		Serial.println( (d.mat()/10),DEC);

		Serial.print("Coolant Temperature: ");
		Serial.println( d.coolant()/10,DEC);

		Serial.print("Throttle Position: ");
		Serial.println( d.tps()/10, DEC);

		Serial.print("Battery Voltage: ");
		Serial.println( d.batteryVoltage()/10, DEC);

		Serial.print("AFR Sensor 1: ");
		Serial.print( d.afr1() ,DEC);
		
		Serial.print("AFR Sensor 2: ");
		Serial.print( d.afr2(), DEC );

		Serial.print("Knock Sensor: ");
		Serial.print (d.knock(), DEC);

		Serial.print("Lambda 1: ");
		Serial.print(d.lambda1(),DEC );

		Serial.print("Lambda 2: ");
		Serial.print(d.lambda2(), DEC);

		Serial.print("Correction for EGO 1: ");
		Serial.println(d.egoCorrection1(), DEC);

		Serial.print("Correction for EGO 2: ");
		Serial.println(d.egoCorrection2(), DEC);

		Serial.print("Correction for Air Temp: ");
		Serial.print(d.airCorrection(), DEC);

		Serial.print("Enrichment for warmup: ");
		Serial.println(d.warmupEnrich(), DEC);

		Serial.print("Enrichment for Acceleration: ");
		Serial.println(d.accelEnrich(), DEC);

		Serial.print("Derichment for Fuel Cut: ");
		Serial.print(d.tpsfuelcut(), DEC);

		Serial.print("Correction for air pressure: ");
		Serial.print (d.baroCorrection(), DEC);

		Serial.print("Enrichment for Gamma: ");
		Serial.print(d.gammaEnrich());

		Serial.print("VE Curr 1: ");
		Serial.println(d.veCurr1(), DEC);
	
		Serial.print("VE Curr 2: ");
		Serial.println(d.veCurr2(), DEC);
	
		Serial.print("Idle Air Controller: ");
		Serial.println(d.iacstep(), DEC);

		Serial.print("Warmup Advance: ");
		Serial.println(d.coldAdvDeg(), DEC);
				
		Serial.print("Throttle Position DOT: ");
		Serial.println(d.tpsDOT(), DEC);

		Serial.print("Manifold Air Pressure DOT: ");
		Serial.println(d.mapDOT());

		Serial.print("Dwell Angle: ");
		Serial.println(d.dwell());

		Serial.print("Mass Air Flow: ");
		Serial.println(d.maf(), DEC);

		Serial.print("Calculated MAP: ");
		Serial.println(d.calcMAP(),DEC);

		Serial.print("Fuel Correction: ");
		Serial.println(d.fuelCorrection());


		Serial.print("Port 0 Active: : ");
        Serial.println(d.port0() ? "Yes" : "No");

		Serial.print("Port 1 Active: : ");
        Serial.println(d.port1() ? "Yes" : "No");

		Serial.print("Port 2 Active: : ");
        Serial.println(d.port2() ? "Yes" : "No");

		Serial.print("Port 3 Active: : ");
        Serial.println(d.port3() ? "Yes" : "No");

		Serial.print("Port 4 Active: : ");
        Serial.println(d.port4() ? "Yes" : "No");

		Serial.print("Port 5 Active: : ");
        Serial.println(d.port5() ? "Yes" : "No");

		Serial.print("Port 6 Active: : ");
        Serial.println(d.port6() ? "Yes" : "No");

		Serial.print("Knock based ignition retard: ");
		Serial.println(d.knockRetard(), DEC);

		Serial.print("X-TAU fuel correction 1: ");
		Serial.println(d.xTauFuelCorr1(), DEC);

		Serial.print("X-TAU fuel correction 2 ");
		Serial.println(d.xTauFuelCorr2(), DEC);

		Serial.print("Exhaust Gas Oxygen Sensor 1: ");
		Serial.println(d.egoV1(), DEC);
		
		Serial.print("Exhaust Gas Oxygen Sensor 2: ");
		Serial.println(d.egoV2(), DEC);

		Serial.print("AMC Updates: ");
		Serial.println(d.amcUpdates(), DEC);

		Serial.print("KPAix: ");
		Serial.println(d.kpaix(), DEC);

		Serial.print("Trigger Fix: ");
		Serial.println(d.trig_fix(), DEC);
		
		Serial.print("Tach count: ");
		Serial.println(d.tachCount(), DEC);

		Serial.print("Checksum: ");
		Serial.println(d.cksum(), DEC);

		Serial.print("Time between trigger pulses: ");
		Serial.println(d.deltaT(), DEC);

		Serial.print("Uptime: ");
		Serial.println(d.seconds(), DEC);
	
		Serial.print("Pulse Width INJ 1: ");	
		Serial.println(d.pulseWidth1(), DEC);

		Serial.print("Pulse Width INJ 2: ");	
		Serial.println(d.pulseWidth2(), DEC);

		Serial.print("RPM: ");
		Serial.println(d.rpm(), DEC);

		Serial.print("Ignition Advance");
		Serial.println(d.advance());




	}

}


void loop(){}

