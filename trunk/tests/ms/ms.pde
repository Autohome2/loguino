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


void setup(){
	byte table[39];
	MegaSquirtData d;
	
	// Clock ticks
	table[0]=144;

/*
 *  128 - bank 1 squirting 
 *  64  - bank 2 quirting
 *  32  - bank 1 scheduled to squirt
 *  16  - bank 1 squirting
 *  8   - bank 2 schedule to squirt
 *  4   - bank 2 squirting
 *  2   - boost controll off
 *  1   - Not Used
 */
	table[1]=128+64+2;

/*
 *  128 - Engine Running 
 *  64  - Engine cranking
 *  32  - ASE Active
 *  16  - Warmup Active
 *  8   - TPS accelleration active
 *  4   - Deceleration mode active
 *  2   - MAP Acceleration active
 *  1   - IDLE active
 */

	table[2]=128+32+16;

	table[3]=12;
	table[4]=10;
	table[5]=32;
	table[6]=58;
	table[7]=2;
	table[8]=12;
	table[9]=5;
	table[10]=0;
	table[11]=1;
	table[12]=13;
	table[13]=10;
	table[14]=13;
	table[15]=0;
	table[16]=5;
	table[17]=0;
	table[18]=2;
	table[19]=13;
	table[20]=0;
	table[21]=9;
	table[22]=255;
	table[23]=255;
	table[24]=12;
	table[25]=13.7;
	table[26]=12.9;
	table[27]=0;
	table[28]=1;
	table[29]=0;
	table[30]=0;

/*
 *  128 - Fuel Pump on
 *  64  - Fidle/Spark on 
 *  32  - Output 2 On    
 *  16  - Output 1/Boost Cont on
 *  86  - NOS On
 *  4   - FAN on
 *  2   - Flyback
 *  1   - Flyback
 */
	table[31]=128+64;
	table[32]=PORTB_INPUT_MAP;
	table[33]=PORTC_OUTPUT_COILA;
	table[34]=PORTD_OUTPUT_COILD;
	table[35]=0;
	table[36]=1;
	table[37]=255;
	table[38]=0;

	d.populate(table);
	Serial.begin(9600);

	char line[]="g";

	Serial.println("Throttle Position:");
	sprintf(line, " Current Throttle Position: %d",d.tpsADC);
	Serial.println(line);
	sprintf(line, " Previous Throttle Position: %d",d.TPSLast);
	Serial.println(line);

	Serial.println("Air Pressures/Temperatures");
	sprintf(line, " Barometric Pressure: %d", d.baroADC);
	Serial.println(line);
	sprintf(line, " Manifold Pressure: %d", d.mapADC);
	Serial.println(line);
	sprintf(line, " Manifold Air Temperature: %d", d.matADC);
	Serial.println(line);
	
	Serial.println("Engine Temperature:");
	sprintf(line, " Coolant Temperature: %d", d.cltADC);
	Serial.println(line);

	Serial.println("Corrections:");
	sprintf(line, " EGO Correction: %d", d.egoCorrection);
	Serial.println(line);
	sprintf(line, " EGO Correction 2: %d", d.egoCorrection2);
	Serial.println(line);
	sprintf(line, " Atmospheric correction: %d", d.airCorrection);
	Serial.println(line);
	sprintf(line, " Barometric Correction: %d", d.baroCorrection);
	Serial.println(line);
	sprintf(line, " Warmup Enrichment: %d", d.warmupEnrich);
	Serial.println(line);
	sprintf(line, " Acceleration Enrichment: %d", d.accelEnrich);
	Serial.println(line);
	
	Serial.println("Exhaust Gas Output");
	sprintf(line, " AFR Target: %d", d.afrTarget);
	Serial.println(line);
	sprintf(line, " Current EGO Value: %d", d.egoADC);
	Serial.println(line);

	Serial.println("Engine:");
	sprintf(line, " RPM: %d", d.rpm100);
	Serial.println(line);
	sprintf(line, " Ignition Advance: %d", d.advance);
	Serial.println(line);
	sprintf(line, " Knock Reduction: %d", d.KnockAngle);
	Serial.println(line);
	sprintf(line, " CLT Ignition modification: %d", d.CltIatAngle);
	Serial.println(line);

	Serial.println("Flags: ");

	Serial.print(" Running: ");
	Serial.println(d.running ? "Yes" : "No");

	Serial.print(" Cranking: ");
	Serial.println(d.cranking ? "Yes" : "No");
	
	Serial.print(" After Start Enrichment: ");
	Serial.println(d.ase ? "Yes" : "No");

	Serial.print(" Warmup: ");
	Serial.println(d.warmup ? "Yes" : "No");
	
	Serial.print(" TPS Acceleration: ");
	Serial.println(d.tpsaccel ? "Yes" : "No");
	Serial.print(" Deceleration: ");
	Serial.println(d.decel ? "Yes" : "No");
	Serial.print(" MAP Acceleration: ");
	Serial.println(d.mapaccel ? "Yes" : "No");
	Serial.print(" Idle: ");
	Serial.println(d.idle ? "Yes" : "No");
}


void loop(){}

