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
#include "Debug.h"
#include "NMEA.h"



void setup(){
	Serial.begin(9600);

	String message;
	
	message="$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62\r\n";
	
	Serial.println("Trying valid sentance.");

	NMEA n;
	int i=0;
	for (i=0;i<message.length();i++){
		if(n.addChar(message.charAt(i))){
		Serial.println("Message validates.");
		}
	}

	if (n.validFix()){
		Serial.println("PASS: validFix()");
	}else{
		Serial.println("FAIL: validFix()");
	}

	Serial.print( n.getTime()=="081836" ? "PASS: " : "FAIL: ");
	Serial.println("getTime()");

	Serial.print( n.getLatitude()=="3751.65S" ? "PASS: " : "FAIL: ");
	Serial.println("getLatitude()");

	Serial.print( n.getLongitude()=="14507.36E" ? "PASS: " : "FAIL: ");
	Serial.println("getLongitude()");

	Serial.print( n.getSpeed()=="000.0" ? "PASS: " : "FAIL: ");
	Serial.println("getSpeed()");

	Serial.print( n.getCourse()=="360.0" ? "PASS: " : "FAIL: ");
	Serial.println("getCourse()");

	Serial.print( n.getDate()=="130998" ? "PASS: " : "FAIL: ");
	Serial.println("getDate()");
}




void loop(){}
