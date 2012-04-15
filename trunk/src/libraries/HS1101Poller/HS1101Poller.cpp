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
 * along with Loguino.  If not, see "http://www.gnu.org/licenses/".
 * 
 * $Rev$:   
 * $Author$: 
 * $Date$:  

*/



#include <HS1101Poller.h>

#ifdef ENABLE_HS1101_POLLER
#define HS1101_RH_CONSTANT 12169
	
void HS1101Poller::begin(){
}
void HS1101Poller::poll()
{
	unsigned long startTime;
	unsigned long timeoutTime;
	unsigned long endTime;
	bool changed;
	changed=false;

	startTime=micros();
	timeoutTime=startTime+100000;
	
	pinMode(HS1101_PIN, OUTPUT); 
	digitalWrite(HS1101_PIN, HIGH);
	delay(1);   
	pinMode(HS1101_PIN, INPUT); 
	digitalWrite(HS1101_PIN, LOW);
	while(!changed && micros()<timeoutTime){
		changed=!digitalRead(HS1101_PIN);
		if (changed){
			endTime=micros()-startTime;
			endTime=endTime/2;
			endTime=endTime*10;
			endTime=endTime-HS1101_RH_CONSTANT;
			endTime=(endTime)/24;
			m.value=String(endTime);
			m.units="%";
			m.nameSpace="HS1101.Humidity";
			log_message();
		}
	}


}

#endif

