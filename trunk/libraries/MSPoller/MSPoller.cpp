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
	

	


}

MSPoller::MSPoller()
{
	active=false;
}
