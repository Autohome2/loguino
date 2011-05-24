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
	Serial1.begin(115200);
	Serial1.flush();
	Serial1.print("a\x00\x06");
	
	int tries=0;
	while( (tries++ <= MAX_TRIES ) && ( Serial1.available() < MS_BLOCK_SIZE ) ){
		delay(DELAY);
	}
	if (Serial1.available() < MS_BLOCK_SIZE){
		Serial.println("Error: Cannot read data from Megasquirt");
	}
	else
	{
		int i=0;
		for (i=0;i<=MS_BLOCK_SIZE;i++){
			table[i]=Serial1.read();
		}
		d.loadData(table);
	}

}


void loop(){}

