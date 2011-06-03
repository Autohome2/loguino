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
 * $Rev$:   
 * $Author$: 
 * $Date$:  

*/


#include "WProgram.h"
#include "Logger.h"
#include <SerialOutput.h>



bool SerialOutput::active;

bool SerialOutput::flush(){
    if (active){
        SO_SERIAL_PORT.flush();
		return true;
    }
	return false;
}


bool SerialOutput::begin()
{
    SO_SERIAL_PORT.begin(SO_SERIAL_PORT_SPEED);
    active=true;
    return true;
}


bool SerialOutput::log(Message msg){
    if (!active){
        return false;
    }
    SO_SERIAL_PORT.println(msg.toCSV());
	return true;
}


