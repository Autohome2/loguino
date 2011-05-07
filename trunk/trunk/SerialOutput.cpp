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
#include "SerialOutput.h"
#include "message.h"

#define SERIAL Serial1


SerialOutput::SerialOutput(){
    active=false;
}

bool SerialOutput::flush(){
    if (active){
        SERIAL.flush();
    }
    return active;
}


bool SerialOutput::begin()
{
    SERIAL.begin(9600);
    active=true;
    return active;
}


bool SerialOutput::logMessage(Message msg){
    if (!active){
        return false;
    }
    SERIAL.println(msg.toCSV());
}


