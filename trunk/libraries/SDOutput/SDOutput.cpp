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
#include <SDOutput.h>


bool SDOutput::active;
File SDOutput::_File;


bool SDOutput::flush(){
    if (active){
        _File.flush();
		return true;
    }
	return false;
}

bool SDOutput::begin()
{
  
	active=false;

    char* fname;
    // Counter for filename
    byte i=0;


    pinMode(SSPIN, OUTPUT);
    if (!SD.begin(SSPIN)){
        // No SD Card, reader, or other fault, bail out.
        return false;
    }
    // Start at zero, and increase the filenumber until 
    // the filename doesnt conflict with anything on the 
    // card already.
    sprintf(fname, "%08d.log");
    
    while (SD.exists(fname)){
        i++;
        // free fname?
        sprintf(fname, "%08d.log");
    }
    
    if (SD.exists(fname)){
        // Run out of filenames
        return false;
    }

    // Try to open the actual file
    _File=SD.open(fname,FILE_WRITE);
    if (!_File){
        return false;
    }

    active=true;
    return active;

}


bool SDOutput::log(Message msg){
    if (!active){
        return false;
    }
    _File.println(msg.toCSV());
	return true;
}



