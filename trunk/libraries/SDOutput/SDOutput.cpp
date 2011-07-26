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

#include <config.h>
#include "WProgram.h"
#include <SDOutput.h>


#ifdef ENABLE_SD_OUTPUT
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
  
	debug (INFO, "SDOutput::begin - Setting up");
	active=false;
	
	pinMode(10, OUTPUT);
	if (!SD.begin(4)) {
		debug(ERROR, "SDOutput::begin - SD Card initialization failed!");
		return false;
	}

	debug(INFO, "SDOutput::begin - SD Card initialization complete.");

    char fname[13];
    // Counter for filename
    byte i=0;

    // Start at zero, and increase the filenumber until 
    // the filename doesnt conflict with anything on the 
    // card already.
    sprintf(fname, "%08d.log",i);
    
    while (SD.exists(fname)){

        i++;
		debug(INFO, "SDOutput::begin - Trying filename: " + String(fname));

        // free fname?
        sprintf(fname, "%08d.log",i);
    }
    
    if (SD.exists(fname)){
        // Run out of filenames
		debug(ERROR, "SDOutput::begin - No Spare Filenames");
        return false;
    }

    // Try to open the actual file
	debug(INFO, "SDOutput::begin - Opening File: "+String(fname));
    _File=SD.open(fname,O_WRITE|O_CREAT);
    if (!_File){
		debug(ERROR, "SDOutput::begin - Not able to open File.");
        return false;
    }

    active=true;
	pinMode(SD_ACTIVE_PIN, OUTPUT);
	digitalWrite(SD_ACTIVE_PIN, active);
    return active;

}


bool SDOutput::log(Message &msg){
    if (!active){
        return false;
    }
    _File.println(msg.toCSV());
	return true;
}



#endif
