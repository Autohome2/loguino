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


byte Logger::flushCount;

//! Sets up the logger, by calling begin on all attached loggers.
void Logger::begin()
{
	//debug(INFO, "Logger::begin: Setting up Logger");
	flushCount=0;

//	debug(INFO, "Logger::begin: Setting up SerialOutput");
	SerialOutput::begin();


//	debug(INFO, "Logger::begin: Setting up SD Output");
	SDOutput::begin();
}


//! Sends the supplied message object to each logger attached.
//! @param msg - The Message object to log.
void Logger::log(Message &msg)
{
	delay(10);
//	debug(INFO, "Logger::log: Logging message");
	if (flushCount++ > LOGGER_FLUSH_MAX){
		debug(INFO, "Logger::log: Flush count reached, flushing SerialOutput");
		SerialOutput::flush();
		debug(INFO, "Logger::log: Flush count reached, flushing SDOutput");
		SDOutput::flush();

		flushCount=0;
	}


//	debug(INFO, "Logger::log: Sending message to serial output");	
	SerialOutput::log(msg);
//	debug(INFO, "Logger::log: Sending message to SD output");	
	SDOutput::log(msg);

}

