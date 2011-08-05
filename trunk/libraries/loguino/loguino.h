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
 * $Rev$
 * $Author$
 * $Date$

*/
#ifndef LOGUINO_H
#define LOGUINO_H
#include "WProgram.h"
#include "LoguinoConfig.h"
#include "Message.h"
#include "Logger_Serial.h"
#include "Logger_SD.h"
//#include "loguino/Pollers/MSPoller.h"
//#include "loguino/Pollers/LIS331Poller.h"
//#include "loguino/Pollers/DummyPoller.h"
//#include "loguino/Pollers/GPSPoller.h"
//#include "loguino/Pollers/DigitalPoller.h"
//#include "loguino/Pollers/AnalogPoller.h"
//#include "loguino/Pollers/ITG3200Poller.h"

/**
 * The poller is responsible for querying each device and sending out 
 * messages to the messaging system. 
 *
 * In order to make adding and removing support for different hardware 
 * setups easier each query type has its own sub poller, this 
 * implementation simply has to call the methods on each of the sub 
 * pollers based on the configuration options set.
 *
 */
class Poller
{
	public:
		static void begin();
		static void poll();
};




#ifndef LOGGER_FLUSH_MAX
    #error "LOGGER_FLUSH_MAX must be defined as an integer value."
#endif

class Logger
{
	static byte flushCount;
	public:
		static void begin();
		static void log(Message &msg);
};



#endif

