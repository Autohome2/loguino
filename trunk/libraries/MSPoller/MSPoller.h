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

#ifndef MSPoller_h
#define MSPoller_h

#include "WProgram.h"
#include "Message.h"
#include "MegaSquirt.h"
#include "Logger.h"
#include "Debug.h"

//!The time to wait after polling the megasquirt controller for an answer.
#define MSP_WAIT_TIME 10
#define MS_STATUS_PIN 9

//! Queries the megasquirt and logs each value retrieved.
class MSPoller
{
	static bool active;//< when active, the megasquirt is online.
	static byte timeouts;//< The number of times since the last attempt to poll.
	static MegaSquirtData d;//< Megasquirt device
	static void systemState();
	static void keyMetrics();
	static void secondaryMetrics();
	static void afrMetrics();
	static void correctionMetrics();
    public:
        static bool begin();
		static bool poll();
};

#endif


