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

#include "Poller.h"
#include "Debug.h"

void setup(){
//	debug(INFO, "Entering Setup");

//	debug(INFO, "Setup: Setting up Logger");
	Logger::begin();

//	debug(INFO, "Setup: Setting up Poller");
	Poller::begin();

//	debug(INFO, "Setup Complete.");
}

void loop(){
	debug(INFO, "Loop: Starting Poll Run");
	Poller::poll();
	debug(INFO, "Loop: Poll Run Complete.");
}


