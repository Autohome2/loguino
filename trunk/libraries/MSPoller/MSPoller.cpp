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
#include "MSPoller.h"

bool MSPoller::active;
byte MSPoller::timeouts;


bool MSPoller::begin(){
	MegaSquirt::begin();
	active=true;
	timeouts=0;
}

bool MSPoller::poll( ){
	// IF inactive, check if its time to try again.
	if (!active)
	{
		// If timeouts > max time out time, then become active, and reset timeouts.
		if (timeouts++ > MSP_WAIT_TIME){
			active=true;
			timeouts=0;
		}
		// timeouts not reached, just return.
		else
		{
			return false;
		}
	}
	
	byte status;
	MegaSquirtData d;
	byte table[MS_TABLE_LENGTH];
	status=MegaSquirt::getData(table);
	if (status != MS_COMM_SUCCESS)
	{
		active=false;
		return false;
	}

	d.loadData(table);

	Message m;
	m.units="Boolean";
	m.nameSpace="MegaSquirt.Cranking";
	m.value=d.crank() ? "True" : "False";
	Logger::log(m);
	
	return true;
}

