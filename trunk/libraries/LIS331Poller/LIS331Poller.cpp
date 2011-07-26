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


#include <config.h>
#include "WProgram.h"
#include "LIS331Poller.h"


#ifdef ENABLE_LIS331_POLLER

LIS331 LIS331Poller::lis;
bool LIS331Poller::active;
byte LIS331Poller::timeouts;


bool LIS331Poller::begin(){
	active=true;
	timeouts=0;

	lis.setPowerStatus(LR_POWER_NORM);
	lis.setXEnable(true);
	lis.setYEnable(true);
	lis.setZEnable(true);
}


bool LIS331Poller::poll(){
	Message m;
	int16_t val;
	m.units="mG";
	
	lis.getXValue(&val);
	m.nameSpace="Accelerometer.LIS331.X";
	m.value=val;
	Logger::log(m);

	lis.getYValue(&val);
	m.nameSpace="Accelerometer.LIS331.Y";
	m.value=val;
	Logger::log(m);

	lis.getZValue(&val);
	m.nameSpace="Accelerometer.LIS331.Z";
	m.value=val;
	Logger::log(m);

}
#endif

