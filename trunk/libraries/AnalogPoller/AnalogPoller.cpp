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
#ifdef ENABLE_ANALOG_POLLER
#include "WProgram.h"
#include "AnalogPoller.h"


int AnalogPoller::called;

bool AnalogPoller::begin(){
	called=0;
	analogReference(EXTERNAL);
}

bool AnalogPoller::poll()
{
	int pins[]={ANALOG_PINS};
	int numpins=sizeof(pins)/sizeof(int);
	Message m;
	int16_t val;
	m.units="Bool";
	int i;			    
	for (i=0;i<numpins;i++){
		m.nameSpace=String("AnalogInput.Pin")+String(pins[i]);
		m.value=String(analogRead(pins[i]));
		Logger::log(m);
	}

}
#endif

