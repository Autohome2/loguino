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
 * $Rev$:   
 * $Author$: 
 * $Date$:  

*/



#include <BMP085Poller.h>

#ifdef ENABLE_BMP085_POLLER
	
BMP085 BMP085Poller::bmp;


void BMP085Poller::begin(){
	BMP085Poller::bmp.begin();
}
void BMP085Poller::poll()
{

    m.units="Degrees C*100";
   	m.value=String(int(bmp.readTemperature()*100));
	m.nameSpace="BMP085.Temp";
	log_message();

	m.units="Pa";
	m.value=String(bmp.readPressure());
	m.nameSpace="BMP085.Pressure";
	log_message();

//	m.units="Meters*100";
//	m.value=String(int(BMP085Poller::bmp.readAltitude()*100));
//	m.nameSpace="BMP085.Altitude";
//	log_message();
}

#endif

