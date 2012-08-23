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
#ifndef DS18B20POLLER_H_
#define DS18B20POLLER_H_

#include <Arduino.h>
#include <config.h>
#ifdef ENABLE_DS18B20_POLLER

#include <message.h>
#include <logger.h>

#include<stdlib.h> //This is for dtostrf as per http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1205038401

#include <OneWire.h>
#include <DallasTemperature.h>

/**
 * A DS18B20 OneWire class to grab temperature data
 *
 * Use Doxygen to document classes.
*/ 


class DS18B20Poller{
	public:
		static void poll();
		static void begin();
	private:
		static OneWire oneWire;
		static DallasTemperature sensors;
		static DeviceAddress addr;
};


#endif
#endif

