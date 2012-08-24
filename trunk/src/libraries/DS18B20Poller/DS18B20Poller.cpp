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
#include <DS18B20Poller.h>

#ifdef ENABLE_DS18B20_POLLER

OneWire DS18B20Poller::oneWire(ONE_WIRE_PIN);
DallasTemperature DS18B20Poller::sensors(&oneWire);
DeviceAddress DS18B20Poller::addr;

void DS18B20Poller::begin(){
	sensors.begin();
	// Search the bus for any devices and set the resolution to 10 bit
	while(oneWire.search(addr)) {  				// This returns true if a device is found with this address
		if ( OneWire::crc8( addr, 7) == addr[7]) { 	// Only if the CRC matches
			sensors.setResolution(10);
		}
	}
}

void DS18B20Poller::poll(){
	// Do stuff to read the sensors
	m.units="Degrees C";
  	oneWire.reset_search();
	while(oneWire.search(addr)) {
		if ( OneWire::crc8( addr, 7) == addr[7]) {
			sensors.requestTemperatures();
			float tempC = sensors.getTempC(addr);
			//m.value=String(called++);
			m.value=String(int(tempC));
			char buf[15];
			m.value=dtostrf(tempC,6,2,buf);
			
			m.nameSpace="DS18B20.";
			int i;
			for (i=0;i<8;i++){
				m.nameSpace+=String(addr[i],HEX);
			}
			m.nameSpace+=".Temp";
			log_message();
   		}
  	}
}

#endif
