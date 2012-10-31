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
 *
 */
#include <GPSPoller.h>
#ifdef ENABLE_GPS_POLLER

NMEA GPSPoller::n;

/**
 * Configures the serial port in order to connect to the GPS.
 */
void GPSPoller::begin(){
	DEBUG ("GPS Poller: Starting Begin");
    GPS_SERIAL_DEV.begin(GPS_SERIAL_DEV_SPEED);
	DEBUG ("GPS Poller: Begin Finished");
}

/**
 * NMEA GPS devices continually output data over the serial line, data forms
 * NMEA sentances.  The GPS may send multiple types of NMEA sentances depending
 * on the GPS, type of fix etc.  The GPS may send data only when it has a fix, or
 * at a specified interval.
 *
 * As such rather than block until a valid sentance is received, each time the 
 * poller is called, it appends any data in the serial buffer to the NMEA object.
 * 
 * When the appended data completes the sentance, the poller queries the NMEA object
 * to see if the fix is valid, if so it logs each metric.
 *
 */
void GPSPoller::poll(){
    extern Message m;
	DEBUG ("GPS Poller: Beginning Poll ");
    while(GPS_SERIAL_DEV.available()){
	DEBUG ("GPS Poller:  Bytes Available from serial device");
	#ifdef DEBUG_MODE
	Serial.println(n.readSentence);
	#endif
		if (n.addChar(GPS_SERIAL_DEV.read())){
			DEBUG ("GPS Poller:  Sentance Recieved from GPS ");
			if (n.validFix()){
				DEBUG ("GPS Poller:   Valid Fix, logging. ");
                
				DEBUG ("GPS Poller:   Logging Course");
				m.units="Degrees";
				m.nameSpace="GPS.Course";
				m.value=n.getCourse();
				log_message();
				DEBUG ("GPS Poller:   Logged");
                
				DEBUG ("GPS Poller:   Logging Speed");
				m.units="Knots";
				m.nameSpace="GPS.Speed";
				m.value=n.getSpeed();
				log_message();
				DEBUG ("GPS Poller:   Logged");
                
				DEBUG ("GPS Poller:   Logging Latitude");
				m.units="Latitude";
				m.nameSpace="GPS.Latitude";
				m.value=n.getLatitude();
				log_message();
				DEBUG ("GPS Poller:   Logged");
                
				DEBUG ("GPS Poller:   Logging Longitude");
				m.units="Longitude";
				m.nameSpace="GPS.Longitude";
				m.value=n.getLongitude();
				log_message();
				DEBUG ("GPS Poller:   Logged");
                
				DEBUG ("GPS Poller:   Logging Date");
				m.units="Date";
				m.nameSpace="GPS.Date";
				m.value=n.getDate();
                log_message();
				DEBUG ("GPS Poller:   Logged");
                
				DEBUG ("GPS Poller:   Time");
				m.units="Time";
				m.nameSpace="GPS.Time";
				m.value=n.getTime();
                log_message();
				DEBUG ("GPS Poller:   Logged");
            }
		}
	}
}

#endif

