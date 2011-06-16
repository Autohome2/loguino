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
#include "GPSPoller.h"


bool GPSPoller::begin(){
	GPS_SERIAL_DEV.begin(GPS_SERIAL_DEV_SPEED);
}


bool GPSPoller::poll(){
	
	GPS_SERIAL_DEV.flush();
	delay(GPS_SERIAL_WAIT);

	byte tries=0;
	TinyGPS gps;
	Message m;
	long lat, lon;
	unsigned long fix_age;
	unsigned long time, date;

	// Try to get a position fix (GPMRC) from the GPS
	while(tries++ < GPS_SERIAL_MAX_RETRIES)
	{
		while(GPS_SERIAL_DEV.available())
		{
			if(gps.encode(GPS_SERIAL_DEV.read()))
			{
				// Successful sentance
				gps.get_position(&lat, &lon, &fix_age);

				// Check if it actually got a fix or not.
				if (fix_age != TinyGPS::GPS_INVALID_AGE)
				{
					// It has a fix.
					gps.get_datetime(&date, &time, &fix_age);
				
					m.units="Bool";
					m.nameSpace="GPS.hasFix";
					m.value="Yes";
					Logger::log(m);

					m.units="Degrees/100000";
					m.nameSpace="GPS.Latitude";
					m.value=String(lat,DEC);
					Logger::log(m);

					m.units="Degrees/100000";
					m.nameSpace="GPS.Longitude";
					m.value=String(lon,DEC);
					Logger::log(m);
					
					char dbuff[6];
					sprintf(dbuff, "%06d",date);
					char tbuff[8];
					sprintf(tbuff, "%08d",time);
						
					m.units="ddmmyy - hhmmsscc";
					m.nameSpace="GPS.DateTime";
					m.value=String(dbuff) + " - " + String(tbuff);
					Logger::log(m);

					m.units="M/Sec";
					m.nameSpace="GPS.Speed";
					m.value=String(gps.speed());
					Logger::log(m);
				
					m.units="Degrees";
					m.nameSpace="GPS.Course";
					m.value=String(gps.course());
					Logger::log(m);
					return true;
				}
			}
		}

	}

	// No fix, log and exit.
	m.units="Bool";
	m.nameSpace="GPS.hasFix";
	m.value="No";
	Logger::log(m);

	return false;
}
