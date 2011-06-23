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

#ifndef NMEADEV_H
#define NMEADEV


#include "WProgram.h"
#include <stdlib.h>
#include <string.h>
#include "Debug.h"

#define NMEA_SERIAL_DEV Serial2
#define NMEA_SERIAL_WAIT 10

#define NMEA_READ_DATA 1
#define NMEA_READ_END 2

#define NMEA_SERIAL_WAIT 10
#define NMEA_TYPE_GGA 0
#define NMEA_TYPE_GLL 1
#define NMEA_TYPE_GSA 2
#define NMEA_TYPE_GSV 3
#define NMEA_TYPE_RMC 4
#define NMEA_TYPE_VTG 5
#define NMEA_TYPE_MSS 6
#define NMEA_TYPE_ZDA 8

class NMEADev{
	private:
		void createMsg(String &message);
		char sumMsg(String &message);
	public:
		void setSpeed(long speed);
		void setQueryRate(int messageType, uint8_t rate);
		bool query(int messageType, uint8_t retries, String &message);

	
};

#endif
