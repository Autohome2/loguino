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

#include "NMEADev.h"

//! Adds a checksum and the $ + newline to the supplied string.
char NMEADev::sumMsg(String &message){
	debug(TRACE, "NMEADev::sumMsg - Entering Function");

	int i=0;
	char checksum=0;
	int l=message.length();

	for (i=0;i<l;i++){
		checksum=checksum ^ message.charAt(i);
	}
	return checksum;

}


void NMEADev::createMsg(String &message){
	debug(TRACE, "NMEADev::createMsg - Entering Function");
	char c;
	c=sumMsg(message);

	char buff[3];
	sprintf(buff, "%02X",c);
	
	debug(INFO, "NMEADev::createMsg - Raw Message: "+message+" - Checksum: "+buff);
	message="$"+message+String('*')+String(buff);
	debug(INFO, "NMEADev::createMsg - Completed Message: "+message);

}




void NMEADev::setSpeed(long speed)
{
	debug(TRACE, "NMEADev::setSpeed - Entering Function");
	debug(TRACE, "NMEADev::setSpeed - Setting Speed to "+String(speed,DEC));
	String message;
	message = String("PSRF100,1,") + String(speed, DEC) + String(",8,1,0");
	createMsg(message);
	NMEA_SERIAL_DEV.println(message);
	NMEA_SERIAL_DEV.begin(speed);
}

void NMEADev::setQueryRate(int messageType, uint8_t rate){
	String message;
	message=String("PSRF103,0")+String(messageType,DEC)+String(",00,0")+String(rate,DEC)+String(",01");
	createMsg(message);
	NMEA_SERIAL_DEV.println(message);
}



bool NMEADev::query(int messageType, uint8_t retries, String &message){
	message=String("PSRF103,0")+String(messageType,DEC)+String(",01,00,01");
	createMsg(message);
	NMEA_SERIAL_DEV.flush();
	Serial.print(message);
	NMEA_SERIAL_DEV.println(message);
	delay(NMEA_SERIAL_WAIT);
	int tries=0;
	bool found=false;
	int state=0;
	char c;
	String sentance;
	while(!found &&tries++<retries)
	{
		while (NMEA_SERIAL_DEV.available()){
			c=NMEA_SERIAL_DEV.read();
			Serial.write(c);
			if (c=='$'){
				state=NMEA_READ_DATA;
				sentance='$';
			}
			else if (c == '\n'){
				state=NMEA_READ_END;
			}
			else if (c== '\r'){
				state=NMEA_READ_END;
			}
			else{
				if (state==NMEA_READ_DATA){
					sentance+=String(c);
				}
			}
			if (state==NMEA_READ_END){
				// End of data has been read, check if its the right one.
				switch( messageType ){
					case NMEA_TYPE_GGA:
						if (sentance.startsWith("$GPGGA")){
							found=true;
						}
						break;
					case NMEA_TYPE_GLL:
						if (sentance.startsWith("$GPGLL")){
							found=true;
						}
						break;
					case NMEA_TYPE_GSA:
						if (sentance.startsWith("$GPGSA")){
							found=true;
						}
						break;
					case NMEA_TYPE_GSV:
						if (sentance.startsWith("$GPGSV")){
							found=true;
						}
						break;
					case NMEA_TYPE_RMC:
						if (sentance.startsWith("$GPRMC")){
							found=true;
						}
						break;
					case NMEA_TYPE_VTG:
						if (sentance.startsWith("$GPVTG")){
							found=true;
						}
						break;
					case NMEA_TYPE_MSS:
						if (sentance.startsWith("$GPMSS")){
							found=true;
						}
						break;
					case NMEA_TYPE_ZDA:
						if (sentance.startsWith("$GPZDA")){
							found=true;
						}
						break;
				}
			}

			if (found){
				message=sentance;
			}
		}
		delay(NMEA_SERIAL_WAIT);
	}
	return false;
}


