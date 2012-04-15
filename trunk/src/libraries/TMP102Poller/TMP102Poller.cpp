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



#include <TMP102Poller.h>

#ifdef ENABLE_TMP102_POLLER
	
void TMP102Poller::begin(){
    Wire.begin();
}
void TMP102Poller::poll()
{
    byte firstbyte, secondbyte; //these are the bytes we read from the TMP102 temperature registers
    int val; /* an int is capable of storing two bytes, this is where we "chuck" the two bytes together. */
    float convertedtemp; /* We then need to multiply our two bytes by a scaling factor, mentioned in the datasheet. */
   
    /* Reset the register pointer (by default it is ready to read temperatures)
     You can alter it to a writeable register and alter some of the configuration -
     the sensor is capable of alerting you if the temperature is above or below a specified threshold. */
    
    Wire.beginTransmission(TMP102_I2C_ADDRESS); //Say hi to the sensor.
	byte i = 0x00;

    Wire.write(i);
    Wire.endTransmission();
    Wire.requestFrom(TMP102_I2C_ADDRESS, 2);
    Wire.endTransmission();
    
    firstbyte      = (Wire.read());
    /*read the TMP102 datasheet - here we read one byte from 
     each of the temperature registers on the TMP102*/
    
    secondbyte     = (Wire.read());
    /*The first byte contains the most significant bits, 
    and the second the less significant */
    
    val = ((firstbyte) << 4);  
    /* MSB */
    
    val |= (secondbyte >> 4);    
    /* LSB is ORed into the second 4 bits of our byte.
     Bitwise maths is a bit funky, but there's a good tutorial on the playground
     */
    convertedtemp = val*0.0625;
    m.units="Degrees 10C";
	char s[32];
	dtostrf(val, 1, 2, s);

    m.value=s;

	m.nameSpace="TMP102.Temp";
    log_message();
}

#endif

