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



#ifndef BMP085POLLER_H
#define BMP085POLLER_H

#include <Arduino.h>
#include <config.h>
#include <message.h>
#include <logger.h>
#include <BMP085.h>


#ifdef ENABLE_BMP085_POLLER

//! The BMP085 is a high precision, low power barometric pressure 
// sensor capable of measuring 300 to 1100hPa with an absolute
// accuracy down to 0.03 hPa connected via the I2C bus. In addition
// there is a built in temperature sensor.
class BMP085Poller
{
	static BMP085 bmp;
    public:
        static void begin();
        static void poll();
};
#endif

#endif

