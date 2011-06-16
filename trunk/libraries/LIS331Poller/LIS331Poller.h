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

#ifndef LIS331Poller_h
#define LIS331Poller_h


#include "WProgram.h"
#include "Wire.h"
#include "LIS331.h"
#include "Logger.h"
#include "Message.h"


/*
 * 0
 * 0
 * 16 1
 * 8  1
 * 4  0
 * 2  0
 * 1  0
 */

/* 
  The Slave ADdress (SAD) associated to the LIS331HH is 001100xb. SDO/SA0 pad 
  can be used to modify less significant bit of the device address. If SA0 pad 
  is connected to voltage supply, LSb is ‘1’ (address 0011001b) else if SA0 pad 
  is connected to ground, LSb value is ‘0’ (address 0011000b). This solution 
  permits to connect and address two different accelerometers to the same I2C 
  lines.

  Bus address is either 24 or 25 
*/

#define LIS_BUS_ADDRESS 24

class LIS331Poller
{
	static LIS331 lis;
	static bool active;
	static byte timeouts;
    public:
        static bool begin();
		static bool poll();
};

#endif


