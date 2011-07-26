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

// Debugging Options
//! The log level to use, 0 is off, 7 is trace level.
#define DEBUG_LEVEL 0

// Logging Options

//! How many log messages may be sent before they must be flushed.
#define LOGGER_FLUSH_MAX 30 

// Output

//! Enable the serial output module
#define ENABLE_SERIAL_OUTPUT
#ifdef ENABLE_SERIAL_OUTPUT
	//! Serial Port to use, select 1 of the following:
	#define SO_SERIAL_PORT Serial
	//#define SO_SERIAL_PORT Serial1
	//#define SO_SERIAL_PORT Serial2
	//#define SO_SERIAL_PORT Serial3

	//! The BAUD rate to log at, default is 115200	
	#define SO_SERIAL_PORT_SPEED 115200
#endif

//! Enable the SD Output Module
#define ENABLE_SD_OUTPUT
#ifdef ENABLE_SD_OUTPUT
	//! The pin that the output LED is connected to, this lights up when 
	//! the SD card module has found a card, and is writing data to it.
	#define SD_ACTIVE_PIN 8 
#endif


// Input

//! Enable the GPS Poller
#define ENABLE_GPS_POLLER
#ifdef ENABLE_GPS_POLLER
	//! Sets the serial device of the GPS
	//#define GPS_SERIAL_DEV Serial
	//#define GPS_SERIAL_DEV Serial1
	#define GPS_SERIAL_DEV Serial2
	//#define GPS_SERIAL_DEV Serial3

	//! Set the BAUD rate of the GPS device
	#define GPS_SERIAL_DEV_SPEED 4800
#endif

//! Enable the MegaSquirt Poller
#define ENABLE_MS_POLLER
#ifdef ENABLE_MS_POLLER
	//! The time to wait after polling the megasquirt controller for an answer.
	#define MSP_WAIT_TIME 10
	//! When the megasquirt is online, this pin will be high.  Used to show a status LED.
	#define MS_STATUS_PIN 9
#endif

//! Enable the LIS331 Poller
#define ENABLE_LIS331_POLLER
#ifdef ENABLE_LIS331_POLLER
	/**
	 * The Slave ADdress (SAD) associated to the LIS331HH is 001100xb. SDO/SA0 pad 
	 * can be used to modify less significant bit of the device address. If SA0 pad 
	 * is connected to voltage supply, LSb is ‘1’ (address 0011001b) else if SA0 pad 
	 * is connected to ground, LSb value is ‘0’ (address 0011000b). This solution 
	 * permits to connect and address two different accelerometers to the same I2C 
	 * lines.
	 * 		
	 * Bus address is either 24 or 25 
	 */
	#define LIS_BUS_ADDRESS 25
#endif

//! Enable the Dummy Poller
#define ENABLE_DUMMY_POLLER


