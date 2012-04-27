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

#include <Arduino.h>
#include <config.h>
#include <message.h>
#include <logger.h>
#include <HS1101Poller.h>
#include <TMP102Poller.h>
#include <BMP085Poller.h>
#include <DummyPoller.h>
#include <GPSPoller.h>
#include <ITG3200Poller.h>
#include <LIS331Poller.h>
#include <AnalogPoller.h>
#include <DigitalPoller.h>
#include <MSPoller.h>
#include <PulsePoller.h>


void loop(){
    m.time=millis();
#ifdef ENABLE_GPS_POLLER
	GPSPoller::poll();
#endif
#ifdef ENABLE_PULSE_POLLER
	PulsePoller::poll();
#endif
#ifdef ENABLE_ITG3200_POLLER
	ITG3200Poller::poll();
#endif
#ifdef ENABLE_DUMMY_POLLER
	DummyPoller::poll();
#endif
#ifdef ENABLE_LIS331_POLLER
	LIS331Poller::poll();
#endif
#ifdef ENABLE_ANALOG_POLLER
	AnalogPoller::poll();
#endif
#ifdef ENABLE_DIGITAL_POLLER
	DigitalPoller::poll();
#endif
#ifdef ENABLE_MS_POLLER
	MSPoller::poll();
#endif
#ifdef ENABLE_TMP102_POLLER
	TMP102Poller::poll();
#endif
#ifdef ENABLE_BMP085_POLLER
	BMP085Poller::poll();
#endif
#ifdef ENABLE_HS1101_POLLER
	HS1101Poller::poll();
#endif

}


void setup(){
    
    
#ifdef ENABLE_DUMMY_POLLER
	DummyPoller::begin();
#endif
#ifdef ENABLE_GPS_POLLER
	GPSPoller::begin();
#endif
#ifdef ENABLE_ITG3200_POLLER
	ITG3200Poller::begin();
#endif
#ifdef ENABLE_LIS331_POLLER
	LIS331Poller::begin();
#endif
#ifdef ENABLE_ANALOG_POLLER
	AnalogPoller::begin();
#endif
#ifdef ENABLE_DIGITAL_POLLER
   	DigitalPoller::begin();
#endif
#ifdef ENABLE_MS_POLLER
	MSPoller::begin();
#endif
#ifdef ENABLE_TMP102_POLLER
	TMP102Poller::begin();
#endif
#ifdef ENABLE_BMP085_POLLER
	BMP085Poller::begin();
#endif
#ifdef ENABLE_HS1101_POLLER
	HS1101Poller::begin();
#endif

#ifdef ENABLE_PULSE_POLLER
	PulsePoller::begin();
#endif


}







