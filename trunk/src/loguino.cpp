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
#ifdef ENABLE_DUMMY_POLLER
    #include <DummyPoller.h>
#endif
#ifdef ENABLE_GPS_POLLER
    #include <GPSPoller.h>
#endif
#ifdef ENABLE_ITG3200_POLLER
    #include <ITG3200Poller.h>
#endif
#ifdef ENABLE_LIS331_POLLER
    #include <LIS331Poller.h>
#endif
#ifdef ENABLE_ANALOG_POLLER
    #include <AnalogPoller.h>
#endif
#ifdef ENABLE_DIGITAL_POLLER
    #include <DigitalPoller.h>
#endif

#ifdef ENABLE_MS_POLLER
    #include <MSPoller.h>
#endif


void loop(){
    m.time=millis();
#ifdef ENABLE_GPS_POLLER
    GPSpoll();
#endif
#ifdef ENABLE_ITG3200_POLLER
    itg3200poll();
#endif
#ifdef ENABLE_DUMMY_POLLER
    Dummypoll();
#endif
#ifdef ENABLE_LIS331_POLLER
    LIS331poll();
#endif
#ifdef ENABLE_ANALOG_POLLER
    analogpoll();
#endif
#ifdef ENABLE_DIGITAL_POLLER
    digitalpoll();
#endif
#ifdef ENABLE_MS_POLLER
    MSpoll();
#endif

}


void setup(){
    loggerBegin();
    
    
#ifdef ENABLE_DUMMY_POLLER
    Dummybegin();
#endif
#ifdef ENABLE_GPS_POLLER
    GPSbegin();
#endif
#ifdef ENABLE_ITG3200_POLLER
    itg3200begin();
#endif
#ifdef ENABLE_LIS331_POLLER
    LIS331begin();
#endif
#ifdef ENABLE_ANALOG_POLLER
    analogbegin();
#endif
#ifdef ENABLE_DIGITAL_POLLER
    digitalbegin();
#endif
#ifdef ENABLE_MS_POLLER
    MSbegin();
#endif

}







