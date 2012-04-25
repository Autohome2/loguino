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



#ifndef ETHERNETLOGGER_H
#define ETHERNETLOGGER_H

#include <Arduino.h>
#include <config.h>
#include <message.h>
#include <Ethernet.h>



#ifdef ENABLE_ETHERNET_LOGGER
	#ifndef ETHERNET_MAC_ADDRESS
		#error ETHERNET_MAC_ADDRESS must be defined
    #endif
	#ifndef ETHERNET_IP_ADDRESS
		#error ETHERNET_IP_ADDRESS must be defined
    #endif
	#ifndef ETHERNET_DNS_ADDRESS
		#error ETHERNET_DNS_ADDRESS must be defined
    #endif
	#ifndef ETHERNET_GW_ADDRESS
		#error ETHERNET_GW_ADDRESS must be defined
    #endif
	#ifndef ETHERNET_NETMASK
		#error ETHERNET_NETMASK must be defined
    #endif
	

class EthernetLogger
{
    public:
		static EthernetServer server;
        static void log();
        static void begin();
		static void flush();
};
#endif

#endif

