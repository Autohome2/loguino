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




#include <EthernetLogger.h>


#ifdef ENABLE_ETHERNET_LOGGER

EthernetServer EthernetLogger::server=EthernetServer(8000);;

/**
 * Prints the message as CSV to any connected clients.
 */
void EthernetLogger::log(){
	server.println(m.toCSV());
	delay(120);
}

/**
 * Configures the TCP Server using the Ethernet library.  Takes the 
 * configuration settings for MAC, IP, GW etc from the preprocessor
 * directives and configures the network interface.  
 *
 * Starts a server on the specified port ready for clients to connected.
 */
void EthernetLogger::begin(){
	byte mac[]={ETHERNET_MAC_ADDRESS};
	byte ip[]={ETHERNET_IP_ADDRESS};
	byte dns[]={ETHERNET_DNS_ADDRESS};
	byte gw[]={ETHERNET_GW_ADDRESS};
	byte nm[]={ETHERNET_NETMASK};
	Ethernet.begin(mac,ip,dns,gw,nm);
	server.begin();

}

/**
 * There are no buffers to flush, however is mandatory to implement a flush
 * method.
 */
void EthernetLogger::flush(){
	// Nothing to flush...
	return;
}
#endif

