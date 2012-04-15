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
#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <config.h>
#include <message.h>

#ifdef ENABLE_ETHERNET_LOGGER
#include <EthernetLogger.h>
#endif

#ifdef ENABLE_SERIAL_OUTPUT
    #include <SerialOutput.h>
#endif

#ifdef ENABLE_SD_OUTPUT
#include <SDOutput.h>
#endif

void log_message();
void flush_output();
void loggerBegin();
extern int num_messages;





#endif








