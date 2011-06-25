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

#ifndef SDOutput_h
#define SDOutput_h

#define SSPIN 10

#include "WProgram.h"
#include "Debug.h"
#include "SD.h"
#include "Message.h"
#include "Logger.h"

class SDOutput
{
    static bool active;
    static File _File;
    public:
        static bool begin();
        static bool log(Message &msg);
        static bool flush();
};


#endif

