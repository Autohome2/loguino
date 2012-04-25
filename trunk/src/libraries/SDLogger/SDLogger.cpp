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
#include <SDLogger.h>
#ifdef ENABLE_SD_OUTPUT

bool SDLogger::sd_active;
File SDLogger::SD_File;

void SDLogger::log(){
    if (sd_active){
                SD_File.println(m.toCSV());
            }
}

void SDLogger::begin(){
    sd_active=true;
    char sd_fname[13];
    Serial.println("Starting");
    
    pinMode(10, OUTPUT);
    sd_active=SD.begin(4);
    
    if (sd_active){
        Serial.println("Looking for filename");
        // Counter for filename
        byte sd_i=0;
        sprintf(sd_fname, "%08d.log",sd_i);
        Serial.print("Trying Files ");
        Serial.println(sd_fname);
        while (sd_i<=250 && SD.exists(sd_fname)){
            sd_i++;
            sprintf(sd_fname, "%08d.log",sd_i);
            Serial.print("Trying Files ");
            Serial.println(sd_fname);
        }
        Serial.println("End of loop");
        if (SD.exists(sd_fname)){
            sd_active=false;
            Serial.println("Still exists");
        } 
    }
    
    if (sd_active){
        SD_File=SD.open(sd_fname,O_WRITE|O_CREAT);
        if (!SD_File){
            sd_active=false;
        }
        
        pinMode(SD_ACTIVE_PIN, OUTPUT);
        digitalWrite(SD_ACTIVE_PIN, true);
    }
    
}
void SDLogger::flush(){
    if (sd_active){
           SD_File.flush();
    }
}
#endif

