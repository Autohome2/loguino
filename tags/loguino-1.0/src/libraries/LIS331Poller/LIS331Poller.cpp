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

#include <LIS331Poller.h>


LIS331 lis;
bool lis_active;
bool lis_timeouts;
int16_t lis_val;


void LIS331begin(){
    lis.setPowerStatus(LR_POWER_NORM);
    lis.setXEnable(true);
    lis.setYEnable(true);
    lis.setZEnable(true);

}
void LIS331poll(){
    m.units="mG";
	lis.getXValue(&lis_val);
	m.nameSpace="Accelerometer.LIS331.X";
	m.value=String(int(lis_val));
    log_message();
	lis.getYValue(&lis_val);
	m.nameSpace="Accelerometer.LIS331.Y";
	m.value=String(int(lis_val));
    log_message();
	lis.getZValue(&lis_val);
	m.nameSpace="Accelerometer.LIS331.Z";
	m.value=String(int(lis_val));
    log_message();
}