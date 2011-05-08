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


#include "WProgram.h"
#include "MegaSquirt.h"
	



bool MegaSquirtData::populate( byte table[39]){
	 byte squirt;
	 byte engine;

	clock=table[0];

	// Decode these into the boolean values somehow
	squirt=table[1];
	engine=table[2];
	
	inj1squirt=get_bit(squirt,0);
	inj2squirt=get_bit(squirt,1);
	schedtosquirt1=get_bit(squirt,2);
	squirting1=get_bit(squirt,3);
	schedtosquirt2=get_bit(squirt,4);
	squirting2=get_bit(squirt,5);
	boostctrl=get_bit(squirt,6);

	running=get_bit(engine, 0);
	cranking=get_bit(engine, 1);
	ase=get_bit(engine,2);
	warmup=get_bit(engine,3);
	tpsaccel=get_bit(engine,4);
	decel=get_bit(engine,5);
	mapaccel=get_bit(engine,6);
	idle=get_bit(engine,7);

	baroADC=table[3];
	mapADC=table[4];
	matADC=table[5];
	cltADC=table[6];
	tpsADC=table[7];
	batADC=table[8];
	egoADC=table[9];
	egoCorrection=table[10];
	airCorrection=table[11];
	warmupEnrich=table[12];
	rpm100=table[13];
	pulsewidth1=table[14];
	accelEnrich=table[15];
	baroCorrection=table[16];
	gammaEnrich=table[17];
	veCurr1=table[18];
	pulsewidth2=table[19];
	veCurr2=table[20];
	idleDC=table[21];
	iTimeM=table[22];
	iTimeL=table[23];
	advance=table[24];
	afrTarget=table[25];
	fuelADC=table[26];
	egtADC=table[27];
	CltIatAngle=table[28];
	KnockAngle=table[29];
	egoCorrection2=table[30];
	TPSLast=table[36];
	iTimeH=table[37];

	return true;
}


bool MegaSquirtData::get_bit(byte b, int p)
{
    b<<=(p);
	if (b>=127){
		return true;
	}
	return false;	
}

