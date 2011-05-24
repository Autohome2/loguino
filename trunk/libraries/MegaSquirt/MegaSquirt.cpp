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
#define LAMBDA 14.1

//! Loads a new set of registers.
bool MegaSquirtData::loadData(byte newReg[112]){
	uint8_t i=0;
	for (i=0;i<=112;i++){
		reg[i]=newReg[i];
	}
	return true;
}

//! The number of seconds since the controller was initialized.
uint16_t MegaSquirtData::MegaSquirtData::seconds(){
	return ((reg[0] <<8)|reg[1]);
}


//! The pulse width for the primary injection circuit.
uint16_t MegaSquirtData::MegaSquirtData::pulseWidth1(){
	return ((reg[2] <<8)|reg[3]);
}
//! The pulse width for the secondary injection circuit.
uint16_t MegaSquirtData::MegaSquirtData::pulseWidth2(){
	return ((reg[4] <<8)|reg[5]);
}

//! The current engine RPM.
uint16_t MegaSquirtData::MegaSquirtData::rpm(){
	return ((reg[6] <<8)|reg[7]);

}

//! The ignition advance in Degrees Before Top Dead Center.
int16_t MegaSquirtData::advance(){
	return ((reg[8] <<8)|reg[9]);
}


//! A byte containing the register for the squirt status.
uint8_t MegaSquirtData::squirt(){
	return reg[10];
}

//! Bank1 Ignition firing.
bool MegaSquirtData::firing1(){
	return get_bit(squirt(), 1);
}

//! Bank2 Ignition firing.
bool MegaSquirtData::firing2(){
	return get_bit(squirt(), 2);
}

//! Injector Circuit 1 scheduled to squirt.
bool MegaSquirtData::sched1(){
	return get_bit(squirt(), 3);
}

//! Injector Circuit 1 Squirting.
bool MegaSquirtData::inj1(){
	return get_bit(squirt(), 4);
}

//! Injector Circuit 2 scheduled to squirt.
bool MegaSquirtData::sched2(){
	return get_bit(squirt(), 5);
}

//! Injector Circuit 2 Squirting.
bool MegaSquirtData::inj2(){
	return get_bit(squirt(), 6);
}



//! Returns the register containing the Engine Operating/Status variables.
uint8_t MegaSquirtData::engine(){
	return reg[11];
}

//! Engine ready to run.
bool MegaSquirtData::ready(){
	return get_bit(engine(),1);
}

//! Engine Cranking.
bool MegaSquirtData::crank(){
	return get_bit(engine(),2);
}

//! Engine in After Start Warmup Enrichment.
bool MegaSquirtData::startw(){
	return get_bit(engine(),3);
}

//! Engine in Warmup Enrichment.
bool MegaSquirtData::warmup(){
	return get_bit(engine(), 4);
}

//! TPS Based Acceleration active.
bool MegaSquirtData::tpsaen(){
	return get_bit(engine(), 5);
}

//! TPS Based Deceleration active.
bool MegaSquirtData::tpsden(){
	return get_bit(engine(), 6);
}


//! MAP Based Acceleration enrichment active.
bool MegaSquirtData::mapaen(){
	return get_bit(engine(), 7);
}


//! AFR target Table 1.
uint8_t MegaSquirtData::afrtgt1(){
	return reg[12];
}

//! AFR target Table 2.
uint8_t MegaSquirtData::afrtgt2(){
	return reg[13];
}

//! Wideband O2 Sensor1 enabled.  From wbo2 - indicates whether wb afr valid.
uint8_t MegaSquirtData::wbo2_en1(){
	return reg[14];
}

//! Wideband O2 Sensor2enabled.  From wbo2 - indicates whether wb afr valid.
uint8_t MegaSquirtData::wbo2_en2(){
	return reg[15];
}



//! Barometric pressure.
int16_t MegaSquirtData::barometer(){
	return ((reg[16] <<8)|reg[17]);
}

//! Manifold Air Pressure.
int16_t MegaSquirtData::map(){
	return ((reg[18] <<8)|reg[19]);
}

//! Manifold Air Temperature.
int16_t MegaSquirtData::mat(){
	return ((reg[20] <<8)|reg[21]);
}

//! Coolant Temperature.
int16_t MegaSquirtData::coolant(){
	return ((reg[22] <<8)|reg[23]);
}

//! Throttle Position.
int16_t MegaSquirtData::tps(){
	return ((reg[24] <<8)|reg[25]);
}

//! Battery Voltage.
int16_t MegaSquirtData::batteryVoltage(){
	return ((reg[26] <<8)|reg[27]);
}

//! AFR Sensor 1.
int16_t MegaSquirtData::afr1(){
	return ((reg[28] <<8)|reg[29]);
}

//! AFR Sensor 2.
int16_t MegaSquirtData::afr2(){
	return ((reg[30] <<8)|reg[31]);

}

//! Knock Sensor. 
int16_t MegaSquirtData::knock(){
	return ((reg[32] <<8)|reg[33]);
}

//! AFR 1 as Lambda
int16_t MegaSquirtData::lambda1(){
	return afr1()/LAMBDA;
}

//! AFR 2 as Lambda
int16_t MegaSquirtData::lambda2(){
	return afr2()/LAMBDA;
}

//! Percent adjustment due to EGO Correction.
int16_t MegaSquirtData::egoCorrection1(){
	return ((reg[34] <<8)|reg[35]);
}


//! Percent adjustment due to EGO Correction.
int16_t MegaSquirtData::egoCorrection(){
	return ((egoCorrection1()+egoCorrection2())/2);
}


//! Percent adjustment due to EGO Correction.
int16_t MegaSquirtData::egoCorrection2(){
	return ((reg[36] <<8)|reg[37]);
}

//! Percent adjustment due to air temperature.
int16_t MegaSquirtData::airCorrection(){
	return ((reg[38] <<8)|reg[39]);
}

int16_t MegaSquirtData::warmupEnrich(){
	return ((reg[40] <<8)|reg[41]);

}


int16_t MegaSquirtData::accelEnrich(){
	return ((reg[42] <<8)|reg[43]);

}


int16_t MegaSquirtData::tpsfuelcut(){
	return ((reg[44] <<8)|reg[45]);

}


int16_t MegaSquirtData::baroCorrection(){
	return ((reg[46] <<8)|reg[47]);

}


int16_t MegaSquirtData::gammaEnrich(){
	return ((reg[48] <<8)|reg[49]);

}


int16_t MegaSquirtData::veCurr1(){
	return ((reg[50] <<8)|reg[51]);

}


int16_t MegaSquirtData::veCurr2(){
	return ((reg[52] <<8)|reg[53]);

}


int16_t MegaSquirtData::veCurr(){
	return veCurr1();
}


int16_t MegaSquirtData::iacstep(){
	return ((reg[54] <<8)|reg[55]);
}


int16_t MegaSquirtData::coldAdvDeg(){
	return ((reg[56] <<8)|reg[57]);
}


int16_t MegaSquirtData::tpsDOT(){
	return ((reg[58] <<8)|reg[59]);
}


int16_t MegaSquirtData::mapDOT(){
	return ((reg[60] <<8)|reg[61]);
}


int16_t MegaSquirtData::dwell(){
	return ((reg[62] <<8)|reg[63]);
}


int16_t MegaSquirtData::maf(){
	return ((reg[64] <<8)|reg[65]);
}


int16_t MegaSquirtData::calcMAP(){
	return ((reg[66] <<8)|reg[67]);

}


int16_t MegaSquirtData::fuelCorrection(){
	return ((reg[68] <<8)|reg[69]);

}

uint16_t MegaSquirtData::MegaSquirtData::portStatus()
{
	return reg[70];
}
bool MegaSquirtData::port0()
{
	return get_bit(portStatus(),1);
}
bool MegaSquirtData::port1()
{

	return get_bit(portStatus(),2);
}
bool MegaSquirtData::port2()
{
	return get_bit(portStatus(),3);

}
bool MegaSquirtData::port3()
{
	return get_bit(portStatus(),4);

}
bool MegaSquirtData::port4()
{
	return get_bit(portStatus(),5);

}
bool MegaSquirtData::port5()
{
	return get_bit(portStatus(),6);

}
bool MegaSquirtData::port6()
{
	return get_bit(portStatus(),7);

}

uint8_t MegaSquirtData::knockRetard()
{
	return reg[71];
}

int16_t MegaSquirtData::xTauFuelCorr1()
{
	return ((reg[72] <<8)|reg[73]);

}
int16_t MegaSquirtData::egoV1()
{
	return ((reg[74] <<8)|reg[75]);

}
int16_t MegaSquirtData::egoV2()
{
	return ((reg[76] <<8)|reg[77]);

}
int16_t MegaSquirtData::amcUpdates()
{
	return ((reg[78] <<8)|reg[79]);

}


int16_t MegaSquirtData::kpaix()
{
	return ((reg[80] <<8)|reg[81]);
}

//! X TAU Adjustment for VE table 2.
int16_t MegaSquirtData::xTauFuelCorr2()
{
	return ((reg[82] <<8)|reg[83]);
}






int16_t MegaSquirtData::spare1()
{
	return ((reg[84] <<8)|reg[85]);

}
int16_t MegaSquirtData::spare2()
{
	return ((reg[86] <<8)|reg[87]);

}
int16_t MegaSquirtData::trig_fix()
{
	return ((reg[88] <<8)|reg[89]);

}
int16_t MegaSquirtData::spare4()
{
	return ((reg[90] <<8)|reg[91]);

}
int16_t MegaSquirtData::spare5()
{
	return ((reg[92] <<8)|reg[93]);

}
int16_t MegaSquirtData::spare6()
{
	return ((reg[94] <<8)|reg[95]);

}
int16_t MegaSquirtData::spare7()
{
	return ((reg[96] <<8)|reg[97]);

}
int16_t MegaSquirtData::spare8()
{
	return ((reg[98] <<8)|reg[99]);

}
int16_t MegaSquirtData::spare9()
{
	return ((reg[100] <<8)|reg[101]);

}

//! Spare Byte 10.
int16_t MegaSquirtData::spare10()
{
	return ((reg[102] <<8)|reg[103]);

}

//! Incremented on each tach pulse received.
uint16_t MegaSquirtData::tachCount()
{
	return ((reg[104] <<8)|reg[105]);
}


uint8_t MegaSquirtData::ospare()
{
	return reg[106];
}

uint8_t MegaSquirtData::cksum()
{
	return reg[107];
}



//! Normalized time between trigger pulses, "fills in" missing teeth.
uint32_t MegaSquirtData::deltaT()
{
	return ( (reg[108]<<24) | (reg[109]<<16) | (reg[110]<<8) | (reg[111]) );
}






bool MegaSquirtData::get_bit(byte b, int p)
{
    b<<=(p);
	if (b>=127){
		return true;
	}
	return false;	
}

