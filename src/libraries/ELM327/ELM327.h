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

 * $Rev$
 * $Author$
 * $Date$

*/

#ifndef ELM327_h
#define ELM327_h
#include "Arduino.h"

#define ELM_TIMEOUT 15000
#define ELM_BAUD_RATE 9600
#define ELM_PORT Serial3

/**
 * Return code when communication with the ELM device returned succesfully
 */
#define ELM_SUCCESS 0


/**
 * Return code when communication with the ELM device failed as there was 
 *  no response.
 */
#define ELM_NO_RESPONSE 1

/**
 * Return code when the ELM device returned more data than the given buffer
 * was able to hold. 
 */
#define ELM_BUFFER_OVERFLOW 2


/**
 *  Return code when the ELM device returns unexpected data.  This is 
 *  usually when the response does not contain byte values, or contains
 *  unexpected output.
 */
#define ELM_GARBAGE 3


/**
 *  Return code when the ELM device is unable to connect to the vehicle 
 *  The ELM327 has tried all of the available protocols, and could not
 *  detect a compatible one. This could be because your vehicle uses an
 *  unsupported protocol, or could be as simple as forgetting to turn the
 *  ignition key on. Check all of your connections, and the ignition,
 *  then try the command again.
 */
#define ELM_UNABLE_TO_CONNECT 4


/**
 *  Return code when the ELM device returns NO DATA.  The IC waited for 
 *  the period of time that was set by AT ST, and detected no response
 *  from the vehicle. It may be that the vehicle had no data to offer
 *  for that particular PID, that the mode requested was not supported,
 *  that the vehicle was attending to higher priority issues, or in the
 *  case of the CAN systems, the filter may have been set so that the
 *  response was ignored, even though one was sent. If you are certain
 *  that there should have been a response, try increasing the ST time
 *  (to be sure that you have allowed enough time for the ECU to
 *  respond), or restoring the CAN filter to its default setting.
 */
#define ELM_NO_DATA 5



/** Interface for the ELM327 multi-function OBDII Interpreter IC.  Providing
 * both a low level interface to run commands directly on the IC, and a
 * higher level API that provides convenience functions to read from the OBD2
 * PIDS, read and reset error codes, and otherwise interrogate the connected
 * vehicle.
 */
class Elm327
{
	public:

		/**
		 *  Configures the ELM Device by sending a series of commands:  
		 *		- Reset 
		 *		- Echo Off
		 *		- Auto detect connection.
		 *  @return ELM_SUCCESS when the ELM device returned the anticipated
		 *  response. Else returns an ELM error code.
		 */
		byte begin();


		/** 
		 *  Issues the ATI command to the ELM device.  The ATI command 
		 *  causes the chip to identify itself, by printing the startup
		 *  product ID string (currently ‘ELM327 v1.3a’). Software can
		 *  use this to determine exactly which integrated circuit it is
		 *  talking to, without having to reset the IC.
		 *  @param[in,out]	rev	String used to store the revision and 
		 *  identifier.
		 *
		 */
		byte getVersion(String &rev);


		/**
		 * 	Issues the AT IGN command to the elm device.  This command provides
		 * 	a means of checking the voltage level at pin 15. It assumes that 
		 * 	the logic level is related to the ignition voltage, so if the input
		 * 	is at a high level, then rev shall be set to true (On) where as a 
		 * 	low level shall set rev to false. (Off)
		 *  @param[in,out]	powered	Boolean value that is set to the ignition
		 *  status.
		 */
		byte getIgnMon(bool &powered);


		/** 
		 *  Issues the AT RV command to the Elm device.  This initiates the
		 *  reading of the voltage present at pin 2, and the conversion of it to
		 *  a decimal voltage. By default, it is assumed that the input is
		 *  connected to the voltage to be measured through a 47KΩ and 10KΩ
		 *  resistor divider (with the 10KΩ connected from pin 2 to Vss), and that
		 *  the ELM327 supply is a nominal 5V. This will allow for the measurement
		 *  of input voltages up to about 28V, with an uncalibrated accuracy of
		 *  typically about 2%.
		 *  @param[in,out]	voltage	Float that is set to the detected voltage.
		 */
		byte getVoltage(float &voltage);


		/**
		 * Gets the Calculated Engine Load.  Reads PID 04 from the OBD interface and sets 
		 * load to the the value returned after conversion.
		 * - Minimum value: 0
		 * - Maximum value: 100
		 * - Units: %
		 * @param[out] load Integer value is set to the calculated Engine Load.
		 */
		byte engineLoad(int &load);


		/**
		 * Gets the Coolant Temperature.  Reads PID 05 from the OBD interface and sets
		 * temp to the value returned after conversion.
		 * - Minimum: -40
		 * - Maximum Value: 215
		 * - Units: °C
		 * @param[out]	temp	Signed integer value is set to the coolant temperature.
		 */
		byte coolantTemperature(int &temp);


		/**
		 * Gets the fuel trim for bank 1 in the short term.  Reads PID 06 from the OBD 
		 * interface and sets percent to the value returned after conversion.
		 * - Minimum Value: -100
		 * - Maximum Value: 99.2
		 * - Units: %
		 * @param[out]	percent	- Signed integer is set to the percent fuel trim.
		 */
		byte fuelTrimBank1ShortTerm(int &percent);


		/**
		 * Gets the fuel trim for bank 2 in the short term.  Reads PID 07 from the OBD 
		 * interface and sets percent to the value returned after conversion.
		 * - Minimum Value: -100
		 * - Maximum Value: 99.2
		 * - Units: %
		 * @param[out]	percent	- Signed integer is set to the percent fuel trim.
		 */
		byte fuelTrimBank2ShortTerm(int &percent);


		/**
		 * Gets the fuel trim for bank 1 in the long term.  Reads PID 08 from the OBD 
		 * interface and sets percent to the value returned after conversion.
		 * - Minimum Value: -100
		 * - Maximum Value: 99.2
		 * - Units: %
		 * @param[out]	percent	- Signed integer is set to the percent fuel trim.
		 */
		byte fuelTrimBank1LongTerm(int &percent);


		/**
		 * Gets the fuel trim for bank 2 in the long term.  Reads PID 09 from the OBD 
		 * interface and sets percent to the value returned after conversion.
		 * - Minimum Value: -100
		 * - Maximum Value: 99.2
		 * - Units: %
		 * @param[out]	percent	- Signed integer is set to the percent fuel trim.
		 */
		byte fuelTrimBank2LongTerm(int &percent);


		byte fuelPressure(int &pressure);
		byte intakeManifoldAbsolutePressure(int &pressure);
		byte engineRPM(int &rpm);
		byte vehicleSpeed(byte &speed);
		byte timingAdvance(byte &advance);
		byte intakeAirTemperature(byte &temperature);
		byte MAFAirFlowRate(unsigned int &rate);
		byte throttlePosition(byte &position);
		byte o2SensorBank1Sensor1(byte &voltage, byte &trim);
		byte o2SensorBank1Sensor2(byte &voltage, byte &trim);
		byte o2SensorBank1Sensor3(byte &voltage, byte &trim);
		byte o2SensorBank1Sensor4(byte &voltage, byte &trim);
		byte o2SensorBank2Sensor1(byte &voltage, byte &trim);
		byte o2SensorBank2Sensor2(byte &voltage, byte &trim);
		byte o2SensorBank2Sensor3(byte &voltage, byte &trim);
		byte o2SensorBank2Sensor4(byte &voltage, byte &trim);
		byte o2sensorRead(const char *bank, byte &voltage, byte &trim);
		byte auxiliaryInputStatus(bool &auxStatus);
		byte engineRunTime(unsigned int &runTime);
		byte distanceMIL(unsigned int &distance);
		byte relativeFuelRailPressure(unsigned int &pressure);
		byte absoluteFuelRailPressure(unsigned int &pressure);
		byte o2S1WRVoltage(unsigned int &equivRatio, unsigned int &voltage);
		byte o2S2WRVoltage(unsigned int &equivRatio, unsigned int &voltage);
		byte o2S3WRVoltage(unsigned int &equivRatio, unsigned int &voltage);
		byte o2S4WRVoltage(unsigned int &equivRatio, unsigned int &voltage);
		byte o2S5WRVoltage(unsigned int &equivRatio, unsigned int &voltage);
		byte o2S6WRVoltage(unsigned int &equivRatio, unsigned int &voltage);
		byte o2S7WRVoltage(unsigned int &equivRatio, unsigned int &voltage);
		byte o2S8WRVoltage(unsigned int &equivRatio, unsigned int &voltage);
		byte commandedEGR(byte &egr);
		byte EGRError(byte &error);
		byte commandedEvaporativePurge(byte &purge);
		byte fuelLevel(byte &level);
		byte warmUpsSinceLastCleared(byte &warmUps);
		byte distanceSinceLastCleared(unsigned int &distance);
		byte evapPressure(int &pressure);
		byte barometricPressure(byte  &pressure);
		byte o2S1WRCurrent(unsigned int &equivRatio, int &current);
		byte o2S2WRCurrent(unsigned int &equivRatio, int &current);
		byte o2S3WRCurrent(unsigned int &equivRatio, int &current);
		byte o2S4WRCurrent(unsigned int &equivRatio, int &current);
		byte o2S5WRCurrent(unsigned int &equivRatio, int &current);
		byte o2S6WRCurrent(unsigned int &equivRatio, int &current);
		byte o2S7WRCurrent(unsigned int &equivRatio, int &current);
		byte o2S8WRCurrent(unsigned int &equivRatio, int &current);
		byte catalystTemperatureBank1Sensor1( int &temperature);
		byte catalystTemperatureBank2Sensor1( int &temperature);
		byte catalystTemperatureBank1Sensor2( int &temperature);
		byte catalystTemperatureBank2Sensor2( int &temperature);
		byte controlModuleVoltage(unsigned int &voltage);
		byte absoluteLoadValue(unsigned int &load);
		byte commandEquivalenceRatio(float &ratio);
		byte relativeThrottlePosition(byte &position);
		byte ambientAirTemperature(int &temperature);
		byte absoluteThrottlePositionB(byte &position);
		byte absoluteThrottlePositionC(byte &position);
		byte acceleratorPedalPositionD(byte &position);
		byte acceleratorPedalPositionE(byte &position);
		byte acceleratorPedalPositionF(byte &position);
		byte commandedThrottleActuator(byte &position);
	private:
		byte o2WRVoltage(const char *sensor, unsigned int &equivRatio, unsigned int &voltage);
		byte o2WRCurrent(const char *sensor, unsigned int &equivRatio, int &current);
		byte catTemperature(const char *sensor, int &temperature);
		byte getBytes( const char *mode, const char *chkMode, const char *pid, byte *values, unsigned int numValues);
		byte runCommand(const char *cmd, char *data, unsigned int dataLength);
		bool getBit(byte b, byte p);
		byte getFuelTrim(const char *pid, int &percent);
		void flush();
};
		

#endif


