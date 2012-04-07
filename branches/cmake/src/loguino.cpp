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



//! Enable the GPS Poller
#define ENABLE_GPS_POLLER
#ifdef ENABLE_GPS_POLLER
	//! Sets the serial device of the GPS
	//#define GPS_SERIAL_DEV Serial
	//#define GPS_SERIAL_DEV Serial1
	#define GPS_SERIAL_DEV Serial2
	//#define GPS_SERIAL_DEV Serial3

	//! Set the BAUD rate of the GPS device
	#define GPS_SERIAL_DEV_SPEED 4800
#endif


//! Enable the Dummy Poller
#define ENABLE_DUMMY_POLLER

//! Enable the Digital Input Poller
#define ENABLE_DIGITAL_POLLER
#ifdef ENABLE_DIGITAL_POLLER
	//! A list, comma seperated of digital pins that will be read for a value.
	#define DIGITAL_PINS 31,32,33,34,35,36
	//! A list, comma seperated of pins that will be held high.
	#define INVERT_DIGITAL_PINS 32,33
#endif

//! Enable the Analog Input Poller
#define ENABLE_ANALOG_POLLER
#ifdef ENABLE_ANALOG_POLLER
	//! A list, comma seperated of digital pins that will be read for a value.
	#define ANALOG_PINS 2,3,4
#endif



//! Enable the serial output module
#define ENABLE_SERIAL_OUTPUT
#ifdef ENABLE_SERIAL_OUTPUT
	//! Serial Port to use, select 1 of the following:
	#define SO_SERIAL_PORT Serial
	//#define SO_SERIAL_PORT Serial1
	//#define SO_SERIAL_PORT Serial2
	//#define SO_SERIAL_PORT Serial3

	//! The BAUD rate to log at, default is 115200	
	#define SO_SERIAL_PORT_SPEED 115200
#endif


//! Enable the ITG 3200 Poller
#define ENABLE_ITG3200_POLLER
#ifdef ENABLE_ITG3200_POLLER
	//! The I2C Address of the itg3200
	#define ITG3200_ADDRESS 0x69
#endif




//! Enable the LIS331 Poller
#define ENABLE_LIS331_POLLER
#ifdef ENABLE_LIS331_POLLER
  #include <Wire.h>
	/**
	 * The Slave ADdress (SAD) associated to the LIS331HH is 001100xb. SDO/SA0 pad 
	 * can be used to modify less significant bit of the device address. If SA0 pad 
	 * is connected to voltage supply, LSb is ‘1’ (address 0011001b) else if SA0 pad 
	 * is connected to ground, LSb value is ‘0’ (address 0011000b). This solution 
	 * permits to connect and address two different accelerometers to the same I2C 
	 * lines.
	 * 		
	 * Bus address is either 24 or 25 
	 */
	#define LIS_BUS_ADDRESS 25
#endif



void f_output();
void log();

#ifdef ENABLE_GPS_POLLER
class NMEA{
	String getField(int field);
	char sumMsg(String &message);
	String readSentence;
	String activeSentence;
	int state;
	public:
		bool addChar(char c);
		bool validFix();
		String getTime();
		char fixType();
		String getLatitude();
		String getLongitude();
		String getSpeed();
		String getCourse();
		String getDate();
};


bool NMEA::addChar(char c)
{
	if (c=='$'){
		NMEA::readSentence="";
		return false;
	}

	if (c=='\r' or c=='\n'){
		if (!NMEA::readSentence.startsWith("GPRMC")){
			return false;
		}
		char msgsum[3];
		char actsum[3];
		// Check if the sentence 
		if (NMEA::readSentence.length()>3 
			&& NMEA::readSentence.charAt(NMEA::readSentence.length()-3)=='*')
		{
			// Grab the checksum from the back of the message.
			msgsum[0]=NMEA::readSentence.charAt(NMEA::readSentence.length()-2);
			msgsum[1]=NMEA::readSentence.charAt(NMEA::readSentence.length()-1);
			// Truncate the message at the * so only the raw data remains
			NMEA::readSentence=NMEA::readSentence.substring(0,NMEA::readSentence.length()-3);

			// Sum the message
			char s;
			s=sumMsg(readSentence);
			sprintf(actsum, "%02X",s);
	
			// Compare the two sums.
			if (msgsum[0]==actsum[0] && msgsum[1]==actsum[1]){
				// Checksums match, its a valid gprmc sentence.
				activeSentence=NMEA::readSentence;
				return true;
			}
		}
	}
	else{
		readSentence+=c;
	}
	return false;
}

bool NMEA::validFix(){
	if (activeSentence.length()>0 && fixType() == 'A'){
		return true;
	}
	else
	{
		return false;
	}
}

String NMEA::getTime(){
	return getField(1);
}
char NMEA::fixType(){
	return getField(2).charAt(0);	
}

String NMEA::getLatitude()
{
	return getField(3)+getField(4);
}

String NMEA::getLongitude(){
	return getField(5)+getField(6);

}
String NMEA::getSpeed(){
	return getField(7);
}

String NMEA::getCourse(){
	return getField(8);
}
String NMEA::getDate(){
	return getField(9);
}







String NMEA::getField(int field){
	byte start=0;
	byte end=0;

	byte i=0;
	while( i < field   ){
		start=activeSentence.indexOf(',',start);
		i++;
		if (start!=0){
			start++;
		}
	}
	end=activeSentence.indexOf(',',start);

	return activeSentence.substring(start,end);
}




		

	

//! Returns the checksum of the message.
char NMEA::sumMsg(String &message){
//    debug(TRACE, "GPSPoller::sumMsg - Entering Function");

    byte i=0;
    char checksum=0;
    byte l=message.length();

    for (i=0;i<l;i++){
        checksum=checksum ^ message.charAt(i);
    }
    return checksum;

}


#endif

//! Enable the SD Output Module
#define ENABLE_SD_OUTPUT
#ifdef ENABLE_SD_OUTPUT
	//! The pin that the output LED is connected to, this lights up when 
	//! the SD card module has found a card, and is writing data to it.
	#define SD_ACTIVE_PIN 8 
#endif



#ifdef  ENABLE_GPS_POLLER

#ifndef GPS_SERIAL_DEV 
	#error GPS_SERIAL_DEV must be set to a valid arduino serial port.
#endif
#ifndef GPS_SERIAL_DEV_SPEED
	#error GPS_SERIAL_DEV_SPEED not set.
#endif
#endif

#ifdef ENABLE_GPS_POLLER
NMEA n;

#endif


#ifdef ENABLE_SD_OUTPUT

#include "SD.h"

#ifndef SD_ACTIVE_PIN
		#error Define SD_ACTIVE_PIN to the pin that will go high when the SD module is writing to a file.
	#endif




static bool sd_active;
static File SD_File;





#endif







#ifdef ENABLE_ITG3200_POLLER




class ITG3200
{
  public:
    static const float GYRO_SENSITIVITY = 14.375;
    static const byte GYRO_RESET=0x80;
    static const byte GYRO_SLEEP=0x40;
    static const byte GYRO_STBY_X=0x20;
    static const byte GYRO_STBY_Y=0x10;
    static const byte GYRO_STBY_Z=0x08;
    static const byte GYRO_CLK_INT=0x0;
    static const byte GYRO_CLK_X=0x1;
    static const byte GYRO_CLK_Y=0x2;
    static const byte GYRO_CLK_Z=0x3;
    static const byte GYRO_CLK_EXT32=0x4;
    static const byte GYRO_CLK_EXT19=0x5;
    
    static const byte GYRO_X=0x1;
    static const byte GYRO_Y=0x2;
    static const byte GYRO_Z=0x4;
    
    static const byte GYRO_ADDR_0=0x68;
    static const byte GYRO_ADDR_1=0x69;
    
   
    static const byte GYRO_FS_2000=0x18;
    static const byte GYRO_LPF_256_8K=0x0;
    static const byte GYRO_LPF_188_1K=0x1;
    static const byte GYRO_LPF_98_1K=0x2;
    static const byte GYRO_LPF_42_1K=0x3;
    static const byte GYRO_LPF_20_1K=0x4;
    static const byte GYRO_LPF_10_1K=0x5;
    static const byte GYRO_LPF_5_1K=0x6; // 5Hz LPF
    
    static const byte GYRO_INT_ACTLO=0x80; // INT logic level = Active Low
    static const byte GYRO_INT_ACTHI=0x00; // INT logic level = Active High
    static const byte GYRO_INT_OPEND=0x40; // INT drive type = Open Drain
    static const byte GYRO_INT_PUSHP=0x00; // INT drive type = Push Pull
    static const byte GYRO_INT_LATCH=0x20; // INT Latch mode = until interrupt is cleared
    static const byte GYRO_INT_PULSE=0x00; // INT Latch mode = 50us pulse
    static const byte GYRO_INT_CLRRD=0x10; // INT clear method = any register ready
    static const byte GYRO_INT_STSRD=0x00; // INT clear method = status reg. read
    static const byte GYRO_INT_RDYEN=0x04; // INT Ready Enable
    static const byte GYRO_INT_RAWEN=0x01; // INT Raw Data Ready Enable 
    
    ITG3200();
    void begin(int gyro_address);
    void begin(int gyro_address, byte pwr_mgm, byte fs_lpf, byte smplrt_div, byte int_cfg);
    void reset();
    void sleep();
    void standBy(byte axis);
    void wake();
    void setInterruptConfig(byte config);
    byte getInterruptConfig();
    bool isInterruptRawDataReady();
    bool isInterruptReady();
    float getX();
    float getY();
    float getZ();
    float getTemperature();
    byte getAddress();
    void setAddress(byte newAddress);
    void setClockSource(byte clockSource);
    

  private:
    static const byte GYRO_REG_WHOAMI=0x00;
    static const byte GYRO_REG_SMPLRT_DIV=0x15;
    static const byte GYRO_REG_DLPF_FS=0x16;
    static const byte GYRO_REG_INT_CFG=0x17;
    static const byte GYRO_REG_INT_STS=0X1A;

    static const byte GYRO_REG_TEMP_H=0x1B;
    static const byte GYRO_REG_TEMP_L=0x1C;
    static const byte GYRO_REG_X_H=0x1D;
    static const byte GYRO_REG_X_L=0x1E;
    static const byte GYRO_REG_Y_H=0x1F;
    static const byte GYRO_REG_Y_L=0x20;
    static const byte GYRO_REG_Z_H=0x21;
    static const byte GYRO_REG_Z_L=0x22;
    static const byte GYRO_REG_PWR_MGM=0x3E;

    static const byte GYRO_INT_READY=0x04; // Enable interrupt when device is ready
    static const byte GYRO_INT_DATA=0x01; // Enable interrupt when data is available
    
    static const float GYRO_TEMP_SENSITIVITY = 280.0;
    static const int GYRO_TEMP_OFFSET = 13200;
    static const float GYRO_TEMP_OFFSET_CELSIUS = 35.0;
    int _gyro_address;
    void write(byte reg, byte val);
    byte read(byte reg);
};


ITG3200::ITG3200()
{
  Wire.begin();
};

void ITG3200::begin(int address)
{
  _gyro_address = address;
  write(GYRO_REG_PWR_MGM, 0);
  write(GYRO_REG_SMPLRT_DIV, 0xFF);
  write(GYRO_REG_DLPF_FS, 0x1E);
  write(GYRO_REG_INT_CFG, 0);
}

void ITG3200::begin(int address,byte pwr_mgm, byte fs_lpf, byte smplrt_div, byte int_cfg) {
  // Power Management
  _gyro_address = address;
  write(GYRO_REG_PWR_MGM, pwr_mgm);
  
  // Sample rate divider
  //_smplrt_div = smplrt_div;
  write(GYRO_REG_SMPLRT_DIV, smplrt_div);
  
  //Frequency select and digital low pass filter
  //_fs_lpf = fs_lpf;
  write(GYRO_REG_DLPF_FS, 0x1F & fs_lpf);
  
  //Interrupt configuration
  //_int_cfg = int_cfg;
  write(GYRO_REG_INT_CFG, 0xF5 & int_cfg);
}

float ITG3200::getX() {
  return (float)(read(GYRO_REG_X_L) | read(GYRO_REG_X_H)<<8)/GYRO_SENSITIVITY; 
}

float ITG3200::getY() {
  return (float)(read(GYRO_REG_Y_L) | read(GYRO_REG_Y_H)<<8)/GYRO_SENSITIVITY; 
}

float ITG3200::getZ() {
  return (float)(read(GYRO_REG_Z_L) | read(GYRO_REG_Z_H)<<8)/GYRO_SENSITIVITY; 
}

float ITG3200::getTemperature(){
  return (((float)((read(GYRO_REG_TEMP_L) | read(GYRO_REG_TEMP_H)<<8) + GYRO_TEMP_OFFSET))/GYRO_TEMP_SENSITIVITY) + GYRO_TEMP_OFFSET_CELSIUS;
}

void ITG3200::reset() {
  write(GYRO_REG_PWR_MGM, GYRO_RESET);
}

void ITG3200::sleep() {
  byte t = read(GYRO_REG_PWR_MGM);
  write(GYRO_REG_PWR_MGM, t | GYRO_SLEEP);
}

void ITG3200::wake(){
  byte t = read(GYRO_REG_PWR_MGM);
  write(GYRO_REG_PWR_MGM, t & ~GYRO_SLEEP);
}

void ITG3200::standBy(byte axis) {
  byte t = read(GYRO_REG_PWR_MGM);
  write(GYRO_REG_PWR_MGM, t & ~axis);
}

byte ITG3200::getAddress()
{
  return read(GYRO_REG_WHOAMI);
}

void ITG3200::setAddress(byte newAddress)
{
  write(GYRO_REG_WHOAMI, newAddress);
}

void ITG3200::setInterruptConfig(byte config)
{
  // bit 3 and 1 must be zero
  write(GYRO_REG_INT_CFG, 0xF5 & config);
}

bool ITG3200::isInterruptRawDataReady()
{
  byte result = read(GYRO_REG_INT_STS);
  return (result & GYRO_INT_DATA) == GYRO_INT_DATA;
}

bool ITG3200::isInterruptReady()
{
  byte result = read(GYRO_REG_INT_STS);
  return (result & GYRO_INT_READY) == GYRO_INT_READY;
}

byte ITG3200::getInterruptConfig()
{
  return read(GYRO_REG_INT_CFG);
}

void ITG3200::setClockSource(byte clockSource)
{
  if (clockSource >= 6) // 6 and 7 are reserved
    return;
  write(GYRO_REG_PWR_MGM, 0xF8 & clockSource);
}

void ITG3200::write(byte reg, byte val) {
  Wire.beginTransmission(_gyro_address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

byte ITG3200::read(byte reg) {
  Wire.beginTransmission(_gyro_address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.beginTransmission(_gyro_address);
  Wire.requestFrom(_gyro_address,  1);
  while (Wire.available() ==0) {}; // block unil data is available
  byte buf = Wire.read();
  Wire.endTransmission();
  return buf;
}



#ifndef ITG3200_ADDRESS
	#error ITG3200_ADDRESS must be defined.
#endif
	static ITG3200 gyro;

#endif


#ifdef ENABLE_LIS331_POLLER
#ifndef LIS_BUS_ADDRESS
  #error LIS_BUS_ADDRESS must be set to the address of the LIS331 device
#endif

#define LR_MAX_TRIES 12


#define LR_CTRL_REG1 0x20
#define LR_CTRL_REG2 0x21
#define LR_CTRL_REG3 0x22
#define LR_CTRL_REG4 0x23
#define LR_CTRL_REG5 0x24
#define LR_HP_FILTER_RESET 0x25
#define LR_REFERENCE 0x26
#define LR_STATUS_REG 0x27
#define LR_OUT_X_L 0x28
#define LR_OUT_X_H 0x29
#define LR_OUT_Y_L 0x2A
#define LR_OUT_Y_H 0x2B
#define LR_OUT_Z_L 0x2C
#define LR_OUT_Z_H 0x2D
#define LR_INT1_CFG 0x30
#define LR_INT1_SOURCE 0x31
#define LR_INT1_THS 0x32
#define LR_INT1_DURATION 0x33
#define LR_INT2_CFG 0x34
#define LR_INT2_SOURCE 0x35
#define LR_INT2_THS 0x36
#define LR_INT2_DURATION 0x37


// Power Modes

#define LR_POWER_OFF B00000000
#define LR_POWER_NORM B00100000 
#define LR_POWER_LOW1 B01000000 
#define LR_POWER_LOW2 B01100000 
#define LR_POWER_LOW3 B10000000 

// Data Rates

#define LR_DATA_RATE_50 B00000000
#define LR_DATA_RATE_100 B00001000
#define LR_DATA_RATE_400 B00010000
#define LR_DATA_RATE_1000 B00011000


// Enable and disable channel.

#define LR_Z_ENABLE B00000100
#define LR_Z_DISABLE B00000000
#define LR_Y_ENABLE B00000010
#define LR_Y_DISABLE B00000000
#define LR_X_ENABLE B00000001
#define LR_X_DISABLE B00000000

class LIS331
{
	bool writeReg(byte addr, byte val);
	bool readReg(byte addr, byte *val);
	bool getBit(byte b, byte bit);
	public:
		LIS331();
		int i2cAddress;
		bool statusHasOverrun();
		bool statusHasZOverrun();
		bool statusHasYOverrun();
		bool statusHasXOverrun();
		bool statusHasDataAvailable();
		bool statusHasZDataAvailable();
		bool statusHasYDataAvailable();
		bool statusHasXDataAvailable();
		byte getPowerStatus();
		bool setPowerStatus(byte status);
		byte getDataRate();
		bool setDataRate(byte rate);
		byte getZEnable();
		byte getYEnable();
		byte getXEnable();
		bool setXEnable(bool state);
		bool setYEnable(bool state);
		bool setZEnable(bool state);
		bool getXValue(int16_t *val);
		bool getZValue(int16_t *val);
		bool getYValue(int16_t *val);
};
LIS331::LIS331(){
	i2cAddress=25;
	}


bool LIS331::readReg(byte addr, byte *val){
  Wire.beginTransmission(i2cAddress);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(i2cAddress,1);
  int timeouts=0;
  while(!Wire.available() && timeouts++<=LR_MAX_TRIES){
    delay(10);
  }
  if (Wire.available()){
		*val=Wire.read();
		return true;
  }else{
	Serial.println("FAIL");
    return false;
  }
}
bool LIS331::writeReg(byte addr, byte val){
  Wire.beginTransmission(i2cAddress);
  Wire.write(addr);
  Wire.write(val);
  Wire.endTransmission();
  return true;
}

  bool LIS331::getBit(byte b,byte bit){
	b<<=(bit);
	if (b>=127){
		return true;
	}
	return false;
  }


  bool LIS331::statusHasOverrun(){
	byte status;
	if (readReg(LR_STATUS_REG, &status)){
		return getBit(status,0);
	}
	return false;
  }

  
  bool LIS331::statusHasZOverrun(){
	byte status;
	if (readReg(LR_STATUS_REG, &status)){
		return getBit(status,1);
	}
	return false;
  }
  bool LIS331::statusHasYOverrun(){
	byte status;
	if (readReg(LR_STATUS_REG, &status)){
		return getBit(status,2);
	}
	return false;
  }
  bool LIS331::statusHasXOverrun(){
	byte status;
	if (readReg(LR_STATUS_REG, &status)){
		return getBit(status,3);
	}
	return false;
  }


  bool LIS331::statusHasDataAvailable(){
	byte status;
	if (readReg(LR_STATUS_REG, &status)){
		return getBit(status,4);
	}
	return false;
  }

  bool LIS331::statusHasZDataAvailable(){
	byte status;
	if (readReg(LR_STATUS_REG, &status)){
		return getBit(status,5);
	}
	return false;
  }
  bool LIS331::statusHasYDataAvailable(){
	byte status;
	if (readReg(LR_STATUS_REG, &status)){
		return getBit(status,6);
	}
	return false;
  }
  bool LIS331::statusHasXDataAvailable(){
	byte status;
	if (readReg(LR_STATUS_REG, &status)){
		return getBit(status,7);
	}
	return false;
  }

	byte LIS331::getPowerStatus(){
		byte state;
		if (readReg(LR_CTRL_REG1, &state)){
			// shift 5 bits to the right
			state=state>>5;
			// shift 5 bits to the left
			state=state<<5;
			// the result is that the right 5 bits are now zero
			// this means that you can compare the output with the constants defined above.
			return state;
	    }
		return 0;
	}

	bool LIS331::setPowerStatus(byte power){
		byte setting;
		if (readReg(LR_CTRL_REG1, &setting)){
			// drop the power bits by leftshifting by 3
			setting=setting<<3;
			// move the rest of the settings back to normal
			setting=setting>>3;
			setting=setting|power;
			return writeReg(LR_CTRL_REG1,setting);
		}
		return false;
	}
	byte LIS331::getDataRate(){
		byte rate;
		if (readReg(LR_CTRL_REG1, &rate)){
			rate=rate<<3;
			rate=rate>>6;
			rate=rate<<3;
			return rate;
	    }
		return 0;
	}
	bool LIS331::setDataRate(byte rate){
		byte setting;
		if (readReg(LR_CTRL_REG1, &setting)){
			// drop the power bits by leftshifting by 3
			setting=setting<<6;
			// move the rest of the settings back to normal
			setting=setting>>6;
			setting=setting|rate;

			byte power;
			power=getPowerStatus();
			setting=setting|power;

			return writeReg(LR_CTRL_REG1,setting);
		}
		return false;
		}


	byte LIS331::getZEnable(){
		byte reg;
		if (readReg(LR_CTRL_REG1, &reg)){
			if (getBit(reg,5)){
				return LR_Z_ENABLE;
			}else{
				return LR_Z_DISABLE;
			}
		}
		return 0;
	}

	byte LIS331::getYEnable(){
		byte reg;
		if (readReg(LR_CTRL_REG1, &reg)){
			if (getBit(reg,6)){
				return LR_Z_ENABLE;
			}else{
				return LR_Z_DISABLE;
			}
		}
		return 0;
	}

	byte LIS331::getXEnable(){
		byte reg;
		if (readReg(LR_CTRL_REG1, &reg)){
			if (getBit(reg,7)){
				return LR_Z_ENABLE;
			}else{
				return LR_Z_DISABLE;
			}
		}
		return 0;
	}


bool LIS331::setZEnable(bool state){
	byte setting;
	if (readReg(LR_CTRL_REG1, &setting)){
		setting &= ~(1<<2);
		if (state){
			setting |= 1<<2;
		}
		return writeReg(LR_CTRL_REG1,setting);
	}
	return false;
}
	


bool LIS331::setYEnable(bool state){
	byte setting;
	if (readReg(LR_CTRL_REG1, &setting)){
		setting &= ~(1<<1);
		if (state){
			setting |= 1<<1;
		}
		return writeReg(LR_CTRL_REG1,setting);
	}
	return false;
}
	


bool LIS331::setXEnable(bool state){
	byte setting;
	if (readReg(LR_CTRL_REG1, &setting)){
		setting &= ~(1<<0);
		if (state){
			setting |= 1<<0;
		}
		return writeReg(LR_CTRL_REG1,setting);
	}
	return false;
}

bool LIS331::getZValue(int16_t *val){
	byte high;
	byte low;
	if (!readReg(LR_OUT_Z_L, &low)){
		return false;
	}
	if (!readReg(LR_OUT_Z_H, &high)){
		return false;
	}
	*val=(low|(high << 8));
	return true;
}


	
	


bool LIS331::getYValue(int16_t *val){
	byte high;
	byte low;
	if (!readReg(LR_OUT_Y_L, &low)){
		return false;
	}
	if (!readReg(LR_OUT_Y_H, &high)){
		return false;
	}
	*val=(low|(high << 8));
	return true;
}


	
	


bool LIS331::getXValue(int16_t *val){
	byte high;
	byte low;
	if (!readReg(LR_OUT_X_L, &low)){
		return false;
	}
	if (!readReg(LR_OUT_X_H, &high)){
		return false;
	}
	*val=(low|(high << 8));
	return true;
}



#endif



/**
 * Messages are sent from pollers to the logging system, the message contains
 * everything that is required to be logged.  Each time a metric is required 
 * to be logged, a message is passed to Logger::log().
 *
 * The message has a toCSV method which is used by each logger to get the
 * correct fields, this format is defined here:
 *
 * http://code.google.com/p/loguino/wiki/DataFormat
 */
class Message
{
    public:
		/**
		 * Time is the time in milliseconds, this is initialized automatically
		 * to the current (up)time by the initializer, however if required
		 * it can be set to whatever value is needed.
		 */
		unsigned long time;
		/**
		 * Namespace is a string used to denote where the message came from.
		 * The loguino convention is to use a period to denote subsystems.
		 * For example:
		 *  Accellerometer.LIS331.XAxis
		 */
		String nameSpace;
		/**
		 * Freeform string denoting the units of the value, for example
		 * MPH or Hz.
		 */
		String units;
		/**
		 * Stringified value of the metric, such as "On", 100, or "Mode A"
		 */
		String value;
        Message();
        String toCSV();
};
/*
 * Initializes the object by setting the default values of the attributes.
 * time is set to the current uptime, nameSpace and units are set to "Unset" 
 * and the value is set to 0.
 */
Message::Message(){
    time=millis();
    nameSpace="Unset";
    units="Unset";
    value="0";
}

/**
 * Returns a comma delimited string containing the message data.
 */
String Message::toCSV(){
    String CSV;
    CSV=String(time);
    CSV+=",";
    CSV+=nameSpace;
    CSV+=",";
    CSV+=value;
    CSV+=",";
    CSV+=units;
    CSV+=",";
    return CSV;
}



#ifdef ENABLE_ANALOG_POLLER
	#ifndef ANALOG_PINS
		#error ANALOG_PINS is not defined.
	#endif

#endif

Message m;

#ifdef ENABLE_LIS331_POLLER
LIS331 lis;
bool lis_active;
bool lis_timeouts;
int16_t lis_val;

#endif


void flush_output(){
#ifdef ENABLE_SERIAL_OUTPUT
  SO_SERIAL_PORT.flush();
#endif
#ifdef ENABLE_SD_OUTPUT
if (sd_active){
  SD_File.flush();
    }

#endif
  
  
}

#ifdef ENABLE_DUMMY_POLLER
static int dummy_called;
#endif
#ifdef ENABLE_ANALOG_POLLER
int anpins[]={ANALOG_PINS};
int annumPins;

#endif


void loop(){
        m.time=millis();
#ifdef ENABLE_DUMMY_POLLER
	m.units="Times";
	m.nameSpace="Dummy.TimesCalled";
	m.value=String(dummy_called++);
        log();
    
        m.units="Milliseconds";
	m.nameSpace="Dummy.Uptime";
	m.value=String(millis());
	log();
#endif

#ifdef ENABLE_ANALOG_POLLER
m.units="Volts*5/1023";
        int i;
	for (i=0; i<annumPins; i++){
		m.nameSpace=String("AnalogInput.Pin")+String(anpins[i]);
		m.value=String(analogRead(anpins[i]));
		log();
	}
#endif
#ifdef ENABLE_GPS_POLLER
while(GPS_SERIAL_DEV.available()){
		if (n.addChar(GPS_SERIAL_DEV.read())){
			if (n.validFix()){

				m.units="Degrees";
				m.nameSpace="GPS.Course";
				m.value=n.getCourse();
				log();
	
				m.units="Knots";
				m.nameSpace="GPS.Speed";
				m.value=n.getSpeed();
				log();
	
				m.units="Latitude";
				m.nameSpace="GPS.Latitude";
				m.value=n.getLatitude();
				log();
	
				m.units="Longitude";
				m.nameSpace="GPS.Longitude";
				m.value=n.getLongitude();
				log();

				m.units="Date";
				m.nameSpace="GPS.Date";
				m.value=n.getDate();
                                log();
	
				m.units="Time";
				m.nameSpace="GPS.Time";
				m.value=n.getTime();
                                log();
				
        		}
		}
	}
#endif

#ifdef ENABLE_DIGITAL_POLLER

	// pins - array of pin numbers to check.
	int dpins[]={DIGITAL_PINS};
	// numdPins - total number of pins to check.
	int numdPins=sizeof(dpins)/sizeof(int);
	// m - message object containing each message.
	m.units="Bool";
	int di;			    
	// For each pin, set the nameSpace, the value, then send the 
	// message.
	for (di=0;di<numdPins;di++){
		m.nameSpace=String("DigitalInput.Pin")+String(dpins[i]);
		m.value=digitalRead(dpins[i]) ? "HIGH":"LOW";
              log();
	}

#endif

#ifdef ENABLE_LIS331_POLLER
m.units="mG";
	lis.getXValue(&lis_val);
	m.nameSpace="Accelerometer.LIS331.X";
	m.value=String(int(lis_val));
        log();
	lis.getYValue(&lis_val);
	m.nameSpace="Accelerometer.LIS331.Y";
	m.value=String(int(lis_val));
        log();
	lis.getZValue(&lis_val);
	m.nameSpace="Accelerometer.LIS331.Z";
	m.value=String(int(lis_val));
        log();

#endif
#ifdef ENABLE_ITG3200_POLLER
	m.units="Degrees*1000/Second";
	m.nameSpace="Gyro.ITG3200.X";
	m.value=String(int(gyro.getX()*1000));
	log();

	m.nameSpace="Gyro.ITG3200.Y";
	m.value=String(int(gyro.getY()*1000));
	log();

	m.nameSpace="Gyro.ITG3200.Z";
	m.value=String(int(gyro.getZ()*1000));
	log();

	m.units="Degrees Celsius*1000";
	m.nameSpace="Gyro.ITG3200.Temperature";
	m.value=String(int(gyro.getTemperature()*1000));
	log();


#endif



}






int num_messages;


void setup(){
  num_messages=0;
#ifdef ENABLE_SERIAL_OUTPUT
  SO_SERIAL_PORT.begin(SO_SERIAL_PORT_SPEED);
#endif

#ifdef ENABLE_DUMMY_POLLER
dummy_called=0;
#endif

#ifdef ENABLE_ANALOG_POLLER
	#ifndef NO_ANALOG_EXTERN_REF
                annumPins=sizeof(anpins)/sizeof(int);
		analogReference(EXTERNAL);
	#endif

#endif

#ifdef ENABLE_LIS331_POLLER

lis.setPowerStatus(LR_POWER_NORM);
lis.setXEnable(true);
lis.setYEnable(true);
lis.setZEnable(true);



#endif



#ifdef ENABLE_ITG3200_POLLER
gyro.begin(ITG3200_ADDRESS);

#endif


#ifdef ENABLE_DIGITAL_POLLER
	// pins - an array of pin numbers to check the state of
	int dpins[]={DIGITAL_PINS};
	// invPins - an array of pin numbers that should be held high
	int invdPins[]={INVERT_DIGITAL_PINS};
	// numPins - The number of pins in total.
	int numdPins=sizeof(dpins)/sizeof(int);
	// numIPins - the number of inverted pins.
	int numdIPins=sizeof(invdPins)/sizeof(int);

	// Iterate through each pin, set it to an input pin.
	// Hold the pin LOW.
	int i;
	for ( i=0; i<numdPins; i++){
		pinMode(dpins[i],INPUT);
		digitalWrite(dpins[i], LOW);
	}

	// For each pin marked inverted, hold it HIGH.
	for(i=0; i<numdIPins; i++){
		digitalWrite(invdPins[i], HIGH);
	}
#endif
#ifdef ENABLE_GPS_POLLER


GPS_SERIAL_DEV.begin(GPS_SERIAL_DEV_SPEED);
#endif



#ifdef ENABLE_SD_OUTPUT
sd_active=true;
char sd_fname[13];
Serial.println("Starting");

pinMode(10, OUTPUT);
sd_active=SD.begin(4);

if (sd_active){
Serial.println("Looking for filename");
    // Counter for filename
  byte sd_i=0;
  sprintf(sd_fname, "%08d.log",i);
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
#endif

}



void log(){
  num_messages++;
  if (num_messages>500){
    flush_output();
    num_messages=0;
  }
  
#ifdef ENABLE_SERIAL_OUTPUT
  SO_SERIAL_PORT.println(m.toCSV());
#endif  
#ifdef ENABLE_SD_OUTPUT
if (sd_active){
    SD_File.println(m.toCSV());
}
#endif
}
