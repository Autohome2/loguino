#include "Debug.h"
#include "WProgram.h"
#include "NMEA.h"

NMEA n;
void setup(){
	Serial2.begin(4800);
	Serial.begin(115200);
}

void loop(){
	while (Serial2.available()){
		if (n.addChar(Serial2.read())){
			if (n.validFix()){
				Serial.println(n.getLatitude());
			}
		}
	}

}
