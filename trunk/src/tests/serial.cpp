#include <Arduino.h>
#include <ELM327.h>
void setup(){
	Serial3.begin(9600);
	Serial.begin(115200);

}
void loop(){
while(Serial.available()){
	Serial3.write(Serial.read());
}
while (Serial3.available()){
	Serial.write(Serial3.read());
}

}



