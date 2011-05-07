#include "message.h"
#include "output.h"
#include "SD.h"

Output Out;

void setup(){
  Out.begin();
 
}

void loop(){
 Message m;
 Serial.begin(9600);
 Serial.println(m.toCSV()); 
}


