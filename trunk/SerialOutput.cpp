#include "WProgram.h"
#include "SerialOutput.h"
#include "message.h"

#define SERIAL Serial1


SerialOutput::SerialOutput(){
    active=false;
}

bool SerialOutput::flush(){
    if (active){
        SERIAL.flush();
    }
    return active;
}


bool SerialOutput::begin()
{
    SERIAL.begin(9600);
    active=true;
    return active;
}


bool SerialOutput::logMessage(Message msg){
    if (!active){
        return false;
    }
    SERIAL.println(msg.toCSV());
}


