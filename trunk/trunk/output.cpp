
#include "WProgram.h"
#include "message.h"
#include "output.h"
#include <SD.h>



bool Output::flush(){
   // sd.flush();
   // ser.flush();  
   return true;
}


bool Output::begin()
{
    return true;
}


bool Output::logMessage(Message msg){
   // sd.logMessage();
    //ser.logMessage();
return true;
}

















