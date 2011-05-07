#include "WProgram.h"

#include "message.h"
#include "OutputBase.h"


OutputBase::OutputBase(){
  active=false;
}

bool OutputBase::flush(){
   return true;
}


bool OutputBase::begin()
{
  
    return true;
}


bool OutputBase::logMessage(Message msg){
    return true;
}


Output::Output(){
}



