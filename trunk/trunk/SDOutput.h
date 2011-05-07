#ifndef SDOutput_h
#define SDOutput_h


#include "WProgram.h"
#include "OutputBase.h"
#include "message.h"

#include <SD.h>


class SDOutput
{
    bool active;
    File _File;
    public:
        SDOutput();
        bool begin();
        bool logMessage(Message msg);
        bool flush();
};


#endif

