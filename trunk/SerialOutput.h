


#ifndef SerialOutput_h
#define SerialOutput_h

#include "WProgram.h"
#include "OutputBase.h"
#include "message.h"




class SerialOutput : public OutputBase
{
    bool active;
    public:
        SerialOutput();
        bool begin();
        bool logMessage(Message msg);
        bool flush();
};

#endif

