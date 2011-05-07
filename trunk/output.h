#ifndef Output_h
#define Output_h


#include "WProgram.h"
#include "message.h"
#include "SDOutput.h"
#include "SerialOutput.h"


class Output
{
    public:
        Output();
        bool begin();
        bool logMessage(Message msg);
        bool flush();
};



#endif


