#ifndef OutputBase_h
#define OutputBase_h

#include "WProgram.h"

#include "output.h"
#include "message.h"


class OutputBase
{
    bool active;
    public:
        OutputBase();
        bool begin();
        bool logMessage(Message msg);
        bool flush();
};

#endif

