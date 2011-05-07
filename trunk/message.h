#ifndef Message_h
#define Message_h

#include "WProgram.h"


class Message
{
    unsigned long time;
    String nameSpace;
    String units;
    String value;
    public:
        Message();
        String toCSV();
};

#endif

