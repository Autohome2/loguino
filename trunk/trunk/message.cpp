#include "WProgram.h"
#include "message.h"

Message::Message(){
    time=millis();
    nameSpace="Unset";
    units="Unset";
    value="0";
}


String Message::toCSV(){
    String CSV;
    CSV=String(time);
    CSV.concat(",");
    CSV.concat(nameSpace);
    CSV.concat(",");
    CSV.concat(value);
    CSV.concat(",");
    CSV.concat(units);
    CSV.concat(",");
    return CSV;
}


