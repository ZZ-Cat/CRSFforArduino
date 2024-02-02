#if defined(ARDUINO) && defined(PLATFORMIO)
#include "Arduino.h"
#include "CRSFforArduino.hpp"

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    CRSFforArduino *crsf = new CRSFforArduino();

    crsf->printTest();

    delete crsf;
    crsf = nullptr;
}

void loop()
{
}
#endif
