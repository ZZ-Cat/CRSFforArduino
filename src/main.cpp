#if defined(ARDUINO) && defined(PLATFORMIO)
#include "new"
#include "Arduino.h"
#include "CRSFforArduino.hpp"

using namespace std;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    CRSFforArduino *crsf = new(nothrow) CRSFforArduino();
    if (crsf == nullptr)
    {
        Serial.println("Failed to allocate memory for CRSFforArduino");
        while (true)
        {
            delay(1000);
        }
    }

    crsf->printTest();

    delete crsf;
    crsf = nullptr;
}

void loop()
{
}
#endif
