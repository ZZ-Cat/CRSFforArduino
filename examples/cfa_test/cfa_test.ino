#include "CRSFforArduino.hpp"

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    CRSFforArduino *crsf = new CRSFforArduino();

    delete crsf;
    crsf = nullptr;
}

void loop()
{
}
