#include "Arduino.h"
#include "CRSFforArduino.hpp"

CRSFforArduino *crsf = nullptr;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    crsf = new CRSFforArduino();

    if (!crsf->begin())
    {
        Serial.println("CRSF for Arduino failed to initialise.");

        delete crsf;
        crsf = nullptr;

        while (1)
        {
            delay(10);
        }
    }
}

void loop()
{
    crsf->update();

    /* Print RC channels every 100 ms. Do this using the millis() function to avoid blocking the main loop. */
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100)
    {
        lastPrint = millis();
        Serial.print("RC Channels <A: ");
        Serial.print(crsf->rcToUs(crsf->getChannel(1)));
        Serial.print(", E: ");
        Serial.print(crsf->rcToUs(crsf->getChannel(2)));
        Serial.print(", T: ");
        Serial.print(crsf->rcToUs(crsf->getChannel(3)));
        Serial.print(", R: ");
        Serial.print(crsf->rcToUs(crsf->getChannel(4)));
        Serial.print(", Aux1: ");
        Serial.print(crsf->rcToUs(crsf->getChannel(5)));
        Serial.print(", Aux2: ");
        Serial.print(crsf->rcToUs(crsf->getChannel(6)));
        Serial.print(", Aux3: ");
        Serial.print(crsf->rcToUs(crsf->getChannel(7)));
        Serial.print(", Aux4: ");
        Serial.print(crsf->rcToUs(crsf->getChannel(8)));
        Serial.println(">");
    }
}
