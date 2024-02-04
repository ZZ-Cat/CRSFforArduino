#include "CRSFforArduino.hpp"
#define USE_SERIAL_PLOTTER 0

CRSFforArduino *crsf = nullptr;

int rcChannelCount = crsfProtocol::RC_CHANNEL_COUNT;
const char *rcChannelNames[] = {
    "A",
    "E",
    "T",
    "R",
    "Aux1",
    "Aux2",
    "Aux3",
    "Aux4",

    "Aux5",
    "Aux6",
    "Aux7",
    "Aux8",
    "Aux9",
    "Aux10",
    "Aux11",
    "Aux12"
};

void setup()
{
    // Initialise the serial port & wait for the port to open.
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }

    // Initialise CRSF for Arduino.
    crsf = new CRSFforArduino();
    if (!crsf->begin())
    {
        crsf->end();

        delete crsf;
        crsf = nullptr;

        Serial.println("CRSF for Arduino initialisation failed!");
        while (1)
        {
            delay(10);
        }
    }

    rcChannelCount = rcChannelCount > crsfProtocol::RC_CHANNEL_COUNT ? crsfProtocol::RC_CHANNEL_COUNT : rcChannelCount;

    // Show the user that the sketch is ready.
    Serial.println("RC Channels Example");
    delay(1000);
    Serial.println("Ready");
    delay(1000);
}

void loop()
{
    crsf->update();

    /* Print RC channels every 100 ms. */
    unsigned long thisTime = millis();
    static unsigned long lastTime = millis();

    /* Compensate for millis() overflow. */
    if (thisTime < lastTime)
    {
        lastTime = thisTime;
    }

    if (thisTime - lastTime >= 100)
    {
        lastTime = thisTime;
#if USE_SERIAL_PLOTTER > 0
        for (int i = 1; i <= rcChannelCount; i++)
        {
            Serial.print(i);
            Serial.print(":");
            Serial.print(crsf->rcToUs(crsf->getChannel(i)));
            Serial.print("\t");
        }
        Serial.println();
#else
        Serial.print("RC Channels <");
        for (int i = 1; i <= rcChannelCount; i++)
        {
            Serial.print(rcChannelNames[i - 1]);
            Serial.print(": ");
            Serial.print(crsf->rcToUs(crsf->getChannel(i)));

            if (i < rcChannelCount)
            {
                Serial.print(", ");
            }
        }
        Serial.println(">");
#endif
    }
}
