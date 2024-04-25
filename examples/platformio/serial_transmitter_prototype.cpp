#if __has_include("Arduino.h")
#include "Arduino.h"
#define ARDUINO_IS_INCLUDED 1
#else
#define ARDUINO_IS_INCLUDED 0
#endif

#if ARDUINO_IS_INCLUDED == 1
void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }
    Serial.println("Hello, World!");
}

void loop()
{}
#endif
