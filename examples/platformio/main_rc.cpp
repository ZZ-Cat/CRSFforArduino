/**
 * @file main_rc.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This file demonstrates the full capabilities of CRSF for Arduino.
 * @version 0.5.0
 * @date 2023-11-1
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU General Public License v3.0
 * This main file is a part of the CRSF for Arduino library.
 * CRSF for Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CRSF for Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CRSF for Arduino.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */

/* The Arduino IDE links main_rc.cpp to any sketch that uses the CRSFforArduino library.
 * EG When you open the "RC Channels" example sketch, the Arduino IDE will link main_rc.cpp to it.
 * To work around this, preprocessor directives are used to exclude the main_rc.cpp code from your sketch.
 */
#if defined(ARDUINO) && defined(PLATFORMIO)
#include "Arduino.h"

#include "CRSFforArduino.hpp"

#define SERIAL_RX_PIN 0 // Set SERIAL_RX_PIN to the pin that the CRSF receiver's TX pin is connected to.
#define SERIAL_TX_PIN 1 // Set SERIAL_TX_PIN to the pin that the CRSF receiver's RX pin is connected to.

CRSFforArduino crsf = CRSFforArduino(SERIAL_RX_PIN, SERIAL_TX_PIN);

void setup()
{
    // Initialize the serial port & wait for the port to open.
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }

    Serial.println("RC Channels Example");

    // Initialize the CRSFforArduino library.
    if (!crsf.begin())
    {
        Serial.println("CRSF for Arduino initialization failed!");
        while (1)
        {
            ;
        }
    }

    // Show the user that the sketch is ready.
    Serial.println("Ready");
    delay(1000);
}

void loop()
{
    crsf.update();

    /* Print RC channels every 100 ms. Do this using the millis() function to avoid blocking the main loop. */
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100)
    {
        lastPrint = millis();
        Serial.print("RC Channels <A: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(1)));
        Serial.print(", E: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(2)));
        Serial.print(", T: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(3)));
        Serial.print(", R: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(4)));
        Serial.print(", Aux1: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(5)));
        Serial.print(", Aux2: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(6)));
        Serial.print(", Aux3: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(7)));
        Serial.print(", Aux4: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(8)));
        Serial.println(">");
    }
}
#endif // defined(ARDUINO) && defined(PLATFORMIO)
