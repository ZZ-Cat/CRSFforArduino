/**
 * @file main.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This is the main development file for CRSF for Arduino.
 * @version 1.0.0
 * @date 2024-2-5
 *
 * @copyright Copyright (c) 2024, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU General Public License v3.0
 * This source file is a part of the CRSF for Arduino library.
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
