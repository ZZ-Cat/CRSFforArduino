/**
 * @file main.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This is the main development file for CRSF for Arduino.
 *
 * @copyright Copyright (c) 2024, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU Affero General Public License v3.0
 * This source file is a part of the CRSF for Arduino library.
 * CRSF for Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CRSF for Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with CRSF for Arduino.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */

#include "Arduino.h"
#include "CRSFforArduino.hpp"

CRSFforArduino *crsf = nullptr;

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

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

    crsf->setRcChannelsCallback(onReceiveRcChannels);
}

void loop()
{
    crsf->update();
}

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    static unsigned long lastPrint = millis();
    if (millis() - lastPrint >= 100)
    {
        lastPrint = millis();

        static bool initialised = false;
        static bool lastFailSafe = false;
        if (rcChannels->failsafe != lastFailSafe || !initialised)
        {
            initialised = true;
            lastFailSafe = rcChannels->failsafe;
            Serial.print("FailSafe: ");
            Serial.println(lastFailSafe ? "Active" : "Inactive");
        }

        if (rcChannels->failsafe == false)
        {
            Serial.print("RC Channels <A: ");
            Serial.print(crsf->rcToUs(rcChannels->value[0]));
            Serial.print(", E: ");
            Serial.print(crsf->rcToUs(rcChannels->value[1]));
            Serial.print(", T: ");
            Serial.print(crsf->rcToUs(rcChannels->value[2]));
            Serial.print(", R: ");
            Serial.print(crsf->rcToUs(rcChannels->value[3]));
            Serial.print(", Aux1: ");
            Serial.print(crsf->rcToUs(rcChannels->value[4]));
            Serial.print(", Aux2: ");
            Serial.print(crsf->rcToUs(rcChannels->value[5]));
            Serial.print(", Aux3: ");
            Serial.print(crsf->rcToUs(rcChannels->value[6]));
            Serial.print(", Aux4: ");
            Serial.print(crsf->rcToUs(rcChannels->value[7]));
            Serial.println(">");
        }
    }
}
