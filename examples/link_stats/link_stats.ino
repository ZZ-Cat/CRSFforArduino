/**
 * @file link_stats.ino
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief Example of how to read link statistics from a receiver.
 * @version 1.0.3
 * @date 2024-7-20
 *
 * @copyright Copyright (c) 2024, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU Affero General Public License v3.0
 * This example is a part of the CRSF for Arduino library.
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

#include "CRSFforArduino.hpp"

/* Tested with the following equipment:
- Controller: RadioMaster TX16S Max Edition Mk1
  - Firmware: EdgeTX 2.10.0 Nightly
  - Lua Script: iNav OpenTX Telemetry Widget 2.2.3
  - Transmitter Module: RadioMaster Ranger
    - Firmware: ExpressLRS 3.3.1
- Receiver: RadioMaster RP3 Diversity
  - Firmware: ExpressLRS 3.3.1
- Development Board: Adafruit Metro M4 Express
  - Board Package: Adafruit SAMD Boards 1.7.5
  - Framework: Arduino 1.8.13
  - Library: CRSF for Arduino 1.0.0
 */

CRSFforArduino *crsf = nullptr;

void onLinkStatisticsUpdate(serialReceiverLayer::link_statistics_t);

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    Serial.println("Link Statistics Example.");

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

    // Set link statistics callback.
    crsf->setLinkStatisticsCallback(onLinkStatisticsUpdate);

    Serial.println("Ready.");
    delay(1000);
}

void loop()
{
    crsf->update();
}

void onLinkStatisticsUpdate(serialReceiverLayer::link_statistics_t linkStatistics)
{
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 200)
    {
        lastPrint = millis();
        Serial.print("Link Statistics: ");
        Serial.print("RSSI: ");
        Serial.print(linkStatistics.rssi);
        Serial.print(", Link Quality: ");
        Serial.print(linkStatistics.lqi);
        Serial.print(", Signal-to-Noise Ratio: ");
        Serial.print(linkStatistics.snr);
        Serial.print(", Transmitter Power: ");
        Serial.println(linkStatistics.tx_power);
    }
}
