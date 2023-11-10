/**
 * @file main_flight_modes.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief Demonstrates the use of CRSF for Arduino's flight mode functionality.
 * @version 0.5.0
 * @date 2023-11-1
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU General Public License v3.0
 * This [file type] is a part of the CRSF for Arduino library.
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
#if defined(ARDUINO) && defined(PLATFORMIO)
#include "Arduino.h"

#include "CRSFforArduino.hpp"

#define FLIGHT_MODE_ARM_CHANNEL 5 // Set FLIGHT_MODE_ARM_CHANNEL to the channel that you want to use to simulate arming your drone.
#define FLIGHT_MODE_ARM_MIN     1000
#define FLIGHT_MODE_ARM_MAX     1500

#define FLIGHT_MODE_ACRO_CHANNEL 8 // Set FLIGHT_MODE_ACRO_CHANNEL to the channel that you want to use to simulate acro mode.
#define FLIGHT_MODE_ACRO_MIN     900
#define FLIGHT_MODE_ACRO_MAX     1300

#define FLIGHT_MODE_ANGLE_CHANNEL 8 // Set FLIGHT_MODE_ANGLE_CHANNEL to the channel that you want to use to simulate angle mode.
#define FLIGHT_MODE_ANGLE_MIN     1300
#define FLIGHT_MODE_ANGLE_MAX     1700

#define FLIGHT_MODE_HORIZON_CHANNEL 8 // Set FLIGHT_MODE_HORIZON_CHANNEL to the channel that you want to use to simulate horizon mode.
#define FLIGHT_MODE_HORIZON_MIN     1700
#define FLIGHT_MODE_HORIZON_MAX     2100

#define SERIAL_RX_PIN 0 // Set SERIAL_RX_PIN to the pin that the CRSF receiver's TX pin is connected to.
#define SERIAL_TX_PIN 1 // Set SERIAL_TX_PIN to the pin that the CRSF receiver's RX pin is connected to.

CRSFforArduino crsf = CRSFforArduino(SERIAL_RX_PIN, SERIAL_TX_PIN);

void onFlightModeUpdate(serialReceiver::flightModeId_t);

void setup()
{
    // Initialise the Serial Port and wait for it to open.
    Serial.begin(115200);
    // while (!Serial)
    // {
    //     ;
    // }

    Serial.println("Flight Modes Example");

    // Initialise CRSF for Arduino.
    if (!crsf.begin())
    {
        Serial.println("CRSF initialisation failed!");
        while (1)
        {
            ;
        }
    }

    // Set flight modes.
    if (!crsf.setFlightMode(serialReceiver::FLIGHT_MODE_DISARMED, FLIGHT_MODE_ARM_CHANNEL, FLIGHT_MODE_ARM_MIN, FLIGHT_MODE_ARM_MAX))
    {
        Serial.println("Failed to set \"DISARMED\" flight mode!");
        while (1)
        {
            ;
        }
    }

    if (!crsf.setFlightMode(serialReceiver::FLIGHT_MODE_ACRO, FLIGHT_MODE_ACRO_CHANNEL, FLIGHT_MODE_ACRO_MIN, FLIGHT_MODE_ACRO_MAX))
    {
        Serial.println("Failed to set \"ACRO\" flight mode!");
        while (1)
        {
            ;
        }
    }

    if (!crsf.setFlightMode(serialReceiver::FLIGHT_MODE_ANGLE, FLIGHT_MODE_ANGLE_CHANNEL, FLIGHT_MODE_ANGLE_MIN, FLIGHT_MODE_ANGLE_MAX))
    {
        Serial.println("Failed to set \"ANGLE\" flight mode!");
        while (1)
        {
            ;
        }
    }

    if (!crsf.setFlightMode(serialReceiver::FLIGHT_MODE_HORIZON, FLIGHT_MODE_HORIZON_CHANNEL, FLIGHT_MODE_HORIZON_MIN, FLIGHT_MODE_HORIZON_MAX))
    {
        Serial.println("Failed to set \"HORIZON\" flight mode!");
        while (1)
        {
            ;
        }
    }

    // Set flight mode callback.
    crsf.setFlightModeCallback(onFlightModeUpdate);

    Serial.print("\tArm channel: ");
    Serial.println(FLIGHT_MODE_ARM_CHANNEL);
    Serial.print("\tFlight Modes Channel: ");
    Serial.println(FLIGHT_MODE_ACRO_CHANNEL);
}

void loop()
{
    // Update CRSF for Arduino.
    crsf.update();
}

void onFlightModeUpdate(serialReceiver::flightModeId_t flightMode)
{
    static serialReceiver::flightModeId_t lastFlightMode = serialReceiver::FLIGHT_MODE_DISARMED;

    if (flightMode != lastFlightMode)
    {
        Serial.print("Flight Mode: ");
        switch (flightMode)
        {
            case serialReceiver::FLIGHT_MODE_DISARMED:
                Serial.println("Disarmed");
                break;
            case serialReceiver::FLIGHT_MODE_ACRO:
                Serial.println("Acro");
                break;
            case serialReceiver::FLIGHT_MODE_WAIT:
                Serial.println("Wait for GPS Lock");
                break;
            case serialReceiver::FLIGHT_MODE_FAILSAFE:
                Serial.println("Failsafe");
                break;
            case serialReceiver::FLIGHT_MODE_GPS_RESCUE:
                Serial.println("GPS Rescue");
                break;
            case serialReceiver::FLIGHT_MODE_PASSTHROUGH:
                Serial.println("Passthrough");
                break;
            case serialReceiver::FLIGHT_MODE_ANGLE:
                Serial.println("Angle");
                break;
            case serialReceiver::FLIGHT_MODE_HORIZON:
                Serial.println("Horizon");
                break;
            case serialReceiver::FLIGHT_MODE_AIRMODE:
                Serial.println("Airmode");
                break;
            default:
                Serial.println("Unknown");
                break;
        }
        lastFlightMode = flightMode;

        crsf.telemetryWriteFlightMode(flightMode);
    }
}

#endif
