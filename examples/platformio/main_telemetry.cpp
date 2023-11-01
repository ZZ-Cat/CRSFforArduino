/**
 * @file main_telemetry.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This file demonstrates how to use CRSF for Arduino to send telemetry to your RC transmitter using the CRSF protocol.
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

/* The Arduino IDE links main_telemetry.cpp to any sketch that uses the CRSFforArduino library.
EG When you open the "telemetry.ino" example sketch, the Arduino IDE will link main_telemetry.cpp to it.
To work around this, preprocessor directives are used to exclude the main_telemetry.cpp code from your sketch. */
#if defined(ARDUINO) && defined(PLATFORMIO)
#include "Arduino.h"

#include "CRSFforArduino.hpp"

/* Configuration Options. */
#define VIEW_RC_CHANNELS             0 // Set VIEW_RC_CHANNELS to 1 to view the RC channel data in the serial monitor.
#define GENERATE_RANDOM_BATTERY_DATA 0 // Set GENERATE_RANDOM_BATTERY_DATA to 1 to generate random battery sensor telemetry data.
#define GENERATE_RANDOM_GPS_DATA     0 // Set GENERATE_RANDOM_GPS_DATA to 1 to generate random GPS telemetry data.
#define SERIAL_RX_PIN                0 // Set SERIAL_RX_PIN to the pin that the CRSF receiver's TX pin is connected to.
#define SERIAL_TX_PIN                1 // Set SERIAL_TX_PIN to the pin that the CRSF receiver's RX pin is connected to.

uint32_t timeNow = 0;

/* Initialise the battery sensor telemetry with default values. */
float batteryVoltage = 385.0F; // Battery voltage is in millivolts (mV * 100).
float batteryCurrent = 150.0F; // Battery current is in milliamps (mA * 10).
uint32_t batteryFuel = 700;    // Battery fuel is in milliamp hours (mAh).
uint8_t batteryPercent = 50;   // Battery percentage remaining is in percent (0 - 100).

/* Initialise the GPS telemetry data with default values. */
float latitude = -41.18219482686493F; // Latitude is in decimal degrees.
float longitude = 174.9497131419602F; // Longitude is in decimal degrees.
float altitude = 100.0F;              // Altitude is in centimetres.
float speed = 500.0F;                 // Speed is in cm/s
float groundCourse = 275.8F;          // Ground Course is in degrees.
uint8_t satellites = 7;               // 7 satellites are in view (implies a 3D fix).

CRSFforArduino crsf = CRSFforArduino(SERIAL_RX_PIN, SERIAL_TX_PIN);

#if VIEW_RC_CHANNELS > 0
const int channelCount = 8;
const char *channelNames[crsfProtocol::RC_CHANNEL_COUNT] = {
    "A", "E", "T", "R", "Aux1", "Aux2", "Aux3", "Aux4", "Aux5", "Aux6", "Aux7", "Aux8", "Aux9", "Aux10", "Aux11", "Aux12"};

static_assert(channelCount <= crsfProtocol::RC_CHANNEL_COUNT, "The number of RC channels must be less than or equal to the maximum number of RC channels supported by CRSF.");
#endif

void setup()
{
#if VIEW_RC_CHANNELS > 0 || defined(CRSF_DEBUG)
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }

    Serial.println("GPS Telemetry Example");
#endif

    /* Initialise CRSF for Arduino */
    if (!crsf.begin())
    {
#if VIEW_RC_CHANNELS > 0 || defined(CRSF_DEBUG)
        Serial.println("CRSF for Arduino initialization failed!");
#endif
        while (1)
        {
            ;
        }
    }

#if VIEW_RC_CHANNELS > 0 || defined(CRSF_DEBUG)
    /* Show the user that the sketch is ready. */
    Serial.println("Ready");
    delay(1000);
#endif
}

void loop()
{

    // Use timeNow to store the current time in milliseconds.
    timeNow = millis();

    /* Attitude Telemetry

    Normally, you would read the raw attitude data from your IMU and convert it to roll, pitch and yaw values.
    For the purposes of this example, we will just update the following with random values:
    - Roll
    - Pitch
    - Yaw

    These values are updated at a rate of 100 Hz.
    */

    /* Update the attitude telemetry at a rate of 100 Hz. */
    static unsigned long lastAttitudeUpdate = 0;
    if (timeNow - lastAttitudeUpdate >= 10)
    {
        lastAttitudeUpdate = timeNow;

        /* Update the attitude telemetry with the new values. */
        crsf.telemetryWriteAttitude(10.0F, 0.0F, 0.0F);
    }

    /* Battery Telemetry

    Normally, you read the battery voltage and current from two analog pins on your Arduino board or from a
    battery sensor connected to your Arduino board.
    For the purposes of this example, we will just update the following with random values:
    - Battery Voltage
    - Battery Current

    These values are updated at a rate of 10 Hz.
    Battery fuel and percentage remaining are calculated from the battery voltage and current values, with
    a simulated battery capacity of 1000 mAh. */

    /* Update the battery sensor telemetry at a rate of 10 Hz. */
    static unsigned long lastBatteryUpdate = 0;
    if (timeNow - lastBatteryUpdate >= 100)
    {
        lastBatteryUpdate = timeNow;

#if GENERATE_RANDOM_BATTERY_DATA > 0
        // Generate random values for the battery sensor telemetry.
        batteryVoltage = random(300, 420);
        batteryCurrent = random(0, 1000);
#endif

        // Calculate the battery fuel and percentage remaining.
        // batteryFuel = (uint32_t)(batteryFuel + (batteryCurrent / 36000));
        // batteryPercent = (uint8_t)(batteryFuel / 10);

        // Update the battery sensor telemetry with the new values.
        crsf.telemetryWriteBattery(batteryVoltage, batteryCurrent, batteryFuel, batteryPercent);
    }

    /* GPS Telemetry

    Normally, you would update the GPS telemetry data with the latest values from your GPS module.
    For the purposes of this example, we will just update the following with random values:
    - Latitude
    - Longitude
    - Altitude
    - Speed
    - Ground Course
    - Satellites

    These values are updated at a rate of 1 Hz.
    Normally, you would update these values at a rate of 5 Hz or higher.
    */

    /* Update the GPS telemetry data at a rate of 1 Hz. */
    static unsigned long lastGpsUpdate = 0;
    if (timeNow - lastGpsUpdate >= 1000)
    {
        lastGpsUpdate = timeNow;

#if GENERATE_RANDOM_GPS_DATA > 0
        // Generate random values for the GPS telemetry data.
        latitude = random(-90, 90);
        longitude = random(-180, 180);
        altitude = random(0, 500000);
        speed = random(0, 6625);
        groundCourse = random(0, 359);
        satellites = random(0, 16);
#endif

        // Update the GPS telemetry data with the new values.
        crsf.telemetryWriteGPS(latitude, longitude, altitude, speed, groundCourse, satellites);
    }

    /* Call crsf.update() in the main loop to process CRSF packets. */
    crsf.update();

/* You can process RC channel data here. */
#if VIEW_RC_CHANNELS > 0
    static unsigned long lastPrint = 0;
    if (timeNow - lastPrint >= 100)
    {
        lastPrint = timeNow;
        Serial.print("RC Channels <");
        for (uint8_t i = 0; i < channelCount; i++)
        {
            Serial.print(channelNames[i]);
            Serial.print(": ");
            Serial.print(crsf.rcToUs(crsf.getChannel(i + 1)));
            if (i < channelCount - 1)
            {
                Serial.print(", ");
            }
        }
        Serial.println(">");
    }
#endif
}

#endif // defined(ARDUINO) && defined(PLATFORMIO)
