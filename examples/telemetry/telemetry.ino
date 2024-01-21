/**
 * @file telemetry.ino
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This demonstrates how to use CRSF for Arduino to send telemetry to your RC transmitter using the CRSF protocol.
 * @version 1.0.0
 * @date 2024-1-20
 * 
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU General Public License v3.0
 * This example sketch is a part of the CRSF for Arduino library.
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
#error "This example sketch is not compatible with PlatformIO. Please use the main_gps-telemetry.cpp example instead."
#elif defined(ARDUINO) && !defined(PLATFORMIO)

#include "CRSFforArduino.hpp"

/* Configuration Options. */
#define VIEW_RC_CHANNELS             0 // Set VIEW_RC_CHANNELS to 1 to view the RC channel data in the serial monitor.
#define USE_SERIAL_PLOTTER           1 // Set USE_SERIAL_PLOTTER to 1 to view this example's data in the Arduino IDE's serial plotter.
#define GENERATE_RANDOM_BATTERY_DATA 0 // Set GENERATE_RANDOM_BATTERY_DATA to 1 to generate random battery sensor telemetry data.
#define GENERATE_RANDOM_GPS_DATA     0 // Set GENERATE_RANDOM_GPS_DATA to 1 to generate random GPS telemetry data.

uint32_t timeNow = 0;

/* Initialise the attitude telemetry with default values. */
int16_t roll = -200; // Roll is in decided degrees (eg -200 = -20.0 degrees).
int16_t pitch = 150; // Pitch is in decided degrees (eg 150 = 15.0 degrees).
uint16_t yaw = 2758; // Yaw is in decided degrees (eg 2758 = 275.8 degrees).

/* Initialise the barometric altitude telemetry with default values. */
uint16_t baroAltitude = 10; // Barometric altitude is in decimetres (eg 10 = 1.0 metres).
int16_t verticalSpeed = 50; // Vertical speed is in centimetres per second (eg 50 = 0.5 metres per second).

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
uint8_t satellites = 4;

const int channelCount = crsfProtocol::RC_CHANNEL_COUNT; // I'm not sure if this is right, but we can always manually put in the number of channels desired

/* The CRSF Protocol only supports up to 16 proportional RC channels.
So, an assert is needed to prevent channelCount from being set to any arbitary number that is higher than 16. */
static_assert(channelCount <= crsfProtocol::RC_CHANNEL_COUNT, "The number of RC channels must be less than or equal to the maximum number of RC channels supported by CRSF.");

CRSFforArduino crsf = CRSFforArduino(&Serial1);

#if USE_SERIAL_PLOTTER == 0 && VIEW_RC_CHANNELS > 0
const char *channelNames[crsfProtocol::RC_CHANNEL_COUNT] = {
    "A", "E", "T", "R", "Aux1", "Aux2", "Aux3", "Aux4", "Aux5", "Aux6", "Aux7", "Aux8", "Aux9", "Aux10", "Aux11", "Aux12"};
#endif

void setup()
{
#if VIEW_RC_CHANNELS > 0 || defined(CRSF_DEBUG) || USE_SERIAL_PLOTTER > 0
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }

#if USE_SERIAL_PLOTTER == 0
    Serial.println("GPS Telemetry Example");
#endif
#endif

    /* Initialise CRSF for Arduino */
    if (!crsf.begin())
    {
#if (VIEW_RC_CHANNELS > 0 || defined(CRSF_DEBUG)) && USE_SERIAL_PLOTTER == 0
        Serial.println("CRSF for Arduino initialization failed!");
#endif
        while (1)
        {
            ;
        }
    }

#if (VIEW_RC_CHANNELS > 0 || defined(CRSF_DEBUG)) && USE_SERIAL_PLOTTER == 0
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
        crsf.telemetryWriteAttitude(roll, pitch, yaw);
    }

    /* Barometric Altitude Telemetry

    Normally, you would read the barometric altitude and vertical speed from a barometric pressure sensor
    connected to your Arduino board.

    For the purposes of this example, we will just update the following with random values:
    - Barometric Altitude
    - Vertical Speed

    These values are updated at a rate of 10 Hz.
    */

    /* Update the barometric altitude telemetry at a rate of 10 Hz. */
    static unsigned long lastBaroAltitudeUpdate = 0;
    if (timeNow - lastBaroAltitudeUpdate >= 100)
    {
        lastBaroAltitudeUpdate = timeNow;

        /* Update the barometric altitude telemetry with the new values. */
        crsf.telemetryWriteBaroAltitude(baroAltitude, verticalSpeed);
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

#if USE_SERIAL_PLOTTER > 0
        Serial.print("Lat:");
        Serial.print(latitude);
        Serial.print("\tLon:");
        Serial.print(longitude);
        Serial.print("\tAlt:");
        Serial.print(altitude);
        Serial.print("\tSpeed:");
        Serial.print(speed);
        Serial.print("\tGroundCourse:");
        Serial.print(groundCourse);
        Serial.print("\tSatellites:");
        Serial.print(satellites);
        Serial.println();
#endif
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
#if USE_SERIAL_PLOTTER == 0
        for (int i = 1; i <= channelCount; i++)
        {
            //Serial.print("Channel");
            Serial.print(i);
            Serial.print(":");
            Serial.print(crsf.rcToUs(crsf.getChannel(i)));
            Serial.print("\t");
        }
        Serial.println();
#else
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
#endif
    }
#endif
}

#endif // defined(ARDUINO) && defined(PLATFORMIO)
