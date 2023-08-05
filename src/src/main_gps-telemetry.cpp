/**
 * @file main_telemetry.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This file demonstrates how to use CRSF for Arduino to send GPS telemetry to an ExpressLRS RC receiver.
 * @version 0.4.0
 * @date 2023-08-01
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
  * EG When you open the "GPS Telemetry" example sketch, the Arduino IDE will link main_telemetry.cpp to it.
  * To work around this, preprocessor directives are used to exclude the main_telemetry.cpp code from your sketch.
  */
#if defined(ARDUINO) && defined(PLATFORMIO)
#include "Arduino.h"

#include "CRSFforArduino.h"

/* Configuration Options. */
#define VIEW_RC_CHANNELS         0 // Set VIEW_RC_CHANNELS to 1 to view the RC channel data in the serial monitor.
#define GENERATE_RANDOM_GPS_DATA 0 // Set GENERATE_RANDOM_GPS_DATA to 1 to generate random GPS telemetry data.
#define SERIAL_RX_PIN            0 // Set SERIAL_RX_PIN to the pin that the CRSF receiver's TX pin is connected to.
#define SERIAL_TX_PIN            1 // Set SERIAL_TX_PIN to the pin that the CRSF receiver's RX pin is connected to.

uint32_t timeNow = 0;

/* Initialise the GPS telemetry data with default values. */
float latitude = -41.18219482686493F; // Latitude is in decimal degrees.
float longitude = 174.9497131419602F; // Longitude is in decimal degrees.
float altitude = 100.0F;              // Altitude is in centimetres.
float speed = 500.0F;                 // Speed is in cm/s
float groundCourse = 275.8F;          // Ground Course is in degrees.
uint8_t satellites = 4;

CRSFforArduino crsf = CRSFforArduino(SERIAL_RX_PIN, SERIAL_TX_PIN);

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

    /* Here, you would normally update the GPS telemetry data with the latest values from your GPS module.
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
    static unsigned long lastUpdate = 0;
    if (timeNow - lastUpdate >= 1000)
    {
        lastUpdate = timeNow;

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
#endif
}

#endif // defined(ARDUINO) && defined(PLATFORMIO)
