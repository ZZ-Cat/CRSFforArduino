/**
 * @file gps_telemetry.ino
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This example sketch demonstrates how to pass data from a GPS module into CRSF for Arduino & transmit it as telemetry.
 * @version 0.5.0
 * @date 2023-09-17
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
#error "This example sketch is not compatible with PlatformIO. Please use the main_gps-telemetry.cpp example instead."
#elif defined(ARDUINO) && !defined(PLATFORMIO)

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
        for(int i = 1; i <= crsfProtocol::RC_CHANNEL_COUNT; i++){
          //Serial.print("Channel");
          Serial.print(i);
          Serial.print(":");
          Serial.print(crsf.rcToUs(crsf.getChannel(i)));
          Serial.print("\t");
        }
        Serial.println();     
    }
#endif
}

#endif // defined(ARDUINO) && defined(PLATFORMIO)
