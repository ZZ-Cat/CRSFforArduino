/**
 * @file gps_telemetry.ino
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This example sketch shows how to send GPS telemetry data to a CRSF receiver using the CRSFforArduino library.
 * @version 0.1.0
 * @date 2023-02-01
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 */

#include "CRSFforArduino.h"
#include "gps.h"

// Create a GPS object.
const uint32_t GPSRxPin = 4;
const uint32_t GPSTxPin = 5;
Uart Serial2(&sercom4, GPSRxPin, GPSTxPin, SERCOM_RX_PAD_1, UART_TX_PAD_2);
GPS gps = GPS(&Serial2, GPSRxPin, GPSTxPin);

// Create a CRSFforArduino object.
CRSFforArduino crsf = CRSFforArduino(&Serial1);

void setup()
{
    // Initialize the serial port & wait for the port to open.
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }

    // Initialize the CRSFforArduino library.
    crsf.begin();

    // Initialize the GPS sensor.
    gps.begin();

    // Show the user that the sketch is ready.
    Serial.println("GPS Telemetry Example");
    delay(1000);
    Serial.println("Ready");
    delay(1000);
}

void loop()
{
    if (crsf.update())
    {
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

    // Update the GPS sensor.
    if (gps.update())
    {
        // Send the GPS telemetry data to the CRSF receiver.
        crsf.writeGPStelemetry(gps.data.latitude, gps.data.longitude, gps.data.altitude, gps.data.speed, gps.data.heading, gps.data.satellites);
    }
}

void SERCOM4_0_Handler()
{
    Serial2.IrqHandler();
}

void SERCOM4_1_Handler()
{
    Serial2.IrqHandler();
}

void SERCOM4_2_Handler()
{
    Serial2.IrqHandler();
}

void SERCOM4_3_Handler()
{
    Serial2.IrqHandler();
}
