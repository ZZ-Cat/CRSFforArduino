/**
 * @file channels.ino
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This example sketch shows how to receive RC channels from a CRSF receiver using the CRSFforArduino library.
 * @version 0.1.0
 * @date 2023-01-15
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section Introduction
 *
 * This example sketch shows how to receive RC channels from a CRSF receiver using the CRSFforArduino library.
 *
 * @section Hardware
 *
 * This example sketch was tested with the following hardware:
 * - Adafruit Metro M4 Express
 * - RadioMaster TX16S Max Edition (EdgeTX 2.8.0 or later)
 * - RadioMaster RP3 ExpressLRS 2.4GHz Nano Receiver (ExpressLRS 3.2.0 or later)
 * - RadioMaster Ranger ExpressLRS 2.4GHz Transmitter Module (ExpressLRS 3.2.0 or later)
 * - TBS Crossfire Micro TX Module
 * - TBS Crossfire Nano Diversity Receiver
 *
 * @section Dependencies
 *
 * This example sketch depends on the following libraries:
 * - Arduino.h
 * - CRSFforArduino.h
 *
 * @section Quick Start
 *
 * 1. Connect the receiver to the Metro M4 Express using the following pinout:
 *   - TX pin to Metro M4 Express RX (Pin 0).
 *   - RX pin to Metro M4 Express TX (Pin 1).
 *   - GND pin to Metro M4 Express GND.
 *   - 5V pin to Metro M4 Express 5V.
 * 2. Connect the Metro M4 Express to your computer using a USB cable.
 * 3. Select the Metro M4 Express board and the correct port in the Arduino IDE.
 *   - Tools > Board > Adafruit Metro M4 (SAMD51)
 *   - Tools > Port > COM# (where # is the port number)
 * 4. Open the CRSFforArduino library example sketch channels.ino.
 *   - File > Examples > CRSFforArduino > channels
 * 5. Upload the sketch to the Metro M4 Express.
 *   - Sketch > Upload
 * 6. Open the Serial Monitor.
 *   - Tools > Serial Monitor
 * 7. Set the Serial Monitor baud rate to 115200.
 * 8. Set the Serial Monitor line ending to "Both NL & CR".
 * 9. Turn on your transmitter.
 *
 * @section Binding (Optional & only needed for the first time)
 *
 * 1. Turn on your transmitter.
 * 2. On your transmitter, open up the TBS Agent Lite app.
 *   - Radio Settings > Tools > TBS Agent Lite
 * 3. In TBS Agent Lite, select the Micro TX & enable Binding Mode.
 *   - XF Micro TX > Bind > Execute
 * 4. Power on the Nano Diversity Receiver.
 *   - If you have already powered on the receiver, power it off and then back on again.
 *   - You do not need to hold the bind button on the receiver.
 *   - Binding will begin automatically.
 *   - The Status LED will turn solid green when the receiver is bound.
 *   - The Status LED on the Micro TX will turn solid green when the receiver is bound.
 * 5. Close TBS Agent Lite.
 *
 * @section Output
 *
 * The Serial Monitor will display the following output:
 * @todo Add output
 *
 * @section Troubleshooting
 *
 * @todo Add troubleshooting
 *
 */

#include "CRSFforArduino.h"

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

    // Show the user that the sketch is ready.
    Serial.println("Channels Example");
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
}
