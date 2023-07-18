/**
 * @file channels.ino
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This example sketch shows how to receive RC channels from a CRSF receiver using the CRSF for Arduino library.
 * @version 0.3.3
 * @date 2023-07-18
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
 * PlatformIO will automatically install the following dependencies:
 * - Adafruit ZeroDMA Library
 *
 * @section Quick Start
 *
 * 1. Connect the receiver to the Metro M4 Express using the following pinout:
 *    - TX pin to Metro M4 Express RX (Pin 0).
 *    - RX pin to Metro M4 Express TX (Pin 1).
 *    - GND pin to Metro M4 Express GND.
 *    - 5V pin to Metro M4 Express 5V.
 * 2. Connect the Metro M4 Express to your computer using a USB cable.
 * 3. Build the firmware:
 *    - Method 1: pio run
 *    - Method 2: ctlr+alt+B
 * 4. Flash your Metro M4 Express.
 *    - Method 1: pio run -t upload
 *    - Method 2: ctlr+alt+U
 * 5. Configure the Serial Monitor:
 *    - Port: Select the port that your Metro M4 Express is connected to.
 *    - Baud Rate: 115200
 *    - Line Ending: Both NL & CR
 * 6. Turn on your transmitter.
 * 7. Click "Start Monitoring" on the Serial Monitor.
 *
 * @section Binding (Optional & only needed for the first time)
 *
 * @par ExpressLRS:
 * Before you begin binding, make sure that you have the latest version of ExpressLRS flashed on your transmitter and receiver.
 * When you flash ExpressLRS, make sure that your transmitter and receiver have the same binding phrase.
 * 1. Turn on your transmitter.
 * 2. Power on your ELRS receiver - EG RadioMaster RP3.
 * 3. Binding will begin automatically, and the Status LED will turn solid when the receiver is bound.
 *
 * @par TBS Crossfire:
 * 1. Turn on your transmitter.
 * 2. On your transmitter, open up the TBS Agent Lite app.
 *    - Radio Settings > Tools > TBS Agent Lite
 * 3. In TBS Agent Lite, select the Micro TX & enable Binding Mode.
 *    - XF Micro TX > Bind > Execute
 * 4. Power on the Nano Diversity Receiver.
 *    - If you have already powered on the receiver, power it off and then back on again.
 *    - You do not need to hold the bind button on the receiver.
 *    - Binding will begin automatically.
 *    - The Status LED will turn solid green when the receiver is bound.
 *    - The Status LED on the Micro TX will turn solid green when the receiver is bound.
 * 5. Close TBS Agent Lite.
 *
 * @section Output
 *
 * The Serial Monitor will display the following output:
 * @code
 * Channels Example
 * Ready
 * RC Channels <A: 1500, E: 1500, T: 1500, R: 1500, Aux1: 1500, Aux2: 1500, Aux3: 1500, Aux4: 1500>
 * @endcode
 * The values for each channel will change as you move the sticks on your transmitter.
 * The values will be in microseconds (us).
 * Standard range is 988us to 2012us, with 1500us being the center position.
 * Extended range is 885us to 2115us, with 1500us being the center position.
 *
 */

#if defined(PLATFORMIO)
#error "This example is for the Arduino IDE only."
#endif

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
    Serial.println("RC Channels Example");
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
