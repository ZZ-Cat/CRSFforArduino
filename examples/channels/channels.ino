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
 * This example sketch was written for the following hardware:
 * - Adafruit Metro M4 Express
 * - RadioMaster TX16S Max (EdgeTX 2.8.0 or later)
 * - TBS Crossfire Nano Diversity Receiver
 * - TBS Crossfire Micro TX
 * 
 * @section Dependencies
 * 
 * This example sketch depends on the following libraries:
 * - Arduino.h
 * - CRSFforArduino.h
 * 
 * @section Quick Start
 * 
 * 1. Connect the CRSF receiver to the Metro M4 Express using the following pinout:
 *   - CRSF TX (CH 1) to Metro M4 Express TX (pin 1)
 *   - CRSF RX (CH 2) to Metro M4 Express RX (pin 0)
 *   - CRSF GND to Metro M4 Express GND
 *   - CRSF VCC to Metro M4 Express 5V
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
#include "wiring_private.h"

Uart crsfUart = Uart(&sercom5, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);
CRSFforArduino crsf = CRSFforArduino(&crsfUart);

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

    // Route the SERCOM5 peripheral to the pins 0 & 1.
    pinPeripheral(0, PIO_SERCOM_ALT);
    pinPeripheral(1, PIO_SERCOM_ALT);

    // Change the data order to MSB first.
    // This is required for the CRSF protocol.
    // The CTRLA register is enable-protected, so it must be disabled before writing to it.
    SERCOM5->USART.CTRLA.bit.ENABLE = 0;
    while (SERCOM5->USART.SYNCBUSY.bit.ENABLE)
    {
        // Wait for synchronization to complete before proceeding.
        ;
    }
    SERCOM5->USART.CTRLA.bit.DORD = 1;

    // Enable the SERCOM5 peripheral again.
    SERCOM5->USART.CTRLA.bit.ENABLE = 1;
    while (SERCOM5->USART.SYNCBUSY.bit.ENABLE)
    {
        // Wait for synchronization to complete before proceeding.
        ;
    }

    // Change the interrupt priority for SERCOM5.
    NVIC_DisableIRQ(SERCOM5_0_IRQn);
    NVIC_DisableIRQ(SERCOM5_1_IRQn);
    NVIC_DisableIRQ(SERCOM5_2_IRQn);
    NVIC_DisableIRQ(SERCOM5_3_IRQn);
    NVIC_ClearPendingIRQ(SERCOM5_0_IRQn);
    NVIC_ClearPendingIRQ(SERCOM5_1_IRQn);
    NVIC_ClearPendingIRQ(SERCOM5_2_IRQn);
    NVIC_ClearPendingIRQ(SERCOM5_3_IRQn);
    NVIC_SetPriority(SERCOM5_0_IRQn, 0);
    NVIC_SetPriority(SERCOM5_1_IRQn, 0);
    NVIC_SetPriority(SERCOM5_2_IRQn, 0);
    NVIC_SetPriority(SERCOM5_3_IRQn, 0);
    NVIC_EnableIRQ(SERCOM5_0_IRQn);
    NVIC_EnableIRQ(SERCOM5_1_IRQn);
    NVIC_EnableIRQ(SERCOM5_2_IRQn);
    NVIC_EnableIRQ(SERCOM5_3_IRQn);

    // Show the user that the sketch is ready.
    Serial.println("Channels Example");
    delay(1000);
    Serial.println("Ready");
    delay(1000);

}

void loop()
{
    crsf.update();
    if (crsf.packetReceived())
    {
        Serial.print("Channel 1: ");
        Serial.println(crsf.getChannel(1));
        Serial.print("Channel 2: ");
        Serial.println(crsf.getChannel(2));
        Serial.print("Channel 3: ");
        Serial.println(crsf.getChannel(3));
        Serial.print("Channel 4: ");
        Serial.println(crsf.getChannel(4));
        // Serial.print("Channel 5: ");
        // Serial.println(crsf.getChannel(5));
        // Serial.print("Channel 6: ");
        // Serial.println(crsf.getChannel(6));
        // Serial.print("Channel 7: ");
        // Serial.println(crsf.getChannel(7));
        // Serial.print("Channel 8: ");
        // Serial.println(crsf.getChannel(8));
        Serial.println();
    }
}

void SERCOM5_0_Handler()
{
    crsfUart.IrqHandler();
}

void SERCOM5_1_Handler()
{
    crsfUart.IrqHandler();
}

void SERCOM5_2_Handler()
{
    crsfUart.IrqHandler();
}

void SERCOM5_3_Handler()
{
    crsfUart.IrqHandler();
}
