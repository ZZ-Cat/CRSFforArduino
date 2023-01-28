/**
 * @file CRSFforArduino.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief Library for CRSF (Crossfire Serial Protocol) for Arduino.
 * @version 0.1.0
 * @date 2023-01-15
 * 
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 * 
 */

#include "CRSFforArduino.h"

/**
 * @par CRSF Protocol
 * 
 * CRSF is a full duplex serial protocol. It is the bus protocol used by both TBS Crossfire & ExpressLRS receivers to
 * communicate with the flight controller.
 * 
 * Each CRSF packet is sent to the flight controller at an interval of 4ms. The flight controller then responds with a
 * packet within every second packet sent by the receiver.
 * 
 * The configuration of the serial port is 420000 baud, 8-bit data, no parity, 1 stop bit.
 * The entire protocol is big endian (MSB first).
 * The maximum packet size is 64 bytes.
 * The time needed per packet is 1.5ms.
 * A 64 byte packet plus a sync byte takes approximately 1.4ms to transmit.
 * 
 * The protocol frame format is as follows:
 * <Device Address><Frame Length><Frame Type><Payload><CRC>
 * 
 * The Device Address is a single byte that identifies the device that is receiving the frame.
 * The Frame Length is a single byte that indicates the length of the frame, including the Frame Type.
 * The Frame Type is a single byte that indicates the what type of frame was received.
 * The Payload is a variable length byte array that contains the data for the frame.
 * The CRC is a single byte that is the CRC-8-DVB-S2 of the entire frame, including the Device Address, Frame Length, and Frame
 * Type.
 * 
 */

/**
 * @brief Construct a new CRSFforArduino object.
 * 
 * @param serial A pointer to the HardwareSerial object to use.
 * 
 */
CRSFforArduino::CRSFforArduino(HardwareSerial *serial)
{
    _serial = serial;
}

/**
 * @brief Destroy the CRSFforArduino object
 * 
 */
CRSFforArduino::~CRSFforArduino()
{
    _serial = NULL;
}

/**
 * @brief Opens the hardware serial port for use with CRSF.
 * 
 */
void CRSFforArduino::begin()
{
    /* CRSF is 420000 baud 8-bit data, no parity, 1 stop bit. */
    _serial->begin(420000, SERIAL_8N1);
    _serial->setTimeout(10);

    _packetReceived = false;
    _bufferIndex = 0;

    memset(_buffer, 0, sizeof(_buffer));
    memset(_channels, 0, sizeof(_channels));
}

/**
 * @brief Closes the hardware serial port.
 * 
 */
void CRSFforArduino::end()
{
    _serial->end();
}

/**
 * @brief Updates the CRSFforArduino object.
 * It is recommended to call this function in the main loop of your sketch.
 * 
 */
void CRSFforArduino::update()
{
    do
    {
        uint8_t c = _serial->read();
    } while (_serial->available() > 0);

    /*
    // Clear the packet received flag.
    _packetReceived = false;

    // Read the serial port.
    uint8_t length = _serial->available();
    if (length > 0)
    {
        uint8_t buffer[length];

        _serial->readBytes(buffer, length);

        // Check if the packet is a CRSF frame.
        if (buffer[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER)
        {
            // Check if the packet is a CRSF RC frame.
            if (buffer[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
            {
                // Read the RC channels.
                for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++)
                {
                    _channels[i] = buffer[3 + (i * 2)] | (buffer[4 + (i * 2)] << 8);
                }

                // Set the packet received flag.
                _packetReceived = true;
            }
        }
    }
    */
}

bool CRSFforArduino::packetReceived()
{
    return _packetReceived;
}

uint16_t CRSFforArduino::getChannel(uint8_t channel)
{
    return _channels[channel];
}
