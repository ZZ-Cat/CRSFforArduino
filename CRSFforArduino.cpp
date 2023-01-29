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

volatile bool _dmaTransferDone = false;

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
bool CRSFforArduino::begin()
{
    /* CRSF is 420000 baud 8-bit data, no parity, 1 stop bit. */
    _serial->begin(420000, SERIAL_8N1);
    _serial->setTimeout(10);

    _packetReceived = false;

    memset(_crsfFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
    memset(_channels, 0, sizeof(_channels));

    /* Configure the DMA. */
    _dmaSerialRx.setTrigger(SERCOM5_DMAC_ID_RX);
    _dmaSerialRx.setAction(DMA_TRIGGER_ACTON_BEAT);
    _dmaStatus = _dmaSerialRx.allocate();
    if (_dmaStatus != DMA_STATUS_OK)
    {

// Debug.
#ifdef CRSF_DEBUG
        Serial.println("DMA allocation failed.");
#endif

        return false;
    }

    /* Configure the DMA descriptor. */
    _dmaSerialRxDescriptor = _dmaSerialRx.addDescriptor(
        (void *)(&SERCOM5->USART.DATA.reg),
        _crsfFrame.raw,
        CRSF_FRAME_SIZE_MAX,
        DMA_BEAT_SIZE_BYTE,
        false,
        true);

    if (_dmaSerialRxDescriptor == NULL)
    {

// Debug.
#ifdef CRSF_DEBUG
        Serial.println("DMA descriptor allocation failed.");
#endif

        return false;
    }

    // Disabled because it is non-functional for some reason.
    // _dmaSerialRx.loop(true);

    /* Configure the DMA callback. */
    _dmaSerialRx.setCallback(_dmaTransferDoneCallback);

    /* Start the DMA. */
    _dmaStatus = _dmaSerialRx.startJob();
    if (_dmaStatus != DMA_STATUS_OK)
    {

// Debug.
#ifdef CRSF_DEBUG
        Serial.println("DMA start failed.");
#endif

        return false;
    }

    return true;
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
bool CRSFforArduino::update()
{
    if (_dmaTransferDone == true)
    {
        _dmaTransferDone = false;

        // Check if the packet is a CRSF frame.
        if (_crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER)
        {
            uint8_t payloadSize = _crsfFrame.frame.frameLength - 2;

            // Check if the packet is a CRSF RC frame.
            if (_crsfFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
            {
                // Read the RC channels.
                // 11 bits per channel, 16 channels = 176 bits = 22 bytes.
                _channels[0] = (uint16_t)((_crsfFrame.frame.payload[0] | _crsfFrame.frame.payload[1] << 8) & 0x07FF);
                _channels[1] = (uint16_t)((_crsfFrame.frame.payload[1] >> 3 | _crsfFrame.frame.payload[2] << 5) & 0x07FF);
                _channels[2] = (uint16_t)((_crsfFrame.frame.payload[2] >> 6 | _crsfFrame.frame.payload[3] << 2 | _crsfFrame.frame.payload[4] << 10) & 0x07FF);
                _channels[3] = (uint16_t)((_crsfFrame.frame.payload[4] >> 1 | _crsfFrame.frame.payload[5] << 7) & 0x07FF);
                _channels[4] = (uint16_t)((_crsfFrame.frame.payload[5] >> 4 | _crsfFrame.frame.payload[6] << 4) & 0x07FF);
                _channels[5] = (uint16_t)((_crsfFrame.frame.payload[6] >> 7 | _crsfFrame.frame.payload[7] << 1 | _crsfFrame.frame.payload[8] << 9) & 0x07FF);
                _channels[6] = (uint16_t)((_crsfFrame.frame.payload[8] >> 2 | _crsfFrame.frame.payload[9] << 6) & 0x07FF);
                _channels[7] = (uint16_t)((_crsfFrame.frame.payload[9] >> 5 | _crsfFrame.frame.payload[10] << 3) & 0x07FF);
                _channels[8] = (uint16_t)((_crsfFrame.frame.payload[11] | _crsfFrame.frame.payload[12] << 8) & 0x07FF);
                _channels[9] = (uint16_t)((_crsfFrame.frame.payload[12] >> 3 | _crsfFrame.frame.payload[13] << 5) & 0x07FF);
                _channels[10] = (uint16_t)((_crsfFrame.frame.payload[13] >> 6 | _crsfFrame.frame.payload[14] << 2 | _crsfFrame.frame.payload[15] << 10) & 0x07FF);
                _channels[11] = (uint16_t)((_crsfFrame.frame.payload[15] >> 1 | _crsfFrame.frame.payload[16] << 7) & 0x07FF);
                _channels[12] = (uint16_t)((_crsfFrame.frame.payload[16] >> 4 | _crsfFrame.frame.payload[17] << 4) & 0x07FF);
                _channels[13] = (uint16_t)((_crsfFrame.frame.payload[17] >> 7 | _crsfFrame.frame.payload[18] << 1 | _crsfFrame.frame.payload[19] << 9) & 0x07FF);
                _channels[14] = (uint16_t)((_crsfFrame.frame.payload[19] >> 2 | _crsfFrame.frame.payload[20] << 6) & 0x07FF);
                _channels[15] = (uint16_t)((_crsfFrame.frame.payload[20] >> 5 | _crsfFrame.frame.payload[21] << 3) & 0x07FF);

                // Set the packet received flag.
                _packetReceived = true;
            }
        }

        /*
        // Check if the packet is a CRSF frame.
        if (_buffer[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER)
        {
            uint8_t payloadSize = _buffer[1] - 2;

            // Check if the packet is a CRSF RC frame.
            if (_buffer[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
            {
                // Read the RC channels.
                for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++)
                {
                    _channels[i] = _buffer[3 + (i * 2)] | (_buffer[4 + (i * 2)] << 8);
                }

                // Set the packet received flag.
                _packetReceived = true;
            }
        }
        */

        // Clear the buffer.
        memset(_crsfFrame.raw, 0, CRSF_FRAME_SIZE_MAX);

        // Restart the DMA.
        _dmaStatus = _dmaSerialRx.startJob();
        if (_dmaStatus != DMA_STATUS_OK)
        {

// Debug.
#ifdef CRSF_DEBUG
            Serial.println("DMA restart failed.");
#endif

            return false;
        }

        // Return true to indicate that the packet was received.
        return true;
    }

    else
    {
        // Return false to indicate that the packet was not received.
        return false;
    }
}

bool CRSFforArduino::packetReceived()
{
    return _packetReceived;
}

uint16_t CRSFforArduino::getChannel(uint8_t channel)
{
    return _channels[channel];
}

void _dmaTransferDoneCallback(Adafruit_ZeroDMA *dma)
{
    (void)dma;

    // Debug.
#ifdef CRSF_DEBUG
    Serial.println("DMA transfer done.");
#endif

    /* Set the DMA Transfer Done flag. */
    _dmaTransferDone = true;
}
