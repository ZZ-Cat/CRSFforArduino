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
    _bufferIndex = 0;

    memset(_buffer, 0, sizeof(_buffer));
    memset(_channels, 0, sizeof(_channels));

    /* Configure the DMA. */
    _dmaSerialRx.setTrigger(SERCOM5_DMAC_ID_RX);
    _dmaSerialRx.setAction(DMA_TRIGGER_ACTON_BEAT);
    _dmaStatus = _dmaSerialRx.allocate();
    if (_dmaStatus != DMA_STATUS_OK)
    {
        return false;
    }

    /* Configure the DMA descriptor. */
    _dmaSerialRxDescriptor = _dmaSerialRx.addDescriptor(
        (void *)(&SERCOM5->USART.DATA.reg),
        _buffer,
        sizeof(_buffer),
        DMA_BEAT_SIZE_BYTE,
        false,
        true);
    
    if (_dmaSerialRxDescriptor == NULL)
    {
        return false;
    }

    _dmaSerialRx.loop(true);

    /* Configure the DMA callback. */
    _dmaSerialRx.setCallback(_dmaTransferDoneCallback);

    /* Start the DMA. */
    _dmaStatus = _dmaSerialRx.startJob();
    if (_dmaStatus != DMA_STATUS_OK)
    {
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
    // Debug.
    // Serial.println("DMA transfer done.");

    /* Set the DMA Transfer Done flag. */
    _dmaTransferDone = true;
}
