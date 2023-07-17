/**
 * @file CRSFforArduino.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief CRSF for Arduino facilitates the use of ExpressLRS RC receivers in Arduino projects.
 * @version 0.4.0
 * @date 2023-07-17
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU General Public License v3.0
 * This source file is a part of the CRSF for Arduino library.
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

#include "CRSFforArduino.h"
#if defined(ARDUINO) && defined(PLATFORMIO)
#include "CompatibilityTable.h"
#else
#include "lib/CompatibilityTable/CompatibilityTable.h"
#endif

CompatibilityTable CT = CompatibilityTable();

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

#ifdef USE_DMA
volatile bool _dmaTransferDone = false;
#endif

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

    /* Check at runtime if the devboard is compatible with CRSF for Arduino. */
    const char *devboardName = CT.getDevboardName();
#ifdef CRSF_DEBUG
    Serial.print("[CRSF for Arduino | INFO] Devboard Name: ");
    Serial.println(devboardName);
#endif

    // Check if the devboard is compatible.
    if (CT.isDevboardCompatible(devboardName) != true)
    {
#ifdef CRSF_DEBUG
        Serial.println("[CRSF for Arduino | ERROR] Devboard is not compatible with CRSF for Arduino.");
#endif
        // Return false to indicate that the devboard is not compatible.
        return false;
    }

    /* CRSF is 420000 baud 8-bit data, no parity, 1 stop bit. */
    _serial->begin(420000, SERIAL_8N1);
    _serial->setTimeout(10);

    _packetReceived = false;

    memset(_crsfFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
    memset(_crsfRcChannelsPackedFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
    memset(_channels, 0, sizeof(_channels));

#ifdef USE_DMA
#if defined(ARDUINO_ARCH_SAMD)
    Sercom *_sercom = _getSercom();
#endif

    /* Configure the DMA. */
    _dmaSerialRx.setTrigger(SERCOM3_DMAC_ID_RX);
    _dmaSerialRx.setAction(DMA_TRIGGER_ACTON_BEAT);
    _dmaStatus = _dmaSerialRx.allocate();
    if (_dmaStatus != DMA_STATUS_OK)
    {
        return false;
    }

    /* Configure the DMA descriptor. */
    _dmaSerialRxDescriptor = _dmaSerialRx.addDescriptor(
#if defined(ARDUINO_ARCH_SAMD)
        (void *)(&_sercom->USART.DATA.reg),
#endif
        _crsfFrame.raw,
        CRSF_FRAME_SIZE_MAX,
        DMA_BEAT_SIZE_BYTE,
        false,
        true);

    if (_dmaSerialRxDescriptor == NULL)
    {
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
        return false;
    }
#endif

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
#ifdef USE_DMA
    if (_dmaTransferDone == true)
    {
        _dmaTransferDone = false;
#else
    while (_serial->available() > 0)
    {
        _serial->readBytes(_crsfFrame.raw, CRSF_FRAME_SIZE_MAX);
#endif

        const int fullFrameLength = _crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;
        const uint8_t crc = _crsfFrameCRC();

        if (crc == _crsfFrame.raw[fullFrameLength - 1])
        {
            // Check if the packet is a CRSF frame.
            if (_crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER)
            {

                // Check if the packet is a CRSF RC frame.
                if (_crsfFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
                {
                    // Read the RC channels.
                    memcpy(&_crsfRcChannelsPackedFrame, &_crsfFrame, CRSF_FRAME_SIZE_MAX);

                    // Set the packet received flag.
                    _packetReceived = true;
                }
            }
        }

        // Clear the buffer.
        memset(_crsfFrame.raw, 0, CRSF_FRAME_SIZE_MAX);

#ifdef USE_DMA
        // Restart the DMA.
        _dmaStatus = _dmaSerialRx.startJob();
        if (_dmaStatus != DMA_STATUS_OK)
        {
            return false;
        }
#endif

        // Return true to indicate that the packet was received.
        return true;
#ifndef USE_DMA
    }

    // Return false to indicate that the packet was not received.
    return false;
#else
    }

    else
    {
        // Return false to indicate that the packet was not received.
        return false;
    }
#endif
}

bool CRSFforArduino::packetReceived()
{
    return _packetReceived;
}

uint16_t CRSFforArduino::getChannel(uint8_t channel)
{
    const __crsf_rcChannelsPacked_t *rcChannelsPacked = (__crsf_rcChannelsPacked_t *)&_crsfRcChannelsPackedFrame.frame.payload;

    // Unpack the RC channels.
    _channels[RC_CHANNEL_ROLL] = rcChannelsPacked->channel0;
    _channels[RC_CHANNEL_PITCH] = rcChannelsPacked->channel1;
    _channels[RC_CHANNEL_THROTTLE] = rcChannelsPacked->channel2;
    _channels[RC_CHANNEL_YAW] = rcChannelsPacked->channel3;
    _channels[RC_CHANNEL_AUX1] = rcChannelsPacked->channel4;
    _channels[RC_CHANNEL_AUX2] = rcChannelsPacked->channel5;
    _channels[RC_CHANNEL_AUX3] = rcChannelsPacked->channel6;
    _channels[RC_CHANNEL_AUX4] = rcChannelsPacked->channel7;
    _channels[RC_CHANNEL_AUX5] = rcChannelsPacked->channel8;
    _channels[RC_CHANNEL_AUX6] = rcChannelsPacked->channel9;
    _channels[RC_CHANNEL_AUX7] = rcChannelsPacked->channel10;
    _channels[RC_CHANNEL_AUX8] = rcChannelsPacked->channel11;
    _channels[RC_CHANNEL_AUX9] = rcChannelsPacked->channel12;
    _channels[RC_CHANNEL_AUX10] = rcChannelsPacked->channel13;
    _channels[RC_CHANNEL_AUX11] = rcChannelsPacked->channel14;
    _channels[RC_CHANNEL_AUX12] = rcChannelsPacked->channel15;

    // Return the requested channel.
    return _channels[(channel - 1) % RC_CHANNEL_COUNT];
}

uint16_t CRSFforArduino::rcToUs(uint16_t rc)
{
    /**
     * @brief Converts a RC value to microseconds.
     * min RC = 172 (988us)
     * mid RC = 992 (1500us)
     * max RC = 1811 (2012us)
     * scale factor = (2012 - 988) / (1811 - 172) = 0.62477120195241
     * offset = 988 - 172 * 0.62477120195241 = 880.53935326418548
     *
     */
    return (uint16_t)((rc * 0.62477120195241F) + 881);
}

uint8_t CRSFforArduino::_crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (crc & 0x80)
        {
            crc = (crc << 1) ^ 0xD5;
        }
        else
        {
            crc <<= 1;
        }
    }
    return crc;
}

uint8_t CRSFforArduino::_crsfFrameCRC()
{
    // CRC includes the packet type and payload.
    uint8_t crc = _crc8_dvb_s2(0, _crsfFrame.frame.type);
    for (uint8_t i = 0; i < _crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; i++)
    {
        crc = _crc8_dvb_s2(crc, _crsfFrame.frame.payload[i]);
    }
    return crc;
}

#if defined(ARDUINO_ARCH_SAMD)
Sercom *CRSFforArduino::_getSercom()
{
    Sercom *sercom = NULL;

    /* Get the SERCOM instance for the current UART.
    This adds compatibility with most development boards on the market today. */

#if USB_VID == 0x239A
    // Adafruit devboards

#if defined(__SAMD21G18A__)
    // Devboards that use the SAMD21G18A chip.

#if USB_PID == 0x800B || USB_PID == 0x801B || USB_PID == 0x800F || USB_PID == 0x8013
    // Adafruit Feather M0, Feather M0 Express, ItsyBitsy M0 & Metro M0 Express.

/* Seeed Studio XIAO SAMD21. */
#elif defined(SEEED_XIAO_M0)
    sercom = SERCOM4;

/* Arduino Zero. */
#elif defined(ARDUINO_SAMD_ZERO)
    sercom = SERCOM0;
#endif

#elif defined(__SAMD51G19A__)
    // Devboards that use the SAMD51G19A chip.

#if USB_PID == 0x802B
    // Adafruit ItsyBitsy M4 Express.

    sercom = SERCOM3;
#endif

#elif defined(__SAMD51J19A__)
    // Devboards that use the SAMD51J19A chip.

#if USB_PID == 0x8031
    // Adafruit Feather M4 Express.

    sercom = SERCOM5;
#elif USB_PID == 0x8037 || USB_PID == 0x8020
    // Adafruit Metro M4 Airlift Lite & Metro M4 Express.

    sercom = SERCOM3;
#endif

#elif defined(__SAMD51P20A__)
    // Devboards that use the SAMD51P20A chip.

#if USB_PID == 0x8020
    // Adafruit Grand Central M4.

    sercom = SERCOM0;
#endif

#elif defined(__SAME51J19A__)
    // Devboards that use the SAME51J19A chip.

#if USB_PID == 0x80CD
    // Adafruit Feather M4 CAN.

    sercom = SERCOM5;
#endif

#endif

#elif USB_VID == 0x2341
    // Arduino devboards

#if defined(__SAMD21G18A__)
    // Devboards that use the SAMD21G18A chip.

    // All Arduino MKR boards use the same SERCOM instance.
#if USB_PID == 0x8050 || USB_PID == 0x8052 || USB_PID == 0x8055 || USB_PID == 0x8056 || USB_PID == 0x8053 || USB_PID == 0x8059 || USB_PID == 0x8054 || USB_PID == 0x804F
    // Arduino MKR FOX 1200, MKR GSM 1400, MKR NB 1500, MKR Vidor 4000, MKR WAN 1300, MKR WAN 1310, MKR WIFI 1010 & MKR ZERO.

    sercom = SERCOM5;

#elif USB_PID == 0x804D
    // Arduino Zero.

    sercom = SERCOM0;
#endif
#endif
#endif

    return sercom;
}
#endif

#ifdef USE_DMA
void _dmaTransferDoneCallback(Adafruit_ZeroDMA *dma)
{
    (void)dma;

    /* Set the DMA Transfer Done flag. */
    _dmaTransferDone = true;
}
#endif
