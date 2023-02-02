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

#ifdef USE_DMA
Adafruit_ZeroDMA _dmaSerialRx;
Adafruit_ZeroDMA _dmaSerialTx;
volatile bool _dmaSerialRxDone = false;
volatile bool _dmaSerialTxDone = false;
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
    /* CRSF is 420000 baud 8-bit data, no parity, 1 stop bit. */
    _serial->begin(420000, SERIAL_8N1);
    _serial->setTimeout(10);

#if defined(ARDUINO_ARCH_SAMD)
    Sercom *_sercom = _getSercom();

    /* Change the data order to MSB First.
    The CTRLA Register is enable protected, so it needs to be disabled before writing to it.
    The Enable Bit is write syncronised. Therefore, a wait for sync is necessary.
    Then re-enable the peripheral again. */
    _sercom->USART.CTRLA.bit.ENABLE = 0;
    while (_sercom->USART.SYNCBUSY.bit.ENABLE)
        ;
    _sercom->USART.CTRLA.bit.DORD = 1;
    _sercom->USART.CTRLA.bit.ENABLE = 1;
    while (_sercom->USART.SYNCBUSY.bit.ENABLE)
        ;
#endif

    memset(_crsfFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
    memset(_channels, 0, sizeof(_channels));

#ifdef USE_DMA
    /* Configure the DMA. */
    _dmaSerialRx.setTrigger(_getDmaRxId());
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

    /* Configure the DMA callback. */
    _dmaSerialRx.setCallback(_dmaTransferDoneCallback);

#if (CRSF_USE_TELEMETRY > 0)
    /* Configure telemetry. */
    uint8_t index = 0;

#if (CRSF_TELEMETRY_DEVICE_ATTITUDE > 0)
    _crsfFrameSchedule[index++] = CRSF_FRAME_ATTITUDE_INDEX;
#endif

#if (CRSF_TELEMETRY_DEVICE_BATTERY_SENSOR > 0)
    _crsfFrameSchedule[index++] = CRSF_FRAME_BATTERY_SENSOR_INDEX;
#endif

#if (CRSF_TELEMETRY_DEVICE_GPS > 0)
    _crsfFrameSchedule[index++] = CRSF_FRAME_GPS_INDEX;
#endif

    _crsfFrameScheduleCount = index;
#endif

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
 * This function will read the CRSF frame from the serial port and parse it.
 * It will also update the channel data.
 *
 * @return true If the CRSF frame was successfully read and parsed.
 * @return false If the CRSF frame was not successfully read and parsed.
 */
bool CRSFforArduino::update()
{
#ifdef USE_DMA
    if (_dmaSerialRxDone == true)
    {
        _dmaSerialRxDone = false;
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
                }

#if (CRSF_USE_TELEMETRY > 0)
                // Send telemetry data.
                _flagSendTelemetry = true;
#endif
            }
        }

        // Clear the buffer.
        memset(_crsfFrame.raw, 0, CRSF_FRAME_SIZE_MAX);

#if (CRSF_USE_TELEMETRY > 0)
        // Send telemetry data.
        if (_flagSendTelemetry)
        {
            _sendTelemetry();
            _flagSendTelemetry = false;
        }
#endif

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

/**
 * @brief Gets the value of a channel.
 *
 * @param channel The channel number.
 * @return uint16_t The value of the channel.
 *
 */
uint16_t CRSFforArduino::getChannel(uint8_t channel)
{
    return _channels[(channel - 1) % RC_CHANNEL_COUNT];
}

/**
 * @brief Converts an RC value to microseconds.
 *
 * @param rc The RC value to convert.
 * @return uint16_t The converted value in microseconds.
 */
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

#if (CRSF_USE_TELEMETRY > 0 && CRSF_TELEMETRY_DEVICE_GPS > 0)
void CRSFforArduino::setGPS(float latitude, float longitude, float altitude, float speed, float heading, float sats)
{
    _crsfGps.latitude = latitude;
    _crsfGps.longitude = longitude;
    _crsfGps.altitude = altitude;
    _crsfGps.speed = speed;
    _crsfGps.heading = heading;
    _crsfGps.sats = sats;
}
#endif

/**
 * @brief Calculates the CRC for a CRSF frame.
 *
 * @param crc
 * @param a
 * @return uint8_t
 */
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

/**
 * @brief Calculates the CRC for a CRSF frame.
 *
 * @return uint8_t
 */
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

uint8_t CRSFforArduino::_crsfGetCRC(uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        crc = _crc8_dvb_s2(crc, data[i]);
    }
    return crc;
}

#if (CRSF_USE_TELEMETRY > 0)
/**
 * @brief Sends telemetry data.
 * @par CRSF telemetry frames have the following structure:
 * <Device address><Frame length><Frame type><Payload><CRC>
 * - Device address: (uint8_t)
 * - Frame length: Length of the frame in bytes, including frame type. (uint8_t)
 * - Frame type: (uint8_t)
 * - Payload: (uint8_t[])
 * - CRC: (uint8_t)
 *
 */
void CRSFforArduino::_sendTelemetry()
{

    const uint8_t thisFrameToSchedule = _crsfFrameSchedule[_crsfFrameScheduleIndex];

    if ((thisFrameToSchedule & CRSF_FRAME_GPS_INDEX) == CRSF_FRAME_GPS_INDEX)
    {
#if (CRSF_TELEMETRY_DEVICE_GPS > 0)
        _sendTelemetryGPS();
#endif
    }

    // Update the telemetry frame index.
    _crsfFrameScheduleIndex = (_crsfFrameScheduleIndex + 1) % _crsfFrameScheduleCount;
}

#if (CRSF_TELEMETRY_DEVICE_GPS > 0)
/**
 * @brief Sends GPS telemetry data.
 * 
 */
void CRSFforArduino::_sendTelemetryGPS()
{
    /* GPS frame. */
    uint8_t length = CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_GPS_PAYLOAD_SIZE;
    _streamBufferClear();

    // Populate the sync byte.
    _streamBufferPush8u(CRSF_SYNC_BYTE);

    // Populate the payload.
    _streamBufferPush8u(CRSF_FRAME_GPS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC); // Set the frame length.
    _streamBufferPush8u(CRSF_FRAMETYPE_GPS);                                       // Set the frame type.
    _streamBufferPush32sBigEndian(_crsfGps.latitude / 60 * 1E7);                   // Set the latitude.
    _streamBufferPush32sBigEndian(_crsfGps.longitude / 60 * 1E7);                  // Set the longitude.
    _streamBufferPush16uBigEndian(_crsfGps.speed * 1.852 * 10);                    // Set the speed.
    _streamBufferPush16uBigEndian(_crsfGps.heading * 100);                         // Set the heading.
    _streamBufferPush16uBigEndian(_crsfGps.altitude + 1000);                       // Set the altitude.
    _streamBufferPush8u(_crsfGps.sats);                                            // Set the number of satellites.

    // Calculate the CRC.
    // Start at index 2, because the CRC does not include the address and length.
    uint8_t crc = _crsfGetCRC(_streamBuffer + 2, length - 3);

    // Populate the frame.
    _streamBufferPush8u(crc); // Set the CRC.

#if (CRSF_DEBUG_TELEMETRY > 0)
    Serial.print("GPS: ");
    for (uint8_t i = 0; i < length; i++)
    {
        Serial.print(_streamBuffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
#endif

    // Send the frame.
    _serial->write(_streamBuffer, length);
}
#endif

void CRSFforArduino::_streamBufferClear()
{
    // Clear the buffer.
    memset(_streamBuffer, 0, CRSF_FRAME_SIZE_MAX);

    // Reset the buffer index.
    _streamBufferIndex = 0;
}

void CRSFforArduino::_streamBufferPush8u(uint8_t data)
{
    // Check if the buffer is full.
    if (_streamBufferIndex >= CRSF_FRAME_SIZE_MAX)
    {
        // Clear the buffer.
        _streamBufferClear();
    }

    // Push the data to the buffer.
    _streamBuffer[_streamBufferIndex++] = data;
}

void CRSFforArduino::_streamBufferPush16s(int16_t data)
{
    // Check if the buffer is full. Need two spaces for the data.
    if (_streamBufferIndex >= (CRSF_FRAME_SIZE_MAX - 1))
    {
        // Clear the buffer.
        _streamBufferClear();
    }

    // Push the data to the buffer.
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data >> 8);
}

void CRSFforArduino::_streamBufferPush16u(uint16_t data)
{
    // Check if the buffer is full. Need two spaces for the data.
    if (_streamBufferIndex >= (CRSF_FRAME_SIZE_MAX - 1))
    {
        // Clear the buffer.
        _streamBufferClear();
    }

    // Push the data to the buffer.
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data >> 8);
}

void CRSFforArduino::_streamBufferPush24s(int32_t data)
{
    // Check if the buffer is full. Need three spaces for the data.
    if (_streamBufferIndex >= (CRSF_FRAME_SIZE_MAX - 2))
    {
        // Clear the buffer.
        _streamBufferClear();
    }

    // Push the data to the buffer.
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)((data >> 8) & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data >> 16);
}

void CRSFforArduino::_streamBufferPush24u(uint32_t data)
{
    // Check if the buffer is full. Need three spaces for the data.
    if (_streamBufferIndex >= (CRSF_FRAME_SIZE_MAX - 2))
    {
        // Clear the buffer.
        _streamBufferClear();
    }

    // Push the data to the buffer.
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)((data >> 8) & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data >> 16);
}

void CRSFforArduino::_streamBufferPush32s(int32_t data)
{
    // Check if the buffer is full. Need four spaces for the data.
    if (_streamBufferIndex >= (CRSF_FRAME_SIZE_MAX - 3))
    {
        // Clear the buffer.
        _streamBufferClear();
    }

    // Push the data to the buffer.
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)((data >> 8) & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)((data >> 16) & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data >> 24);
}

void CRSFforArduino::_streamBufferPush32u(uint32_t data)
{
    // Check if the buffer is full. Need four spaces for the data.
    if (_streamBufferIndex >= (CRSF_FRAME_SIZE_MAX - 3))
    {
        // Clear the buffer.
        _streamBufferClear();
    }

    // Push the data to the buffer.
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)((data >> 8) & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)((data >> 16) & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data >> 24);
}

void CRSFforArduino::_streamBufferPush16sBigEndian(int16_t data)
{
    // Check if the buffer is full. Need two spaces for the data.
    if (_streamBufferIndex >= (CRSF_FRAME_SIZE_MAX - 1))
    {
        // Clear the buffer.
        _streamBufferClear();
    }

    // Push the data to the buffer.
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data >> 8);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data & 0xFF);
}

void CRSFforArduino::_streamBufferPush16uBigEndian(uint16_t data)
{
    // Check if the buffer is full. Need two spaces for the data.
    if (_streamBufferIndex >= (CRSF_FRAME_SIZE_MAX - 1))
    {
        // Clear the buffer.
        _streamBufferClear();
    }

    // Push the data to the buffer.
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data >> 8);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data & 0xFF);
}

void CRSFforArduino::_streamBufferPush24sBigEndian(int32_t data)
{
    // Check if the buffer is full. Need three spaces for the data.
    if (_streamBufferIndex >= (CRSF_FRAME_SIZE_MAX - 2))
    {
        // Clear the buffer.
        _streamBufferClear();
    }

    // Push the data to the buffer.
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data >> 16);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)((data >> 8) & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data & 0xFF);
}

void CRSFforArduino::_streamBufferPush24uBigEndian(uint32_t data)
{
    // Check if the buffer is full. Need three spaces for the data.
    if (_streamBufferIndex >= (CRSF_FRAME_SIZE_MAX - 2))
    {
        // Clear the buffer.
        _streamBufferClear();
    }

    // Push the data to the buffer.
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data >> 16);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)((data >> 8) & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data & 0xFF);
}

void CRSFforArduino::_streamBufferPush32sBigEndian(int32_t data)
{
    // Check if the buffer is full. Need four spaces for the data.
    if (_streamBufferIndex >= (CRSF_FRAME_SIZE_MAX - 3))
    {
        // Clear the buffer.
        _streamBufferClear();
    }

    // Push the data to the buffer.
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data >> 24);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)((data >> 16) & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)((data >> 8) & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data & 0xFF);
}

void CRSFforArduino::_streamBufferPush32uBigEndian(uint32_t data)
{
    // Check if the buffer is full. Need four spaces for the data.
    if (_streamBufferIndex >= (CRSF_FRAME_SIZE_MAX - 3))
    {
        // Clear the buffer.
        _streamBufferClear();
    }

    // Push the data to the buffer.
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data >> 24);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)((data >> 16) & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)((data >> 8) & 0xFF);
    _streamBuffer[_streamBufferIndex++] = (uint8_t)(data & 0xFF);
}
#endif

#if defined(ARDUINO_ARCH_SAMD)
#ifdef USE_DMA
uint8_t CRSFforArduino::_getDmaRxId()
{
    uint8_t id = 0;

    /* Get the DMA RX ID for the current UART.
    This adds compatibility with most development boards on the market today. */

#if defined(__SAMD21E18A__)

/* Adafruit Gemma M0, QtPy M0 & Trinket M0. */
#if defined(ADAFRUIT_GEMMA_M0) || defined(ADAFRUIT_QTPY_M0) || defined(ADAFRUIT_TRINKET_M0)
    id = SERCOM0_DMAC_ID_RX;
#endif

#elif defined(__SAMD21G18A__)

/* Adafruit Crickit M0. */
#if defined(ADAFRUIT_CRICKIT_M0)
    id = SERCOM5_DMAC_ID_RX;

/* Adafruit Feather M0 , Feather M0 Express, ItsyBitsy M0 Express & Metro M0 Express. */
#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ADAFRUIT_ITSYBITSY_M0) || \
    defined(ADAFRUIT_METRO_M0_EXPRESS)
    id = SERCOM0_DMAC_ID_RX;

/* The entire lineup of Arduino MKR boards. */
#elif defined(ARDUINO_SAMD_MKR1000) || defined(ARDUINO_SAMD_MKRFox1200) || defined(ARDUINO_SAMD_MKRGSM1400) ||   \
    defined(ARDUINO_SAMD_MKRNB1500) || defined(ARDUINO_SAMD_MKRVIDOR4000) || defined(ARDUINO_SAMD_MKRWAN1300) || \
    defined(ARDUINO_SAMD_MKRWAN1310) || defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_MKRZERO) ||    \
    defined(ARDUINO_SAMD_NANO_33_IOT)
    id = SERCOM5_DMAC_ID_RX;

/* Arduino Zero. */
#elif defined(ARDUINO_SAMD_ZERO)
    id = SERCOM0_DMAC_ID_RX;
#endif

#elif defined(__SAMD51G19A__)

/* Adafruit ItsyBitsy M4 Express. */
#if defined(ADAFRUIT_ITSYBITSY_M4_EXPRESS)
    id = SERCOM3_DMAC_ID_RX;
#endif

#elif defined(__SAMD51J19A__)

/* Adafruit Feather M4 Express. */
#if defined(ADAFRUIT_FEATHER_M4_EXPRESS)
    id = SERCOM5_DMAC_ID_RX;

/* Adafruit Metro M4 Airlift Lite & Metro M4 Express. */
#elif defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_METRO_M4_EXPRESS)
    id = SERCOM3_DMAC_ID_RX;
#endif

#elif defined(__SAME51J19A__)

/* Adafruit Feather M4 CAN. */
#if defined(ADAFRUIT_FEATHER_M4_CAN)
    id = SERCOM5_DMAC_ID_RX;
#endif
#endif

    return id;
}
#endif

/**
 * @brief Gets the SERCOM instance for the current UART.
 *
 * @return Sercom* The SERCOM instance.
 */
Sercom *CRSFforArduino::_getSercom()
{
    Sercom *sercom = NULL;

    /* Get the SERCOM instance for the current UART.
    This adds compatibility with most development boards on the market today. */

#if defined(__SAMD21E18A__)

/* Adafruit Gemma M0, QtPy M0 & Trinket M0. */
#if defined(ADAFRUIT_GEMMA_M0) || defined(ADAFRUIT_QTPY_M0) || defined(ADAFRUIT_TRINKET_M0)
    sercom = SERCOM0;
#endif

#elif defined(__SAMD21G18A__)

/* Adafruit Crickit M0. */
#if defined(ADAFRUIT_CRICKIT_M0)
    sercom = SERCOM5;

/* Adafruit Feather M0 , Feather M0 Express, ItsyBitsy M0 & Metro M0 Express. */
#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ADAFRUIT_ITSYBITSY_M0) || \
    defined(ADAFRUIT_METRO_M0_EXPRESS)
    sercom = SERCOM0;

/* The entire lineup of Arduino MKR boards. */
#elif defined(ARDUINO_SAMD_MKR1000) || defined(ARDUINO_SAMD_MKRFox1200) || defined(ARDUINO_SAMD_MKRGSM1400) ||   \
    defined(ARDUINO_SAMD_MKRNB1500) || defined(ARDUINO_SAMD_MKRVIDOR4000) || defined(ARDUINO_SAMD_MKRWAN1300) || \
    defined(ARDUINO_SAMD_MKRWAN1310) || defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_MKRZERO) ||    \
    defined(ARDUINO_SAMD_NANO_33_IOT)
    sercom = SERCOM5;

/* Arduino Zero. */
#elif defined(ARDUINO_SAMD_ZERO)
    sercom = SERCOM0;
#endif

#elif defined(__SAMD51G19A__)

/* Adafruit ItsyBitsy M4 Express. */
#if defined(ADAFRUIT_ITSYBITSY_M4_EXPRESS)
    sercom = SERCOM3;
#endif

#elif defined(__SAMD51J19A__)

/* Adafruit Feather M4 Express. */
#if defined(ADAFRUIT_FEATHER_M4_EXPRESS)
    sercom = SERCOM5;

/* Adafruit Metro M4 Airlift Lite & Metro M4 Express. */
#elif defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_METRO_M4_EXPRESS)
    sercom = SERCOM3;
#endif

#elif defined(__SAMD51P20A__)

/* Adafruit Grand Central M4. */
#if defined(ADAFRUIT_GRAND_CENTRAL_M4)
    sercom = SERCOM0;
#endif

#elif defined(__SAME51J19A__)

/* Adafruit Feather M4 CAN. */
#if defined(ADAFRUIT_FEATHER_M4_CAN)
    sercom = SERCOM5;
#endif
#endif

    return sercom;
}
#endif

#ifdef USE_DMA
/**
 * @brief DMA transfer done callback.
 *
 * @param dma A pointer to the DMA instance.
 */
void _dmaTransferDoneCallback(Adafruit_ZeroDMA *dma)
{
    // Get the DMA instance that triggered the callback.
    if (dma == &_dmaSerialRx)
    {
        // RX DMA transfer done.
        _dmaSerialRxDone = true;
    }
    else if (dma == &_dmaSerialTx)
    {
        // TX DMA transfer done.
        _dmaSerialTxDone = true;
    }
}
#endif
