/**
 * @file CRSFforArduino.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief CRSF for Arduino facilitates the use of ExpressLRS RC receivers in Arduino projects.
 * @version 0.4.0
 * @date 2023-07-27
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
// using namespace __crsf_private_dma;
// using namespace __crsf_private_rx;

// using namespace __crsf_private_rx;

// namespace __crsf_private_dma
// {
//     uint8_t rxByte = 0;
//     volatile uint32_t frameStartTime = 0;
//     volatile bool dmaTransferDone = false;

//     /**
//      * @brief Handles the reception of CRSF packets.
//      * This function is called by the DMA callback function.
//      *
//      */
//     void crsfSerialRxHandler()
//     {
//         static uint8_t framePosition = 0;
//         const uint32_t currentTime = micros();

//         // Reset the frame position, if the timeout has elapsed.
//         if (currentTime - frameStartTime > CRSF_FRAME_TIMEOUT)
//         {
//             framePosition = 0;

//             // Compensate for micros() overflow.
//             if (currentTime < frameStartTime)
//             {
//                 frameStartTime = currentTime;
//             }
//         }

//         // Set the frame start time, if the frame position is 0.
//         if (framePosition == 0)
//         {
//             frameStartTime = currentTime;
//         }

//         // Assume the CRSF frame length is 5 bytes until the frame length is known.
//         // Use the maximum buffer size to prevent buffer overflows.
//         const int fullFrameLength = framePosition < 3 ? 5 : min(__crsf_private_rx::buffer.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH, CRSF_FRAME_SIZE_MAX);

//         if (framePosition < fullFrameLength)
//         {
//             __crsf_private_rx::buffer.raw[framePosition++] = rxByte;

//             // Check if the frame is complete.
//             if (framePosition >= fullFrameLength)
//             {
//                 // Reset the frame position.
//                 framePosition = 0;

//                 // Copy data from the buffer to the CRSF frame.
//                 memcpy(&__crsf_private_rx::crsfFrame, &__crsf_private_rx::buffer, CRSF_FRAME_SIZE_MAX);

//                 // Set the packet received flag.
//                 __crsf_private_rx::frameReceived = true;
//             }
//         }
//     }

//     /**
//      * @brief DMA callback function.
//      *
//      * @param dma A pointer to the Adafruit_ZeroDMA object.
//      *
//      * @return nothing
//      *
//      */
//     void _dmaSerialCallback(Adafruit_ZeroDMA *dma)
//     {
//         (void)dma;

//         /* Process received data. */
//         crsfSerialRxHandler();

//         /* Set the DMA Transfer Done flag. */
//         dmaTransferDone = true;
//     }
// } // namespace __crsf_private_dma
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
 * @brief Destroy the CRSFforArduino object.
 *
 */
CRSFforArduino::~CRSFforArduino()
{
    _serial = NULL;
}

/**
 * @brief Opens the hardware serial port for use with CRSF.
 *
 * @return true if the serial port was opened successfully, false if it was not.
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

    /* Disable interrupts. */
    noInterrupts();

    /* CRSF is 420000 baud 8-bit data, no parity, 1 stop bit. */
    _serial->begin(420000, SERIAL_8N1);
    _serial->setTimeout(10);

    /* Initialise telemetry. */
    _telemetryBegin();

    /* Initialise RC channels. */
    _packetReceived = false;

#ifdef USE_DMA
    memset(__crsf_private_rx::buffer.raw, 0, CRSF_FRAME_SIZE_MAX);
    // memset(__crsf_private_rx::dataFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
    memset(__crsf_private_rx::rcChannelsPackedFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
#endif

    memset(_crsfFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
    memset(_crsfRcChannelsPackedFrame.raw, 0, CRSF_FRAME_SIZE_MAX);

    // Set the initial Throttle channel value to 172 (988us).
    _channels[RC_CHANNEL_THROTTLE] = 172;

    // Set all other channel values to 992 (1500us).
    for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++)
    {
        if (i != RC_CHANNEL_THROTTLE)
        {
            _channels[i] = 992;
        }
    }

#ifdef USE_DMA
#if defined(ARDUINO_ARCH_SAMD)
    Sercom *_sercom = _getSercom();

    /* Check if the SERCOM instance was found. */
    if (_sercom == NULL)
    {
#ifdef CRSF_DEBUG
        Serial.println("[CRSF for Arduino | ERROR] SERCOM instance not found.");
#endif
        return false;
    }
#endif

    /* Configure the DMA. */
    _dmaSerial.setTrigger(SERCOM3_DMAC_ID_RX);
    _dmaSerial.setAction(DMA_TRIGGER_ACTON_BEAT);
    _dmaStatus = _dmaSerial.allocate();
    if (_dmaStatus != DMA_STATUS_OK)
    {
        interrupts();
#ifdef CRSF_DEBUG
        Serial.print("[CRSF for Arduino | ERROR] DMA allocation failed with status: ");
        Serial.println(_dmaStatusString[_dmaStatus]);
#endif
        return false;
    }

    /* Configure the DMA descriptors. */
    _dmaSerialDescriptor = _dmaSerial.addDescriptor(
#if defined(ARDUINO_ARCH_SAMD)
        (void *)(&_sercom->USART.DATA.reg),
#endif
        &__crsf_private_dma::rxByte,
        1,
        DMA_BEAT_SIZE_BYTE,
        false,
        false);

    if (_dmaSerialDescriptor == NULL)
    {
        interrupts();
#ifdef CRSF_DEBUG
        Serial.println("[CRSF for Arduino | ERROR] DMA descriptor allocation failed.");
#endif

        return false;
    }

    // Disabled because it is non-functional for some reason.
    _dmaSerial.loop(true);

    /* Configure the DMA callback. */
    _dmaSerial.setCallback(__crsf_private_dma::_dmaSerialCallback);

    /* Enable interrupts. */
    interrupts();

    /* Flush the serial buffer. */
    _flushSerial();

    /* Start the DMA. */
    _dmaStatus = _dmaSerial.startJob();
    if (_dmaStatus != DMA_STATUS_OK)
    {
#ifdef CRSF_DEBUG
        Serial.print("[CRSF for Arduino | ERROR] DMA start failed with status: ");
        Serial.println(_dmaStatusString[_dmaStatus]);
#endif
        return false;
    }
#else
    /* Enable interrupts. */
    interrupts();

    /* Flush the serial buffer. */
    _flushSerial();
#endif

    return true;
}

/**
 * @brief Closes the hardware serial port.
 *
 * @return nothing
 *
 */
void CRSFforArduino::end()
{
    _flushSerial();
    _serial->end();
}

/**
 * @brief The main function of CRSF for Arduino.
 * This function must be called in the main loop of your sketch.
 * It handles the reception of CRSF packets, and the transmission of CRSF telemetry.
 *
 * @return true if a packet was received, false if a packet was not received.
 *
 */
bool CRSFforArduino::update()
{
#ifdef USE_DMA
    if (__crsf_private_dma::dmaTransferDone == true)
    {
        __crsf_private_dma::dmaTransferDone = false;

        if (__crsf_private_rx::frameReceived == true)
        {
            // Clear the frame received flag.
            __crsf_private_rx::frameReceived = false;

            // Get the full frame length & its CRC.
            const int fullFrameLength = __crsf_private_rx::crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;
            const uint8_t crc = _crsfFrameCRC();

            // Check if the CRC is correct.
            if (crc == __crsf_private_rx::crsfFrame.raw[fullFrameLength - 1])
            {
                // Check if the packet is a CRSF frame.
                if (__crsf_private_rx::crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER)
                {
                    // Check if the packet is a CRSF RC frame.
                    if (__crsf_private_rx::crsfFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
                    {
                        // Unpack the RC channels.
                        const __crsf_rcChannelsPacked_t *rcChannelsPacked = (__crsf_rcChannelsPacked_t *)&__crsf_private_rx::crsfFrame.frame.payload;

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
                    }
                }

                // Increment the packet counter.
                _crsfFrameCount = (_crsfFrameCount + 1) % 2;

                // Check if it is time to send telemetry.
                if (_crsfFrameCount == 0)
                {
                    _telemetryProcessFrame();
                }
            }
        }
    }
#endif

    // #ifdef USE_DMA
    //     if (__crsf_private_dma::_dmaTransferDone == true)
    //     {
    //         __crsf_private_dma::_dmaTransferDone = false;
    // #else
    //     while (_serial->available() > 0)
    //     {
    //         _serial->readBytes(_crsfFrame.raw, CRSF_FRAME_SIZE_MAX);
    // #endif

    //         const int fullFrameLength = _crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;
    //         const uint8_t crc = _crsfFrameCRC();

    //         if (crc == _crsfFrame.raw[fullFrameLength - 1])
    //         {
    //             // Check if the packet is a CRSF frame.
    //             if (_crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    //             {

    //                 // Check if the packet is a CRSF RC frame.
    //                 if (_crsfFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    //                 {
    //                     // Read the RC channels.
    //                     memcpy(&_crsfRcChannelsPackedFrame, &_crsfFrame, CRSF_FRAME_SIZE_MAX);

    //                     // Set the packet received flag.
    //                     _packetReceived = true;
    //                 }
    //             }

    //             // Increment the packet counter.
    //             _crsfFrameCount = (_crsfFrameCount + 1) % 2;

    //             // Check if it is time to send telemetry.
    //             if (_crsfFrameCount == 0)
    //             {
    //                 _telemetryProcessFrame();
    //             }
    //         }

    //         // Clear the buffer.
    //         memset(_crsfFrame.raw, 0, CRSF_FRAME_SIZE_MAX);

    // #ifdef USE_DMA
    //         // Restart the DMA.
    //         _dmaStatus = _dmaSerial.startJob();
    //         if (_dmaStatus != DMA_STATUS_OK)
    //         {
    // #ifdef CRSF_DEBUG
    //             Serial.print("[CRSF for Arduino | ERROR] DMA start failed with status: ");
    //             Serial.println(_dmaStatusString[_dmaStatus]);
    // #endif
    //             return false;
    //         }
    // #endif

    //         // Return true to indicate that the packet was received.
    //         return true;
    // #ifndef USE_DMA
    //     }

    //     // Return false to indicate that the packet was not received.
    //     return false;
    // #else
    //     }

    //     else
    //     {
    //         // Return false to indicate that the packet was not received.
    //         return false;
    //     }
    // #endif
}

/**
 * @brief Checks if a CRSF packet was received.
 *
 * @attention This function is deprecated. Use the return value of CRSFforArduino::update() instead.
 *
 * @return true if a packet was received, false if a packet was not received.
 *
 */
bool CRSFforArduino::packetReceived()
{
    return _packetReceived;
}

/**
 * @brief Gets the raw 11-bit value of an RC channel from the CRSF RC frame.
 * Channel range is 172-1811, with 992 being the centre.
 *
 * @param channel The channel ID that you want to get the value of.
 * @return uint16_t The raw 11-bit value of the requested channel.
 *
 */
uint16_t CRSFforArduino::getChannel(uint8_t channel)
{
    // const __crsf_rcChannelsPacked_t *rcChannelsPacked = (__crsf_rcChannelsPacked_t *)&_crsfRcChannelsPackedFrame.frame.payload;

    // // Check if a packet was received.
    // if (_packetReceived == true)
    // {
    //     // Unpack the RC channels.
    //     _channels[RC_CHANNEL_ROLL] = rcChannelsPacked->channel0;
    //     _channels[RC_CHANNEL_PITCH] = rcChannelsPacked->channel1;
    //     _channels[RC_CHANNEL_THROTTLE] = rcChannelsPacked->channel2;
    //     _channels[RC_CHANNEL_YAW] = rcChannelsPacked->channel3;
    //     _channels[RC_CHANNEL_AUX1] = rcChannelsPacked->channel4;
    //     _channels[RC_CHANNEL_AUX2] = rcChannelsPacked->channel5;
    //     _channels[RC_CHANNEL_AUX3] = rcChannelsPacked->channel6;
    //     _channels[RC_CHANNEL_AUX4] = rcChannelsPacked->channel7;
    //     _channels[RC_CHANNEL_AUX5] = rcChannelsPacked->channel8;
    //     _channels[RC_CHANNEL_AUX6] = rcChannelsPacked->channel9;
    //     _channels[RC_CHANNEL_AUX7] = rcChannelsPacked->channel10;
    //     _channels[RC_CHANNEL_AUX8] = rcChannelsPacked->channel11;
    //     _channels[RC_CHANNEL_AUX9] = rcChannelsPacked->channel12;
    //     _channels[RC_CHANNEL_AUX10] = rcChannelsPacked->channel13;
    //     _channels[RC_CHANNEL_AUX11] = rcChannelsPacked->channel14;
    //     _channels[RC_CHANNEL_AUX12] = rcChannelsPacked->channel15;

    //     // Clear the packet received flag.
    //     _packetReceived = false;
    // }

    // Return the requested channel.
    return _channels[(channel - 1) % RC_CHANNEL_COUNT];
}

/**
 * @brief Converts a raw RC channel value to microseconds.
 * 
 * @param rc The raw RC channel value to convert.
 * @return uint16_t The RC channel value in microseconds.
 *
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

/**
 * @brief Converts GPS data to the CRSF telemetry frame format.
 * Use this in your main loop to update the GPS telemetry data.
 * Sending of the telemetry frame is handled internally.
 *
 * @param latitude Latitude in decimal degrees.
 * @param longitude Longitude in decimal degrees.
 * @param altitude Altitude in centimetres.
 * @param speed Speed in centimetres per second.
 * @param groundCourse Ground course in degrees.
 * @param satellites Number of satellites in view.
 *
 * @returns nothing
 *
 */
void CRSFforArduino::telemetryWriteGPS(float latitude, float longitude, float altitude, float speed, float groundCourse, uint8_t satellites)
{
    // Latitude & longitude are in decimal degrees. Convert it to degrees, minutes & seconds.
    _telemetryData.gps.latitude = latitude * 10000000;
    _telemetryData.gps.longitude = longitude * 10000000;

    // Altitude is in centimetres. Convert it to metres, then constrain it to 0-5000m with an offset of 1000m.
    _telemetryData.gps.altitude = (constrain(altitude, 0, 5000 * 100) / 100) + 1000;

    // Ground speed is in cm/s.
    _telemetryData.gps.speed = ((speed * 36 + 50) / 100);

    // Ground course is in degrees * 100.
    _telemetryData.gps.groundCourse = (groundCourse * 100);

    // Satellites in view is a uint8_t.
    _telemetryData.gps.satellites = satellites;
}

/**
 * @brief Initialises the telemetry frame schedule.
 *
 * @attention This function contains code from Betaflight. Betaflight is licensed under the GNU General Public License v3.0.
 * The code is adapted to CRSF for Arduino by Cassandra "ZZ Cat" Robinson, on 2023-07-25.
 * The original code can be found at: https://tinyurl.com/yjm3f4j6
 *
 * @return nothing
 *
 */
void CRSFforArduino::_telemetryBegin()
{
    uint8_t index = 0;
    memset(_telemetryFrameSchedule, 0, sizeof(_telemetryFrameSchedule));
    _serialBufferReset();

    // Add the GPS Telemetry frame to the schedule.
    _telemetryFrameSchedule[index++] = (1 << CRSF_TELEMETRY_FRAME_GPS_INDEX);

    // Set the maximum number of telemetry frames.
    _telemetryFrameIndex = index;
}

/**
 * @brief Initialises the telemetry frame with the sync byte.
 *
 * @attention This function contains code from Betaflight. Betaflight is licensed under the GNU General Public License v3.0.
 * The code is adapted to CRSF for Arduino by Cassandra "ZZ Cat" Robinson, on 2023-07-25.
 * The original code can be found at: https://tinyurl.com/53b7s6v5
 *
 * @returns nothing
 *
 */
void CRSFforArduino::_telemetryInitialiseFrame()
{
    _serialBufferReset();
    _serialBufferWriteU8(CRSF_SYNC_BYTE);
}

/**
 * @brief Appends the GPS telemetry data to the telemetry frame.
 *
 * @attention This function contains code from Betaflight. Betaflight is licensed under the GNU General Public License v3.0.
 * The code is adapted to CRSF for Arduino by Cassandra "ZZ Cat" Robinson, on 2023-07-25.
 * The original code can be found at: https://tinyurl.com/44knvs5s
 *
 * @returns nothing
 *
 */
void CRSFforArduino::_telemetryAppendGPSframe()
{
    _serialBufferWriteU8(CRSF_FRAME_GPS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    _serialBufferWriteU8(CRSF_FRAMETYPE_GPS);

    /* Populate the GPS frame with data from the _telemetryData struct. */
    _serialBufferWrite32BE(_telemetryData.gps.latitude);
    _serialBufferWrite32BE(_telemetryData.gps.longitude);
    _serialBufferWriteU16BE(_telemetryData.gps.speed);
    _serialBufferWriteU16BE(_telemetryData.gps.groundCourse);
    _serialBufferWriteU16BE(_telemetryData.gps.altitude);
    _serialBufferWriteU8(_telemetryData.gps.satellites);
}

/**
 * @brief Appends the CRC to the telemetry frame & sends it.
 *
 * @attention This function contains code from Betaflight. Betaflight is licensed under the GNU General Public License v3.0.
 * The code is adapted to CRSF for Arduino by Cassandra "ZZ Cat" Robinson, on 2023-07-25.
 * The original code can be found at: https://tinyurl.com/5amwu7es
 *
 * @returns nothing
 *
 */
void CRSFforArduino::_telemetryFinaliseFrame()
{
    // Start at byte 2, because the first & second bytes are the sync byte & frame length, which are not included in the CRC.
    uint8_t crc = _crc8_dvb_s2(0, _serialBuffer[2]);
    for (uint8_t i = 3; i < _serialBufferLength; i++)
    {
        crc = _crc8_dvb_s2(crc, _serialBuffer[i]);
    }
    _serialBufferWriteU8(crc);

#ifdef USE_DMA
// For some reason, the DMA does not work with the telemetry frame.
#warning "DMA is temporarily disabled for the telemetry frame."

    // Until I can figure out why, the telemetry frame will be sent using serial.
    _serial->write(_serialBuffer, _serialBufferLength);
#else
    // Send the telemetry frame.
    _serial->write(_serialBuffer, _serialBufferLength);
#endif
}

/**
 * @brief Processes the telemetry frames.
 *
 * @attention This function contains code from Betaflight. Betaflight is licensed under the GNU General Public License v3.0.
 * The code is adapted to CRSF for Arduino by Cassandra "ZZ Cat" Robinson, on 2023-07-25.
 * The original code can be found at: https://tinyurl.com/3fzf9bpw
 *
 * @returns nothing
 *
 */
void CRSFforArduino::_telemetryProcessFrame()
{
    static uint8_t telemetryScheduleIndex = 0;
    const uint8_t currentSchedule = _telemetryFrameSchedule[telemetryScheduleIndex];

    if (currentSchedule & (1 << CRSF_TELEMETRY_FRAME_GPS_INDEX))
    {
        _telemetryInitialiseFrame();
        _telemetryAppendGPSframe();
        _telemetryFinaliseFrame();
    }

    telemetryScheduleIndex = (telemetryScheduleIndex + 1) % _telemetryFrameIndex;
}

/**
 * @brief Calculates the CRC-8-DVB-S2 checksum using the polynomial 0xD5.
 * 
 * @param crc The current CRC value.
 * @param a The byte to add to the CRC.
 *
 * @return uint8_t The new CRC value.
 *
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
 * @brief Calculates the CRC of the CRSF frame.
 * 
 * @return uint8_t The CRC of the CRSF frame.
 *
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

/**
 * @brief Resets the serial buffer.
 *
 * @return nothing
 *
 */
void CRSFforArduino::_serialBufferReset()
{
    memset(_serialBuffer, 0, CRSF_FRAME_SIZE_MAX);
    _serialBufferIndex = 0;
    _serialBufferLength = 0;
}

/**
 * @brief Writes a 32-bit signed integer to the serial buffer.
 *
 * @param value The 32-bit signed integer to write to the serial buffer.
 * @return uint8_t The number of bytes written to the serial buffer.
 *
 */
uint8_t CRSFforArduino::_serialBufferWrite32(int32_t value)
{
    // There must be at least 4 bytes available in the buffer.
    if (_serialBufferIndex < CRSF_FRAME_SIZE_MAX - 4)
    {
        _serialBuffer[_serialBufferIndex++] = (uint8_t)(value & 0xFF);
        _serialBuffer[_serialBufferIndex++] = (uint8_t)((value >> 8) & 0xFF);
        _serialBuffer[_serialBufferIndex++] = (uint8_t)((value >> 16) & 0xFF);
        _serialBuffer[_serialBufferIndex++] = (uint8_t)((value >> 24) & 0xFF);
        _serialBufferLength = _serialBufferIndex;
        return _serialBufferLength;
    }

    // Return 0 to indicate that the buffer is full.
    return 0;
}

/**
 * @brief Writes a 32-bit signed integer to the serial buffer in big endian format.
 * 
 * @param value The 32-bit signed integer to write to the serial buffer.
 * @return uint8_t The number of bytes written to the serial buffer.
 *
 */
uint8_t CRSFforArduino::_serialBufferWrite32BE(int32_t value)
{
    // There must be at least 4 bytes available in the buffer.
    if (_serialBufferIndex < CRSF_FRAME_SIZE_MAX - 4)
    {
        _serialBuffer[_serialBufferIndex++] = (uint8_t)((value >> 24) & 0xFF);
        _serialBuffer[_serialBufferIndex++] = (uint8_t)((value >> 16) & 0xFF);
        _serialBuffer[_serialBufferIndex++] = (uint8_t)((value >> 8) & 0xFF);
        _serialBuffer[_serialBufferIndex++] = (uint8_t)(value & 0xFF);
        _serialBufferLength = _serialBufferIndex;
        return _serialBufferLength;
    }

    // Return 0 to indicate that the buffer is full.
    return 0;
}

/**
 * @brief Writes a 16-bit unsigned integer to the serial buffer.
 * 
 * @param value The 16-bit unsigned integer to write to the serial buffer.
 * @return uint8_t The number of bytes written to the serial buffer.
 *
 */
uint8_t CRSFforArduino::_serialBufferWriteU8(uint8_t value)
{
    // There must be at least 1 byte available in the buffer.
    if (_serialBufferIndex < CRSF_FRAME_SIZE_MAX - 1)
    {
        _serialBuffer[_serialBufferIndex++] = value;
        _serialBufferLength = _serialBufferIndex;
        return _serialBufferLength;
    }

    // Return 0 to indicate that the buffer is full.
    return 0;
}

/**
 * @brief Writes a 16-bit unsigned integer to the serial buffer.
 * 
 * @param value The 16-bit unsigned integer to write to the serial buffer.
 * @return uint8_t The number of bytes written to the serial buffer.
 *
 */
uint8_t CRSFforArduino::_serialBufferWriteU16(uint16_t value)
{
    // There must be at least 2 bytes available in the buffer.
    if (_serialBufferIndex < CRSF_FRAME_SIZE_MAX - 2)
    {
        _serialBuffer[_serialBufferIndex++] = (uint8_t)(value & 0xFF);
        _serialBuffer[_serialBufferIndex++] = (uint8_t)((value >> 8) & 0xFF);
        _serialBufferLength = _serialBufferIndex;
        return _serialBufferLength;
    }

    // Return 0 to indicate that the buffer is full.
    return 0;
}

/**
 * @brief Writes a 32-bit unsigned integer to the serial buffer.
 * 
 * @param value The 32-bit unsigned integer to write to the serial buffer.
 * @return uint8_t The number of bytes written to the serial buffer.
 *
 */
uint8_t CRSFforArduino::_serialBufferWriteU32(uint32_t value)
{
    // There must be at least 4 bytes available in the buffer.
    if (_serialBufferIndex < CRSF_FRAME_SIZE_MAX - 4)
    {
        _serialBuffer[_serialBufferIndex++] = (uint8_t)(value & 0xFF);
        _serialBuffer[_serialBufferIndex++] = (uint8_t)((value >> 8) & 0xFF);
        _serialBuffer[_serialBufferIndex++] = (uint8_t)((value >> 16) & 0xFF);
        _serialBuffer[_serialBufferIndex++] = (uint8_t)((value >> 24) & 0xFF);
        _serialBufferLength = _serialBufferIndex;
        return _serialBufferLength;
    }

    // Return 0 to indicate that the buffer is full.
    return 0;
}

/**
 * @brief Writes a 16-bit unsigned integer to the serial buffer in big endian format.
 * 
 * @param value The 16-bit unsigned integer to write to the serial buffer.
 * @return uint8_t The number of bytes written to the serial buffer.
 *
 */
uint8_t CRSFforArduino::_serialBufferWriteU16BE(uint16_t value)
{
    // There must be at least 2 bytes available in the buffer.
    if (_serialBufferIndex < CRSF_FRAME_SIZE_MAX - 2)
    {
        _serialBuffer[_serialBufferIndex++] = (uint8_t)((value >> 8) & 0xFF);
        _serialBuffer[_serialBufferIndex++] = (uint8_t)(value & 0xFF);
        _serialBufferLength = _serialBufferIndex;
        return _serialBufferLength;
    }

    // Return 0 to indicate that the buffer is full.
    return 0;
}

/**
 * @brief Writes a 32-bit unsigned integer to the serial buffer in big endian format.
 * 
 * @param value The 32-bit unsigned integer to write to the serial buffer.
 * @return uint8_t The number of bytes written to the serial buffer.
 *
 */
uint8_t CRSFforArduino::_serialBufferWriteU32BE(uint32_t value)
{
    // There must be at least 4 bytes available in the buffer.
    if (_serialBufferIndex < CRSF_FRAME_SIZE_MAX - 4)
    {
        _serialBuffer[_serialBufferIndex++] = (uint8_t)((value >> 24) & 0xFF);
        _serialBuffer[_serialBufferIndex++] = (uint8_t)((value >> 16) & 0xFF);
        _serialBuffer[_serialBufferIndex++] = (uint8_t)((value >> 8) & 0xFF);
        _serialBuffer[_serialBufferIndex++] = (uint8_t)(value & 0xFF);
        _serialBufferLength = _serialBufferIndex;
        return _serialBufferLength;
    }

    // Return 0 to indicate that the buffer is full.
    return 0;
}

#if defined(ARDUINO_ARCH_SAMD)
/**
 * @brief Gets the SERCOM instance for the current UART.
 *
 * @return Sercom* The SERCOM instance for the current UART.
 *
 */
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

#elif USB_VID == 0x2886
    // Seeed Studio devboards

#if defined(__SAMD21G18A__)
    // Devboards that use the SAMD21G18A chip.

#if USB_PID == 0x802F
    // Seed Studio XIAO SAMD21.

    sercom = SERCOM4;
#endif

#endif
#endif

    return sercom;
}
#endif

/**
 * @brief Flushes the serial buffer.
 *
 * @return nothing
 *
 */
void CRSFforArduino::_flushSerial()
{
    _serial->flush();
    while (_serial->available() > 0)
    {
        _serial->read();
    }
}
