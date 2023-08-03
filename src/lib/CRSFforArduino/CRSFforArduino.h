/**
 * @file CRSFforArduino.h
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief CRSF for Arduino facilitates the use of ExpressLRS RC receivers in Arduino projects.
 * @version 0.4.0
 * @date 2023-08-01
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU General Public License v3.0
 * This header file is a part of the CRSF for Arduino library.
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

#pragma once

#if defined(USE_ABSTRACTION_LAYER)

#include "Arduino.h"
#include "SerialReceiver.h"

namespace sketchLayer
{
    class CRSFforArduino : private serialReceiver::SerialReceiver
    {
    public:
        CRSFforArduino();
        CRSFforArduino(Uart *serial);
        ~CRSFforArduino();
        bool begin();
        void end();
        void update();
        uint16_t getChannel(uint8_t channel);
        uint16_t rcToUs(uint16_t rc);
        uint16_t readRcChannel(uint8_t channel, bool raw = false);
    private:
        serialReceiver::SerialReceiver *_serialReceiver;
    };
} // namespace sketchLayer

using namespace sketchLayer;

#else

#include "Arduino.h"

#if defined(ARDUINO_ARCH_SAMD)
// DMA is disabled for now, as it is not working.
// #define USE_DMA
#endif
#ifdef USE_DMA
#include "Adafruit_ZeroDMA.h"
#endif

#define CRSF_FRAME_TIMEOUT 1500 // 1500 microseconds.

// Uncomment this line to enable debug output.
// #define CRSF_DEBUG

enum __rc_channels_e
{
    RC_CHANNEL_ROLL = 0,
    RC_CHANNEL_PITCH,
    RC_CHANNEL_THROTTLE,
    RC_CHANNEL_YAW,
    RC_CHANNEL_AUX1,
    RC_CHANNEL_AUX2,
    RC_CHANNEL_AUX3,
    RC_CHANNEL_AUX4,
    RC_CHANNEL_AUX5,
    RC_CHANNEL_AUX6,
    RC_CHANNEL_AUX7,
    RC_CHANNEL_AUX8,
    RC_CHANNEL_AUX9,
    RC_CHANNEL_AUX10,
    RC_CHANNEL_AUX11,
    RC_CHANNEL_AUX12,
    RC_CHANNEL_COUNT
};

namespace crsfProtocol
{
    enum syncByte_e
    {
        CRSF_SYNC_BYTE = 0xC8,
    };

    enum frameSize_e
    {
        CRSF_FRAME_SIZE_MAX = 64,
        CRSF_PAYLOAD_SIZE_MAX = CRSF_FRAME_SIZE_MAX - 6
    };

    enum payloadSize_e
    {
        CRSF_FRAME_GPS_PAYLOAD_SIZE = 15,
        CRSF_FRAME_VARIO_PAYLOAD_SIZE = 2,
        CRSF_FRAME_BARO_ALTITUDE_PAYLOAD_SIZE = 4, // TBS is 2, ExpressLRS is 4 (combines vario)
        CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
        CRSF_FRAME_DEVICE_INFO_PAYLOAD_SIZE = 48,
        CRSF_FRAME_FLIGHT_MODE_PAYLOAD_SIZE = 16,
        CRSF_FRAME_HEARTBEAT_PAYLOAD_SIZE = 2,
        CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE = 10,
        CRSF_FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE = 6,
        CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE = 22,
        CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE = 6
    };

    enum frameLength_e
    {
        CRSF_FRAME_LENGTH_ADDRESS = 1,                                               // Length of the address field in bytes.
        CRSF_FRAME_LENGTH_FRAMELENGTH = 1,                                           // Length of the frame length field in bytes.
        CRSF_FRAME_LENGTH_TYPE = 1,                                                  // Length of the type field in bytes.
        CRSF_FRAME_LENGTH_CRC = 1,                                                   // Length of the CRC field in bytes.
        CRSF_FRAME_LENGTH_TYPE_CRC = CRSF_FRAME_LENGTH_TYPE + CRSF_FRAME_LENGTH_CRC, // Length of the type and CRC fields in bytes.
        CRSF_FRAME_LENGTH_EXT_TYPE_CRC = 4,                                          // Length of the extended Dest/Origin, type, and CRC fields in bytes.
        CRSF_FRAME_LENGTH_NON_PAYLOAD = 4                                            // Combined length of all fields except the payload in bytes.
    };

    typedef enum frameType_e
    {
        CRSF_FRAMETYPE_GPS = 0x02,
        CRSF_FRAMETYPE_VARIO = 0x07,
        CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
        CRSF_FRAMETYPE_BARO_ALTITUDE = 0x09,
        CRSF_FRAMETYPE_HEARTBEAT = 0x0B,
        CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
        CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
        CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17,
        CRSF_FRAMETYPE_LINK_STATISTICS_RX = 0x1C,
        CRSF_FRAMETYPE_LINK_STATISTICS_TX = 0x1D,
        CRSF_FRAMETYPE_ATTITUDE = 0x1E,
        CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,

        CRSF_FRAMETYPE_DEVICE_PING = 0x28,
        CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
        CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
        CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
        CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
        CRSF_FRAMETYPE_COMMAND = 0x32,

        CRSF_FRAMETYPE_MSP_REQ = 0x7A,
        CRSF_FRAMETYPE_MSP_RESP = 0x7B,
        CRSF_FRAMETYPE_MSP_WRITE = 0x7C,
        CRSF_FRAMETYPE_DISPLAYPORT_CMD = 0x7D,
    } frameType_t;

    typedef enum address_e
    {
        CRSF_ADDRESS_BROADCAST = 0x00,
        CRSF_ADDRESS_USB = 0x10,
        CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
        CRSF_ADDRESS_RESERVED1 = 0x8A,
        CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
        CRSF_ADDRESS_GPS = 0xC2,
        CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
        CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
        CRSF_ADDRESS_RESERVED2 = 0xCA,
        CRSF_ADDRESS_RACE_TAG = 0xCC,
        CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
        CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
        CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE
    } address_t;

    // Schedule array to send telemetry frames.
    typedef enum
    {
        CRSF_TELEMETRY_FRAME_START_INDEX = 0,
        // CRSF_TELEMETRY_FRAME_ATTITUDE_INDEX,
        // CRSF_TELEMETRY_FRAME_BARO_ALTITUDE_INDEX,
        // CRSF_TELEMETRY_FRAME_BATTERY_SENSOR_INDEX,
        // CRSF_TELEMETRY_FRAME_FLIGHT_MODE_INDEX,
        CRSF_TELEMETRY_FRAME_GPS_INDEX,
        // CRSF_TELEMETRY_FRAME_HEARTBEAT_INDEX,
        // CRSF_TELEMETRY_FRAME_VARIO_INDEX,
        CRSF_TELEMETRY_FRAME_SCHEDULE_MAX
    } telemetryFrame_t;

    // RC Channels Packed. 22 bytes (11 bits per channel, 16 channels) total.
    struct rcChannelsPacked_s
    {
        uint16_t channel0  : 11;
        uint16_t channel1  : 11;
        uint16_t channel2  : 11;
        uint16_t channel3  : 11;
        uint16_t channel4  : 11;
        uint16_t channel5  : 11;
        uint16_t channel6  : 11;
        uint16_t channel7  : 11;
        uint16_t channel8  : 11;
        uint16_t channel9  : 11;
        uint16_t channel10 : 11;
        uint16_t channel11 : 11;
        uint16_t channel12 : 11;
        uint16_t channel13 : 11;
        uint16_t channel14 : 11;
        uint16_t channel15 : 11;
    } __attribute__((packed));

    typedef struct rcChannelsPacked_s rcChannelsPacked_t;

    typedef struct frameDefinition_s
    {
        uint8_t deviceAddress;                                          // Frame address.
        uint8_t frameLength;                                            // Frame length. Includes payload and CRC.
        uint8_t type;                                                   // Frame type.
        uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + CRSF_FRAME_LENGTH_CRC]; // Frame payload.
    } frameDefinition_t;

    typedef union frame_u
    {
        uint8_t raw[CRSF_FRAME_SIZE_MAX];
        frameDefinition_t frame;
    } frame_t;

    // GPS Data to pass to the telemetry frame.
    typedef struct gpsData_s
    {
        int32_t latitude;
        int32_t longitude;
        uint16_t altitude;
        uint16_t speed;
        uint16_t groundCourse;
        uint8_t satellites;
    } gpsData_t;

    // Struct to hold data for the telemetry frame.
    typedef struct telemetryData_s
    {
        gpsData_t gps;
    } telemetryData_t;
} // namespace crsfProtocol

class CRSFforArduino
{
  public:
    /* Constructor & Destructor */
    CRSFforArduino(HardwareSerial *serial);
    ~CRSFforArduino();

    /* CRSF Functions */
    bool begin();
    void end();
    void update();
    bool packetReceived();
    uint16_t getChannel(uint8_t channel);
    uint16_t rcToUs(uint16_t rc);

    /* Telemetry Functions */
    void telemetryWriteGPS(float latitude, float longitude, float altitude, float speed, float groundCourse, uint8_t satellites);

  protected:
    /* CRSF */
    bool _rcFrameReceived;
    HardwareSerial *_serial;
    uint16_t _crsfFrameCount;
    uint16_t _channels[RC_CHANNEL_COUNT];
    crsfProtocol::frame_t _crsfFrame;
    crsfProtocol::frame_t _crsfRcChannelsPackedFrame;

    /* Telemetry */
    uint8_t _telemetryFrameIndex;
    uint8_t _telemetryFrameSchedule[crsfProtocol::CRSF_TELEMETRY_FRAME_SCHEDULE_MAX];
    crsfProtocol::telemetryData_t _telemetryData;
    void _telemetryBegin(void);
    void _telemetryInitialiseFrame(void);
    void _telemetryAppendGPSframe(void);
    void _telemetryFinaliseFrame(void);
    void _telemetryProcessFrame(void);

    /* CRC */
    uint8_t _crc8_dvb_s2(uint8_t crc, uint8_t a);
    uint8_t _crsfFrameCRC(void);

    /* Serial Buffer */
    uint8_t _serialBuffer[crsfProtocol::CRSF_FRAME_SIZE_MAX];
    uint8_t _serialBufferIndex;
    uint8_t _serialBufferLength;
    void _serialBufferReset(void);
    uint8_t _serialBufferWrite32(int32_t data);
    uint8_t _serialBufferWrite32BE(int32_t data);
    uint8_t _serialBufferWriteU8(uint8_t data);
    uint8_t _serialBufferWriteU16(uint16_t data);
    uint8_t _serialBufferWriteU32(uint32_t data);
    uint8_t _serialBufferWriteU16BE(uint16_t data);
    uint8_t _serialBufferWriteU32BE(uint32_t data);

#if defined(ARDUINO_ARCH_SAMD)
    Sercom *_getSercom(void);
#endif

    void _flushSerial(void);
};

#endif // USE_ABSTRACTION_LAYER
