/**
 * @file CRSFforArduino.h
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief CRSF for Arduino facilitates the use of ExpressLRS RC receivers in Arduino projects.
 * @version 0.3.0
 * @date 2023-02-25
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

#if defined(ARDUINO_ARCH_SAMD)
#define USE_DMA
#endif

#include "Arduino.h"

#ifdef USE_DMA
#include "Adafruit_ZeroDMA.h"
#endif

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

enum __crsf_syncByte_e
{
    CRSF_SYNC_BYTE = 0xC8,
};

enum __crsf_frameSize_e
{
    CRSF_FRAME_SIZE_MAX = 64,
    CRSF_PAYLOAD_SIZE_MAX = CRSF_FRAME_SIZE_MAX - 6
};

enum __crsf_frame_payloadSize_e
{
    CRSF_FRAME_GPS_PAYLOAD_SIZE = 15,
    CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
    CRSF_FRAME_HEARTBEAT_PAYLOAD_SIZE = 2,
    CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE = 10,
    CRSF_FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE = 6,
    CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE = 22,
    CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE = 6
};

enum __crsf_frameLength_e
{
    CRSF_FRAME_LENGTH_ADDRESS = 1,                                               // Length of the address field in bytes.
    CRSF_FRAME_LENGTH_FRAMELENGTH = 1,                                           // Length of the frame length field in bytes.
    CRSF_FRAME_LENGTH_TYPE = 1,                                                  // Length of the type field in bytes.
    CRSF_FRAME_LENGTH_CRC = 1,                                                   // Length of the CRC field in bytes.
    CRSF_FRAME_LENGTH_TYPE_CRC = CRSF_FRAME_LENGTH_TYPE + CRSF_FRAME_LENGTH_CRC, // Length of the type and CRC fields in bytes.
    CRSF_FRAME_LENGTH_EXT_TYPE_CRC = 4,                                          // Length of the extended Dest/Origin, type, and CRC fields in bytes.
    CRSF_FRAME_LENGTH_NON_PAYLOAD = 4                                            // Combined length of all fields except the payload in bytes.
};

typedef enum __crsf_frameType_e
{
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
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
} __crsf_frameType_t;

typedef enum __crsf_address_e
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
} __crsf_address_t;

typedef struct __crsf_frameDefinition_s
{
    uint8_t deviceAddress;                                          // Frame address.
    uint8_t frameLength;                                            // Frame length. Includes payload and CRC.
    uint8_t type;                                                   // Frame type.
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + CRSF_FRAME_LENGTH_CRC]; // Frame payload.
} __crsf_frameDefinition_t;

typedef union __crsf_frame_u
{
    uint8_t raw[CRSF_FRAME_SIZE_MAX];
    __crsf_frameDefinition_t frame;
} __crsf_frame_t;

class CRSFforArduino
{
  public:
    CRSFforArduino(HardwareSerial *serial);
    ~CRSFforArduino();
    bool begin();
    void end();
    bool update();
    bool packetReceived();
    uint16_t getChannel(uint8_t channel);
    uint16_t rcToUs(uint16_t rc);

  protected:
    /* CRSF */
    bool _packetReceived;
    HardwareSerial *_serial;
    uint16_t _channels[RC_CHANNEL_COUNT];
    __crsf_frame_t _crsfFrame;

    /* CRC */
    uint8_t _crc8_dvb_s2(uint8_t crc, uint8_t a);
    uint8_t _crsfFrameCRC(void);

#ifdef USE_DMA
    /* DMA */
    Adafruit_ZeroDMA _dmaSerialRx;
    DmacDescriptor *_dmaSerialRxDescriptor;
    ZeroDMAstatus _dmaStatus;
#endif

#if defined(ARDUINO_ARCH_SAMD)
    Sercom *_getSercom(void);
#endif
};

#ifdef USE_DMA
/**
 * @brief DMA transfer done callback
 *
 */
void _dmaTransferDoneCallback(Adafruit_ZeroDMA *);
#endif
