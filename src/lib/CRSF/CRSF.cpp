/**
 * @file CRSF.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief CRSF class implementation.
 * @version 0.4.0
 * @date 2023-08-01
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

#include "CRSF.h"

using namespace crsfProtocol;

namespace serialReceiver
{
    CRSF::CRSF()
    {
    }

    CRSF::~CRSF()
    {
    }

    void CRSF::begin()
    {
        rcFrameReceived = false;
        frameCount = 0;
        timePerFrame = 0;

        memset(rxFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
        memset(rcChannelsFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
        // memset(channelData, 0, RC_CHANNEL_COUNT);
    }

    void CRSF::end()
    {
        // memset(channelData, 0, RC_CHANNEL_COUNT);
        memset(rcChannelsFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
        memset(rxFrame.raw, 0, CRSF_FRAME_SIZE_MAX);

        timePerFrame = 0;
        frameCount = 0;
        rcFrameReceived = false;
    }

    void CRSF::setFrameTime(uint32_t baudRate, uint8_t packetCount)
    {
        timePerFrame = ((1000000 * packetCount) / (baudRate / (CRSF_FRAME_SIZE_MAX - 1)));
    }

    bool CRSF::receiveFrames(uint8_t rxByte)
    {
        static uint8_t framePosition = 0;
        static uint32_t frameStartTime = 0;
        const uint32_t currentTime = micros();

        // Reset the frame position if the frame time has elapsed.
        if (currentTime - frameStartTime > timePerFrame)
        {
            framePosition = 0;

            // This compensates for micros() overflow.
            if (currentTime < frameStartTime)
            {
                frameStartTime = currentTime;
            }
        }

        // Reset the frame start time if the frame position is 0.
        if (framePosition == 0)
        {
            frameStartTime = currentTime;
        }

        // Assume the full frame lenthg is 5 bytes until the frame length byte is received.
        const int fullFrameLength = framePosition < 3 ? 5 : min(rxFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH, CRSF_FRAME_SIZE_MAX);

        if (framePosition < fullFrameLength)
        {
            rxFrame.raw[framePosition] = rxByte;
            framePosition++;

            if (framePosition >= fullFrameLength)
            {
                const uint8_t crc = calculateFrameCRC();

                if (crc == rxFrame.raw[fullFrameLength - 1])
                {
                    switch (rxFrame.frame.type)
                    {
                        case crsfProtocol::CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                            if (rxFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER)
                            {
                                memcpy(&rcChannelsFrame, &rxFrame, CRSF_FRAME_SIZE_MAX);
                                rcFrameReceived = true;
                            }
                            break;
                    }
                }

                memset(rxFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
                framePosition = 0;

                return true;
            }
        }

        return false;
    }

    void CRSF::getRcChannels(uint16_t *rcChannels)
    {
        if (rcFrameReceived)
        {
            rcFrameReceived = false;
            if (rcChannelsFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
            {
                // Unpack RC Channels.
                const rcChannelsPacked_t *rcChannelsPacked = (rcChannelsPacked_t *)&rcChannelsFrame.frame.payload;

                rcChannels[RC_CHANNEL_ROLL] = rcChannelsPacked->channel0;
                rcChannels[RC_CHANNEL_PITCH] = rcChannelsPacked->channel1;
                rcChannels[RC_CHANNEL_THROTTLE] = rcChannelsPacked->channel2;
                rcChannels[RC_CHANNEL_YAW] = rcChannelsPacked->channel3;
                rcChannels[RC_CHANNEL_AUX1] = rcChannelsPacked->channel4;
                rcChannels[RC_CHANNEL_AUX2] = rcChannelsPacked->channel5;
                rcChannels[RC_CHANNEL_AUX3] = rcChannelsPacked->channel6;
                rcChannels[RC_CHANNEL_AUX4] = rcChannelsPacked->channel7;
                rcChannels[RC_CHANNEL_AUX5] = rcChannelsPacked->channel8;
                rcChannels[RC_CHANNEL_AUX6] = rcChannelsPacked->channel9;
                rcChannels[RC_CHANNEL_AUX7] = rcChannelsPacked->channel10;
                rcChannels[RC_CHANNEL_AUX8] = rcChannelsPacked->channel11;
                rcChannels[RC_CHANNEL_AUX9] = rcChannelsPacked->channel12;
                rcChannels[RC_CHANNEL_AUX10] = rcChannelsPacked->channel13;
                rcChannels[RC_CHANNEL_AUX11] = rcChannelsPacked->channel14;
                rcChannels[RC_CHANNEL_AUX12] = rcChannelsPacked->channel15;

                // Print the first 8 RC channels to the Serial Monitor.
                // Serial.print("RC Channels <A: ");
                // Serial.print(rcChannels[RC_CHANNEL_ROLL]);
                // Serial.print(", E: ");
                // Serial.print(rcChannels[RC_CHANNEL_PITCH]);
                // Serial.print(", T: ");
                // Serial.print(rcChannels[RC_CHANNEL_THROTTLE]);
                // Serial.print(", R: ");
                // Serial.print(rcChannels[RC_CHANNEL_YAW]);
                // Serial.print(", AUX1: ");
                // Serial.print(rcChannels[RC_CHANNEL_AUX1]);
                // Serial.print(", AUX2: ");
                // Serial.print(rcChannels[RC_CHANNEL_AUX2]);
                // Serial.print(", AUX3: ");
                // Serial.print(rcChannels[RC_CHANNEL_AUX3]);
                // Serial.print(", AUX4: ");
                // Serial.print(rcChannels[RC_CHANNEL_AUX4]);
                // Serial.println(">");
            }
        }
    }

    uint8_t CRSF::calculateFrameCRC()
    {
        uint8_t crc = crc_8_dvb_s2(0, rxFrame.frame.type);
        for (uint8_t i = 0; i < rxFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; i++)
        {
            crc = crc_8_dvb_s2(crc, rxFrame.frame.payload[i]);
        }
        return crc;
    }
} // namespace serialReceiver
