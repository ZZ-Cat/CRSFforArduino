/**
 * @file CRSF.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This decodes CRSF frames from a serial port.
 * @version 1.1.0
 * @date 2024-4-18
 *
 * @copyright Copyright (c) 2024, Cassandra "ZZ Cat" Robinson. All rights reserved.
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

#include "CRSF.hpp"
#include "Arduino.h"

using namespace crsfProtocol;
using namespace genericCrc;

namespace serialReceiverLayer
{
    CRSF::CRSF()
    {
        rcFrameReceived = false;
        frameCount = 0;
        timePerFrame = 0;

        crc8 = new GenericCRC();
    }

    CRSF::CRSF(const CRSF &crsf)
    {
        rcFrameReceived = crsf.rcFrameReceived;
        frameCount = crsf.frameCount;
        timePerFrame = crsf.timePerFrame;

        memcpy(rxFrame.raw, crsf.rxFrame.raw, CRSF_FRAME_SIZE_MAX);
        memcpy(rcChannelsFrame.raw, crsf.rcChannelsFrame.raw, CRSF_FRAME_SIZE_MAX);

        crc8 = new GenericCRC(*crsf.crc8);
    }

    CRSF &CRSF::operator=(const CRSF &crsf)
    {
        if (this != &crsf)
        {
            rcFrameReceived = crsf.rcFrameReceived;
            frameCount = crsf.frameCount;
            timePerFrame = crsf.timePerFrame;

            memcpy(rxFrame.raw, crsf.rxFrame.raw, CRSF_FRAME_SIZE_MAX);
            memcpy(rcChannelsFrame.raw, crsf.rcChannelsFrame.raw, CRSF_FRAME_SIZE_MAX);

            *crc8 = *crsf.crc8;
        }

        return *this;
    }

    CRSF::~CRSF()
    {
        delete crc8;
        crc8 = nullptr;
    }

    void CRSF::begin()
    {
        rcFrameReceived = false;
        frameCount = 0;
        timePerFrame = 0;

        memset(rxFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
        memset(rcChannelsFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
    }

    void CRSF::end()
    {
        memset(rcChannelsFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
        memset(rxFrame.raw, 0, CRSF_FRAME_SIZE_MAX);

        timePerFrame = 0;
        frameCount = 0;
        rcFrameReceived = false;
    }

    void CRSF::setFrameTime(uint32_t baudRate, uint8_t packetCount)
    {
        /* Calculate the time per frame based on the baud rate and packet count. */
        timePerFrame = ((1000000 * packetCount) / (baudRate / (CRSF_FRAME_SIZE_MAX - 1)));
    }

    bool CRSF::receiveFrames(uint8_t rxByte)
    {
        static uint8_t framePosition = 0;
        static uint32_t frameStartTime = 0;
        const uint32_t currentTime = micros();

        /* Reset the frame position if the frame time has expired. */
        if (currentTime - frameStartTime > timePerFrame)
        {
            framePosition = 0;

            if (currentTime < frameStartTime)
            {
                frameStartTime = currentTime;
            }
        }

        if (framePosition == 0)
        {
            frameStartTime = currentTime;
        }

        /* Assume the full frame lenthg is 5 bytes until the frame length byte is received. */
        const int fullFrameLength = framePosition < 3 ? 5 : min(rxFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH, (int)CRSF_FRAME_SIZE_MAX);

        if (framePosition < fullFrameLength)
        {
            /* Store the received byte in the frame buffer. */
            rxFrame.raw[framePosition] = rxByte;
            framePosition++;

            if (framePosition >= fullFrameLength)
            {
                /* Frame is complete, calculate the CRC and check if it is valid. */
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

#if CRSF_LINK_STATISTICS_ENABLED > 0
                        case CRSF_FRAMETYPE_LINK_STATISTICS:
                            if ((rxFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) && (rxFrame.frame.frameLength == CRSF_FRAME_ORIGIN_DEST_SIZE + CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE))
                            {
                                crsf_payload_link_statistics_t linkStatisticsPayload;
                                memcpy(&linkStatisticsPayload, rxFrame.frame.payload, sizeof(crsf_payload_link_statistics_t));

                                linkStatistics.rssi = (linkStatisticsPayload.active_antenna ? linkStatisticsPayload.uplink_rssi_2 : linkStatisticsPayload.uplink_rssi_1);
                                linkStatistics.lqi = linkStatisticsPayload.uplink_link_quality;
                                linkStatistics.snr = linkStatisticsPayload.uplink_snr;
                                linkStatistics.tx_power = (linkStatisticsPayload.uplink_tx_power < 9) ? tx_power_table[linkStatisticsPayload.uplink_tx_power] : 0;
                            }
                            break;
#endif
                    }
                }

                /* Clear the frame buffer and reset the frame position. */
                memset(rxFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
                framePosition = 0;
                return true;
            }
        }

        return false;
    }

    void CRSF::getFailSafe(bool *failSafe)
    {
        /* Set the failsafe flag based on the link statistics thresholds. */
        if (linkStatistics.lqi <= CRSF_FAILSAFE_LQI_THRESHOLD || linkStatistics.rssi >= CRSF_FAILSAFE_RSSI_THRESHOLD)
        {
            *failSafe = true;
        }
        else
        {
            *failSafe = false;
        }
    }

    void CRSF::getRcChannels(uint16_t *rcChannels)
    {
        /* Decode RC frames if one has been received. */
        if (rcFrameReceived)
        {
            rcFrameReceived = false;
            if (rcChannelsFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
            {
                rcChannelsPacked_t rcChannelsPacked;
                memcpy(&rcChannelsPacked, rcChannelsFrame.frame.payload, sizeof(rcChannelsPacked_t));

                rcChannels[RC_CHANNEL_ROLL] = rcChannelsPacked.channel0;
                rcChannels[RC_CHANNEL_PITCH] = rcChannelsPacked.channel1;
                rcChannels[RC_CHANNEL_THROTTLE] = rcChannelsPacked.channel2;
                rcChannels[RC_CHANNEL_YAW] = rcChannelsPacked.channel3;
                rcChannels[RC_CHANNEL_AUX1] = rcChannelsPacked.channel4;
                rcChannels[RC_CHANNEL_AUX2] = rcChannelsPacked.channel5;
                rcChannels[RC_CHANNEL_AUX3] = rcChannelsPacked.channel6;
                rcChannels[RC_CHANNEL_AUX4] = rcChannelsPacked.channel7;
                rcChannels[RC_CHANNEL_AUX5] = rcChannelsPacked.channel8;
                rcChannels[RC_CHANNEL_AUX6] = rcChannelsPacked.channel9;
                rcChannels[RC_CHANNEL_AUX7] = rcChannelsPacked.channel10;
                rcChannels[RC_CHANNEL_AUX8] = rcChannelsPacked.channel11;
                rcChannels[RC_CHANNEL_AUX9] = rcChannelsPacked.channel12;
                rcChannels[RC_CHANNEL_AUX10] = rcChannelsPacked.channel13;
                rcChannels[RC_CHANNEL_AUX11] = rcChannelsPacked.channel14;
                rcChannels[RC_CHANNEL_AUX12] = rcChannelsPacked.channel15;
            }
        }
    }

    void CRSF::getLinkStatistics(link_statistics_t *linkStats)
    {
#if CRSF_LINK_STATISTICS_ENABLED > 0
        memcpy(linkStats, &linkStatistics, sizeof(link_statistics_t));
#else
        (void)linkStats;
#endif
    }

    uint8_t CRSF::calculateFrameCRC()
    {
        return crc8->calculate(rxFrame.frame.type, rxFrame.frame.payload, rxFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC);
    }
} // namespace serialReceiverLayer
