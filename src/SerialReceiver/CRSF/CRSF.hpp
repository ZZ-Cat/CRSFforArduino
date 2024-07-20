/**
 * @file CRSF.hpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This decodes CRSF frames from a serial port.
 * @version 1.0.3
 * @date 2024-7-20
 *
 * @copyright Copyright (c) 2024, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU Affero General Public License v3.0
 * This source file is a part of the CRSF for Arduino library.
 * CRSF for Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CRSF for Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with CRSF for Arduino.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */

#pragma once

#include "../CRC/CRC.hpp"
#include "CRSFProtocol.hpp"

namespace serialReceiverLayer
{
    typedef struct link_statistics_s
    {
        int16_t rssi = 0;
        int16_t lqi = 0;
        int16_t snr = 0;
        int16_t tx_power = 0;
    } link_statistics_t;

    const uint16_t tx_power_table[9] = {
        0,    // 0 mW
        10,   // 10 mW
        25,   // 25 mW
        100,  // 100 mW
        500,  // 500 mW
        1000, // 1 W
        2000, // 2 W
        250,  // 250 mW
        50    // 50 mW
    };

    class CRSF
    {
      public:
        CRSF();
        CRSF(const CRSF &crsf);
        CRSF &operator=(const CRSF &crsf);
        virtual ~CRSF();
        void begin();
        void end();
        void setFrameTime(uint32_t baudRate, uint8_t packetCount = 10);
        bool receiveFrames(uint8_t rxByte);
        void getFailSafe(bool *failSafe);
        void getRcChannels(uint16_t *rcChannels);
        void getLinkStatistics(link_statistics_t *linkStats);

      private:
        bool rcFrameReceived;
        uint16_t frameCount;
        uint32_t timePerFrame;
        crsfProtocol::frame_t rxFrame;
        crsfProtocol::frame_t rcChannelsFrame;
        link_statistics_t linkStatistics;
        genericCrc::GenericCRC *crc8 = nullptr;
        uint8_t calculateFrameCRC();
    };
} // namespace serialReceiverLayer
