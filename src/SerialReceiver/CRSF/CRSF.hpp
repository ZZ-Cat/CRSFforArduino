/**
 * @file CRSF.hpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This decodes CRSF frames from a serial port.
 * @version 1.0.0
 * @date 2024-2-6
 *
 * @copyright Copyright (c) 2024, Cassandra "ZZ Cat" Robinson. All rights reserved.
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

#include "../CRC/CRC.hpp"
#include "CRSFProtocol.hpp"

namespace serialReceiverLayer
{
    class CRSF
    {
      public:
        CRSF();
        virtual ~CRSF();
        void begin();
        void end();
        void setFrameTime(uint32_t baudRate, uint8_t packetCount = 10);
        bool receiveFrames(uint8_t rxByte);
        void getRcChannels(uint16_t *rcChannels);

      private:
        bool rcFrameReceived;
        uint16_t frameCount;
        uint32_t timePerFrame;
        crsfProtocol::frame_t rxFrame;
        crsfProtocol::frame_t rcChannelsFrame;
        genericCrc::CRC *crc8;
        uint8_t calculateFrameCRC();
    };
} // namespace serialReceiverLayer
