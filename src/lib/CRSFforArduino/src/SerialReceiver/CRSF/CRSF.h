/**
 * @file CRSF.h
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief CRSF class definition.
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

#include "Arduino.h"
#include "CRSFProtocol.h"
#include "SerialReceiver/CRC/CRC.h"
// #include "Hardware.h"

namespace serialReceiver
{
    class CRSF : private CRC
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
        uint16_t channelData[crsfProtocol::RC_CHANNEL_COUNT];
        uint32_t timePerFrame;
        crsfProtocol::frame_t rxFrame;
        crsfProtocol::frame_t rcChannelsFrame;
        uint8_t calculateFrameCRC();
    };
} // namespace serialReceiver
