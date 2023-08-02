/**
 * @file SerialReceiver.h
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This is the header file for the Serial Receiver Interface.
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
#include "CRSF/CRSF.h"
#include "Hardware/Hardware.h"

namespace serialReceiver
{
    class SerialReceiver : /* private CRSF, */ private CompatibilityTable, private DevBoards
    {
      public:
        SerialReceiver();
        SerialReceiver(int rxPin, int txPin);
        ~SerialReceiver();

        bool begin();
        void end();

        void processFrames();

        uint16_t readRcChannel(uint8_t channel, bool raw = false);

      private:
        CRSF *crsf;
        int _rxPin = -1;
        int _txPin = -1;
        uint16_t _rcChannels[16];
        void flushRemainingFrames();
    };
} // namespace serialReceiver
