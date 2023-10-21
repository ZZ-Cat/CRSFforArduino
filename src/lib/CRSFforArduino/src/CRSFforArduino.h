/**
 * @file CRSFforArduino.h
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief CRSF for Arduino facilitates the use of ExpressLRS RC receivers in Arduino projects.
 * @version 0.5.0
 * @date 2023-10-22
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

#if defined(ARDUINO) && defined(PLATFORMIO)
#include "SerialReceiver/SerialReceiver.h"
#elif defined(ARDUINO) && !defined(PLATFORMIO)
#include "lib/CRSFforArduino/src/SerialReceiver/SerialReceiver.h"
#endif

namespace sketchLayer
{
    class CRSFforArduino : private serialReceiver::SerialReceiver
    {
      public:
        CRSFforArduino();
        CRSFforArduino(uint8_t RxPin, uint8_t TxPin);
        ~CRSFforArduino();
        bool begin();
        void end();
        void update();
        uint16_t getChannel(uint8_t channel);
        uint16_t rcToUs(uint16_t rc);
        uint16_t readRcChannel(uint8_t channel, bool raw = false);

        void telemetryWriteGPS(float latitude, float longitude, float altitude, float speed, float groundCourse, uint8_t satellites);

      private:
        SerialReceiver *_serialReceiver;
    };
} // namespace sketchLayer

using namespace sketchLayer;
