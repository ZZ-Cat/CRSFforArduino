/**
 * @file SerialBuffer.hpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief A generic serial buffer for the CRSF for Arduino library.
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

#include "stddef.h"
#include "stdint.h"

namespace genericStreamBuffer
{
    class SerialBuffer
    {
      public:
        explicit SerialBuffer(size_t size = 64);
        SerialBuffer(const SerialBuffer &serialBuffer);
        SerialBuffer &operator=(const SerialBuffer &serialBuffer);
        ~SerialBuffer();

        void reset();

        size_t write8(int8_t value);
        size_t write16(int16_t value);
        size_t write32(int32_t value);

        size_t writeU8(uint8_t value);
        size_t writeU16(uint16_t value);
        size_t writeU32(uint32_t value);

        size_t write8BE(int8_t value);
        size_t write16BE(int16_t value);
        size_t write32BE(int32_t value);

        size_t writeU8BE(uint8_t value);
        size_t writeU16BE(uint16_t value);
        size_t writeU24BE(uint32_t value);
        size_t writeU32BE(uint32_t value);

        size_t writeString(const char *string);

        size_t getLength();

        size_t getMaxSize();

        size_t getIndex();

        uint8_t getByte(size_t index);

        uint8_t *getBuffer();

      private:
        size_t bufferSizeMax;
        size_t bufferLength;
        size_t bufferIndex;
        uint8_t *buffer;
    };
} // namespace genericStreamBuffer
