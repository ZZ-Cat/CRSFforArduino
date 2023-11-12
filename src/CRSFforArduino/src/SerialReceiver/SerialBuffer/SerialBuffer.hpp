/**
 * @file SerialBuffer.hpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief SerialBuffer class definition.
 * @version 0.5.0
 * @date 2023-11-1
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

#ifdef USE_DMA
#if defined(ARDUINO) && defined(PLATFORMIO)
#include "Hardware/DevBoards/DevBoards.hpp"
#elif defined(ARDUINO) && !defined(PLATFORMIO)
#include "lib/CRSFforArduino/src/Hardware/DevBoards/DevBoards.hpp"
#endif
#endif

namespace genericStreamBuffer
{
    class SerialBuffer
#ifdef USE_DMA
        : public hal::DevBoards
#endif
    {
      public:
        SerialBuffer(size_t size);
        ~SerialBuffer();

        void reset();

        // Write signed integers in little endian
        size_t write8(int8_t value);
        size_t write16(int16_t value);
        size_t write32(int32_t value);

        // Write unsigned integers in little endian
        size_t writeU8(uint8_t value);
        size_t writeU16(uint16_t value);
        size_t writeU32(uint32_t value);

        // Write signed integers in big endian
        size_t write8BE(int8_t value);
        size_t write16BE(int16_t value);
        size_t write32BE(int32_t value);

        // Write unsigned integers in big endian
        size_t writeU8BE(uint8_t value);
        size_t writeU16BE(uint16_t value);
        size_t writeU24BE(uint32_t value);
        size_t writeU32BE(uint32_t value);

        // Write a string
        size_t writeString(const char *string);

        // Get the current buffer length
        size_t getLength();

        // Get the maximum buffer size
        size_t getMaxSize();

        // Get the current buffer index
        size_t getIndex();

        // Get the byte at the specified index
        uint8_t getByte(size_t index);

        // Get the buffer
        uint8_t *getBuffer();

      private:
        size_t bufferSizeMax;
        size_t bufferLength;
        size_t bufferIndex;
        uint8_t *buffer;
    };
} // namespace genericStreamBuffer
