/**
 * @file SerialBuffer.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief A generic serial buffer for the CRSF for Arduino library.
 * @version 1.1.0
 * @date 2024-4-17
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

#include "SerialBuffer.hpp"
#include "cstring"

namespace genericStreamBuffer
{
    SerialBuffer::SerialBuffer(size_t size)
    {
        bufferIndex = 0;
        bufferLength = 0;
        bufferSizeMax = size;
        buffer = new uint8_t[bufferSizeMax];

        memset(buffer, 0, bufferSizeMax);
    }

    SerialBuffer::~SerialBuffer()
    {
        delete[] buffer;
        bufferIndex = 0;
        bufferLength = 0;
        bufferSizeMax = 0;
    }

    void SerialBuffer::reset()
    {
        bufferIndex = 0;
        bufferLength = 0;
        memset(buffer, 0, bufferSizeMax);
    }

    size_t SerialBuffer::write8(int8_t value)
    {
        if (bufferIndex + 1 > bufferSizeMax)
        {
            return 0;
        }

        buffer[bufferIndex++] = value;
        bufferLength = bufferIndex;

        return 1;
    }

    size_t SerialBuffer::write16(int16_t value)
    {
        if (bufferIndex + 2 > bufferSizeMax)
        {
            return 0;
        }

        buffer[bufferIndex++] = value & 0xFF;
        buffer[bufferIndex++] = (value >> 8) & 0xFF;
        bufferLength = bufferIndex;

        return 2;
    }

    size_t SerialBuffer::write32(int32_t value)
    {
        if (bufferIndex + 4 > bufferSizeMax)
        {
            return 0;
        }

        buffer[bufferIndex++] = value & 0xFF;
        buffer[bufferIndex++] = (value >> 8) & 0xFF;
        buffer[bufferIndex++] = (value >> 16) & 0xFF;
        buffer[bufferIndex++] = (value >> 24) & 0xFF;
        bufferLength = bufferIndex;

        return 4;
    }

    size_t SerialBuffer::writeU8(uint8_t value)
    {
        if (bufferIndex + 1 > bufferSizeMax)
        {
            return 0;
        }

        buffer[bufferIndex++] = value;
        bufferLength = bufferIndex;

        return 1;
    }

    size_t SerialBuffer::writeU16(uint16_t value)
    {
        if (bufferIndex + 2 > bufferSizeMax)
        {
            return 0;
        }

        buffer[bufferIndex++] = value & 0xFF;
        buffer[bufferIndex++] = (value >> 8) & 0xFF;
        bufferLength = bufferIndex;

        return 2;
    }

    size_t SerialBuffer::writeU32(uint32_t value)
    {
        if (bufferIndex + 4 > bufferSizeMax)
        {
            return 0;
        }

        buffer[bufferIndex++] = value & 0xFF;
        buffer[bufferIndex++] = (value >> 8) & 0xFF;
        buffer[bufferIndex++] = (value >> 16) & 0xFF;
        buffer[bufferIndex++] = (value >> 24) & 0xFF;
        bufferLength = bufferIndex;

        return 4;
    }

    size_t SerialBuffer::write8BE(int8_t value)
    {
        if (bufferIndex + 1 > bufferSizeMax)
        {
            return 0;
        }

        buffer[bufferIndex++] = value;
        bufferLength = bufferIndex;

        return 1;
    }

    size_t SerialBuffer::write16BE(int16_t value)
    {
        if (bufferIndex + 2 > bufferSizeMax)
        {
            return 0;
        }

        buffer[bufferIndex++] = (value >> 8) & 0xFF;
        buffer[bufferIndex++] = value & 0xFF;
        bufferLength = bufferIndex;

        return 2;
    }

    size_t SerialBuffer::write32BE(int32_t value)
    {
        if (bufferIndex + 4 > bufferSizeMax)
        {
            return 0;
        }

        buffer[bufferIndex++] = (value >> 24) & 0xFF;
        buffer[bufferIndex++] = (value >> 16) & 0xFF;
        buffer[bufferIndex++] = (value >> 8) & 0xFF;
        buffer[bufferIndex++] = value & 0xFF;
        bufferLength = bufferIndex;

        return 4;
    }

    size_t SerialBuffer::writeU8BE(uint8_t value)
    {
        if (bufferIndex + 1 > bufferSizeMax)
        {
            return 0;
        }

        buffer[bufferIndex++] = value;
        bufferLength = bufferIndex;

        return 1;
    }

    size_t SerialBuffer::writeU16BE(uint16_t value)
    {
        if (bufferIndex + 2 > bufferSizeMax)
        {
            return 0;
        }

        buffer[bufferIndex++] = (value >> 8) & 0xFF;
        buffer[bufferIndex++] = value & 0xFF;
        bufferLength = bufferIndex;

        return 2;
    }

    size_t SerialBuffer::writeU24BE(uint32_t value)
    {
        if (bufferIndex + 3 > bufferSizeMax)
        {
            return 0;
        }

        buffer[bufferIndex++] = (value >> 16) & 0xFF;
        buffer[bufferIndex++] = (value >> 8) & 0xFF;
        buffer[bufferIndex++] = value & 0xFF;
        bufferLength = bufferIndex;

        return 3;
    }

    size_t SerialBuffer::writeU32BE(uint32_t value)
    {
        if (bufferIndex + 4 > bufferSizeMax)
        {
            return 0;
        }

        buffer[bufferIndex++] = (value >> 24) & 0xFF;
        buffer[bufferIndex++] = (value >> 16) & 0xFF;
        buffer[bufferIndex++] = (value >> 8) & 0xFF;
        buffer[bufferIndex++] = value & 0xFF;
        bufferLength = bufferIndex;

        return 4;
    }

    size_t SerialBuffer::writeString(const char *string)
    {
        size_t length = strlen(string);

        if (bufferIndex + length > bufferSizeMax)
        {
            return 0;
        }

        memcpy(buffer + bufferIndex, string, length);
        bufferIndex += length;
        bufferLength = bufferIndex;

        return length;
    }

    size_t SerialBuffer::getLength()
    {
        return bufferLength;
    }

    size_t SerialBuffer::getMaxSize()
    {
        return bufferSizeMax;
    }

    size_t SerialBuffer::getIndex()
    {
        return bufferIndex;
    }

    uint8_t SerialBuffer::getByte(size_t index)
    {
        if (index >= bufferSizeMax)
        {
            return 0;
        }

        return buffer[index];
    }

    uint8_t *SerialBuffer::getBuffer()
    {
        return buffer;
    }
} // namespace genericStreamBuffer
