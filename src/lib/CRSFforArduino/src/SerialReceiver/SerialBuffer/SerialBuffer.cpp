/**
 * @file SerialBuffer.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief SerialBuffer class implementation.
 * @version 0.5.0
 * @date 2023-10-22
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

#include "SerialBuffer.h"

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
#ifdef USE_DMA
        memset_dma(buffer, 0, bufferSizeMax);
#else
        memset(buffer, 0, bufferSizeMax);
#endif
    }

    // Write signed integers in little endian
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

    // Write unsigned integers in little endian
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

    // Write signed integers in big endian
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

    // Write unsigned integers in big endian
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

    // Get the current buffer length
    size_t SerialBuffer::getLength()
    {
        return bufferLength;
    }

    // Get the maximum buffer size
    size_t SerialBuffer::getMaxSize()
    {
        return bufferSizeMax;
    }

    // Get the current buffer index
    size_t SerialBuffer::getIndex()
    {
        return bufferIndex;
    }

    // Get the byte at the specified index
    uint8_t SerialBuffer::getByte(size_t index)
    {
        if (index >= bufferSizeMax)
        {
            return 0;
        }

        return buffer[index];
    }

    // Get the buffer
    uint8_t *SerialBuffer::getBuffer()
    {
        return buffer;
    }
} // namespace genericStreamBuffer
