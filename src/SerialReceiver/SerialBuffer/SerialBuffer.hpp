#pragma once

#include "stddef.h"
#include "stdint.h"

namespace genericStreamBuffer
{
    class SerialBuffer
    {
    public:
        SerialBuffer(size_t size = 64);
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
