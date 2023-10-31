/**
 * @file DevBoards.hpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This file contains the DevBoards class, which is used to configure CRSF for Arduino for specific development boards.
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
#warning "DMA is enabled. This is an experimental feature and may not work as expected."
#if defined(ARDUINO_ARCH_SAMD)
#include "Adafruit_ZeroDMA.h"
#endif
#endif

namespace hal
{
    class DevBoards : private Stream
    {
      public:
        DevBoards();
        virtual ~DevBoards();

        void setUART(uint8_t port, uint8_t rx, uint8_t tx);
        void clearUART();

        // Hardware Serial functions.
        void begin(unsigned long baudrate, int config = SERIAL_8N1);
        void end();
        int available(void);
        int peek(void);
        int read(void);
        void flush(void);
        size_t write(uint8_t c);
        size_t write(const uint8_t *buffer, size_t size);
        using Print::write; // pull in write(str) and write(buf, size) from Print
        operator bool();

        // Critical section functions.
        void enterCriticalSection();
        void exitCriticalSection();

#ifdef USE_DMA
        // DMA functions.
        void memcpy_dma(void *dest, void *src, size_t size);
        void memset_dma(void *dest, int value, size_t size);
#endif

      private:
        uint16_t critical_section_counter = 0;

#if defined(ARDUINO_ARCH_SAMD)
        Uart *uart_port;
#else
        HardwareSerial *uart_port;
#endif
    };
} // namespace hal
