/**
 * @file DevBoards.h
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This file contains the DevBoards class, which is used to configure CRSF for Arduino for specific development boards.
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

namespace hal
{
#define USE_ERROR_FLAGS 0
    class DevBoards : private HardwareSerial
    {
      public:
        DevBoards();
        ~DevBoards();

        void setUART(uint8_t port, uint8_t rx, uint8_t tx);
        void clearUART();

        // Hardware Serial functions.
        void begin(unsigned long baudrate, uint16_t config = SERIAL_8N1);
        void end();
        int available(void);
        int peek(void);
        int read(void);
        void flush(void);
        size_t write(uint8_t c);
        using Print::write; // pull in write(str) and write(buf, size) from Print
        operator bool();

        // Critical section functions.
        void enterCriticalSection();
        void exitCriticalSection();

      private:
#if USE_ERROR_FLAGS > 0
        enum error_flags_e
        {
            UART_PORT_OK = 0x00,
            UART_PORT_NOT_SET = 0x01,
            UART_PORT_NOT_AVAILABLE = 0x02,
        };

        typedef enum error_flags_e error_flags_t;

        error_flags_t error_flags = UART_PORT_NOT_SET;

        error_flags_t getErrorFlag();
        void setErrorFlag(error_flags_t flag);
        void clearErrorFlag();
#endif

#if defined(ARDUINO_ARCH_SAMD)
        // Sercom *sercom;
        Uart *uart_port = NULL;
#endif
    };
} // namespace hal
