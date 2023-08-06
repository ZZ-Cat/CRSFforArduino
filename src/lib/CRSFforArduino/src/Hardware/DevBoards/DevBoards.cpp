/**
 * @file DevBoards.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This is the DevBoards implementation file. It is used to configure CRSF for Arduino for specific development boards.
 * @version 0.4.0
 * @date 2023-08-01
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

#include "DevBoards.h"

namespace hal
{
    DevBoards::DevBoards()
    {
    }

    DevBoards::~DevBoards()
    {
    }

    void DevBoards::setUART(uint8_t port, uint8_t rx, uint8_t tx)
    {
#if defined(ARDUINO_ARCH_SAMD)
        // If UART port was defined beforehand, delete it.
        if (uart_port != nullptr)
        {
            delete uart_port;
        }

        // Set the UART port.
        switch (port)
        {
            case 0:
                uart_port = &Serial1;
                break;

            case 1: // TO-DO: Fix this.
                uart_port = new Uart(&sercom2, rx, tx, SERCOM_RX_PAD_1, UART_TX_PAD_0);
                break;

            default:
                uart_port = nullptr;
                break;
        }
#else
        uart_port = nullptr;
#endif
    }

    void DevBoards::clearUART()
    {
        // If UART port was defined beforehand, delete it.
        if (uart_port != nullptr)
        {
            delete uart_port;
        }
    }

    void DevBoards::begin(unsigned long baudrate, uint16_t config)
    {
        // Begin the UART port.
        uart_port->begin(baudrate, config);
    }

    void DevBoards::end()
    {
        // End the UART port.
        uart_port->end();
    }

    int DevBoards::available(void)
    {
        // Return the number of bytes available in the UART port.
        return uart_port->available();
    }

    int DevBoards::peek(void)
    {
        // Return the next byte in the UART port without removing it from the buffer.
        return uart_port->peek();
    }

    int DevBoards::read(void)
    {
        // Return the next byte in the UART port and remove it from the buffer.
        return uart_port->read();
    }

    void DevBoards::flush(void)
    {
        // Flush the UART port.
        uart_port->flush();
    }

    size_t DevBoards::write(uint8_t c)
    {
        // Write a byte to the UART port.
        return uart_port->write(c);
    }

    DevBoards::operator bool()
    {
        // Return if the UART port is available.
        return uart_port->operator bool();
    }

    void DevBoards::enterCriticalSection()
    {
        // Enter a critical section.
#if defined(ARDUINO_ARCH_SAMD)
        __disable_irq();
#else
        noInterrupts();
#endif

        // Increment the critical section counter.
        critical_section_counter++;
    }

    void DevBoards::exitCriticalSection()
    {
        // Decrement the critical section counter.
        critical_section_counter--;

        // Exit a critical section.
        if (critical_section_counter == 0)
        {
#if defined(ARDUINO_ARCH_SAMD)
            __enable_irq();
#else
            interrupts();
#endif
        }
    }
} // namespace hal
