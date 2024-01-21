/**
 * @file hw_uart.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This is the hw_uart implementation file. It is used to configure CRSF for Arduino for specific development boards.
 * @version 1.0.0
 * @date 2024-1-22
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

#include "hw_uart.hpp"

namespace hal
{
    // #if defined(USE_DMA)
    // #if defined(ARDUINO_ARCH_SAMD)
    // #define DMA_BUFFER_SIZE 64

    //     Adafruit_ZeroDMA *dma_memcopy;
    //     Adafruit_ZeroDMA *dma_memset;

    //     DmacDescriptor *descriptor_memcopy;
    //     DmacDescriptor *descriptor_memset;

    //     volatile bool dma_memcopy_transfer_complete = false;
    //     volatile bool dma_memset_transfer_complete = false;

    //     void dma_callback(Adafruit_ZeroDMA *dma)
    //     {
    //         if (dma == dma_memcopy)
    //         {
    //             dma_memcopy_transfer_complete = true;
    //         }
    //         else if (dma == dma_memset)
    //         {
    //             dma_memset_transfer_complete = true;
    //         }
    //     }
    // #endif
    // #endif

    hw_uart::hw_uart()
    {
        _uart_port = nullptr;
    }

    hw_uart::hw_uart(HardwareSerial *uart_port)
    {
        if (uart_port != nullptr)
        {
            _uart_port = uart_port;
        }
    }

    hw_uart::~hw_uart()
    {
        _uart_port = nullptr;
        // #ifdef USE_DMA
        // #if defined(ARDUINO_ARCH_SAMD)
        //         // If DMA memory was initialized, delete it.
        //         if (dma_memcopy != nullptr)
        //         {
        //             delete dma_memcopy;
        //         }

        //         if (dma_memset != nullptr)
        //         {
        //             delete dma_memset;
        //         }
        // #endif
        // #endif
    }

    void hw_uart::setUART(uint8_t port, uint8_t rx, uint8_t tx)
    {
        return;
        // #if defined(ARDUINO_ARCH_SAMD)
        //         // If UART port was defined beforehand, delete it.
        //         if (_uart_port != nullptr)
        //         {
        //             _uart_port->~Uart();

        //             // Debug.
        //             // Serial.println("[Development Board | DEBUG]: Deleted previous UART port.");
        //         }

        //         // Set the UART port.
        //         switch (port)
        //         {
        //             case 0:
        //                 _uart_port = &Serial1;

        //                 // Debug.
        //                 // Serial.println("[Development Board | DEBUG]: Using Serial1.");
        //                 break;

        //             case 1: // TO-DO: Fix this.
        //                 _uart_port = new Uart(&sercom2, rx, tx, SERCOM_RX_PAD_1, UART_TX_PAD_0);

        //                 // Debug.
        //                 // Serial.println("[Development Board | DEBUG]: Using Serial2.");
        //                 break;

        //             default:
        //                 _uart_port = nullptr;

        //                 // Debug.
        //                 // Serial.println("[Development Board | ERROR]: No UART port was defined.");
        //                 break;
        //         }
        // #elif defined(TEENSYDUINO)
        //         // Default to Serial1 if Teensyduino is being used. May expand this in the future, if requested.
        //         _uart_port = &Serial1;

        //         // Debug.
        //         // Serial.println("[Development Board | DEBUG]: Using Serial1.");
        // #elif defined(ARDUINO_ARCH_ESP32)
        //         // Default to Serial1 if ESP32 is being used. May expand this in the future, if requested.
        //         _uart_port = &Serial1;

        //         // Debug.
        //         // Serial.println("[Development Board | DEBUG]: Using Serial1.");
        // #else
        //         _uart_port = nullptr;

        //         // Debug.
        //         // Serial.println("[Development Board | ERROR]: No UART port was defined.");
        // #endif
    }

    void hw_uart::clearUART()
    {
        return;
        // If UART port was defined beforehand, delete it.
        //         if (_uart_port != nullptr)
        //         {
        // #if not(defined(TEENSYDUINO) || defined(ARDUINO_ARCH_ESP32))
        //             _uart_port->~Uart();
        // #endif
        //         }
    }

    void hw_uart::begin(unsigned long baudrate, int config)
    {
        // Begin the UART port.
        _uart_port->begin(baudrate, config);
    }

    void hw_uart::end()
    {
        // End the UART port.
        _uart_port->end();
    }

    int hw_uart::available(void)
    {
        // Return the number of bytes available in the UART port.
        return _uart_port->available();
    }

    int hw_uart::peek(void)
    {
        // Return the next byte in the UART port without removing it from the buffer.
        return _uart_port->peek();
    }

    int hw_uart::read(void)
    {
        // Return the next byte in the UART port and remove it from the buffer.
        return _uart_port->read();
    }

    void hw_uart::flush(void)
    {
        // Flush the UART port.
        _uart_port->flush();
    }

    size_t hw_uart::write(uint8_t c)
    {
        // Write a byte to the UART port.
        return _uart_port->write(c);
    }

    size_t hw_uart::write(const uint8_t *buffer, size_t size)
    {
        // Write a buffer to the UART port.
        return _uart_port->write(buffer, size);
    }

    hw_uart::operator bool()
    {
        // Return if the UART port is available.
        return _uart_port->operator bool();
    }

    void hw_uart::enterCriticalSection()
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

    void hw_uart::exitCriticalSection()
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

    // #if defined(USE_DMA)
    //     void hw_uart::memcpy_dma(void *dest, void *src, size_t size)
    //     {
    // #if defined(ARDUINO_ARCH_SAMD)
    //         // If DMA memory was not initialized, initialize it.
    //         if (dma_memcopy == nullptr)
    //         {
    //             enterCriticalSection();

    //             dma_memcopy = new Adafruit_ZeroDMA();

    //             // Check if DMA memory was initialized.
    //             if (dma_memcopy == nullptr)
    //             {
    //                 exitCriticalSection();
    //                 return;
    //             }

    //             // Allocate DMA channel for memory & check if it was allocated.
    //             if (dma_memcopy->allocate() != DMA_STATUS_OK)
    //             {
    //                 delete dma_memcopy;
    //                 exitCriticalSection();
    //                 return;
    //             }

    //             // Allocate DMA descriptors for memory & check if they were allocated.
    //             descriptor_memcopy = dma_memcopy->addDescriptor(
    //                 NULL, // No source data to start with.
    //                 NULL, // No destination to start with.
    //                 0     // No data to start with.
    //             );

    //             if (descriptor_memcopy == nullptr)
    //             {
    //                 delete dma_memcopy;
    //                 exitCriticalSection();
    //                 return;
    //             }

    //             // Set the DMA callback function.
    //             dma_memcopy->setCallback(dma_callback);

    //             // Set the DMA transfer complete flag.
    //             dma_memcopy_transfer_complete = true;

    //             exitCriticalSection();
    //         }

    //         // Check if the DMA transfer is complete.
    //         if (dma_memcopy_transfer_complete == true)
    //         {
    //             // Reset the DMA transfer complete flag.
    //             dma_memcopy_transfer_complete = false;

    //             // Change the DMA descriptor.
    //             dma_memcopy->changeDescriptor(
    //                 descriptor_memcopy,
    //                 src,
    //                 dest,
    //                 size);

    //             // Start the DMA transfer.
    //             if (dma_memcopy->startJob() == DMA_STATUS_OK)
    //             {
    //                 dma_memcopy->trigger();

    //                 // Wait for the DMA transfer to complete.
    //                 while (dma_memcopy_transfer_complete == false)
    //                 {
    //                     ;
    //                 }
    //             }
    //         }
    // #else
    //         // Default to memcpy if DMA is not supported.
    //         memcpy(dest, src, size);
    // #endif
    //     }

    //     void hw_uart::memset_dma(void *dest, int value, size_t size)
    //     {
    // #if defined(ARDUINO_ARCH_SAMD)
    //         // If DMA memory was not initialized, initialize it.
    //         if (dma_memset == nullptr)
    //         {
    //             enterCriticalSection();

    //             dma_memset = new Adafruit_ZeroDMA();

    //             // Check if DMA memory was initialized.
    //             if (dma_memset == nullptr)
    //             {
    //                 exitCriticalSection();
    //                 return;
    //             }

    //             // Allocate DMA channel for memory & check if it was allocated.
    //             if (dma_memset->allocate() != DMA_STATUS_OK)
    //             {
    //                 delete dma_memset;
    //                 exitCriticalSection();
    //                 return;
    //             }

    //             // Allocate DMA descriptors for memory & check if they were allocated.
    //             descriptor_memset = dma_memset->addDescriptor(
    //                 NULL,               // No source data to start with.
    //                 NULL,               // No destination to start with.
    //                 0,                  // No data to start with.
    //                 DMA_BEAT_SIZE_BYTE, // Transfer 1 byte at a time.
    //                 false,              // Don't increment the source address.
    //                 true                // Increment the destination address.
    //             );

    //             if (descriptor_memset == nullptr)
    //             {
    //                 delete dma_memset;
    //                 exitCriticalSection();
    //                 return;
    //             }

    //             // Set the DMA callback function.
    //             dma_memset->setCallback(dma_callback);

    //             // Set the DMA transfer complete flag.
    //             dma_memset_transfer_complete = true;

    //             exitCriticalSection();
    //         }

    //         // Check if the DMA transfer is complete.
    //         if (dma_memset_transfer_complete == true)
    //         {
    //             // Reset the DMA transfer complete flag.
    //             dma_memset_transfer_complete = false;

    //             // Change the DMA descriptor.
    //             dma_memset->changeDescriptor(
    //                 descriptor_memset,
    //                 &value,
    //                 dest,
    //                 size);

    //             // Start the DMA transfer.
    //             if (dma_memset->startJob() == DMA_STATUS_OK)
    //             {
    //                 dma_memset->trigger();

    //                 // Wait for the DMA transfer to complete.
    //                 while (dma_memset_transfer_complete == false)
    //                 {
    //                     ;
    //                 }
    //             }
    //         }
    // #else
    //         // Default to memset if DMA is not supported.
    //         memset(dest, value, size);
    // #endif
    //     }
    // #endif
} // namespace hal
