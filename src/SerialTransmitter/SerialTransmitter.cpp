/**
 * @file SerialReceiver.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief The Serial Transmitter layer for the CRSF for Arduino library.
 * @version 1.1.0
 * @date 2024-4-9
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

#include "SerialTransmitter.hpp"
#include "../hal/CompatibilityTable/CompatibilityTable.hpp"
#include "Arduino.h"

using namespace hal;

namespace serialTransmitterLayer
{
    SerialTransmitter::SerialTransmitter()
    {
    #if defined(ARDUINO_ARCH_STM32)
#if defined(HAVE_HWSERIAL1)
        _uart = &Serial1;
#elif defined(HAVE_HWSERIAL2)
        _uart = &Serial2;
#elif defined(HAVE_HWSERIAL3)
        _uart = &Serial3;
#endif
#elif defined(ARDUINO_ARCH_ESP32)
        _uart = &Serial1;

#if defined(D0)
        _rxPin = D0;
#else
        _rxPin = 0;
#endif

#if defined(D1)
        _txPin = D1;
#else
        _txPin = 1;
#endif
#else
        _uart = &Serial1;
#endif
    }

    SerialTransmitter::SerialTransmitter(HardwareSerial *hwUartPort)
    {
        _uart = hwUartPort;

#if defined(ARDUINO_ARCH_ESP32)
#if defined(D0)
        _rxPin = D0;
#else
        _rxPin = 0;
#endif

#if defined(D1)
        _txPin = D1;
#else
        _txPin = 1;
#endif
#endif
    }

    SerialTransmitter::SerialTransmitter(HardwareSerial *hwUartPort, int8_t rxPin, int8_t txPin)
    {
        _uart = hwUartPort;

#if defined(ARDUINO_ARCH_ESP32)
        _rxPin = rxPin;
        _txPin = txPin;
#else
        (void)rxPin;
        (void)txPin;
#endif
    }

    SerialTransmitter::~SerialTransmitter()
    {
        _uart = nullptr;

        _rxPin = -1;
        _txPin = -1;
    }
} // namespace serialTransmitterLayer
