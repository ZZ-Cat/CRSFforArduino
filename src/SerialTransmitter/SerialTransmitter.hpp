/**
 * @file SerialTransmitter.hpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief The Serial Transmitter layer for the CRSF for Arduino library.
 * @version 1.1.0
 * @date 2024-4-20
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

#pragma once

#ifndef ENV_DEFECT_DETECTOR
#include "../CFA_Config.hpp"
#endif
#include "Arduino.h"

namespace serialTransmitterLayer
{
    class SerialTransmitter
    {
        public:
            SerialTransmitter();
            explicit SerialTransmitter(HardwareSerial *hwUartPort);
            SerialTransmitter(HardwareSerial *hwUartPort, int8_t rxPin, int8_t txPin);
            virtual ~SerialTransmitter();

        private:
            HardwareSerial *_uart;

            int8_t _rxPin = -1;
            int8_t _txPin = -1;
    };
} // namespace serialTransmitterLayer
