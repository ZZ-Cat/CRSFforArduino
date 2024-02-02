/**
 * @file SerialReceiver.hpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This is the header file for the Serial Receiver Interface.
 * @version 1.0.0
 * @date 2024-2-2
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
#include "CRSF/CRSF.hpp"

namespace serialReceiverLayer
{
    class SerialReceiver : public CRSF
    {
    public:
        SerialReceiver(HardwareSerial *serialPort = &Serial1);
        virtual ~SerialReceiver();

        void printHelloWorld();
    private:
        HardwareSerial *_serialPort = nullptr;
    };
} // namespace serialReceiverLayer
