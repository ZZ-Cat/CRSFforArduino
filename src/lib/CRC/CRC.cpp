/**
 * @file CRC.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief CRC class implementation.
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

#include "CRC.h"

namespace serialReceiver
{
    CRC::CRC()
    {
    }

    CRC::~CRC()
    {
    }

    uint8_t CRC::crc_8_dvb_s2(uint8_t crc, uint8_t data, uint8_t poly)
    {
        crc ^= data;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ poly;
            }
            else
            {
                crc <<= 1;
            }
        }
        return crc;
    }
} // namespace serialReceiver
