/**
 * @file CRC.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief A generic CRC8 implementation for the CRSF for Arduino library.
 * @version 1.1.0
 * @date 2024-4-18
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

#include "CRC.hpp"
#include "stdlib.h"

namespace genericCrc
{
    GenericCRC::GenericCRC()
    {
#if (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SPEED)
        crc_8_dvb_s2_table = new uint8_t[256];

        for (uint16_t i = 0; i < 256; i++)
        {
            uint8_t crc = i;
            for (uint8_t j = 0; j < 8; j++)
            {
                if (crc & 0x80)
                {
                    crc = (crc << 1) ^ 0xd5;
                }
                else
                {
                    crc <<= 1;
                }
            }

            crc_8_dvb_s2_table[i] = crc & 0xff;
        }
#endif
    }

    /* GenericCRC copy constructor. */
    GenericCRC::GenericCRC(const GenericCRC &other)
    {
#if (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SPEED)
        crc_8_dvb_s2_table = new uint8_t[256];
        for (uint16_t i = 0; i < 256; i++)
        {
            crc_8_dvb_s2_table[i] = other.crc_8_dvb_s2_table[i];
        }
#endif
    }

    /* GenericCRC operator= */
    GenericCRC &GenericCRC::operator=(const GenericCRC &other)
    {
        if (this != &other)
        {
#if (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SPEED)
            crc_8_dvb_s2_table = other.crc_8_dvb_s2_table;
#endif
        }
        return *this;
    }

    GenericCRC::~GenericCRC()
    {
#if (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SPEED)
        delete[] crc_8_dvb_s2_table;
        crc_8_dvb_s2_table = nullptr;
#endif
    }

#if (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SIZE)
    uint8_t GenericCRC::crc_8_dvb_s2(uint8_t crc, uint8_t data)
    {
        crc ^= data;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0xd5;
            }
            else
            {
                crc <<= 1;
            }
        }
        return crc;
    }
#endif

    uint8_t GenericCRC::calculate(uint8_t start, const uint8_t *data, uint8_t length)
    {
#if (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SPEED)
        uint8_t crc = crc_8_dvb_s2_table[0 ^ start];

        for (uint8_t i = 0; i < length; i++)
        {
            crc = crc_8_dvb_s2_table[crc ^ data[i]];
        }

        return crc;
#elif (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SIZE)
        uint8_t crc = crc_8_dvb_s2(0, start);

        for (uint8_t i = 0; i < length; i++)
        {
            crc = crc_8_dvb_s2(crc, data[i]);
        }

        return crc;
#elif (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_HARDWARE)
#error "CRC_OPTIMISATION_LEVEL is set to CRC_OPTIMISATION_HARDWARE, but no hardware implementation is available."
#endif
    }

    uint8_t GenericCRC::calculate(uint8_t offset, uint8_t start, const uint8_t *data, uint8_t length)
    {
        (void)start;
#if (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SPEED)
        uint8_t crc = crc_8_dvb_s2_table[0 ^ data[offset]];

        for (uint8_t i = offset + 1; i < length; i++)
        {
            crc = crc_8_dvb_s2_table[crc ^ data[i]];
        }

        return crc;
#elif (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SIZE)
        uint8_t crc = crc_8_dvb_s2(0, data[offset]);

        for (uint8_t i = offset + 1; i < length; i++)
        {
            crc = crc_8_dvb_s2(crc, data[i]);
        }

        return crc;
#elif (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_HARDWARE)
#error "CRC_OPTIMISATION_LEVEL is set to CRC_OPTIMISATION_HARDWARE, but no hardware implementation is available."
#endif
    }
} // namespace genericCrc
