/**
 * @file CRC.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief CRC class implementation.
 * @version 0.5.0
 * @date 2023-11-1
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

#include "CRC.hpp"

namespace serialReceiver
{
    CRC::CRC()
    {
#if (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SPEED)
        // Allocate memory for the CRC8 DVB S2 table.
        crc_8_dvb_s2_table = (uint8_t *)malloc(256 * sizeof(uint8_t));

        // Generate the CRC8 DVB S2 table.
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

    CRC::~CRC()
    {
#if (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SPEED)
        // Free the memory allocated for the CRC8 DVB S2 table.
        free(crc_8_dvb_s2_table);
#endif
    }

#if (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SIZE)
    uint8_t CRC::crc_8_dvb_s2(uint8_t crc, uint8_t data)
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

    uint8_t CRC::calculate(uint8_t start, uint8_t *data, uint8_t length)
    {
#if (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SPEED)
        // start is the first byte of the data to be CRC'd.
        // data is a pointer to the data to be CRC'd.

        // Calculate the CRC8 DVB S2 value.
        uint8_t crc = crc_8_dvb_s2_table[0 ^ start];
        for (uint8_t i = 0; i < length; i++)
        {
            crc = crc_8_dvb_s2_table[crc ^ data[i]];
        }

        // Return the CRC8 DVB S2 value.
        return crc;
#elif (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SIZE)
        uint8_t crc = crc_8_dvb_s2(0, start);
        for (uint8_t i = 0; i < length; i++)
        {
            crc = crc_8_dvb_s2(crc, data[i]);
        }
        return crc;
#elif (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_HARDWARE)
#endif
    }

    uint8_t CRC::calculate(uint8_t offset, uint8_t start, uint8_t *data, uint8_t length)
    {
        (void)start;
#if (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SPEED)
        // start is the first byte of the data to be CRC'd.
        // data is a pointer to the data to be CRC'd.

        // Calculate the CRC8 DVB S2 value.
        uint8_t crc = crc_8_dvb_s2_table[0 ^ data[offset]];
        for (uint8_t i = offset + 1; i < length; i++)
        {
            crc = crc_8_dvb_s2_table[crc ^ data[i]];
        }

        // Return the CRC8 DVB S2 value.
        return crc;
#elif (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SIZE)
        uint8_t crc = crc_8_dvb_s2(0, data[offset]);
        for (uint8_t i = offset + 1; i < length; i++)
        {
            crc = crc_8_dvb_s2(crc, data[i]);
        }
        return crc;
#elif (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_HARDWARE)
#endif
    }

} // namespace serialReceiver
