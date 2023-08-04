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
// #if defined(ARDUINO_ARCH_SAMD)
// #if defined(__SAMD51__)
// #else
//         uint32_t crc32;
//         uint8_t crc8;
//         /* Source: Microchip SAM D21/DA1 Family Data Sheet, Section 13.11.3.1.
//          * https://ww1.microchip.com/downloads/en/DeviceDoc/SAMD21-Family-DataSheet-DS40001882D.pdf
//          *
//          * CRC32 calculation for a memory range is started after writing the start address to the Address Register (ADDR) and
//          * the length of the memory range to the Length Register (LENGTH). Both must be word aligned.
//          *
//          * The initial value used for the CRC32 calculation must be written to the Data Register (DATA). This value is typically
//          * 0xFFFFFFFF, but can be, for example, the result of a previous CRC32 calculation if generating a common CRC32 of
//          * seperate memory blocks.
//          *
//          * Once completed, the calculated CRC32 value can be read from the Data Register (DATA). The read value must be
//          * complemented to match standard CRC32 implementations or kept non-inverted if used as a starting point for
//          * subsequent CRC32 calculations.
//          *
//          * The actual test is started by writing a '1' in the 32-bit Cyclic Redundancy Check bit of the Control Register
//          * (CTRL.CRC). A running CRC32 operation can be cancelled by resetting the module (writing a '1' to CTRL.SWRST).
//          */

//         // Set the CRC32 start address.
//         DSU->ADDR.reg = (uint32_t)data;

//         // Set the CRC32 length.
//         DSU->LENGTH.reg = sizeof(data);

//         // Set the CRC32 initial value.
//         DSU->DATA.reg = crc;

//         // Start the CRC32 calculation.
//         DSU->CTRL.bit.CRC = 1;

//         // When the CRC32 calculation is complete, the STATUSA.DONE bit will be set in the Status A Register (STATUSA).
//         // Then, the Bus Error bit of the Status A Register (STATUSA.BERR) will be set if a bus error occurred during the
//         // CRC32 calculation.
//         while (!DSU->STATUSA.bit.DONE)
//         {
//         }

//         // Check if a bus error occurred during the CRC32 calculation.
//         if (DSU->STATUSA.bit.BERR)
//         {
//             // Return 0 if a bus error occurred during the CRC32 calculation.
//             return 0;
//         }

//         // Read the CRC32 value.
//         crc32 = DSU->DATA.reg;

//         // Convert the CRC32 value to a CRC DVB 8 S2 value, based on the polynomial.
//         crc8 = (uint8_t)(crc32 >> 24) ^ poly;

//         // Return the CRC8 value.
//         return crc8;
// #endif
// #else
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
// #endif
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


} // namespace serialReceiver
