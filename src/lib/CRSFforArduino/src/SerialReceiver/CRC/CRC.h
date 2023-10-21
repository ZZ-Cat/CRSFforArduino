/**
 * @file CRC.h
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief CRC class declaration.
 * @version 0.5.0
 * @date 2023-10-22
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

namespace serialReceiver
{
#define CRC_OPTIMISATION_SPEED    0
#define CRC_OPTIMISATION_SIZE     1
#define CRC_OPTIMISATION_HARDWARE 2

    class CRC
    {
      public:
        CRC();
        virtual ~CRC();

        uint8_t calculate(uint8_t start, uint8_t *data, uint8_t length);
        uint8_t calculate(uint8_t offset, uint8_t start, uint8_t *data, uint8_t length);

      private:
#if (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SPEED)
        uint8_t *crc_8_dvb_s2_table;
#elif (CRC_OPTIMISATION_LEVEL == CRC_OPTIMISATION_SIZE)
        uint8_t crc_8_dvb_s2(uint8_t crc, uint8_t data);
#endif
    };
} // namespace serialReceiver
