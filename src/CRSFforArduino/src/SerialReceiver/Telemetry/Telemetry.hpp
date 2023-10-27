/**
 * @file Telemetry.hpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief Telemetry class definition.
 * @version 0.5.0
 * @date 2023-10-24
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

#if defined(ARDUINO) && defined(PLATFORMIO)
#include "Hardware/DevBoards/DevBoards.hpp"
#include "SerialReceiver/CRC/CRC.hpp"
#include "SerialReceiver/CRSF/CRSFProtocol.hpp"
#include "SerialReceiver/SerialBuffer/SerialBuffer.hpp"
#elif defined(ARDUINO) && !defined(PLATFORMIO)
#include "lib/CRSFforArduino/src/Hardware/DevBoards/DevBoards.hpp"
#include "lib/CRSFforArduino/src/SerialReceiver/CRC/CRC.hpp"
#include "lib/CRSFforArduino/src/SerialReceiver/CRSF/CRSFProtocol.hpp"
#include "lib/CRSFforArduino/src/SerialReceiver/SerialBuffer/SerialBuffer.hpp"
#endif

namespace serialReceiver
{
#define USE_BATTERY_TELEMETRY 1 // Set USE_BATTERY_TELEMETRY to 1 to enable battery telemetry.
#define USE_GPS_TELEMETRY     1 // Set USE_GPS_TELEMETRY to 1 to enable GPS telemetry.

    class Telemetry : private CRC, private genericStreamBuffer::SerialBuffer
#ifndef USE_DMA
        ,
                      private hal::DevBoards
#endif
    {
      public:
        Telemetry();
        ~Telemetry();

        void begin();
        void end();

        bool update();

        // void setAttitudeData(float roll, float pitch, float yaw);
        // void setBaroAltitudeData(float altitude);
        void setBatteryData(float voltage, float current, uint32_t capacity, uint8_t percent);
        // void setFlightModeData(const char *flightMode);
        void setGPSData(float latitude, float longitude, float altitude, float speed, float course, uint8_t satellites);
        // void setVarioData(float vario);

        void sendTelemetryData(DevBoards *db);

      private:
        uint8_t _telemetryFrameScheduleCount;
        uint8_t _telemetryFrameSchedule[crsfProtocol::CRSF_TELEMETRY_FRAME_SCHEDULE_MAX];
        crsfProtocol::telemetryData_t _telemetryData;

        void _initialiseFrame();
        // void _appendAttitudeData();
        // void _appendBaroAltitudeData();
        void _appendBatterySensorData();
        // void _appendFlightModeData();
        void _appendGPSData();
        // void _appendHeartbeatData();
        // void _appendVarioData();
        void _finaliseFrame();
    };
} // namespace serialReceiver
