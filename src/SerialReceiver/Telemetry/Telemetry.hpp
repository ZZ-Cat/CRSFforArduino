/**
 * @file Telemetry.hpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This encodes data into CRSF telemetry frames for transmission to the RC handset.
 * @version 1.0.3
 * @date 2024-7-20
 *
 * @copyright Copyright (c) 2024, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU Affero General Public License v3.0
 * This source file is a part of the CRSF for Arduino library.
 * CRSF for Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CRSF for Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with CRSF for Arduino.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */

#pragma once

#include "Arduino.h"

#include "../CRC/CRC.hpp"
#include "../CRSF/CRSFProtocol.hpp"
#include "../SerialBuffer/SerialBuffer.hpp"

namespace serialReceiverLayer
{
    class Telemetry : private genericCrc::GenericCRC, private genericStreamBuffer::SerialBuffer
    {
      public:
        Telemetry();
        ~Telemetry();

        void begin();
        void end();

        bool update();

        void setAttitudeData(int16_t roll, int16_t pitch, int16_t yaw);
        void setBaroAltitudeData(uint16_t altitude, int16_t vario);
        void setBatteryData(float voltage, float current, uint32_t capacity, uint8_t percent);
        void setFlightModeData(const char *flightMode, bool armed = false);
        void setGPSData(float latitude, float longitude, float altitude, float speed, float course, uint8_t satellites);

        void sendTelemetryData(HardwareSerial *db);

      private:
        uint8_t _telemetryFrameScheduleCount;
        uint8_t _telemetryFrameSchedule[crsfProtocol::CRSF_TELEMETRY_FRAME_SCHEDULE_MAX];
        crsfProtocol::telemetryData_t _telemetryData;

        int16_t _decidegreeToRadians(int16_t decidegrees);

        void _initialiseFrame();
        void _appendAttitudeData();
        void _appendBaroAltitudeData();
        void _appendBatterySensorData();
  #if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_FLIGHTMODE_ENABLED > 0
        void _appendFlightModeData();
  #endif
        void _appendGPSData();
        void _finaliseFrame();
    };
} // namespace serialReceiverLayer
