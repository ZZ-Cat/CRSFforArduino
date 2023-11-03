/**
 * @file Telemetry.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief Telemetry class implementation.
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

#include "Telemetry.hpp"

using namespace crsfProtocol;

namespace serialReceiver
{
#define PI  3.1415926535897932384626433832795F
#define RAD PI / 180.0F

    Telemetry::Telemetry() :
        CRC(), SerialBuffer(CRSF_FRAME_SIZE_MAX)
    {
        _telemetryFrameScheduleCount = 0;
        memset(_telemetryFrameSchedule, 0, sizeof(_telemetryFrameSchedule));
        memset(&_telemetryData, 0, sizeof(_telemetryData));
    }

    Telemetry::~Telemetry()
    {
    }

    void Telemetry::begin()
    {
        SerialBuffer::reset();

        uint8_t index = 0;
#if USE_ATTITUDE_TELEMETRY > 0
        _telemetryFrameSchedule[index++] = (1 << CRSF_TELEMETRY_FRAME_ATTITUDE_INDEX);
#endif

#if USE_BARO_ALT_TELEMETRY > 0
        _telemetryFrameSchedule[index++] = (1 << CRSF_TELEMETRY_FRAME_BARO_ALTITUDE_INDEX);
#endif

#if USE_BATTERY_TELEMETRY > 0
        _telemetryFrameSchedule[index++] = (1 << CRSF_TELEMETRY_FRAME_BATTERY_SENSOR_INDEX);
#endif

#if USE_GPS_TELEMETRY > 0
        _telemetryFrameSchedule[index++] = (1 << CRSF_TELEMETRY_FRAME_GPS_INDEX);
#endif

        _telemetryFrameScheduleCount = index;
    }

    void Telemetry::end()
    {
        SerialBuffer::reset();
    }

    bool Telemetry::update()
    {
        bool sendFrame = false;

        static uint8_t scheduleIndex = 0;
        const uint8_t currentSchedule = _telemetryFrameSchedule[scheduleIndex];

#if USE_ATTITUDE_TELEMETRY > 0
        if (currentSchedule & (1 << CRSF_TELEMETRY_FRAME_ATTITUDE_INDEX))
        {
            _initialiseFrame();
            _appendAttitudeData();
            _finaliseFrame();
            sendFrame = true;
        }
#endif

#if USE_BARO_ALT_TELEMETRY > 0
        if (currentSchedule & (1 << CRSF_TELEMETRY_FRAME_BARO_ALTITUDE_INDEX))
        {
            _initialiseFrame();
            _appendBaroAltitudeData();
            _finaliseFrame();
            sendFrame = true;
        }
#endif

#if USE_BATTERY_TELEMETRY > 0
        if (currentSchedule & (1 << CRSF_TELEMETRY_FRAME_BATTERY_SENSOR_INDEX))
        {
            _initialiseFrame();
            _appendBatterySensorData();
            _finaliseFrame();
            sendFrame = true;
        }
#endif

#if USE_GPS_TELEMETRY > 0
        if (currentSchedule & (1 << CRSF_TELEMETRY_FRAME_GPS_INDEX))
        {
            _initialiseFrame();
            _appendGPSData();
            _finaliseFrame();
            sendFrame = true;
        }
#endif

        scheduleIndex = (scheduleIndex + 1) % _telemetryFrameScheduleCount;

        return sendFrame;
    }

    void Telemetry::setAttitudeData(int16_t roll, int16_t pitch, int16_t yaw)
    {
#if USE_ATTITUDE_TELEMETRY > 0
        _telemetryData.attitude.roll = _decidegreeToRadians(roll);
        _telemetryData.attitude.pitch = -_decidegreeToRadians(pitch);
        _telemetryData.attitude.yaw = _decidegreeToRadians(yaw);
#else
        (void)roll;
        (void)pitch;
        (void)yaw;
#endif
    }

    void Telemetry::setBaroAltitudeData(uint16_t altitude, int16_t vario)
    {
#if USE_BARO_ALT_TELEMETRY > 0
        _telemetryData.baroAltitude.altitude = altitude + 10000;
        _telemetryData.baroAltitude.vario = vario;
#else
        (void)altitude;
        (void)vario;
#endif
    }

    void Telemetry::setBatteryData(float voltage, float current, uint32_t capacity, uint8_t percent)
    {
#if USE_BATTERY_TELEMETRY > 0
        _telemetryData.battery.voltage = (voltage + 5) / 10;
        _telemetryData.battery.current = current / 10;
        _telemetryData.battery.capacity = capacity;
        _telemetryData.battery.percent = percent;
#else
        (void)voltage;
        (void)current;
        (void)capacity;
        (void)percent;
#endif
    }

    void Telemetry::setGPSData(float latitude, float longitude, float altitude, float speed, float course, uint8_t satellites)
    {
#if USE_GPS_TELEMETRY > 0
        _telemetryData.gps.latitude = latitude * 10000000;
        _telemetryData.gps.longitude = longitude * 10000000;
        _telemetryData.gps.altitude = (constrain(altitude, 0, 5000 * 100) / 100) + 1000;
        _telemetryData.gps.speed = ((speed * 36 + 50) / 100);
        _telemetryData.gps.groundCourse = (course * 100);
        _telemetryData.gps.satellites = satellites;
#else
        (void)latitude;
        (void)longitude;
        (void)altitude;
        (void)speed;
        (void)course;
        (void)satellites;
#endif
    }

    void Telemetry::sendTelemetryData(DevBoards *db)
    {
        uint8_t *buffer = SerialBuffer::getBuffer();
        size_t length = SerialBuffer::getLength();

        db->write(buffer, length);
    }

    int16_t Telemetry::_decidegreeToRadians(int16_t decidegrees)
    {
        /* convert angle in decidegree to radians/10000 with reducing angle to +/-180 degree range */
        while (decidegrees > 18000)
        {
            decidegrees -= 36000;
        }
        while (decidegrees < -18000)
        {
            decidegrees += 36000;
        }
        return (int16_t)(RAD * 1000.0F * decidegrees);
    }

    void Telemetry::_initialiseFrame()
    {
        SerialBuffer::reset();
        SerialBuffer::writeU8(CRSF_SYNC_BYTE);
    }

    void Telemetry::_appendAttitudeData()
    {
        SerialBuffer::writeU8(CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
        SerialBuffer::writeU8(CRSF_FRAMETYPE_ATTITUDE);

        SerialBuffer::writeU16BE(_telemetryData.attitude.pitch);
        SerialBuffer::writeU16BE(_telemetryData.attitude.roll);
        SerialBuffer::writeU16BE(_telemetryData.attitude.yaw);
    }

    void Telemetry::_appendBaroAltitudeData()
    {
        SerialBuffer::writeU8(CRSF_FRAME_BARO_ALTITUDE_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
        SerialBuffer::writeU8(CRSF_FRAMETYPE_BARO_ALTITUDE);

        SerialBuffer::writeU16BE(_telemetryData.baroAltitude.altitude);
        SerialBuffer::writeU16BE(_telemetryData.baroAltitude.vario);
    }

    void Telemetry::_appendBatterySensorData()
    {
        SerialBuffer::writeU8(CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
        SerialBuffer::writeU8(CRSF_FRAMETYPE_BATTERY_SENSOR);

        SerialBuffer::writeU16BE(_telemetryData.battery.voltage);
        SerialBuffer::writeU16BE(_telemetryData.battery.current);
        SerialBuffer::writeU24BE(_telemetryData.battery.capacity);
        SerialBuffer::writeU8(_telemetryData.battery.percent);
    }

    void Telemetry::_appendGPSData()
    {
        SerialBuffer::writeU8(CRSF_FRAME_GPS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
        SerialBuffer::writeU8(CRSF_FRAMETYPE_GPS);

        SerialBuffer::write32BE(_telemetryData.gps.latitude);
        SerialBuffer::write32BE(_telemetryData.gps.longitude);
        SerialBuffer::writeU16BE(_telemetryData.gps.speed);
        SerialBuffer::writeU16BE(_telemetryData.gps.groundCourse);
        SerialBuffer::writeU16BE(_telemetryData.gps.altitude);
        SerialBuffer::writeU8(_telemetryData.gps.satellites);
    }

    void Telemetry::_finaliseFrame()
    {
        uint8_t *buffer = SerialBuffer::getBuffer();
        uint8_t length = SerialBuffer::getLength();
        uint8_t crc = CRC::calculate(2, buffer[2], buffer, length);

        SerialBuffer::writeU8(crc);
    }
} // namespace serialReceiver
