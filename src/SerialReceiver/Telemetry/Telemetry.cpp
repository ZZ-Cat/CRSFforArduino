#include "CFA_Config.hpp"
#include "Telemetry.hpp"

using namespace crsfProtocol;

namespace serialReceiverLayer
{
#ifndef PI
#define PI  3.1415926535897932384626433832795F
#endif

#ifndef RAD
#define RAD PI / 180.0F
#endif

    Telemetry::Telemetry()
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
        reset();

        uint8_t index = 0;
#if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_ATTITUDE_ENABLED > 0
        _telemetryFrameSchedule[index++] = (1 << CRSF_TELEMETRY_FRAME_ATTITUDE_INDEX);
#endif

#if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_BAROALTITUDE_ENABLED > 0
        _telemetryFrameSchedule[index++] = (1 << CRSF_TELEMETRY_FRAME_BARO_ALTITUDE_INDEX);
#endif

#if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_BATTERY_ENABLED > 0
        _telemetryFrameSchedule[index++] = (1 << CRSF_TELEMETRY_FRAME_BATTERY_SENSOR_INDEX);
#endif

#if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_FLIGHTMODE_ENABLED > 0
        _telemetryFrameSchedule[index++] = (1 << CRSF_TELEMETRY_FRAME_FLIGHT_MODE_INDEX);
#endif

#if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_GPS_ENABLED > 0
        _telemetryFrameSchedule[index++] = (1 << CRSF_TELEMETRY_FRAME_GPS_INDEX);
#endif

        _telemetryFrameScheduleCount = index;
    }

    void Telemetry::end()
    {
        reset();
    }

    bool Telemetry::update()
    {
#if CRSF_TELEMETRY_ENABLED > 0
        bool sendFrame = false;

        static uint8_t scheduleIndex = 0;
        const uint8_t currentSchedule = _telemetryFrameSchedule[scheduleIndex];

#if CRSF_TELEMETRY_ATTITUDE_ENABLED > 0
        if (currentSchedule & (1 << CRSF_TELEMETRY_FRAME_ATTITUDE_INDEX))
        {
            _initialiseFrame();
            _appendAttitudeData();
            _finaliseFrame();
            sendFrame = true;
        }
#endif

#if CRSF_TELEMETRY_BAROALTITUDE_ENABLED > 0
        if (currentSchedule & (1 << CRSF_TELEMETRY_FRAME_BARO_ALTITUDE_INDEX))
        {
            _initialiseFrame();
            _appendBaroAltitudeData();
            _finaliseFrame();
            sendFrame = true;
        }
#endif

#if CRSF_TELEMETRY_BATTERY_ENABLED > 0
        if (currentSchedule & (1 << CRSF_TELEMETRY_FRAME_BATTERY_SENSOR_INDEX))
        {
            _initialiseFrame();
            _appendBatterySensorData();
            _finaliseFrame();
            sendFrame = true;
        }
#endif

#if CRSF_TELEMETRY_FLIGHTMODE_ENABLED > 0
        if (currentSchedule & (1 << CRSF_TELEMETRY_FRAME_FLIGHT_MODE_INDEX))
        {
            _initialiseFrame();
            _appendFlightModeData();
            _finaliseFrame();
            sendFrame = true;
        }
#endif

#if CRSF_TELEMETRY_GPS_ENABLED > 0
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
#else
        return false;
#endif
    }

    void Telemetry::setAttitudeData(int16_t roll, int16_t pitch, int16_t yaw)
    {
#if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_ATTITUDE_ENABLED > 0
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
#if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_BAROALTITUDE_ENABLED > 0
        _telemetryData.baroAltitude.altitude = altitude + 10000;
        _telemetryData.baroAltitude.vario = vario;
#else
        (void)altitude;
        (void)vario;
#endif
    }

    void Telemetry::setBatteryData(float voltage, float current, uint32_t capacity, uint8_t percent)
    {
#if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_BATTERY_ENABLED > 0
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

    void Telemetry::setFlightModeData(const char *flightMode, bool armed)
    {
#if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_FLIGHTMODE_ENABLED > 0
        size_t length = strlen(flightMode);
        memset(_telemetryData.flightMode.flightMode, 0, sizeof(_telemetryData.flightMode.flightMode));
        memcpy(_telemetryData.flightMode.flightMode, flightMode, length);

        if (armed)
        {
            strcat(_telemetryData.flightMode.flightMode, "*");
        }
#else
        (void)flightMode;
#endif
    }

    void Telemetry::setGPSData(float latitude, float longitude, float altitude, float speed, float course, uint8_t satellites)
    {
#if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_GPS_ENABLED > 0
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

    void Telemetry::sendTelemetryData(HardwareSerial *db)
    {
        uint8_t *buffer = getBuffer();
        size_t length = getLength();

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
        reset();
        writeU8(CRSF_SYNC_BYTE);
    }

    void Telemetry::_appendAttitudeData()
    {
        writeU8(CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
        writeU8(CRSF_FRAMETYPE_ATTITUDE);

        writeU16BE(_telemetryData.attitude.pitch);
        writeU16BE(_telemetryData.attitude.roll);
        writeU16BE(_telemetryData.attitude.yaw);
    }

    void Telemetry::_appendBaroAltitudeData()
    {
        writeU8(CRSF_FRAME_BARO_ALTITUDE_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
        writeU8(CRSF_FRAMETYPE_BARO_ALTITUDE);

        writeU16BE(_telemetryData.baroAltitude.altitude);
        writeU16BE(_telemetryData.baroAltitude.vario);
    }

    void Telemetry::_appendBatterySensorData()
    {
        writeU8(CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
        writeU8(CRSF_FRAMETYPE_BATTERY_SENSOR);

        writeU16BE(_telemetryData.battery.voltage);
        writeU16BE(_telemetryData.battery.current);
        writeU24BE(_telemetryData.battery.capacity);
        writeU8(_telemetryData.battery.percent);
    }

    void Telemetry::_appendFlightModeData()
    {
        // Return if the length of the flight mode string is greater than the flight mode payload size.
        size_t length = strlen(_telemetryData.flightMode.flightMode) + 1;
        if (length > CRSF_FRAME_FLIGHT_MODE_PAYLOAD_SIZE)
        {
            return;
        }

        writeU8(length + CRSF_FRAME_LENGTH_TYPE_CRC);
        writeU8(CRSF_FRAMETYPE_FLIGHT_MODE);

        writeString(_telemetryData.flightMode.flightMode);

        writeU8('\0');
    }

    void Telemetry::_appendGPSData()
    {
        writeU8(CRSF_FRAME_GPS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
        writeU8(CRSF_FRAMETYPE_GPS);

        write32BE(_telemetryData.gps.latitude);
        write32BE(_telemetryData.gps.longitude);
        writeU16BE(_telemetryData.gps.speed);
        writeU16BE(_telemetryData.gps.groundCourse);
        writeU16BE(_telemetryData.gps.altitude);
        writeU8(_telemetryData.gps.satellites);
    }

    void Telemetry::_finaliseFrame()
    {
        uint8_t *buffer = getBuffer();
        uint8_t length = getLength();
        uint8_t crc = calculate(2, buffer[2], buffer, length);

        writeU8(crc);
    }
} // namespace serialReceiverLayer
