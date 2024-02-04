#pragma once

#include "Arduino.h"

#include "../CRC/CRC.hpp"
#include "../CRSF/CRSFProtocol.hpp"
#include "../SerialBuffer/SerialBuffer.hpp"

namespace serialReceiverLayer
{
    class Telemetry : private genericCrc::CRC, private genericStreamBuffer::SerialBuffer
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
        // void setVarioData(float vario);

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
        void _appendFlightModeData();
        void _appendGPSData();
        // void _appendHeartbeatData();
        // void _appendVarioData();
        void _finaliseFrame();
    };
} // namespace serialReceiverLayer
