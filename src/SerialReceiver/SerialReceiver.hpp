/**
 * @file SerialReceiver.hpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief The Serial Receiver layer for the CRSF for Arduino library.
 * @version 1.1.0
 * @date 2024-4-17
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

#pragma once

#include "../CFA_Config.hpp"
#include "Arduino.h"
#include "CRSF/CRSF.hpp"
#include "Telemetry/Telemetry.hpp"

namespace serialReceiverLayer
{
    typedef enum flightModeId_e
    {
        FLIGHT_MODE_DISARMED = 0,
        FLIGHT_MODE_ACRO,
        FLIGHT_MODE_WAIT,
        FLIGHT_MODE_FAILSAFE,
        FLIGHT_MODE_GPS_RESCUE,
        FLIGHT_MODE_PASSTHROUGH,
        FLIGHT_MODE_ANGLE,
        FLIGHT_MODE_HORIZON,
        FLIGHT_MODE_AIRMODE,

#if CRSF_CUSTOM_FLIGHT_MODES_ENABLED > 0
        CUSTOM_FLIGHT_MODE1,
        CUSTOM_FLIGHT_MODE2,
        CUSTOM_FLIGHT_MODE3,
        CUSTOM_FLIGHT_MODE4,
        CUSTOM_FLIGHT_MODE5,
        CUSTOM_FLIGHT_MODE6,
        CUSTOM_FLIGHT_MODE7,
        CUSTOM_FLIGHT_MODE8,
#endif
        FLIGHT_MODE_COUNT
    } flightModeId_t;

    typedef struct rcChannels_s
    {
        bool valid;
        bool failsafe;
        uint16_t value[crsfProtocol::RC_CHANNEL_COUNT];
    } rcChannels_t;

    /* Function pointers for callbacks. */
    typedef void (*rcChannelsCallback_t)(rcChannels_t *);
    typedef void (*flightModeCallback_t)(flightModeId_t);
    typedef void (*linkStatisticsCallback_t)(link_statistics_t);

    class SerialReceiver
    {
      public:
        SerialReceiver();
        SerialReceiver(HardwareSerial *hwUartPort);
        SerialReceiver(HardwareSerial *hwUartPort, int8_t rxPin, int8_t txPin);
        virtual ~SerialReceiver();

        bool begin();
        void end();

#if CRSF_RC_ENABLED > 0 || CRSF_TELEMETRY_ENABLED > 0 || CRSF_LINK_STATISTICS_ENABLED > 0
        void processFrames();
#endif

#if CRSF_LINK_STATISTICS_ENABLED > 0
        void setLinkStatisticsCallback(linkStatisticsCallback_t callback);
#endif

#if CRSF_RC_ENABLED > 0
        void setRcChannelsCallback(rcChannelsCallback_t callback);
        uint16_t getChannel(uint8_t channel);
        uint16_t rcToUs(uint16_t rc);
        uint16_t usToRc(uint16_t us);
        uint16_t readRcChannel(uint8_t channel, bool raw = false);

#if CRSF_FLIGHTMODES_ENABLED > 0
        bool setFlightMode(flightModeId_t flightModeId, const char *flightModeName, uint8_t channel, uint16_t min, uint16_t max);
        bool setFlightMode(flightModeId_t flightMode, uint8_t channel, uint16_t min, uint16_t max);
        void setFlightModeCallback(flightModeCallback_t callback);
        void handleFlightMode();
#endif
#endif

#if CRSF_TELEMETRY_ENABLED > 0
        void telemetryWriteAttitude(int16_t roll, int16_t pitch, int16_t yaw);
        void telemetryWriteBaroAltitude(uint16_t altitude, int16_t vario);
        void telemetryWriteBattery(float voltage, float current, uint32_t fuel, uint8_t percent);
        void telemetryWriteFlightMode(flightModeId_t flightMode, bool disarmed = false);
        void telemetryWriteCustomFlightMode(const char *flightMode, bool armed = true);
        void telemetryWriteGPS(float latitude, float longitude, float altitude, float speed, float groundCourse, uint8_t satellites);
#endif

      private:
        CRSF *crsf;
        HardwareSerial *_uart;

        int8_t _rxPin = -1;
        int8_t _txPin = -1;

#if CRSF_TELEMETRY_ENABLED > 0
        Telemetry *telemetry;
#endif

#if CRSF_RC_ENABLED > 0
        rcChannels_t *_rcChannels = nullptr;
        rcChannelsCallback_t _rcChannelsCallback = nullptr;
#endif

#if CRSF_TELEMETRY_ENABLED > 0
        const char *flightModeStr = "ACRO";
#endif

#if CRSF_LINK_STATISTICS_ENABLED > 0
        link_statistics_t _linkStatistics;
        linkStatisticsCallback_t _linkStatisticsCallback = nullptr;
#endif

#if CRSF_RC_ENABLED > 0 && CRSF_FLIGHTMODES_ENABLED > 0
        typedef struct flightMode_s
        {
            const char *name = nullptr;
            uint8_t channel = 0;
            uint16_t min = 0;
            uint16_t max = 0;
        } flightMode_t;

        flightMode_t *_flightModes = nullptr;
        flightModeCallback_t _flightModeCallback = nullptr;
#endif

#if CRSF_RC_ENABLED > 0 || CRSF_TELEMETRY_ENABLED > 0
        void flushRemainingFrames();
#endif
    };
} // namespace serialReceiverLayer
