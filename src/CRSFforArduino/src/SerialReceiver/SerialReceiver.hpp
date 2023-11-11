/**
 * @file SerialReceiver.hpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This is the header file for the Serial Receiver Interface.
 * @version 0.5.0
 * @date 2023-11-1
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
#include "CRSF/CRSF.hpp"
#include "Telemetry/Telemetry.hpp"
#if defined(ARDUINO) && defined(PLATFORMIO)
#include "Hardware/Hardware.hpp"
#elif defined(ARDUINO) && !defined(PLATFORMIO)
#include "../Hardware/Hardware.hpp"
#endif

namespace serialReceiver
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
        FLIGHT_MODE_COUNT
    } flightModeId_t;

    // Function pointer for Flight Mode Callback
    typedef void (*flightModeCallback_t)(flightModeId_t);

    class SerialReceiver : /* private CRSF, */ private CompatibilityTable, private DevBoards
    {
      public:
        SerialReceiver();
        SerialReceiver(uint8_t rxPin, uint8_t txPin);
        virtual ~SerialReceiver();

        bool begin();
        void end();

#if CRSF_RC_ENABLED > 0 || CRSF_TELEMETRY_ENABLED > 0
        void processFrames();
#endif

#if CRSF_RC_ENABLED > 0
        uint16_t getChannel(uint8_t channel);
        uint16_t rcToUs(uint16_t rc);
        uint16_t usToRc(uint16_t us);
        uint16_t readRcChannel(uint8_t channel, bool raw = false);

#if CRSF_FLIGHTMODES_ENABLED > 0
        bool setFlightMode(flightModeId_t flightMode, uint8_t channel, uint16_t min, uint16_t max);
        void setFlightModeCallback(flightModeCallback_t callback);
        void handleFlightMode();
#endif
#endif

#if CRSF_TELEMETRY_ENABLED > 0
        void telemetryWriteAttitude(int16_t roll, int16_t pitch, int16_t yaw);
        void telemetryWriteBaroAltitude(uint16_t altitude, int16_t vario);
        void telemetryWriteBattery(float voltage, float current, uint32_t fuel, uint8_t percent);
        void telemetryWriteFlightMode(flightModeId_t flightMode);
        void telemetryWriteGPS(float latitude, float longitude, float altitude, float speed, float groundCourse, uint8_t satellites);
#endif

      private:
        CRSF *crsf;
        CompatibilityTable *ct;
        DevBoards *board;

#if CRSF_TELEMETRY_ENABLED > 0
        Telemetry *telemetry;
#endif

        uint8_t _rxPin = 0xffu;
        uint8_t _txPin = 0xffu;

#if CRSF_RC_ENABLED > 0
        uint16_t *_rcChannels;
#endif

#if CRSF_TELEMETRY_ENABLED > 0
        const char *flightModeStr = "ACRO";
#endif

#if CRSF_RC_ENABLED > 0 && CRSF_FLIGHTMODES_ENABLED > 0
        typedef struct flightMode_s
        {
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
} // namespace serialReceiver
