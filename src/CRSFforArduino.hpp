/**
 * @file CRSFforArduino.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This is the Sketch Layer, which is a simplified API for CRSF for Arduino.
 * It is intended to be used by the user in their sketches.
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
#include "SerialReceiver/SerialReceiver.hpp"

namespace sketchLayer
{
    class CRSFforArduino : private serialReceiverLayer::SerialReceiver
    {
      public:
        CRSFforArduino();
        explicit CRSFforArduino(HardwareSerial *serialPort);
        CRSFforArduino(HardwareSerial *serialPort, int rxPin, int txPin);
        ~CRSFforArduino();
        bool begin(const uint32_t baud_rate = crsfProtocol::BAUD_RATE);
        void end();
        void update();

        // RC channel functions.
        uint16_t getChannel(uint8_t channel);
        uint16_t rcToUs(uint16_t rc);
        uint16_t readRcChannel(uint8_t channel, bool raw = false);
        void setRcChannelsCallback(void (*callback)(serialReceiverLayer::rcChannels_t *rcChannels));

        // Link statistics functions.
        void setLinkStatisticsCallback(void (*callback)(serialReceiverLayer::link_statistics_t linkStatistics));

        // Flight mode functions.
        bool setFlightMode(serialReceiverLayer::flightModeId_t flightModeId, const char *flightModeName, uint8_t channel, uint16_t min, uint16_t max);
        bool setFlightMode(serialReceiverLayer::flightModeId_t flightMode, uint8_t channel, uint16_t min, uint16_t max);
        void setFlightModeCallback(void (*callback)(serialReceiverLayer::flightModeId_t flightMode));

        // Telemetry functions.
        void telemetryWriteAttitude(int16_t roll, int16_t pitch, int16_t yaw);
        void telemetryWriteBaroAltitude(uint16_t altitude, int16_t vario);
        void telemetryWriteBattery(float voltage, float current, uint32_t fuel, uint8_t percent);
        void telemetryWriteFlightMode(serialReceiverLayer::flightModeId_t flightMode, bool disarmed = false);
        void telemetryWriteCustomFlightMode(const char *flightMode, bool armed = false);
        void telemetryWriteGPS(float latitude, float longitude, float altitude, float speed, float groundCourse, uint8_t satellites);

      private:
    };
} // namespace sketchLayer

using namespace sketchLayer;
