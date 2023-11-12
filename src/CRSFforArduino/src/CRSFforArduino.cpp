/**
 * @file CRSFforArduino.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief CRSF for Arduino facilitates the use of ExpressLRS RC receivers in Arduino projects.
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

#include "CRSFforArduino.hpp"

namespace sketchLayer
{
    /**
     * @brief Construct a new CRSFforArduino object.
     * 
     */
    CRSFforArduino::CRSFforArduino()
    {
#if CRSF_RC_ENABLED > 0 || CRSF_TELEMETRY_ENABLED > 0
        _serialReceiver = new SerialReceiver();
#endif
    }

    /**
     * @brief Construct a new CRSFforArduino object with the specified RX and TX pins.
     * 
     * @param rxPin 
     * @param txPin 
     */
    CRSFforArduino::CRSFforArduino(uint8_t rxPin, uint8_t txPin)
    {
#if CRSF_RC_ENABLED > 0 || CRSF_TELEMETRY_ENABLED > 0
        _serialReceiver = new SerialReceiver(rxPin, txPin);
#else
        // Prevent compiler warnings
        (void)rxPin;
        (void)txPin;
#endif
    }

    /**
     * @brief Destroy the CRSFforArduino object.
     * 
     */
    CRSFforArduino::~CRSFforArduino()
    {
#if CRSF_RC_ENABLED > 0 || CRSF_TELEMETRY_ENABLED > 0
        delete _serialReceiver;
#endif
    }

    /**
     * @brief Initialises CRSF for Arduino.
     * 
     * @return true if CRSF for Arduino was initialised successfully.
     */
    bool CRSFforArduino::begin()
    {
#if CRSF_RC_ENABLED > 0 || CRSF_TELEMETRY_ENABLED > 0
        return _serialReceiver->begin();
#else
        // Return false if RC is disabled
        return false;
#endif
    }

    /**
     * @brief Ends CRSF for Arduino.
     *
     */
    void CRSFforArduino::end()
    {
#if CRSF_RC_ENABLED > 0 || CRSF_TELEMETRY_ENABLED > 0
        _serialReceiver->end();
#endif
    }

    /**
     * @brief This processes RC and Telemetry frames.
     * It should be called as often as possible.
     *
     */
    void CRSFforArduino::update()
    {
#if CRSF_RC_ENABLED > 0 || CRSF_TELEMETRY_ENABLED > 0
        _serialReceiver->processFrames();
#endif

#if CRSF_RC_ENABLED > 0 && CRSF_FLIGHTMODES_ENABLED > 0
        _serialReceiver->handleFlightMode();
#endif
    }

    /**
     * @brief Reads the specified RC channel.
     * @param channel The channel to read.
     * @param raw If true, returns the raw RC value. If false, returns the scaled RC value in microseconds.
     *
     * @return The RC value.
     */
    uint16_t CRSFforArduino::readRcChannel(uint8_t channel, bool raw)
    {
#if CRSF_RC_ENABLED > 0
        return _serialReceiver->readRcChannel(channel - 1, raw);
#else
        // Prevent compiler warnings
        (void)channel;
        (void)raw;

        // Return 0 if RC is disabled
        return 0;
#endif
    }

    /**
     * @brief Alias for readRcChannel(channel, true).
     * 
     * @param channel The channel to read.
     * @return The RC value.
     */
    uint16_t CRSFforArduino::getChannel(uint8_t channel)
    {
#if CRSF_RC_ENABLED > 0
        return _serialReceiver->getChannel(channel - 1);
#else
        // Prevent compiler warnings
        (void)channel;

        // Return 0 if RC is disabled
        return 0;
#endif
    }

    /**
     * @brief Converts a raw RC value to microseconds.
     * 
     * @param rc The raw RC value to convert.
     * @return The converted RC value in microseconds.
     */
    uint16_t CRSFforArduino::rcToUs(uint16_t rc)
    {
#if CRSF_RC_ENABLED > 0
        return _serialReceiver->rcToUs(rc);
#else
        // Prevent compiler warnings
        (void)rc;

        // Return 0 if RC is disabled
        return 0;
#endif
    }

    /**
     * @brief Assigns a Flight Mode to the specified channel.
     * 
     * @param flightMode The Flight Mode to assign.
     * @param channel The channel to assign the Flight Mode to.
     * @param min The minimum RC value for the Flight Mode to be active.
     * @param max The maximum RC value for the Flight Mode to be active.
     * @return true if the Flight Mode was assigned successfully.
     */
    bool CRSFforArduino::setFlightMode(serialReceiver::flightModeId_t flightMode, uint8_t channel, uint16_t min, uint16_t max)
    {
#if CRSF_RC_ENABLED > 0 && CRSF_FLIGHTMODES_ENABLED > 0
        return _serialReceiver->setFlightMode(flightMode, channel - 1, _serialReceiver->usToRc(min), _serialReceiver->usToRc(max));
#else
        // Prevent compiler warnings
        (void)flightMode;
        (void)channel;
        (void)min;
        (void)max;

        // Return false if RC is disabled
        return false;
#endif
    }

    /**
     * @brief Registers a callback function to be called when a Flight Mode is activated.
     * This is called when the RC value for the Flight Mode channel is between the min and max values.
     * @param callback The callback function to register.
     */
    void CRSFforArduino::setFlightModeCallback(void (*callback)(serialReceiver::flightModeId_t flightMode))
    {
#if CRSF_RC_ENABLED > 0 && CRSF_FLIGHTMODES_ENABLED > 0
        _serialReceiver->setFlightModeCallback(callback);
#else
        // Prevent compiler warnings
        (void)callback;
#endif
    }

    /**
     * @brief Sends a CRSF Telemetry Frame with the current attitude data.
     * 
     * @param roll In decidegrees (eg 15 degrees = 150).
     * @param pitch In decidegrees (eg 20 degrees = 200).
     * @param yaw In decidegrees (eg 30 degrees = 300).
     */
    void CRSFforArduino::telemetryWriteAttitude(int16_t roll, int16_t pitch, int16_t yaw)
    {
#if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_ATTITUDE_ENABLED > 0
        _serialReceiver->telemetryWriteAttitude(roll, pitch, yaw);
#else
        // Prevent compiler warnings
        (void)roll;
        (void)pitch;
        (void)yaw;
#endif
    }

    /**
     * @brief Sends a CRSF Telemetry Frame with the current barometric altitude data.
     * 
     * @param altitude In decimeters (eg 1m = 10)
     * @param vario In centimetres per second (eg 1m/s = 100)
     */
    void CRSFforArduino::telemetryWriteBaroAltitude(uint16_t altitude, int16_t vario)
    {
#if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_BAROALTITUDE_ENABLED > 0
        _serialReceiver->telemetryWriteBaroAltitude(altitude, vario);
#else
        // Prevent compiler warnings
        (void)altitude;
        (void)vario;
#endif
    }

    /**
     * @brief Sends a CRSF Telemetry Frame with the current battery data.
     * 
     * @param voltage In millivolts * 100 (eg 3.8V = 380.0F).
     * @param current In milliamps * 10 (eg 1.5A = 150.0F).
     * @param fuel In milliampere hours (eg 100 mAh = 100).
     * @param percent In percent (eg 50% = 50).
     */
    void CRSFforArduino::telemetryWriteBattery(float voltage, float current, uint32_t fuel, uint8_t percent)
    {
#if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_BATTERY_ENABLED > 0
        _serialReceiver->telemetryWriteBattery(voltage, current, fuel, percent);
#else
        // Prevent compiler warnings
        (void)voltage;
        (void)current;
        (void)fuel;
        (void)percent;
#endif
    }

    /**
     * @brief Sends a CRSF Telemetry Frame with the current Flight Mode.
     * 
     * @param flightMode The Flight Mode to send.
     */
    void CRSFforArduino::telemetryWriteFlightMode(serialReceiver::flightModeId_t flightMode)
    {
#if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_FLIGHTMODE_ENABLED > 0
        _serialReceiver->telemetryWriteFlightMode(flightMode);
#else
        // Prevent compiler warnings
        (void)flightMode;
#endif
    }

    /**
     * @brief Sends a CRSF Telemetry Frame with the current GPS data.
     * 
     * @param latitude In decimal degrees.
     * @param longitude In decimal degrees.
     * @param altitude In centimetres.
     * @param speed in centimeters per second.
     * @param groundCourse In degrees.
     * @param satellites In view.
     */
    void CRSFforArduino::telemetryWriteGPS(float latitude, float longitude, float altitude, float speed, float groundCourse, uint8_t satellites)
    {
#if CRSF_TELEMETRY_ENABLED > 0 && CRSF_TELEMETRY_GPS_ENABLED > 0
        _serialReceiver->telemetryWriteGPS(latitude, longitude, altitude, speed, groundCourse, satellites);
#else
        // Prevent compiler warnings
        (void)latitude;
        (void)longitude;
        (void)altitude;
        (void)speed;
        (void)groundCourse;
        (void)satellites;
#endif
    }
} // namespace sketchLayer
