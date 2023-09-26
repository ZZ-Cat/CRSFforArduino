/**
 * @file SerialReceiver.h
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This is the implementation file for the Serial Receiver Interface.
 * @version 0.5.0
 * @date 2023-09-17
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

#include "SerialReceiver.h"

using namespace crsfProtocol;

namespace serialReceiver
{
    SerialReceiver::SerialReceiver()
    {
        _rxPin = 0;
        _txPin = 1;
        board = new DevBoards();
        ct = new CompatibilityTable();
        _rcChannels = new uint16_t[RC_CHANNEL_COUNT];
    }

    SerialReceiver::SerialReceiver(uint8_t rxPin, uint8_t txPin)
    {
        _rxPin = rxPin;
        _txPin = txPin;
        board = new DevBoards();
        ct = new CompatibilityTable();
        _rcChannels = new uint16_t[RC_CHANNEL_COUNT];
    }

    SerialReceiver::~SerialReceiver()
    {
        _rxPin = 0xffu;
        _txPin = 0xffu;
        delete board;
        delete ct;
        delete[] _rcChannels;
    }

    bool SerialReceiver::begin()
    {
        // board->enterCriticalSection();

        // Initialize the RC Channels.
        // Throttle is set to 172 (988us) to prevent the ESCs from arming. All other channels are set to 992 (1500us).
        for (size_t i = 0; i < RC_CHANNEL_COUNT; i++)
        {
            if (i == RC_CHANNEL_THROTTLE)
            {
                _rcChannels[i] = 172;
            }
            else
            {
                _rcChannels[i] = 992;
            }
        }

        // Check if the board is compatible.
        if (ct->isDevboardCompatible(ct->getDevboardName()))
        {
            if (_rxPin == 0xffu && _txPin == 0xffu)
            {
                // board->exitCriticalSection();
                return false;
            }

            board->enterCriticalSection();

            // Initialize the CRSF Protocol.
            crsf = new CRSF();

            // Check that the CRSF Protocol was initialized successfully.
            if (crsf == nullptr)
            {
                board->exitCriticalSection();
                return false;
            }

            crsf->begin();
            crsf->setFrameTime(BAUD_RATE, 10);
            board->setUART(0, _rxPin, _txPin);
            board->begin(BAUD_RATE);

            // Initialise telemetry.
            telemetry = new Telemetry();

            // Check that the telemetry was initialised successfully.
            if (telemetry == nullptr)
            {
                board->exitCriticalSection();
                return false;
            }

            telemetry->begin();

            board->exitCriticalSection();

            // Clear the UART buffer.
            board->flush();
            while (board->available() > 0)
            {
                board->read();
            }
            return true;
        }
        else
        {
            // board->exitCriticalSection();
            return false;
        }
    }

    void SerialReceiver::end()
    {
        board->flush();
        while (board->available() > 0)
        {
            board->read();
        }

        board->enterCriticalSection();
        board->end();
        board->clearUART();

        // Check if the CRSF Protocol was initialized.
        if (crsf != nullptr)
        {
            crsf->end();
            delete crsf;
        }

        // Check if telemetry was initialized.
        if (telemetry != nullptr)
        {
            telemetry->end();
            delete telemetry;
        }
        board->exitCriticalSection();
    }

    void SerialReceiver::processFrames()
    {
        while (board->available() > 0)
        {
            if (crsf->receiveFrames((uint8_t)board->read()))
            {
                flushRemainingFrames();

                // Check if it is time to send telemetry.
                if (telemetry->update())
                {
                    telemetry->sendTelemetryData(board);
                }
            }
        }

        // Update the RC Channels.
        crsf->getRcChannels(_rcChannels);
    }

    void SerialReceiver::flushRemainingFrames()
    {
        board->flush();
        while (board->available() > 0)
        {
            board->read();
        }
    }

    uint16_t SerialReceiver::readRcChannel(uint8_t channel, bool raw)
    {
        if (channel <= 15)
        {
            if (raw == true)
            {
                return _rcChannels[channel];
            }
            else
            {
                /* Convert RC value from raw to microseconds.
                - Mininum: 172 (988us)
                - Middle: 992 (1500us)
                - Maximum: 1811 (2012us)
                - Scale factor = (2012 - 988) / (1811 - 172) = 0.62477120195241
                - Offset = 988 - 172 * 0.62477120195241 = 880.53935326418548
                */
                return (uint16_t)((_rcChannels[channel] * 0.62477120195241F) + 881);
            }
        }
        else
        {
            return 0;
        }
    }

    uint16_t SerialReceiver::getChannel(uint8_t channel)
    {
        return readRcChannel(channel, true);
    }

    uint16_t SerialReceiver::rcToUs(uint16_t rc)
    {
        return (uint16_t)((rc * 0.62477120195241F) + 881);
    }

    void SerialReceiver::telemetryWriteGPS(float latitude, float longitude, float altitude, float speed, float groundCourse, uint8_t satellites)
    {
        telemetry->setGPSData(latitude, longitude, altitude, speed, groundCourse, satellites);
    }
} // namespace serialReceiver
