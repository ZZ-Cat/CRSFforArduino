/**
 * @file CFA_Config.hpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This is the configuration file for CRSF for Arduino.
 * @version 1.1.0
 * @date 2024-4-17
 *
 * @copyright Copyright (c) 2024, Cassandra "ZZ Cat" Robinson. All rights reserved.
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

/* The following defines are used to configure CRSF for Arduino.
You can change these values to suit your needs. */

namespace crsfForArduinoConfig
{
/* CRSFforArduino version
Versioning is done using Semantic Versioning 2.0.0.
See https://semver.org/ for more information. */
#define CRSFFORARDUINO_VERSION       "1.1.0"
#define CRSFFORARDUINO_VERSION_DATE  "2024-4-17"
#define CRSFFORARDUINO_VERSION_MAJOR 1
#define CRSFFORARDUINO_VERSION_MINOR 1
#define CRSFFORARDUINO_VERSION_PATCH 0

/* Failsafe Options
- CRSF_FAILSAFE_LQI_THRESHOLD: The minimum LQI value for the receiver to be considered connected.
- CRSF_FAILSAFE_RSSI_THRESHOLD: The minimum RSSI value for the receiver to be considered connected.
  - NB: It is considered good practice to set this value to the same as the RSSI Sensitivity Limit in your Lua script.
*/
#define CRSF_FAILSAFE_LQI_THRESHOLD  80
#define CRSF_FAILSAFE_RSSI_THRESHOLD 105

/* RC Options
- RC_ENABLED: Enables or disables the RC API.
- RC_MAX_CHANNELS: The maximum number of RC channels to be received.
- RC_CHANNEL_MIN: The minimum value of an RC channel.
- RC_CHANNEL_MAX: The maximum value of an RC channel.
- RC_CHANNEL_CENTER: The center value of an RC channel.
- RC_INITIALISE_CHANNELS: Whether or not to initialise the RC channels to their center values.
- RC_INITIALISE_ARMCHANNEL: When enabled, the arm channel is set to its minimum value.
  - NB: This refers to the Aux1 channel and is intended for use with ExpressLRS receivers.
- RC_INITIALISE_THROTTLECHANNEL: When enabled, the throttle channel is set to its minimum value. */
#define CRSF_RC_ENABLED                    1
#define CRSF_RC_MAX_CHANNELS               16
#define CRSF_RC_CHANNEL_MIN                172
#define CRSF_RC_CHANNEL_MAX                1811
#define CRSF_RC_CHANNEL_CENTER             992
#define CRSF_RC_INITIALISE_CHANNELS        1
#define CRSF_RC_INITIALISE_ARMCHANNEL      1
#define CRSF_RC_INITIALISE_THROTTLECHANNEL 1

/* Flight Modes
Enables or disables the Flight Mode API.
When enabled, you are given an event-driven API that allows you to easily implement flight modes
and assign them to a switch on your controller.
Pro Tip: You can combine the Flight Mode API with the Telemetry API to send flight mode
information back to your controller. */
#define CRSF_FLIGHTMODES_ENABLED 0

/* Custom Flight Modes
Enables or disables the Custom Flight Modes.
When enabled, this allows you to implement flight modes with custom names
and assign them to a switch on your controller. */
#define CRSF_CUSTOM_FLIGHT_MODES_ENABLED 0

/* Telemetry Options
- TELEMETRY_ENABLED: Enables or disables the Telemetry API.
- TELEMETRY_ATTITUDE_ENABLED: Enables or disables attitude telemetry output.
- TELEMETRY_BAROALTITUDE_ENABLED: Enables or disables barometric altitude telemetry output.
- TELEMETRY_BATTERY_ENABLED: Enables or disables battery telemetry output.
- TELEMETRY_FLIGHTMODE_ENABLED: Enables or disables flight mode telemetry output.
- TELEMETRY_GPS_ENABLED: Enables or disables GPS telemetry output.
- TELEMETRY_SIMULATE_ARBITRARY_VALUES: When enabled, arbitrary values are sent for telemetry. */
#define CRSF_TELEMETRY_ENABLED 1

#define CRSF_TELEMETRY_ATTITUDE_ENABLED     1
#define CRSF_TELEMETRY_BAROALTITUDE_ENABLED 1
#define CRSF_TELEMETRY_BATTERY_ENABLED      1

#ifndef CRSF_TELEMETRY_FLIGHTMODE_ENABLED
#define CRSF_TELEMETRY_FLIGHTMODE_ENABLED 0
#endif

#define CRSF_TELEMETRY_GPS_ENABLED 1

#define CRSF_LINK_STATISTICS_ENABLED 1

/* Debug Options
- DEBUG_ENABLED: Enables or disables debug output over the selected serial port.
- CRSF_DEBUG_SERIAL_PORT: The serial port to use for debug output. Usually the native USB port.
- CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT: Enables or disables debug output from the compatibility table. */
#define CRSF_DEBUG_ENABLED                           0
#define CRSF_DEBUG_SERIAL_PORT                       Serial
#define CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT 0

/* All warnings and asserts below this point are to ensure that the configuration is valid. */

/* Compiler warning if both RC and Telemetry are disabled. */
#if CRSF_RC_ENABLED == 0 && CRSF_TELEMETRY_ENABLED == 0
#warning "Both CRSF_RC_ENABLED and CRSF_TELEMETRY_ENABLED are disabled. CRSF for Arduino will not do anything."
#endif

/* Static assert if Flight Modes are enabled, but RC is disabled. */
#if CRSF_FLIGHTMODES_ENABLED == 1 && CRSF_RC_ENABLED == 0
    static_assert(false, "CRSF_FLIGHTMODES_ENABLED is enabled, but CRSF_RC_ENABLED is disabled. Flight Modes require RC to be enabled.");
#endif

/* Static assert if all telemetry options are disabled.
Better to use CRSF_TELEMETRY_ENABLED instead. */
#if CRSF_TELEMETRY_ATTITUDE_ENABLED == 0 && CRSF_TELEMETRY_BAROALTITUDE_ENABLED == 0 && CRSF_TELEMETRY_BATTERY_ENABLED == 0 && CRSF_TELEMETRY_FLIGHTMODE_ENABLED == 0 && CRSF_TELEMETRY_GPS_ENABLED == 0
    static_assert(false, "All telemetry options are disabled. Set CRSF_TELEMETRY_ENABLED to 0 to disable telemetry instead.");
#endif

}; // namespace crsfForArduinoConfig
