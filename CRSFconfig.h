/**
 * @file CRSFconfig.h
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief Configurtion header for CRSFforArduino library.
 * @version 0.2.0
 * @date 2023-02-02
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 */

#pragma once

// Uncomment this line to enable debug output.
// #define CRSF_DEBUG
#define CRSF_DEBUG_RC 0        // Set to 1 to enable debug output for RC data.
#define CRSF_DEBUG_TELEMETRY 0 // Set to 1 to enable debug output for telemetry.
#define CRSF_DEBUG_GPS 0       // Set to 1 to enable debug output for GPS data.
#define CRSF_DEBUG_GPS_NMEA 0  // Set to 1 to enable debug output for GPS NMEA data.

#define CRSF_USE_RC 1                          // Set to 1 to enable RC support.
#define CRSF_USE_TELEMETRY 1                   // Set to 1 to enable telemetry support.
#define CRSF_TELEMETRY_DEVICE_GPS 1            // Set to 1 to enable GPS telemetry support.
#define CRSF_TELEMETRY_DEVICE_BATTERY_SENSOR 0 // Set to 1 to enable battery sensor telemetry support.
#define CRSF_TELEMETRY_DEVICE_VARIO_SENSOR 0   // Set to 1 to enable variometer sensor telemetry support.
#define CRSF_TELEMETRY_DEVICE_ATTITUDE 0       // Set to 1 to enable attitude/artificial horizon telemetry support.
