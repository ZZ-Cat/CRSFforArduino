/**
 * @file CRSFconfig.h
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief Configurtion header for CRSFforArduino library.
 * @version 0.1.0
 * @date 2023-02-01
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 */

#pragma once

// Uncomment this line to enable debug output.
// #define CRSF_DEBUG

#define CRSF_USE_TELEMETRY 0                   // Set to 1 to enable telemetry support.
#define CRSF_TELEMETRY_DEVICE_GPS 0            // Set to 1 to enable GPS telemetry support.
#define CRSF_TELEMETRY_DEVICE_BATTERY_SENSOR 0 // Set to 1 to enable battery sensor telemetry support.
#define CRSF_TELEMETRY_DEVICE_VARIO_SENSOR 0   // Set to 1 to enable variometer sensor telemetry support.
#define CRSF_TELEMETRY_DEVICE_ATTITUDE 0       // Set to 1 to enable attitude/artificial horizon telemetry support.
