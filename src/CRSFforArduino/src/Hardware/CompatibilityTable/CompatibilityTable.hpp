/**
 * @file CompatibilityTable.hpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This is the Compatibility Table header file.
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

namespace hal
{
    class CompatibilityTable
    {
      public:
        CompatibilityTable();
        virtual ~CompatibilityTable();

        bool isDevboardCompatible(const char *name);
        const char *getDevboardName();

      private:
        typedef enum ct_devboards_e
        {
            // Unknown device.
            DEVBOARD_IS_INCOMPATIBLE = 0,

            // Adafruit ESP32 boards.
            DEVBOARD_ADAFRUIT_FEATHER_ESP32,
            DEVBOARD_ADAFRUIT_FEATHER_ESP32S2,
            DEVBOARD_ADAFRUIT_FEATHER_ESP32S3,
            DEVBOARD_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM,
            DEVBOARD_ADAFRUIT_ITSYBITSY_ESP32,
            DEVBOARD_ADAFRUIT_METRO_ESP32S2,
            DEVBOARD_ADAFRUIT_QTPY_ESP32C3,
            DEVBOARD_ADAFRUIT_QTPY_ESP32_PICO,
            DEVBOARD_ADAFRUIT_QTPY_ESP32S2,
            DEVBOARD_ADAFRUIT_QTPY_ESP32S3,

            // Adafruit SAMD21 boards.
            DEVBOARD_ADAFRUIT_FEATHER_M0,
            DEVBOARD_ADAFRUIT_FEATHER_M0_EXPRESS,
            DEVBOARD_ADAFRUIT_ITSYBITSY_M0_EXPRESS,
            DEVBOARD_ADAFRUIT_METRO_M0_EXPRESS,
            DEVBOARD_ADAFRUIT_QTPY_M0,
            DEVBOARD_ADAFRUIT_TRINKET_M0,

            // Adafruit SAMD51 boards.
            DEVBOARD_ADAFRUIT_FEATHER_M4_EXPRESS,
            DEVBOARD_ADAFRUIT_GRAND_CENTRAL_M4,
            DEVBOARD_ADAFRUIT_ITSYBITSY_M4_EXPRESS,
            DEVBOARD_ADAFRUIT_METRO_M4_AIRLIFT_LITE,
            DEVBOARD_ADAFRUIT_METRO_M4_EXPRESS,

            // Adafruit SAME51 boards.
            DEVBOARD_ADAFRUIT_FEATHER_M4_CAN,

            // Arduino ESP32 boards.
            DEVBOARD_ARDUINO_NANO_ESP32,

            // Arduino SAMD21 boards.
            DEVBOARD_ARDUINO_MKR1000,
            DEVBOARD_ARDUINO_MKRFOX1200,
            DEVBOARD_ARDUINO_MKRGSM1400,
            DEVBOARD_ARDUINO_MKRNB1500,
            DEVBOARD_ARDUINO_MKRVIDOR4000,
            DEVBOARD_ARDUINO_MKRWAN1300,
            DEVBOARD_ARDUINO_MKRWAN1310,
            DEVBOARD_ARDUINO_MKRWIFI1010,
            DEVBOARD_ARDUINO_MKRZERO,
            DEVBOARD_ARDUINO_NANO_33_IOT,
            DEVBOARD_ARDUINO_ZERO,

            // Espresif ESP32 boards.
            DEVBOARD_ESPRESSIF_ESP32C3_DEVKIT,
            DEVBOARD_ESPRESSIF_ESP32S3_DEVKIT,

            // Seeed Studio boards.
            DEVBOARD_SEEEDSTUDIO_XIAO_ESP32C3,
            DEVBOARD_SEEEDSTUDIO_XIAO_ESP32S3,
            DEVBOARD_SEEEDSTUDIO_XIAO_M0,

            // SparkFun Boards
            DEVBOARD_SPARKFUN_REDBOARD_ESP32_IOT,
            DEVBOARD_SPARKFUN_THING_ESP32,
            DEVBOARD_SPARKFUN_THING_PLUS_ESP32,
            DEVBOARD_SPARKFUN_THING_PLUS_ESP32S2,

            // Teensy boards.
            DEVBOARD_TEENSY_30,
            DEVBOARD_TEENSY_31_32,
            DEVBOARD_TEENSY_35,
            DEVBOARD_TEENSY_36,
            DEVBOARD_TEENSY_40,
            DEVBOARD_TEENSY_41,

            DEVBOARD_COUNT
        } ct_devboards_t;

        typedef struct ct_devicetypes_s
        {
            ct_devboards_t devboard;
        } ct_devicetypes_t;

        typedef struct ct_devices_s
        {
            ct_devicetypes_t type;
        } ct_devices_t;

        ct_devices_t device;

        const char *deviceNames[DEVBOARD_COUNT] = {
            "Incompatible device",
            "Adafruit Feather ESP32",
            "Adafruit Feather ESP32-S2",
            "Adafruit Feather ESP32-S3",
            "Adafruit Feather ESP32-S3 (no PSRAM)",
            "Adafruit ItsyBitsy ESP32",
            "Adafruit Metro ESP32-S2",
            "Adafruit QT Py ESP32 C3",
            "Adafruit QT Py ESP32 Pico",
            "Adafruit QT Py ESP32-S2",
            "Adafruit QT Py ESP32-S3",
            "Adafruit Feather M0",
            "Adafruit Feather M0 Express",
            "Adafruit ItsyBitsy M0 Express",
            "Adafruit Metro M0 Express",
            "Adafruit QT Py M0",
            "Adafruit Trinket M0",
            "Adafruit Feather M4 Express",
            "Adafruit Grand Central M4",
            "Adafruit ItsyBitsy M4 Express",
            "Adafruit Metro M4 AirLift Lite",
            "Adafruit Metro M4 Express",
            "Adafruit Feather M4 CAN",
            "Arduino Nano ESP32",
            "Arduino MKR1000",
            "Arduino MKRFOX1200",
            "Arduino MKRGSM1400",
            "Arduino MKRNB1500",
            "Arduino MKRVIDOR4000",
            "Arduino MKRWAN1300",
            "Arduino MKRWAN1310",
            "Arduino MKRWIFI1010",
            "Arduino MKRZERO",
            "Arduino Nano 33 IoT",
            "Arduino Zero",
            "Espressif ESP32-C3 DevKit",
            "Espressif ESP32-S3 DevKit",
            "Seeed Studio Xiao ESP32-C3",
            "Seeed Studio Xiao ESP32-S3",
            "Seeed Studio Xiao SAMD21",
            "SparkFun RedBoard ESP32 IoT",
            "SparkFun Thing ESP32",
            "SparkFun Thing Plus ESP32",
            "SparkFun Thing Plus ESP32-S2",
            "Teensy 3.0",
            "Teensy 3.1/3.2",
            "Teensy 3.5",
            "Teensy 3.6",
            "Teensy 4.0",
            "Teensy 4.1"};
    };
} // namespace hal
