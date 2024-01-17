/**
 * @file CompatibilityTable.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This is the implementation of the Compatibility Table.
 * It is used to determine if the target development board is compatible with CRSF for Arduino.
 * @version 1.0.0
 * @date 2024-1-15
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

#include "CompatibilityTable.hpp"
#if defined(ARDUINO) && defined(PLATFORMIO)
#include "./CFA_Config.hpp"
#elif defined(ARDUINO) && !defined(PLATFORMIO)
#include "./CRSFforArduino/src/CFA_Config.hpp"
#endif

namespace hal
{
    /**
     * @brief Constructs a Compatibility Table object
     *
     */
    CompatibilityTable::CompatibilityTable()
    {
// TEMPORARILY DISABLED: Arduino IDE must be 1.7.0 or greater
// #if ARDUINO >= 10700

// Arduino ESP32 Architecture
#if defined(ARDUINO_ARCH_ESP32)

// Adafruit devboards
#if defined(ARDUINO_FEATHER_ESP32)
        device.type.devboard = DEVBOARD_ADAFRUIT_FEATHER_ESP32;
#elif defined(ARDUINO_METRO_ESP32S2)
        device.type.devboard = DEVBOARD_ADAFRUIT_METRO_ESP32S2;
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_NOPSRAM)
        device.type.devboard = DEVBOARD_ADAFRUIT_FEATHER_ESP32S2;
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3)
        device.type.devboard = DEVBOARD_ADAFRUIT_FEATHER_ESP32S3;
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM)
        device.type.devboard = DEVBOARD_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM;
#elif defined(ARDUINO_ADAFRUIT_ITSYBITSY_ESP32)
        device.type.devboard = DEVBOARD_ADAFRUIT_ITSYBITSY_ESP32;
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32C3)
        device.type.devboard = DEVBOARD_ADAFRUIT_QTPY_ESP32C3;
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2)
        device.type.devboard = DEVBOARD_ADAFRUIT_QTPY_ESP32S2;
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM)
        device.type.devboard = DEVBOARD_ADAFRUIT_QTPY_ESP32S3;
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
        device.type.devboard = DEVBOARD_ADAFRUIT_QTPY_ESP32_PICO;

// Espressif devboards.
#elif defined(ARDUINO_ESP32C3_DEV)
        device.type.devboard = DEVBOARD_ESPRESSIF_ESP32C3_DEVKIT;
#elif defined(ARDUINO_ESP32S3_DEV)
        device.type.devboard = DEVBOARD_ESPRESSIF_ESP32S3_DEVKIT;

// Seeed Studio ESP32 devboards.
#elif defined(ARDUINO_XIAO_ESP32C3)
        device.type.devboard = DEVBOARD_SEEEDSTUDIO_XIAO_ESP32C3;
#elif defined(ARDUINO_XIAO_ESP32S3)
        device.type.devboard = DEVBOARD_SEEEDSTUDIO_XIAO_ESP32S3;

// SparkFun ESP32 devboards.
#elif defined(ARDUINO_ESP32_IOT_REDBOARD)
        device.type.devboard = DEVBOARD_SPARKFUN_REDBOARD_ESP32_IOT;
#elif defined(ARDUINO_ESP32_THING)
        device.type.devboard = DEVBOARD_SPARKFUN_THING_ESP32;
#elif defined(ARDUINO_ESP32_THING_PLUS)
        device.type.devboard = DEVBOARD_SPARKFUN_THING_PLUS_ESP32;
#elif defined(ARDUINO_ESP32S2_THING_PLUS)
        device.type.devboard = DEVBOARD_SPARKFUN_THING_PLUS_ESP32S2;

// Arduino devboards
#elif defined(ARDUINO_NANO_ESP32)
        device.type.devboard = DEVBOARD_ARDUINO_NANO_ESP32;
#else
        // The architecture and chip is known, but the board is not.
#warning "Devboard not supported. Please check the compatibility table."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif

// Arduino SAMD Architecture
#elif defined(ARDUINO_ARCH_SAMD)

// Adafruit devboards
#if USB_VID == 0x239A

#if defined(__SAMD21G18A__)
// Adafruit Feather M0
#if USB_PID == 0x800B
        device.type.devboard = DEVBOARD_ADAFRUIT_FEATHER_M0;
// Adafruit Feather M0 Express
#elif USB_PID == 0x801B
        device.type.devboard = DEVBOARD_ADAFRUIT_FEATHER_M0_EXPRESS;
// Adafruit ItsyBitsy M0
#elif USB_PID == 0x800F
        device.type.devboard = DEVBOARD_ADAFRUIT_ITSYBITSY_M0_EXPRESS;
// Adafruit Metro M0 Express
#elif USB_PID == 0x8013
        device.type.devboard = DEVBOARD_ADAFRUIT_METRO_M0_EXPRESS;
// The architecture and chip is known, but the board is not.
#else
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#warning "Devboard not supported. Please check the compatibility table."
#endif

#elif defined(__SAMD51J19A__)
// Adafruit Feather M4 Express
#if USB_PID == 0x8031
        device.type.devboard = DEVBOARD_ADAFRUIT_FEATHER_M4_EXPRESS;
// Adafruit Metro M4 Express
#elif USB_PID == 0x8020
        device.type.devboard = DEVBOARD_ADAFRUIT_METRO_M4_EXPRESS;
// Adafruit Metro M4 AirLift Lite
#elif USB_PID == 0x8037
        device.type.devboard = DEVBOARD_ADAFRUIT_METRO_M4_AIRLIFT_LITE;
// The architecture and chip is known, but the board is not.
#else
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#warning "Devboard not supported. Please check the compatibility table."
#endif

#elif defined(__SAMD51G19A__)
// Adafruit ItsyBitsy M4 Express
#if USB_PID == 0x802B
        device.type.devboard = DEVBOARD_ADAFRUIT_ITSYBITSY_M4_EXPRESS;
// The architecture and chip is known, but the board is not.
#else
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#warning "Devboard not supported. Please check the compatibility table."
#endif

#elif defined(__SAMD51P20A__)
// Adafruit Grand Central M4
#if USB_PID == 0x8020
        device.type.devboard = DEVBOARD_ADAFRUIT_GRAND_CENTRAL_M4;
// The architecture and chip is known, but the board is not.
#else
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#warning "Devboard not supported. Please check the compatibility table."
#endif

#elif defined(__SAME51J19A__)
// Adafruit Feather M4 CAN
#if USB_PID == 0x80CD
        device.type.devboard = DEVBOARD_ADAFRUIT_FEATHER_M4_CAN;
// The architecture and chip is known, but the board is not.
#else
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#warning "Devboard not supported. Please check the compatibility table."
#endif
#else // The architecture is known, but the board and chip are not.
#warning "Devboard not supported. Please check the compatibility table."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD_AND_CHIP;
#endif

// Arduino devboards
#elif USB_VID == 0x2341

#if defined(__SAMD21G18A__)
// Arduino MKRFOX1200
#if USB_PID == 0x8050
        device.type.devboard = DEVBOARD_ARDUINO_MKRFOX1200;
// Arduino MKRGSM1400
#elif USB_PID == 0x8052
        device.type.devboard = DEVBOARD_ARDUINO_MKRGSM1400;
// Arduino MKRNB1500
#elif USB_PID == 0x8055
        device.type.devboard = DEVBOARD_ARDUINO_MKRNB1500;
// Arduino MKRVIDOR4000
#elif USB_PID == 0x8056
        device.type.devboard = DEVBOARD_ARDUINO_MKRVIDOR4000;
// Arduino MKRWAN1300
#elif USB_PID == 0x8053
        device.type.devboard = DEVBOARD_ARDUINO_MKRWAN1300;
// Arduino MKRWAN1310
#elif USB_PID == 0x8059
        device.type.devboard = DEVBOARD_ARDUINO_MKRWAN1310;
// Arduino MKRWiFi1010
#elif USB_PID == 0x8054
        device.type.devboard = DEVBOARD_ARDUINO_MKRWIFI1010;
// Arduino MKRZERO
#elif USB_PID == 0x804F
        device.type.devboard = DEVBOARD_ARDUINO_MKRZERO;
// Arduino Zero
#elif USB_PID == 0x804D
        device.type.devboard = DEVBOARD_ARDUINO_ZERO;
// Arduino Nano 33 IoT
#elif USB_PID == 0x8057
        device.type.devboard = DEVBOARD_ARDUINO_NANO_33_IOT;
// The architecture and chip is known, but the board is not.
#else
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#warning "Devboard not supported. Please check the compatibility table."
#endif
// The architecture is known, but the board and chip are not.
#else
#warning "Devboard not supported. Please check the compatibility table."
device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD_AND_CHIP;
#endif

// Seeed Studio devboards
#elif USB_VID == 0x2886

#if defined(__SAMD21G18A__)
// Seeed Studio XIAO SAMD21
#if USB_PID == 0x802F
        device.type.devboard = DEVBOARD_SEEEDSTUDIO_XIAO_M0;

// The architecture and chip is known, but the board is not.
#else
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#warning "Devboard not supported. Please check the compatibility table."
#endif

#else // The architecture is known, but the board and chip are not.
#warning "Devboard not supported. Please check the compatibility table."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD_AND_CHIP;
#endif

#else // Unable to verify the vendor ID. Board is incompatible.
#warning "Devboard not supported. Please check the compatibility table."
        device.type.devboard = DEVBOARD_IS_INCOMPATIBLE;
#endif // ARDUINO_SAMD_ADAFRUIT

#elif defined(CORE_TEENSY)
#if defined(__MK20DX128__)
#if defined(ARDUINO_TEENSY30)
        device.type.devboard = DEVBOARD_TEENSY_30;
#pragma message "Teensy 3.x is not recommended for new projects. Please consider using Teensy 4.0 or later instead."
#else
        // The architecture and chip is known, but the board is not.
#warning "Devboard not supported. Please check the compatibility table."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif
#elif defined(__MK20DX256__)
/* PlatformIO treats Teensy 3.1 and Teensy 3.2 as the same board, but the Arduino IDE treats them
as two separate boards. To prevent a false negative, check for both boards. */
#if defined(ARDUINO_TEENSY31) || defined(ARDUINO_TEENSY32)
        device.type.devboard = DEVBOARD_TEENSY_31_32;
#pragma message "Teensy 3.x is not recommended for new projects. Please consider using Teensy 4.0 or later instead."
#else // The architecture and chip is known, but the board is not.
#warning "Devboard not supported. Please check the compatibility table."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif
#elif defined(__MK64FX512__)
#if defined(ARDUINO_TEENSY35)
        device.type.devboard = DEVBOARD_TEENSY_35;
#pragma message "Teensy 3.x is not recommended for new projects. Please consider using Teensy 4.0 or later instead."
#else // The architecture and chip is known, but the board is not.
#warning "Devboard not supported. Please check the compatibility table."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif
#elif defined(__MK66FX1M0__)
#if defined(ARDUINO_TEENSY36)
        device.type.devboard = DEVBOARD_TEENSY_36;
#pragma message "Teensy 3.x is not recommended for new projects. Please consider using Teensy 4.0 or later instead."
#else // The architecture and chip is known, but the board is not.
#warning "Devboard not supported. Please check the compatibility table."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif
#elif defined(__IMXRT1062__)
#if defined(ARDUINO_TEENSY40)
        device.type.devboard = DEVBOARD_TEENSY_40;
#elif defined(ARDUINO_TEENSY41)
        device.type.devboard = DEVBOARD_TEENSY_41;
#else // The architecture and chip is known, but the board is not.
#warning "Devboard not supported. Please check the compatibility table."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif
#else // The architecture is known, but the board and chip are not.
#warning "Devboard not supported. Please check the compatibility table."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD_AND_CHIP;
#endif

#else // Unsupported architecture
#error "Unsupported architecture. Please check the compatibility table."
        device.type.devboard = DEVBOARD_IS_INCOMPATIBLE;
#endif // ARDUINO_ARCH_SAMD

        // #else
        // #error "This library requires Arduino IDE 1.7.0 or greater. Please update your IDE."
        // #endif // ARDUINO >= 10700
    }

    CompatibilityTable::~CompatibilityTable()
    {
    }

    /**
     * @brief Determines if the target development board is compatible with CRSF for Arduino.
     *
     * @return true The target development board is compatible with CRSF for Arduino.
     * @return false The target development board is incompatible with CRSF for Arduino.
     */
    bool CompatibilityTable::isDevboardCompatible(const char *name)
    {
        if (strcmp(name, deviceNames[DEVBOARD_IS_INCOMPATIBLE]) == 0)
        {
#if CRSF_DEBUG_ENABLED > 0
            // Debug.
            CRSF_DEBUG_SERIAL_PORT.print("[Compatibility Table | DEBUG]: Board is ");
            CRSF_DEBUG_SERIAL_PORT.println("incompatible.");
#endif

            return false;
        }

        else if (strcmp(name, deviceNames[DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD_AND_CHIP]) == 0)
        {
#if CRSF_DEBUG_ENABLED > 0
            // Warning.
            CRSF_DEBUG_SERIAL_PORT.print("\r\n[Compatibility Table | WARNING]: Board is ");
            CRSF_DEBUG_SERIAL_PORT.println("permissively incompatible (unknown board and chip).");
#endif

            return true;
        }

        else if (strcmp(name, deviceNames[DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD]) == 0)
        {
#if CRSF_DEBUG_ENABLED > 0
            // Warning.
            CRSF_DEBUG_SERIAL_PORT.print("\r\n[Compatibility Table | WARNING]: Board is ");
            CRSF_DEBUG_SERIAL_PORT.println("permissively incompatible (unknown board).");
#endif

            return true;
        }

        else
        {
#if CRSF_DEBUG_ENABLED > 0
            // Debug.
            CRSF_DEBUG_SERIAL_PORT.print("[Compatibility Table | DEBUG]: Board is ");
            CRSF_DEBUG_SERIAL_PORT.println("compatible.");
#endif

            return true;
        }

        // return strcmp(name, deviceNames[DEVBOARD_IS_INCOMPATIBLE]) != 0 ? true : false;
    }

    /**
     * @brief Gets the name of the target development board.
     *
     * @return const char* The name of the target development board.
     */
    const char *CompatibilityTable::getDevboardName()
    {
        if (device.type.devboard >= DEVBOARD_COUNT)
        {
#if CRSF_DEBUG_ENABLED > 0
            // Debug.
            CRSF_DEBUG_SERIAL_PORT.println("\r\n[Compatibility Table | FATAL ERROR]: Board index is out of bounds.");
#endif

            return deviceNames[DEVBOARD_IS_INCOMPATIBLE];
        }

#if CRSF_DEBUG_ENABLED > 0
        // Debug.
        CRSF_DEBUG_SERIAL_PORT.print("\r\n[Compatibility Table | DEBUG]: Board is ");
        CRSF_DEBUG_SERIAL_PORT.println(deviceNames[device.type.devboard]);
#endif

        return deviceNames[device.type.devboard];
    }
} // namespace hal
