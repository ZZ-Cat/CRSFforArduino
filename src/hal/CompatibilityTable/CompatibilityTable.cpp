/**
 * @file CompatibilityTable.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief The Compatibility Table determines if the target development board is compatible with CRSF for Arduino.
 * @version 1.1.0
 * @date 2024-4-18
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

#include "CompatibilityTable.hpp"
#include "../../CFA_Config.hpp"
#include "Arduino.h"

namespace hal
{
    /**
     * @brief Constructs a Compatibility Table object
     *
     */
    CompatibilityTable::CompatibilityTable()
    {

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
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif

// Raspberry Pi RP2040 Architecture
#elif defined(ARDUINO_ARCH_RP2040)

// Arduino Nano RP2040 Connect
#if defined(ARDUINO_NANO_RP2040_CONNECT)
        device.type.devboard = DEVBOARD_ARDUINO_NANO_RP2040_CONNECT;

// Raspberry Pi Pico
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
        device.type.devboard = DEVBOARD_RASPBERRYPI_PICO;
#else
        // The architecture and chip is known, but the board is not.
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
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
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
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
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
#endif

#elif defined(__SAMD51G19A__)
// Adafruit ItsyBitsy M4 Express
#if USB_PID == 0x802B
        device.type.devboard = DEVBOARD_ADAFRUIT_ITSYBITSY_M4_EXPRESS;
// The architecture and chip is known, but the board is not.
#else
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
#endif

#elif defined(__SAMD51P20A__)
// Adafruit Grand Central M4
#if USB_PID == 0x8020
        device.type.devboard = DEVBOARD_ADAFRUIT_GRAND_CENTRAL_M4;
// The architecture and chip is known, but the board is not.
#else
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
#endif

#elif defined(__SAME51J19A__)
// Adafruit Feather M4 CAN
#if USB_PID == 0x80CD
        device.type.devboard = DEVBOARD_ADAFRUIT_FEATHER_M4_CAN;
// The architecture and chip is known, but the board is not.
#else
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
#endif
#else // The architecture is known, but the board and chip are not.
#warning "The target board and the chipset that it's using are unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
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
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
#endif
// The architecture is known, but the board and chip are not.
#else
#warning "The target board and the chipset that it's using are unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
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
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
#endif

#else // The architecture is known, but the board and chip are not.
#warning "The target board and the chipset that it's using are unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD_AND_CHIP;
#endif

#else // Unable to verify the vendor ID. Board is incompatible.
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
        device.type.devboard = DEVBOARD_IS_INCOMPATIBLE;
#endif // ARDUINO_SAMD_ADAFRUIT

#elif defined(ARDUINO_ARCH_STM32) || defined(TARGET_STM)

#if defined(STM32F1xx)
#if defined(ARDUINO_BLACKPILL_F103C8)
        device.type.devboard = DEVBOARD_STM32_BLACKPILL_STM32F103C8;
#elif defined(ARDUINO_BLUEPILL_F103C6)
        device.type.devboard = DEVBOARD_STM32_BLUEPILL_STM32F103C6;
#elif defined(ARDUINO_BLUEPILL_F103C8)
        device.type.devboard = DEVBOARD_STM32_BLUEPILL_STM32F103C8;
#elif defined(ARDUINO_GENERIC_F103C6TX)
        device.type.devboard = DEVBOARD_STM32F103C6;
#elif defined(ARDUINO_GENERIC_F103C8TX)
        device.type.devboard = DEVBOARD_STM32F103C8;
#elif defined(ARDUINO_GENERIC_F103CBTX)
        device.type.devboard = DEVBOARD_STM32F103CB;
#elif defined(ARDUINO_GENERIC_F103R6TX)
        device.type.devboard = DEVBOARD_STM32F103R6;
#elif defined(ARDUINO_GENERIC_F103R8TX)
        device.type.devboard = DEVBOARD_STM32F103R8;
#elif defined(ARDUINO_GENERIC_F103RBTX)
        device.type.devboard = DEVBOARD_STM32F103RB;
#elif defined(ARDUINO_GENERIC_F103RCTX)
        device.type.devboard = DEVBOARD_STM32F103RC;
#elif defined(ARDUINO_GENERIC_F103RDTX)
        device.type.devboard = DEVBOARD_STM32F103RD;
#elif defined(ARDUINO_GENERIC_F103RETX)
        device.type.devboard = DEVBOARD_STM32F103RE;
#elif defined(ARDUINO_GENERIC_F103RFTX)
        device.type.devboard = DEVBOARD_STM32F103RF;
#elif defined(ARDUINO_GENERIC_F103RGTX)
        device.type.devboard = DEVBOARD_STM32F103RG;
#elif defined(ARDUINO_GENERIC_F103T6UX)
        device.type.devboard = DEVBOARD_STM32F103T6;
#elif defined(ARDUINO_GENERIC_F103T8UX)
        device.type.devboard = DEVBOARD_STM32F103T8;
#elif defined(ARDUINO_GENERIC_F103TBUX)
        device.type.devboard = DEVBOARD_STM32F103TB;
#elif defined(ARDUINO_GENERIC_F103V8TX)
        device.type.devboard = DEVBOARD_STM32F103V8;
#elif defined(ARDUINO_GENERIC_F103VBTX)
        device.type.devboard = DEVBOARD_STM32F103VB;
#elif defined(ARDUINO_GENERIC_F103VCTX)
        device.type.devboard = DEVBOARD_STM32F103VC;
#elif defined(ARDUINO_GENERIC_F103VDTX)
        device.type.devboard = DEVBOARD_STM32F103VD;
#elif defined(ARDUINO_GENERIC_F103VETX)
        device.type.devboard = DEVBOARD_STM32F103VE;
#elif defined(ARDUINO_GENERIC_F103VFTX)
        device.type.devboard = DEVBOARD_STM32F103VF;
#elif defined(ARDUINO_GENERIC_F103VGTX)
        device.type.devboard = DEVBOARD_STM32F103VG;
#elif defined(ARDUINO_GENERIC_F103ZCTX)
        device.type.devboard = DEVBOARD_STM32F103ZC;
#elif defined(ARDUINO_GENERIC_F103ZDTX)
        device.type.devboard = DEVBOARD_STM32F103ZD;
#elif defined(ARDUINO_GENERIC_F103ZETX)
        device.type.devboard = DEVBOARD_STM32F103ZE;
#elif defined(ARDUINO_GENERIC_F103ZFTX)
        device.type.devboard = DEVBOARD_STM32F103ZF;
#elif defined(ARDUINO_GENERIC_F103ZGTX)
        device.type.devboard = DEVBOARD_STM32F103ZG;
#else
        // The architecture and chip is known, but the board is not.
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif

#elif defined(STM32F4xx)
#if defined(ARDUINO_FEATHER_F405)
        device.type.devboard = DEVBOARD_ADAFRUIT_FEATHER_F405;
#elif defined(ARDUINO_NUCLEO_F401RE)
        device.type.devboard = DEVBOARD_ST_NUCLEO_F401RE;
#elif defined(ARDUINO_NUCLEO_F411RE)
        device.type.devboard = DEVBOARD_ST_NUCLEO_F411RE;
#elif defined(ARDUINO_NUCLEO_F429ZI)
        device.type.devboard = DEVBOARD_ST_NUCLEO_F429ZI;
#elif defined(ARDUINO_NUCLEO_F446RE)
        device.type.devboard = DEVBOARD_ST_NUCLEO_F446RE;
#elif defined(ARDUINO_BLACKPILL_F401CC)
        device.type.devboard = DEVBOARD_STM32_BLACKPILL_STM32F401CC;
#elif defined(ARDUINO_BLACKPILL_F411CE)
        device.type.devboard = DEVBOARD_STM32_BLACKPILL_STM32F411CE;
#elif defined(ARDUINO_GENERIC_F401CBUX)
        device.type.devboard = DEVBOARD_STM32F401CB;
#elif defined(ARDUINO_GENERIC_F401CCUX)
        device.type.devboard = DEVBOARD_STM32F401CC;
#elif defined(ARDUINO_GENERIC_F401CDUX)
        device.type.devboard = DEVBOARD_STM32F401CD;
#elif defined(ARDUINO_GENERIC_F401CEUX)
        device.type.devboard = DEVBOARD_STM32F401CE;
#elif defined(ARDUINO_GENERIC_F401RBTX)
        device.type.devboard = DEVBOARD_STM32F401RB;
#elif defined(ARDUINO_GENERIC_F401RCTX)
        device.type.devboard = DEVBOARD_STM32F401RC;
#elif defined(ARDUINO_GENERIC_F401RDTX)
        device.type.devboard = DEVBOARD_STM32F401RD;
#elif defined(ARDUINO_GENERIC_F401RETX)
        device.type.devboard = DEVBOARD_STM32F401RE;
#elif defined(ARDUINO_GENERIC_F405OEYX)
        device.type.devboard = DEVBOARD_STM32F405OE;
#elif defined(ARDUINO_GENERIC_F405OGYX)
        device.type.devboard = DEVBOARD_STM32F405OG;
#elif defined(ARDUINO_GENERIC_F405RGTX)
        device.type.devboard = DEVBOARD_STM32F405RG;
#elif defined(ARDUINO_GENERIC_F405VGTX)
        device.type.devboard = DEVBOARD_STM32F405VG;
#elif defined(ARDUINO_GENERIC_F405ZGTX)
        device.type.devboard = DEVBOARD_STM32F405ZG;
#elif defined(ARDUINO_GENERIC_F407VETX)
        device.type.devboard = DEVBOARD_STM32F407VE;
#elif defined(ARDUINO_GENERIC_F407VGTX)
        device.type.devboard = DEVBOARD_STM32F407VG;
#elif defined(ARDUINO_GENERIC_F410C8TX)
        device.type.devboard = DEVBOARD_STM32F410C8;
#elif defined(ARDUINO_GENERIC_F410CBTX)
        device.type.devboard = DEVBOARD_STM32F410CB;
#elif defined(ARDUINO_GENERIC_F410R8TX)
        device.type.devboard = DEVBOARD_STM32F410R8;
#elif defined(ARDUINO_GENERIC_F410RBTX)
        device.type.devboard = DEVBOARD_STM32F410RB;
#elif defined(ARDUINO_GENERIC_F411CEUX)
        device.type.devboard = DEVBOARD_STM32F411CE;
#elif defined(ARDUINO_GENERIC_F411RCTX)
        device.type.devboard = DEVBOARD_STM32F411RC;
#elif defined(ARDUINO_GENERIC_F411RETX)
        device.type.devboard = DEVBOARD_STM32F411RE;
#elif defined(ARDUINO_GENERIC_F412CEUX)
        device.type.devboard = DEVBOARD_STM32F412CE;
#elif defined(ARDUINO_GENERIC_F412CGUX)
        device.type.devboard = DEVBOARD_STM32F412CG;
#elif defined(ARDUINO_GENERIC_F412RETX)
        device.type.devboard = DEVBOARD_STM32F412RE;
#elif defined(ARDUINO_GENERIC_F412RGTX)
        device.type.devboard = DEVBOARD_STM32F412RG;
#elif defined(ARDUINO_GENERIC_F413CGUX)
        device.type.devboard = DEVBOARD_STM32F413CG;
#elif defined(ARDUINO_GENERIC_F413CHUX)
        device.type.devboard = DEVBOARD_STM32F413CH;
#elif defined(ARDUINO_GENERIC_F413RGTX)
        device.type.devboard = DEVBOARD_STM32F413RG;
#elif defined(ARDUINO_GENERIC_F413RHUX)
        device.type.devboard = DEVBOARD_STM32F413RH;
#elif defined(ARDUINO_GENERIC_F415RGTX)
        device.type.devboard = DEVBOARD_STM32F415RG;
#elif defined(ARDUINO_GENERIC_F417VETX)
        device.type.devboard = DEVBOARD_STM32F417VE;
#elif defined(ARDUINO_GENERIC_F417VGTX)
        device.type.devboard = DEVBOARD_STM32F417VG;
#elif defined(ARDUINO_GENERIC_F423CHUX)
        device.type.devboard = DEVBOARD_STM32F423CH;
#elif defined(ARDUINO_GENERIC_F423RHTX)
        device.type.devboard = DEVBOARD_STM32F423RH;
#elif defined(ARDUINO_GENERIC_F446RCTX)
        device.type.devboard = DEVBOARD_STM32F446RC;
#elif defined(ARDUINO_GENERIC_F446RETX)
        device.type.devboard = DEVBOARD_STM32F446RE;
#elif defined(ARDUINO_SPARKFUN_MICROMOD_F405)
        device.type.devboard = DEVBOARD_SPARKFUN_MICROMOD_F405;
#elif defined(ARDUINO_BLACK_F407VE)
        device.type.devboard = DEVBOARD_ST_BLACK_F407VE;
#elif defined(ARDUINO_BLACK_F407VG)
        device.type.devboard = DEVBOARD_ST_BLACK_F407VG;
#elif defined(ARDUINO_BLACK_F407ZE)
        device.type.devboard = DEVBOARD_ST_BLACK_F407ZE;
#elif defined(ARDUINO_BLACK_F407ZG)
        device.type.devboard = DEVBOARD_ST_BLACK_F407ZG;
#elif defined(ARDUINO_BLUE_F407VE_MINI)
        device.type.devboard = DEVBOARD_ST_BLUE_F407VE_MINI;
#elif defined(ARDUINO_DISCO_F413ZH)
        device.type.devboard = DEVBOARD_ST_DISCOVERY_F413ZH;
#else
        // The architecture and chip is known, but the board is not.
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif

#elif defined(STM32F7xx)

#if defined(ARDUINO_GENERIC_F722ICKX) || defined(ARDUINO_GENERIC_F722ICTX)
        device.type.devboard = DEVBOARD_STM32F722IC;
#elif defined(ARDUINO_GENERIC_F722IEKX) || defined(ARDUINO_GENERIC_F722IETX)
        device.type.devboard = DEVBOARD_STM32F722IE;
#elif defined(ARDUINO_GENERIC_F722RCTX)
        device.type.devboard = DEVBOARD_STM32F722RC;
#elif defined(ARDUINO_GENERIC_F722RETX)
        device.type.devboard = DEVBOARD_STM32F722RE;
#elif defined(ARDUINO_GENERIC_F722VCTX)
        device.type.devboard = DEVBOARD_STM32F722VC;
#elif defined(ARDUINO_GENERIC_F722VETX)
        device.type.devboard = DEVBOARD_STM32F722VE;
#elif defined(ARDUINO_GENERIC_F722ZCTX)
        device.type.devboard = DEVBOARD_STM32F722ZC;
#elif defined(ARDUINO_GENERIC_F722ZETX)
        device.type.devboard = DEVBOARD_STM32F722ZE;
#elif defined(ARDUINO_DISCO_F746NG)
        device.type.devboard = DEVBOARD_ST_DISCOVERY_F746NG;
#elif defined(ARDUINO_NUCLEO_F722ZE)
        device.type.devboard = DEVBOARD_ST_NUCLEO_F722ZE;
#elif defined(ARDUINO_NUCLEO_F746ZG)
        device.type.devboard = DEVBOARD_ST_NUCLEO_F746ZG;
#elif defined(ARDUINO_NUCLEO_F756ZG)
        device.type.devboard = DEVBOARD_ST_NUCLEO_F756ZG;
#elif defined(ARDUINO_NUCLEO_F767ZI)
        device.type.devboard = DEVBOARD_ST_NUCLEO_F767ZI;
#else
        // The architecture and chip is known, but the board is not.
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif

#elif defined(STM32H7xx)

#if defined(ARDUINO_GENERIC_H745BGTX)
        device.type.devboard = DEVBOARD_STM32H745BG;
#elif defined(ARDUINO_GENERIC_H745BITX)
        device.type.devboard = DEVBOARD_STM32H745BI;
#elif defined(ARDUINO_GENERIC_H745IGKX) || defined(ARDUINO_GENERIC_H745IGTX)
        device.type.devboard = DEVBOARD_STM32H745IG;
#elif defined(ARDUINO_GENERIC_H745IIKX) || defined(ARDUINO_GENERIC_H745IITX)
        device.type.devboard = DEVBOARD_STM32H745II;
#elif defined(ARDUINO_GENERIC_H745ZGTX)
        device.type.devboard = DEVBOARD_STM32H745ZG;
#elif defined(ARDUINO_GENERIC_H745ZITX)
        device.type.devboard = DEVBOARD_STM32H745ZI;
#elif defined(ARDUINO_GENERIC_H750VBTX)
        device.type.devboard = DEVBOARD_STM32H750BT;
#elif defined(ARDUINO_NUCLEO_H723ZG)
        device.type.devboard = DEVBOARD_ST_NUCLEO_H723ZG;
#elif defined(ARDUINO_NUCLEO_H743ZI)
        device.type.devboard = DEVBOARD_ST_NUCLEO_H743ZI;
#else
        // The architecture and chip is known, but the board is not.
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif

#elif defined(TARGET_STM32H7) // For some reason, Arduino are using their own define for the STM32H7 series.

#if defined(ARDUINO_NICLA_VISION)
        device.type.devboard = DEVBOARD_ARDUINO_NICLA_VISION;
#elif defined(ARDUINO_OPTA)
        device.type.devboard = DEVBOARD_ARDUINO_OPTA;
#elif defined(ARDUINO_PORTENTA_H7_M4)
        device.type.devboard = DEVBOARD_ARDUINO_PORTENTA_H7_M4;
#elif defined(ARDUINO_PORTENTA_H7_M7)
        device.type.devboard = DEVBOARD_ARDUINO_PORTENTA_H7;
#else
        // The architecture and chip is known, but the board is not.
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif

#else // The architecture is known, but the board and chip are not.
#warning "The target board and the chipset that it's using are unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD_AND_CHIP;
#endif

#elif defined(CORE_TEENSY)
#if defined(__MK20DX128__)
#if defined(ARDUINO_TEENSY30)
        device.type.devboard = DEVBOARD_TEENSY_30;
#pragma message "Teensy 3.x is not recommended for new projects. Please consider using Teensy 4.0 or later instead."
#else
        // The architecture and chip is known, but the board is not.
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif
#elif defined(__MK20DX256__)
/* PlatformIO treats Teensy 3.1 and Teensy 3.2 as the same board, but the Arduino IDE treats them
as two separate boards. To prevent a false negative, check for both boards. */
#if defined(ARDUINO_TEENSY31) || defined(ARDUINO_TEENSY32)
        device.type.devboard = DEVBOARD_TEENSY_31_32;
#pragma message "Teensy 3.x is not recommended for new projects. Please consider using Teensy 4.0 or later instead."
#else // The architecture and chip is known, but the board is not.
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif
#elif defined(__MK64FX512__)
#if defined(ARDUINO_TEENSY35)
        device.type.devboard = DEVBOARD_TEENSY_35;
#pragma message "Teensy 3.x is not recommended for new projects. Please consider using Teensy 4.0 or later instead."
#else // The architecture and chip is known, but the board is not.
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif
#elif defined(__MK66FX1M0__)
#if defined(ARDUINO_TEENSY36)
        device.type.devboard = DEVBOARD_TEENSY_36;
#pragma message "Teensy 3.x is not recommended for new projects. Please consider using Teensy 4.0 or later instead."
#else // The architecture and chip is known, but the board is not.
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif
#elif defined(__IMXRT1062__)
#if defined(ARDUINO_TEENSY40)
        device.type.devboard = DEVBOARD_TEENSY_40;
#elif defined(ARDUINO_TEENSY41)
        device.type.devboard = DEVBOARD_TEENSY_41;
#else // The architecture and chip is known, but the board is not.
#warning "The target board is unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD;
#endif
#else // The architecture is known, but the board and chip are not.
#warning "The target board and the chipset that it's using are unknown. Please enable CRSF_DEBUG_ENABLED and CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT in CFA_Config.hpp for more information."
        device.type.devboard = DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD_AND_CHIP;
#endif

#else // Unsupported architecture
#error "Unsupported architecture. CRSF for Arduino only supports the ESP32, SAMD, and Teensy architectures."
        device.type.devboard = DEVBOARD_IS_INCOMPATIBLE;
#endif // ARDUINO_ARCH_SAMD
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
#if CRSF_DEBUG_ENABLED > 0 && CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT > 0
            // Error.
            CRSF_DEBUG_SERIAL_PORT.println("\r\n[Compatibility Table | ERROR]: The target board is not compatible with CRSF for Arduino.");
#endif

#if defined(F_CPU)
#if F_CPU >= 48000000
#if CRSF_DEBUG_ENABLED > 0 && CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT > 0
            CRSF_DEBUG_SERIAL_PORT.println("\t\t\t       CRSF for Arduino will not run on this board.");

            // Print instructions for requesting board support.
            CRSF_DEBUG_SERIAL_PORT.println("[Compatibility Table | INFO]: If you would like to request support for this board and its chipset, you may do the following:");
            CRSF_DEBUG_SERIAL_PORT.println("[Compatibility Table | INFO]: 1. Go to CRSFforArduino's repository at: http://tinyurl.com/4je6bzv7");
            CRSF_DEBUG_SERIAL_PORT.println("[Compatibility Table | INFO]: 2. Click the Get Started button next to the \"Devboard Compatibility Request\" issue template.");
            CRSF_DEBUG_SERIAL_PORT.println("[Compatibility Table | INFO]: 3. Fill out the template and submit the issue.");
            CRSF_DEBUG_SERIAL_PORT.println("[Compatibility Table | INFO]: Remember to check that your issue does not already exist before submitting it.");
#endif
#else
            // F_CPU is less than 48 MHz.
#if CRSF_DEBUG_ENABLED > 0 && CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT > 0
            CRSF_DEBUG_SERIAL_PORT.println("\r\n[Compatibility Table | ERROR]: The target board's clock speed is less than 48 MHz. CRSF for Arduino requires a clock speed of at least 48 MHz or higher.");
#endif
#endif
#else
            // F_CPU is not defined. This is a fatal error and should not happen under normal circumstances.
#if CRSF_DEBUG_ENABLED > 0 && CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT > 0
            CRSF_DEBUG_SERIAL_PORT.println("\r\n[Compatibility Table | FATAL ERROR]: F_CPU is not defined.");
#endif
#endif

            return false;
        }

        else if (strcmp(name, deviceNames[DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD_AND_CHIP]) == 0)
        {
#if CRSF_DEBUG_ENABLED > 0 && CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT > 0
            // Warning.
            CRSF_DEBUG_SERIAL_PORT.println("\r\n[Compatibility Table | WARNING]: The target board and its chipset may not be compatible with CRSF for Arduino.");
#endif

#if defined(F_CPU)
#if F_CPU >= 48000000
#if CRSF_DEBUG_ENABLED > 0 && CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT > 0
            CRSF_DEBUG_SERIAL_PORT.println("\t\t\t\t The target board and the chipset that it's using are unknown.\r\n\t\t\t\t CRSF for Arduino will attempt to run on this board, but it may not work properly.");

            // Print instructions for requesting board support.
            CRSF_DEBUG_SERIAL_PORT.println("[Compatibility Table | INFO]: If you would like to request support for this board and its chipset, you may do the following:");
            CRSF_DEBUG_SERIAL_PORT.println("[Compatibility Table | INFO]: 1. Go to CRSFforArduino's repository at: http://tinyurl.com/4je6bzv7");
            CRSF_DEBUG_SERIAL_PORT.println("[Compatibility Table | INFO]: 2. Click the Get Started button next to the \"Devboard Compatibility Request\" issue template.");
            CRSF_DEBUG_SERIAL_PORT.println("[Compatibility Table | INFO]: 3. Fill out the template and submit the issue.");
            CRSF_DEBUG_SERIAL_PORT.println("[Compatibility Table | INFO]: Remember to check that your issue does not already exist before submitting it.");
#endif
#else
            // F_CPU is less than 48 MHz.
#if CRSF_DEBUG_ENABLED > 0 && CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT > 0
            CRSF_DEBUG_SERIAL_PORT.println("\r\n[Compatibility Table | ERROR]: The target board's clock speed is less than 48 MHz. CRSF for Arduino requires a clock speed of at least 48 MHz or higher.");
#endif

            return false;
#endif
#else
            // F_CPU is not defined. This is a fatal error and should not happen under normal circumstances.
#if CRSF_DEBUG_ENABLED > 0 && CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT > 0
            CRSF_DEBUG_SERIAL_PORT.println("\r\n[Compatibility Table | FATAL ERROR]: F_CPU is not defined.");
#endif

            return false;
#endif

            return true;
        }

        else if (strcmp(name, deviceNames[DEVBOARD_IS_PERMISSIVELY_INCOMPATIBLE_UNKNOWN_BOARD]) == 0)
        {
#if CRSF_DEBUG_ENABLED > 0 && CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT > 0
            // Warning.
            CRSF_DEBUG_SERIAL_PORT.println("\r\n[Compatibility Table | WARNING]: The target board may not be compatible with CRSF for Arduino.");
#endif

#if defined(F_CPU)
#if F_CPU >= 48000000
#if CRSF_DEBUG_ENABLED > 0 && CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT > 0
            CRSF_DEBUG_SERIAL_PORT.println("\t\t\t\t The target board is unknown.\r\n\t\t\t\t CRSF for Arduino will attempt to run on this board, but it may not work properly.");

            // Print instructions for requesting board support.
            CRSF_DEBUG_SERIAL_PORT.println("[Compatibility Table | INFO]: If you would like to request support for this board and its chipset, you may do the following:");
            CRSF_DEBUG_SERIAL_PORT.println("[Compatibility Table | INFO]: 1. Go to CRSFforArduino's repository at: http://tinyurl.com/4je6bzv7");
            CRSF_DEBUG_SERIAL_PORT.println("[Compatibility Table | INFO]: 2. Click the Get Started button next to the \"Devboard Compatibility Request\" issue template.");
            CRSF_DEBUG_SERIAL_PORT.println("[Compatibility Table | INFO]: 3. Fill out the template and submit the issue.");
            CRSF_DEBUG_SERIAL_PORT.println("[Compatibility Table | INFO]: Remember to check that your issue does not already exist before submitting it.");
#endif
#else
            // F_CPU is less than 48 MHz.
#if CRSF_DEBUG_ENABLED > 0 && CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT > 0
            CRSF_DEBUG_SERIAL_PORT.println("\r\n[Compatibility Table | ERROR]: The target board's clock speed is less than 48 MHz. CRSF for Arduino requires a clock speed of at least 48 MHz or higher.");
#endif

            return false;
#endif
#else
            // F_CPU is not defined. This is a fatal error and should not happen under normal circumstances.
#if CRSF_DEBUG_ENABLED > 0 && CRSF_DEBUG_ENABLE_COMPATIBILITY_TABLE_OUTPUT > 0
            CRSF_DEBUG_SERIAL_PORT.println("\r\n[Compatibility Table | FATAL ERROR]: F_CPU is not defined.");
#endif

            return false;
#endif

            return true;
        }

        else
        {
            return true;
        }
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

        return deviceNames[device.type.devboard];
    }
} // namespace hal
