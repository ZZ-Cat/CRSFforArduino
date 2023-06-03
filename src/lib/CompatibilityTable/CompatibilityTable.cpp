/**
 * @file CompatibilityTable.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief Compatibility Table is used to determine if the current device is compatible with CRSF for Arduino.
 * @version 0.3.1
 * @date 2023-06-03
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

#include "CompatibilityTable.h"

CompatibilityTable::CompatibilityTable()
{
#if defined(ARDUINO_ARCH_SAMD)
#if defined(ADAFRUIT_FEATHER_M0)
    device.type.devboard = DEVBOARD_ADAFRUIT_FEATHER_M0;
#elif defined(ADAFRUIT_FEATHER_M0_EXPRESS)
    device.type.devboard = DEVBOARD_ADAFRUIT_FEATHER_M0_EXPRESS;
#elif defined(ADAFRUIT_ITSYBITSY_M0)
    device.type.devboard = DEVBOARD_ADAFRUIT_ITSYBITSY_M0_EXPRESS;
#elif defined(ADAFRUIT_METRO_M0_EXPRESS)
    device.type.devboard = DEVBOARD_ADAFRUIT_METRO_M0_EXPRESS;
#elif defined(ADAFRUIT_QTPY_M0)
    device.type.devboard = DEVBOARD_ADAFRUIT_QTPY_M0;
#elif defined(ADAFRUIT_TRINKET_M0)
    device.type.devboard = DEVBOARD_ADAFRUIT_TRINKET_M0;
#elif defined(ADAFRUIT_FEATHER_M4_EXPRESS)
    device.type.devboard = DEVBOARD_ADAFRUIT_FEATHER_M4_EXPRESS;
#elif defined(ADAFRUIT_GRAND_CENTRAL_M4)
    device.type.devboard = DEVBOARD_ADAFRUIT_GRAND_CENTRAL_M4;
#elif defined(ADAFRUIT_ITSYBITSY_M4_EXPRESS)
    device.type.devboard = DEVBOARD_ADAFRUIT_ITSYBITSY_M4_EXPRESS;
#elif defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE)
    device.type.devboard = DEVBOARD_ADAFRUIT_METRO_M4_AIRLIFT_LITE;
#elif defined(ADAFRUIT_METRO_M4_EXPRESS)
    device.type.devboard = DEVBOARD_ADAFRUIT_METRO_M4_EXPRESS;
#elif defined(ADAFRUIT_FEATHER_M4_CAN)
    device.type.devboard = DEVBOARD_ADAFRUIT_FEATHER_M4_CAN;
#elif defined(ARDUINO_SAMD_MKR1000)
    device.type.devboard = DEVBOARD_ARDUINO_MKR1000;
#elif defined(ARDUINO_SAMD_MKRFox1200)
    device.type.devboard = DEVBOARD_ARDUINO_MKRFox1200;
#elif defined(ARDUINO_SAMD_MKRGSM1400)
    device.type.devboard = DEVBOARD_ARDUINO_MKRGSM1400;
#elif defined(ARDUINO_SAMD_MKRNB1500)
    device.type.devboard = DEVBOARD_ARDUINO_MKRNB1500;
#elif defined(ARDUINO_SAMD_MKRVIDOR4000)
    device.type.devboard = DEVBOARD_ARDUINO_MKRVIDOR4000;
#elif defined(ARDUINO_SAMD_MKRWAN1300)
    device.type.devboard = DEVBOARD_ARDUINO_MKRWAN1300;
#elif defined(ARDUINO_SAMD_MKRWAN1310)
    device.type.devboard = DEVBOARD_ARDUINO_MKRWAN1310;
#elif defined(ARDUINO_SAMD_MKRWIFI1010)
    device.type.devboard = DEVBOARD_ARDUINO_MKRWIFI1010;
#elif defined(ARDUINO_SAMD_MKRZERO)
    device.type.devboard = DEVBOARD_ARDUINO_MKRZERO;
#elif defined(ARDUINO_SAMD_NANO_33_IOT)
    device.type.devboard = DEVBOARD_ARDUINO_NANO_33_IOT;
#elif defined(ARDUINO_SAMD_ZERO)
    device.type.devboard = DEVBOARD_ARDUINO_ZERO;
#else
    device.type.devboard = DEVBOARD_IS_INCOMPATIBLE;
#warning "Devboard not supported. Please check the compatibility table."
#endif // ADAFRUIT_FEATHER_M0 etc
#else
    device.type.devboard = DEVBOARD_IS_INCOMPATIBLE;
#warning "Devboard not supported. Please check the compatibility table."
#endif // ARDUINO_ARCH_SAMD
}

bool CompatibilityTable::isDevboardCompatible(const char *name)
{
    return strcmp(name, deviceNames[DEVBOARD_IS_INCOMPATIBLE]) != 0 ? true : false;
}

const char *CompatibilityTable::getDevboardName()
{
    if (device.type.devboard > DEVBOARD_COUNT)
    {
        return deviceNames[DEVBOARD_IS_INCOMPATIBLE];
    }

    return deviceNames[device.type.devboard];
}
