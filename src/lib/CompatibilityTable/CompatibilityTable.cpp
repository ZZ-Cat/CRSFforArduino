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
