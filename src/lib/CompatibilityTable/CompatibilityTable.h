#pragma once

#include "Arduino.h"

class CompatibilityTable
{
  public:
    CompatibilityTable();

    bool isDevboardCompatible(const char *name);
    const char *getDevboardName();

  protected:
    typedef enum __ct_devboards_e
    {
        // Unknown device.
        DEVBOARD_IS_INCOMPATIBLE = 0,

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

        DEVBOARD_COUNT
    } __ct_devboards_t;

    typedef struct __ct_devicetypes_s
    {
        __ct_devboards_t devboard;
    } __ct_devicetypes_t;

    typedef struct __ct_devices_s
    {
        __ct_devicetypes_t type;
    } __ct_devices_t;

    __ct_devices_t device;

    const char *deviceNames[DEVBOARD_COUNT] = {
        "Incompatible device",
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
        "Arduino Zero"
    };
};

// CompatibilityTable CT = CompatibilityTable();
