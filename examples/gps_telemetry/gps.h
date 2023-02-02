/**
 * @file gps.h
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief A generic GPS sensor class.
 * @version 0.1.0
 * @date 2023-02-01
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 */

#pragma once

#include "Arduino.h"
#include "Adafruit_ZeroDMA.h"
#include "CRSFconfig.h"
#include "TinyGPS++.h"

#define GPS_RX_BUFFER_SIZE 255

class GPS
{
public:
    typedef struct gps_data_s
    {
        float latitude;
        float longitude;
        float altitude;
        float speed;
        float heading;
        float satellites;
    } gps_data_t;

    gps_data_t data;

    GPS(HardwareSerial *serial, uint32_t rxPin, uint32_t txPin);
    ~GPS();

    bool begin();
    bool update();

    // float getLatitude();
    // float getLongitude();
    // float getAltitude();
    // float getSpeed();
    // float getHeading();
    // float getSatellites();

    // bool isUpdated();

protected:
    HardwareSerial *_serial;

    DmacDescriptor *_dmaGpsRxDescriptor;
    ZeroDMAstatus _dmaGpsRxStatus;
    uint8_t _dmaGpsRxBuffer[GPS_RX_BUFFER_SIZE];

    TinyGPSPlus _gps;

    uint32_t _rxPin;
    uint32_t _txPin;

    float _latitude;
    float _longitude;
    float _altitude;
    float _speed;
    float _heading;
    float _satellites;

    bool _updated;

    void _parseNMEA(uint8_t *buffer, uint8_t length);
};

void _dmaGpsRxCallback(Adafruit_ZeroDMA *dma);
