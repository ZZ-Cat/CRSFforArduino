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
#include "TinyGPS++.h"

class GPS
{
public:
    GPS(HardwareSerial *serial);
    ~GPS();

    void begin();
    void update();

    float getLatitude();
    float getLongitude();
    float getAltitude();
    float getSpeed();
    float getHeading();
    float getSatellites();

    bool isUpdated();
protected:
    HardwareSerial *_serial;

    float _latitude;
    float _longitude;
    float _altitude;
    float _speed;
    float _heading;
    float _satellites;

    bool _updated;

    void _parseNMEA(uint8_t *buffer, uint8_t length);
};
