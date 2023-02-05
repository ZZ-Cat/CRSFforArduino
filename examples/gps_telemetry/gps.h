/**
 * @file gps.h
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief A generic GPS sensor class.
 * @version 0.2.0
 * @date 2023-02-02
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
#define GPS_TX_BUFFER_SIZE 255
#define GPS_RX_TIMEOUT 1000
#define GPS_TX_TIMEOUT 1000

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

protected:
    HardwareSerial *_serial;

    DmacDescriptor *_dmaGpsRxDescriptor;
    DmacDescriptor *_dmaGpsTxDescriptor;
    ZeroDMAstatus _dmaGpsRxStatus;
    ZeroDMAstatus _dmaGpsTxStatus;

    TinyGPSPlus _gps;

    bool _gpsBaudRateLocked;
    uint8_t _gpsBaudRateIndex;
    uint32_t _gpsBaudRate;
    const uint32_t _gpsBaudRates[7] = {
        4800,
        9600,
        14400,
        19200,
        38400,
        57600,
        115200
    };

    uint32_t _rxPin;
    uint32_t _txPin;

    float _latitude;
    float _longitude;
    float _altitude;
    float _speed;
    float _heading;
    float _satellites;

    bool _updated;

    bool _initDMA();
    void _parseNMEA(const char *buffer, size_t length);

    bool _gpsNegotiateBaudRate(uint32_t targetBaudRate);
    void _gpsWrite(uint8_t *buffer, size_t length);
    void _gpsWrite(const char *buffer, size_t length);
    bool _gpsWaitForResponse(uint32_t timeout);
};

void _dmaGpsRxCallback(Adafruit_ZeroDMA *dma);
void _dmaGpsTxCallback(Adafruit_ZeroDMA *dma);
