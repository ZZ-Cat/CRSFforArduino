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

#include "Adafruit_ZeroDMA.h"
#include "Arduino.h"
#include "CRSFconfig.h"
#include "TinyGPS++.h"

#define GPS_RX_BUFFER_SIZE 255
#define GPS_TX_BUFFER_SIZE 255
#define GPS_RX_TIMEOUT     1000
#define GPS_TX_TIMEOUT     1000

#define GPS_BAUD_RATE 115200

typedef enum __gps_baud_rate_e
{
    GPS_BAUD_RATE_4800 = 0,
    GPS_BAUD_RATE_9600,
    GPS_BAUD_RATE_14400,
    GPS_BAUD_RATE_19200,
    GPS_BAUD_RATE_38400,
    GPS_BAUD_RATE_57600,
    GPS_BAUD_RATE_115200,
    GPS_BAUD_RATE_COUNT
} gps_baud_rate_t;

typedef enum __gps_update_rate_e
{
    GPS_UPDATE_RATE_INVALID = 0,
    GPS_UPDATE_RATE_1HZ,
    GPS_UPDATE_RATE_2HZ,
    GPS_UPDATE_RATE_5HZ,
    GPS_UPDATE_RATE_10HZ,
    GPS_UPDATE_RATE_COUNT
} gps_update_rate_t;

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
    bool setUpdateRate(gps_update_rate_t updateRate);
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
        115200};

    uint16_t _gpsUpdateRate;
    const uint16_t _gpsUpdateRates[GPS_UPDATE_RATE_COUNT] = {
        0,    // 0 is not a valid update rate
        1000, // 1 Hz
        500,  // 2 Hz
        200,  // 5 Hz
        100   // 10 Hz
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
