/**
 * @file gps.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief A generic GPS sensor class.
 * @version 0.1.0
 * @date 2023-02-01
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 */

#include "gps.h"
#include "wiring_private.h"

Adafruit_ZeroDMA _dmaGpsRx;
volatile bool _dmaGpsRxComplete = false;

/**
 * @brief Construct a new GPS object.
 *
 */
GPS::GPS(HardwareSerial *serial, uint32_t rxPin, uint32_t txPin)
{
    _serial = serial;
    _rxPin = rxPin;
    _txPin = txPin;
}

/**
 * @brief Destroy the GPS object.
 *
 */
GPS::~GPS()
{
    _serial = NULL;
    pinPeripheral(_rxPin, PIO_DIGITAL);
    pinPeripheral(_txPin, PIO_DIGITAL);
}

/**
 * @brief Initialize the GPS sensor.
 *
 */
bool GPS::begin()
{
    _serial->begin(9600);
    pinPeripheral(_rxPin, PIO_SERCOM);
    pinPeripheral(_txPin, PIO_SERCOM);

    /* Configure DMA. */
    _dmaGpsRx.setTrigger(SERCOM4_DMAC_ID_RX);
    _dmaGpsRx.setAction(DMA_TRIGGER_ACTON_BEAT);
    _dmaGpsRxStatus = _dmaGpsRx.allocate();
    if (_dmaGpsRxStatus != DMA_STATUS_OK)
    {
#if (CRSF_DEBUG_GPS > 0)
        Serial.print("DMA allocation error: ");
        Serial.println(_dmaGpsRxStatus);
#endif
        return false;
    }

    /* Configure the DMA descriptor. */
    _dmaGpsRxDescriptor = _dmaGpsRx.addDescriptor(
        (void *)&SERCOM4->USART.DATA.reg, /* source address */
        _dmaGpsRxBuffer,                  /* destination address */
        GPS_RX_BUFFER_SIZE,               /* beat transfer count */
        DMA_BEAT_SIZE_BYTE,               /* beat size */
        false,                            /* increment source address */
        true                              /* increment desination address */
    );

    /* Configure the DMA callback. */
    _dmaGpsRx.setCallback(_dmaGpsRxCallback);

    /* Start the DMA job. */
    _dmaGpsRxStatus = _dmaGpsRx.startJob();
    if (_dmaGpsRxStatus != DMA_STATUS_OK)
    {
#if (CRSF_DEBUG_GPS > 0)
        Serial.print("DMA start job error: ");
        Serial.println(_dmaGpsRxStatus);
#endif
        return false;
    }

    return true;
}

/**
 * @brief Parse NMEA data.
 *
 * @return true
 * @return false
 */
bool GPS::update()
{
    if (_dmaGpsRxComplete == true)
    {
        _dmaGpsRxComplete = false;

        // Parse the NMEA data.
        _parseNMEA(_dmaGpsRxBuffer, GPS_RX_BUFFER_SIZE);

        // Update the GPS data.
        data.latitude = _latitude;
        data.longitude = _longitude;
        data.altitude = _altitude;
        data.speed = _speed;
        data.heading = _heading;
        data.satellites = _satellites;

        // Clear the DMA buffer.
        memset(_dmaGpsRxBuffer, 0, GPS_RX_BUFFER_SIZE);

        // Start a new DMA job.
        _dmaGpsRxStatus = _dmaGpsRx.startJob();
        if (_dmaGpsRxStatus != DMA_STATUS_OK)
        {
#if (CRSF_DEBUG_GPS > 0)
            Serial.print("DMA start job error: ");
            Serial.println(_dmaGpsRxStatus);
#endif
            return false;
        }

        return true;
    }

    else
    {
        return false;
    }
}

/**
 * @brief Parse NMEA data.
 *
 * @param buffer
 * @param length
 */
void GPS::_parseNMEA(uint8_t *buffer, uint8_t length)
{
#if (CRSF_DEBUG_GPS > 0)
    Serial.print("GPS: ");
    for (uint8_t i = 0; i < length; i++)
    {
        Serial.print((char)buffer[i]);
    }
    Serial.println();
#endif

    for (uint8_t i = 0; i < length; i++)
    {
        _gps.encode(buffer[i]);
    }

    if (_gps.location.isValid())
    {
        _latitude = _gps.location.lat();
        _longitude = _gps.location.lng();
    }

    if (_gps.altitude.isValid())
    {
        _altitude = _gps.altitude.meters();
    }

    if (_gps.speed.isValid())
    {
        _speed = _gps.speed.kmph();
    }

    if (_gps.course.isValid())
    {
        _heading = _gps.course.deg();
    }

    if (_gps.satellites.isValid())
    {
        _satellites = _gps.satellites.value();
    }
}

void _dmaGpsRxCallback(Adafruit_ZeroDMA *dma)
{
    _dmaGpsRxComplete = true;
}
