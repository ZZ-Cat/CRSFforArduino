/**
 * @file gps.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief A generic GPS sensor class.
 * @version 0.2.0
 * @date 2023-02-02
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 */

#include "gps.h"
#include "wiring_private.h"

Adafruit_ZeroDMA _dmaGpsRx;
volatile bool _dmaGpsRxComplete = false;
char _dmaGpsRxChar = 0;
uint8_t _dmaGpsRxBuffer[GPS_RX_BUFFER_SIZE];
size_t _dmaGpsRxIndex = 0;

volatile size_t _dmaGpsNmeaBufferLength = 0;
char _dmaGpsNmeaBuffer[GPS_RX_BUFFER_SIZE];

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
        &_dmaGpsRxChar,                   /* destination address */
        1,                                /* beat transfer count */
        DMA_BEAT_SIZE_BYTE,               /* beat size */
        false,                            /* increment source address */
        false                             /* increment desination address */
    );
    _dmaGpsRxDescriptor->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;
    _dmaGpsRx.loop(true);

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

        // Copy the NMEA data.
        memset(_dmaGpsRxBuffer, 0, GPS_RX_BUFFER_SIZE);
        memcpy(_dmaGpsRxBuffer, _dmaGpsNmeaBuffer, _dmaGpsNmeaBufferLength);

        // Parse the NMEA data.
        _parseNMEA(_dmaGpsRxBuffer, _dmaGpsNmeaBufferLength);

        // Update the GPS data.
        data.latitude = _latitude;
        data.longitude = _longitude;
        data.altitude = _altitude;
        data.speed = _speed;
        data.heading = _heading;
        data.satellites = _satellites;

        // Start a new DMA job.
        //         _dmaGpsRxStatus = _dmaGpsRx.startJob();
        //         if (_dmaGpsRxStatus != DMA_STATUS_OK)
        //         {
        // #if (CRSF_DEBUG_GPS > 0)
        //             Serial.print("DMA start job error: ");
        //             Serial.println(_dmaGpsRxStatus);
        // #endif
        //             return false;
        //         }

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
void GPS::_parseNMEA(uint8_t *buffer, size_t length)
{
#if (CRSF_DEBUG_GPS > 0)
    Serial.print("GPS: ");
    for (size_t i = 0; i < length; i++)
    {
        Serial.print((char)buffer[i]);
    }
#endif

    for (size_t i = 0; i < length; i++)
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

    if (dma == &_dmaGpsRx)
    {
        /* Initialize the NMEA buffer. */
        if (_dmaGpsRxChar == '$')
        {
            _dmaGpsNmeaBufferLength = 0;
            _dmaGpsRxIndex = 0;
            memset(_dmaGpsNmeaBuffer, 0, GPS_RX_BUFFER_SIZE);
        }

        /* Fill the NMEA buffer.
        The NMEA buffer is filled until the buffer is full or the NMEA sentence is complete. */
        if (_dmaGpsRxIndex < GPS_RX_BUFFER_SIZE)
        {
            _dmaGpsNmeaBuffer[_dmaGpsRxIndex] = _dmaGpsRxChar;

            if (_dmaGpsNmeaBuffer[0] == '$' && _dmaGpsNmeaBuffer[_dmaGpsRxIndex - 1] == '\r' && _dmaGpsNmeaBuffer[_dmaGpsRxIndex] == '\n')
            {
                _dmaGpsNmeaBufferLength = _dmaGpsRxIndex + 1;
                _dmaGpsRxComplete = true;
            }

            else
            {
                _dmaGpsRxIndex++;
            }
        }

        /* If I get here, the NMEA buffer has overflowed.
        I need to reset the DMA job. */
        else
        {
            _dmaGpsRxIndex = 0;
            _dmaGpsNmeaBufferLength = 0;
            memset(_dmaGpsNmeaBuffer, 0, GPS_RX_BUFFER_SIZE);

#if (CRSF_DEBUG_GPS > 0)
            Serial.println("Error: GPS NMEA buffer overflowed.");
#endif
        }
    }
}
