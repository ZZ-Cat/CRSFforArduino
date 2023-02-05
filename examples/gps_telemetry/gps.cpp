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
char _dmaGpsRxBuffer[GPS_RX_BUFFER_SIZE];
size_t _dmaGpsRxIndex = 0;

Adafruit_ZeroDMA _dmaGpsTx;
volatile bool _dmaGpsTxComplete = false;
char _dmaGpsTxChar = 0;
uint8_t _dmaGpsTxBuffer[GPS_TX_BUFFER_SIZE];
size_t _dmaGpsTxIndex = 0;

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
#if (CRSF_DEBUG_GPS > 0)
    Serial.println("Initializing GPS module...");
#endif

    // Initialize DMA.
    _initDMA();

    // Set GPS Baud Rate.
    if (_gpsNegotiateBaudRate(9600) != true)
    {
#if (CRSF_DEBUG_GPS > 0)
        Serial.println("GPS initialization failed.");
#endif
        return false;
    }

#if (CRSF_DEBUG_GPS > 0)
    Serial.println("GPS initialization complete.");
#endif

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

        return true;
    }

    else
    {
        return false;
    }
}

/**
 * @brief Initializes DMA.
 *
 */
bool GPS::_initDMA()
{
    /* Configure DMA. */
    _dmaGpsRx.setTrigger(SERCOM5_DMAC_ID_RX);
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

    _dmaGpsTx.setTrigger(SERCOM5_DMAC_ID_TX);
    _dmaGpsTx.setAction(DMA_TRIGGER_ACTON_BEAT);
    _dmaGpsTxStatus = _dmaGpsTx.allocate();
    if (_dmaGpsTxStatus != DMA_STATUS_OK)
    {
#if (CRSF_DEBUG_GPS > 0)
        Serial.print("DMA allocation error: ");
        Serial.println(_dmaGpsTxStatus);
#endif
        return false;
    }

    /* Configure the DMA descriptor. */
    _dmaGpsRxDescriptor = _dmaGpsRx.addDescriptor(
        (void *)&SERCOM5->USART.DATA.reg, /* source address */
        &_dmaGpsRxChar,                   /* destination address */
        1,                                /* beat transfer count */
        DMA_BEAT_SIZE_BYTE,               /* beat size */
        false,                            /* increment source address */
        false                             /* increment desination address */
    );
    _dmaGpsRxDescriptor->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;
    _dmaGpsRx.loop(true);

    _dmaGpsTxDescriptor = _dmaGpsTx.addDescriptor(
        _dmaGpsTxBuffer,                  /* source address */
        (void *)&SERCOM5->USART.DATA.reg, /* destination address */
        GPS_TX_BUFFER_SIZE,               /* beat transfer count */
        DMA_BEAT_SIZE_BYTE,               /* beat size */
        true,                             /* increment source address */
        false                             /* increment desination address */
    );

    /* Configure the DMA callback. */
    _dmaGpsRx.setCallback(_dmaGpsRxCallback);
    _dmaGpsTx.setCallback(_dmaGpsTxCallback);

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
 * @param buffer
 * @param length
 */
void GPS::_parseNMEA(const char *buffer, size_t length)
{
#if (CRSF_DEBUG_GPS > 0) && (CRSF_DEBUG_GPS_NMEA > 0)
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

bool GPS::_gpsNegotiateBaudRate(uint32_t targetBaudRate)
{
    /* Intialize the serial port at the lowest baud rate.
    Send "$PMTK000*32\r\n" to the GPS module & wait for a response.
    If the response either times out or is invalid, go to the next baud rate.
    Keep doing this until the GPS module responds with a valid response.
    When a valid response is received, check if the current baud rate matches the target baud rate.
    If the current baud rate does not match the target baud rate, update the GPS module's baud rate.
    Wait until the GPS sends a valid response before continuing.
    If the current baud rate matches the target baud rate, return true. */

#if (CRSF_DEBUG_GPS > 0)
    Serial.println("Negotiating GPS baud rate...");
    Serial.print("Trying baud rate: ");
#endif
    _gpsBaudRateLocked = false;
    for (uint8_t i = 0; i < 7; i++)
    {
        _gpsBaudRate = _gpsBaudRates[i];

#if (CRSF_DEBUG_GPS > 0)
        Serial.println(_gpsBaudRate);
#endif

        // Initialize the serial port.
        _serial->begin(_gpsBaudRate);
        pinPeripheral(_rxPin, PIO_SERCOM);
        pinPeripheral(_txPin, PIO_SERCOM);

        // Initialize the NMEA buffer.
        memset(_dmaGpsNmeaBuffer, 0, GPS_RX_BUFFER_SIZE);

        // Send the initialization command.
        char initCommand[14] = "$PMTK000*32\r\n";
        _gpsWrite(initCommand, sizeof(initCommand));
        if (_gpsWaitForResponse(GPS_RX_TIMEOUT) != true)
        {
            _serial->end();
            pinPeripheral(_rxPin, PIO_DIGITAL);
            pinPeripheral(_txPin, PIO_DIGITAL);

#if (CRSF_DEBUG_GPS > 0)
            if (i >= 6)
            {
                Serial.println("GPS baud rate negotiation failed.");
            }
            else
            {
                Serial.print("GPS response invalid. Trying next baud rate: ");
            }
#endif
        }
        else
        {
            // Check if the GPS module is already using the target baud rate.
            if (_gpsBaudRate == targetBaudRate)
            {
                _gpsBaudRateLocked = true;
#if (CRSF_DEBUG_GPS > 0)
                Serial.print("GPS response valid. Locking onto baud rate: ");
                Serial.println(_gpsBaudRate);
#endif
                break;
            }

            // Update the GPS module's baud rate.
            else
            {
#if (CRSF_DEBUG_GPS > 0)
                Serial.print("GPS response valid. Updating baud rate to: ");
                Serial.println(targetBaudRate);
#endif

                _gpsBaudRateLocked = false;
                char updateBaudRateCommand[21];

                // Build the command string.
                sprintf(updateBaudRateCommand, "$PMTK251,%lu*", targetBaudRate);
                char checksum = 0;
                size_t length = strlen(updateBaudRateCommand);
                for (uint8_t i = 1; i < strlen(updateBaudRateCommand) - 1; i++)
                {
                    checksum ^= updateBaudRateCommand[i];
                }
                sprintf(updateBaudRateCommand, "%s%02X\r\n", updateBaudRateCommand, checksum);

                // Empty the NMEA buffer.
                memset(_dmaGpsNmeaBuffer, 0, GPS_RX_BUFFER_SIZE);

                // Send the command.
                _gpsWrite(updateBaudRateCommand, sizeof(updateBaudRateCommand));

                // Update the serial port's baud rate.
                _serial->end();
                pinPeripheral(_rxPin, PIO_DIGITAL);
                pinPeripheral(_txPin, PIO_DIGITAL);
                _serial->begin(targetBaudRate);
                pinPeripheral(_rxPin, PIO_SERCOM);
                pinPeripheral(_txPin, PIO_SERCOM);

                // Wait for a response.
                if (_gpsWaitForResponse(GPS_RX_TIMEOUT) != true)
                {
                    _serial->end();
                    pinPeripheral(_rxPin, PIO_DIGITAL);
                    pinPeripheral(_txPin, PIO_DIGITAL);

#if (CRSF_DEBUG_GPS > 0)
                    Serial.println("GPS baud rate update failed.");
                    Serial.print("Restarting baud rate negotiation. Trying baud rate: ");
#endif

                    // Restart the for loop.
                    i = 0;
                }
                else
                {
                    _gpsBaudRateLocked = true;
#if (CRSF_DEBUG_GPS > 0)
                    Serial.print("GPS baud rate update successful. Locking onto baud rate: ");
                    Serial.println(targetBaudRate);
#endif
                    break;
                }
            }
        }
    }

    if (_gpsBaudRateLocked != true)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void GPS::_gpsWrite(uint8_t *buffer, size_t length)
{
    /* Copy the data to the DMA buffer. */
    memset(_dmaGpsTxBuffer, 0, GPS_TX_BUFFER_SIZE);
    memcpy(_dmaGpsTxBuffer, buffer, length);

    /* Reset the DMA job complete flag. */
    _dmaGpsTxComplete = false;

    /* Update the DMA descriptor. */
    _dmaGpsTx.changeDescriptor(
        _dmaGpsTxDescriptor,
        _dmaGpsTxBuffer,                  /* source address */
        (void *)&SERCOM5->USART.DATA.reg, /* destination address */
        length                            /* beat transfer count */
    );

    /* Start the DMA job. */
    _dmaGpsTxStatus = _dmaGpsTx.startJob();
    if (_dmaGpsTxStatus != DMA_STATUS_OK)
    {
#if (CRSF_DEBUG_GPS > 0)
        Serial.print("DMA start job error: ");
        Serial.println(_dmaGpsTxStatus);
#endif
    }

    /* Wait for the DMA job to complete. */
    uint32_t _timestamp = millis();
    while (_dmaGpsTxComplete != true)
    {
        if (millis() - _timestamp >= GPS_TX_TIMEOUT)
        {
#if (CRSF_DEBUG_GPS > 0)
            Serial.println("DMA timeout");
#endif
            break;
        }
    }
}

void GPS::_gpsWrite(const char *buffer, size_t length)
{
    _gpsWrite((uint8_t *)buffer, length);
}

bool GPS::_gpsWaitForResponse(uint32_t timeout)
{
    uint32_t _timestamp = millis();
    while (millis() - _timestamp < timeout)
    {
        if (_dmaGpsRxComplete == true)
        {
            _dmaGpsRxComplete = false;

            // Copy the NMEA data.
            memset(_dmaGpsRxBuffer, 0, GPS_RX_BUFFER_SIZE);
            memcpy(_dmaGpsRxBuffer, _dmaGpsNmeaBuffer, _dmaGpsNmeaBufferLength);

#if (CRSF_DEBUG_GPS > 0) && (CRSF_DEBUG_GPS_NMEA > 0)
            Serial.print("GPS module initialization response: ");
            Serial.print(_dmaGpsRxBuffer);
#endif

            // Calculate the checksum of the response.
            uint8_t checksum = 0;
            for (size_t i = 1; i < _dmaGpsNmeaBufferLength - 5; i++)
            {
                checksum ^= _dmaGpsRxBuffer[i];
            }

            // Convert the checksum to a hex string.
            char checksumString[2];
            sprintf(checksumString, "%02X", checksum);

            // Compare the checksums.
            if (strncmp(checksumString, &_dmaGpsRxBuffer[_dmaGpsNmeaBufferLength - 4], 2) == 0)
            {
                // Wait for acknowledgement.
                if (strstr(_dmaGpsRxBuffer, "$PMTK001") != NULL)
                {
                    // Get the acknowledgement code and convert it to an integer.
                    char ackCode[1];
                    strncpy(ackCode, &_dmaGpsRxBuffer[13], 1);
                    uint8_t ack = atoi(ackCode);

                    // Check the acknowledgement code.
                    if (ack == 3)
                    {
                        return true;
                    }

                    // Any other acknowledgement code is an error.
                    else
                    {
#if (CRSF_DEBUG_GPS > 0)
                        Serial.print("GPS module initialization error: ");
                        Serial.println(ack);
#endif
                        return false;
                    }
                }
            }

            // Checksum failed.
            else
            {
                return false;
            }
        }
    }

    return false;
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
            _dmaGpsRxComplete = false;
            memset(_dmaGpsNmeaBuffer, 0, GPS_RX_BUFFER_SIZE);

#if (CRSF_DEBUG_GPS > 0)
            Serial.println("Error: GPS NMEA buffer overflowed.");
#endif
        }
    }
}

void _dmaGpsTxCallback(Adafruit_ZeroDMA *dma)
{
    if (dma == &_dmaGpsTx)
    {
        _dmaGpsTxComplete = true;
    }
}
