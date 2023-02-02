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

/**
 * @brief Construct a new GPS object.
 *
 */
GPS::GPS(HardwareSerial *serial)
{
    _serial = serial;
}

/**
 * @brief Destroy the GPS object.
 *
 */
GPS::~GPS()
{
    _serial = NULL;
}

/**
 * @brief Initialize the GPS sensor.
 *
 */
void GPS::begin()
{
    _serial->begin(9600);
}
