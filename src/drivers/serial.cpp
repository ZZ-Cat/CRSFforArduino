/**
 * @file serial.cpp
 * @author Cassandra "ZZ Cat" Robinson
 * @brief Hardware serial driver for the Adafruit Metro M4 board.
 * @version 0.1.0
 * @date 2023-01-27
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 */

#include "serial.h"

serial::serial(serial_port_t port, uint32_t txPin, uint32_t rxPin)
{
}

serial::~serial()
{
}

bool serial::begin(uint32_t baudRate, uint32_t config = SERIAL_8N1)
{
    return false;
}
