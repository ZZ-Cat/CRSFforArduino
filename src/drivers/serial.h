/**
 * @file serial.h
 * @author Cassandra "ZZ Cat" Robinson
 * @brief Hardware serial driver for the Adafruit Metro M4 board.
 * @version 0.1.0
 * @date 2023-01-27
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 */

#pragma once

#include "Arduino.h"

typedef enum serial_port_e
{
    SERIAL_PORT_0 = 0,
    SERIAL_PORT_1,
    SERIAL_PORT_COUNT
} serial_port_t;

class serial
{
public:
    serial(serial_port_t port, uint32_t txPin, uint32_t rxPin);
    ~serial();

    bool begin(uint32_t baudRate, uint32_t config = SERIAL_8N1);
    void end();

    void flush();

    uint8_t available();
    uint8_t read();
    void read(uint8_t *data, uint8_t size);
    
    void write(uint8_t data);
    void write(uint8_t *data, uint8_t size);

private:
};
