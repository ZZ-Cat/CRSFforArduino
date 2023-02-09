/**
 * @file i2c.h
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief A hardware I2C driver for SAMD51, based on the TWI/I2C library for Arduino Zero.
 * @version 0.2.0
 * @date 2023-02-08
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 */

#pragma once

#include "Stream.h"

#include "SERCOM.h"
#include "RingBuffer.h"

#define I2C_BAUDRATE_STANDARD 100000
#define I2C_BAUDRATE_FAST 400000
#define I2C_BAUDRATE_FASTPLUS 1000000
#define I2C_BAUDRATE_HIGH 3400000

#define I2C_SERCOM sercom4
#define I2C_PIN_SDA A4
#define I2C_PIN_SCL A5
#define I2C_IRQ_HANDLER_0 SERCOM4_0_Handler
#define I2C_IRQ_HANDLER_1 SERCOM4_1_Handler
#define I2C_IRQ_HANDLER_2 SERCOM4_2_Handler
#define I2C_IRQ_HANDLER_3 SERCOM4_3_Handler

class I2C: public Stream
{
public:
    I2C(SERCOM* s, uint8_t pinSDA, uint8_t pinSCL);
    void begin(uint32_t baudrate = I2C_BAUDRATE_STANDARD);
    void begin(uint8_t, bool enableGeneralCall = false);
    void end();
    void setClock(uint32_t);

    void beginTransmission(uint8_t);
    uint8_t endTransmission(bool stopBit);
    uint8_t endTransmission(void);

    uint8_t requestFrom(uint8_t address, size_t quantity, bool stopBit);
    uint8_t requestFrom(uint8_t address, size_t quantity);

    size_t write(uint8_t data);
    size_t write(const uint8_t* data, size_t quantity);

    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive(void(*)(int));
    void onRequest(void(*)(void));

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;

    void onService(void);

private:
    SERCOM* sercom;
    uint8_t _uc_pinSDA;
    uint8_t _uc_pinSCL;

    bool transmissionBegun;

    // RX Buffer
    RingBufferN<256> rxBuffer;

    //TX buffer
    RingBufferN<256> txBuffer;
    uint8_t txAddress;

    // Callback user functions
    void (*onRequestCallback)(void);
    void (*onReceiveCallback)(int);
};

extern I2C Wire;
