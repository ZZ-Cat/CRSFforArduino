/**
 * @file busio_i2c_target.h
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief A BusIO library for I2C Target Mode, based on Adafruit's BusIO library.
 * @version 0.2.0
 * @date 2023-02-08
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 */

#pragma once

#include "Arduino.h"
#include "i2c.h"

///< The class which defines how we will talk to this device over I2C
class BusIO_I2C_Target
{
  public:
    BusIO_I2C_Target(uint8_t addr, I2C *theWire = &Wire);
    uint8_t address(void);
    bool begin(bool addr_detect = true);
    void end(void);
    bool detected(void);

    bool read(uint8_t *buffer, size_t len, bool stop = true);
    bool write(const uint8_t *buffer, size_t len, bool stop = true,
               const uint8_t *prefix_buffer = nullptr, size_t prefix_len = 0);
    bool write_then_read(const uint8_t *write_buffer, size_t write_len,
                         uint8_t *read_buffer, size_t read_len,
                         bool stop = false);
    bool setSpeed(uint32_t desiredclk);

    /*!   @brief  How many bytes we can read in a transaction
     *    @return The size of the Wire receive/transmit buffer */
    size_t maxBufferSize()
    {
        return _maxBufferSize;
    }

  private:
    uint8_t _addr;
    I2C *_wire;
    bool _begun;
    size_t _maxBufferSize;
    bool _read(uint8_t *buffer, size_t len, bool stop);
};
