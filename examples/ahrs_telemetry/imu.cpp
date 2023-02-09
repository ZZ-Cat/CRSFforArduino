/**
 * @file imu.h
 * @author Cassandra "ZZ Cat" Robinson
 * @brief A generic IMU sensor class.
 * @version 0.2.0
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 */

#include "Arduino.h"
#include "imu.h"
#include "unified_sensor.h"
#include "imu_imumaths.h"

IMU::IMU(I2C* i2cBus)
{
    _imu = nullptr;
    _i2c = i2cBus;
}

IMU::~IMU()
{
    if (_imu != nullptr)
    {
        delete _imu;
    }
    _i2c = nullptr;
}

bool IMU::begin()
{
    uint8_t address = 0x00;
    uint8_t id = 0x00;

    // Scan the I2C bus for the IMU.
    _i2c->begin();
    for (uint8_t i = 0; i < 128; i++)
    {
        _i2c->beginTransmission(i);
        if (_i2c->endTransmission() == 0)
        {
            // Got a response, check if it's the right one.
            if (i == 0x28 || i == 0x29)
            {
                // Check the ID register.
                _i2c->beginTransmission(i);
                _i2c->write(0x00);
                _i2c->endTransmission();
                _i2c->requestFrom(i, 1);
                id = _i2c->read();

                // IMU found, it's a BNO055.
                if (id == 0xA0)
                {
                    address = i;
                    break;
                }
            }
        }
    }

    // If I didn't find the IMU, return false.
    if (address == 0x00)
    {
        return false;
    }

    // Create the BNO055 object.
    _imu = new IMU_BNO055(id, address, _i2c);

    // Initialize the IMU.
    if (_imu->begin() != true)
    {
        return false;
    }

    // Set the IMU to use external crystal.
    _imu->setExtCrystalUse(true);

    // Set the IMU to only use the gyroscope.
    _imu->setMode(OPERATION_MODE_GYRONLY);

    return true;
}

bool IMU::update()
{

    static uint32_t sampleTimestamp = millis();

    // Check if it's time to sample the IMU.
    if (millis() - sampleTimestamp >= 10)
    {
        // Get the IMU data.
        _imu->getEvent(&_event, IMU_BNO055::VECTOR_GYROSCOPE);

        // Update the sample timestamp.
        sampleTimestamp = _event.timestamp;

        // Convert the IMU data to floats.
        _data.gyro.x = _event.gyro.x;
        _data.gyro.y = _event.gyro.y;
        _data.gyro.z = _event.gyro.z;

        return true;
    }

    return false;
}

bool IMU::getData(IMU_Data_t* data)
{
    // Copy the data to the output.
    *data = _data;

    return true;
}
