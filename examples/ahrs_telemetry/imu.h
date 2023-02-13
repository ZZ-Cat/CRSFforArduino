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

#pragma once

#include "i2c.h"
#include "imu_bno055.h"

typedef struct fvec_s
{
    float x;
    float y;
    float z;
} fvec_t;

typedef struct fquat_s
{
    float w;
    float x;
    float y;
    float z;
} fquat_t;

typedef struct feuler_s
{
    float x;
    float y;
    float z;
} feuler_t;

typedef struct IMU_Data_s
{
    fvec_t accel;
    fvec_t gyro;
    fvec_t mag;
    fquat_t quat;
    feuler_t euler;
} IMU_Data_t;

typedef struct IMU_Calibration_s
{
    uint8_t accel;
    uint8_t gyro;
    uint8_t mag;
    uint8_t sys;
} IMU_Calibration_t;

class IMU
{
  public:
    IMU(I2C *);
    ~IMU();

    bool begin();
    bool update();

    bool getData(IMU_Data_t *);

  protected:
    I2C *_i2c;
    IMU_BNO055 *_imu;
    IMU_Data_t _data;
    sensors_event_t _event;
};
