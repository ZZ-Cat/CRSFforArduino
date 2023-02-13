/**
 * @file unified_sensor.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief A generic unified sensor library, based on Adafruit's Unified Sensor library.
 * @version 0.2.0
 * @date 2023-02-08
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 */
#include "unified_sensor.h"

/**************************************************************************/
/*!
     @brief  Prints sensor information to serial console
 */
/**************************************************************************/
void Unified_Sensor::printSensorDetails(void)
{
    sensor_t sensor;
    getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.print(F("Sensor:       "));
    Serial.println(sensor.name);
    Serial.print(F("Type:         "));
    switch ((sensors_type_t)sensor.type)
    {
        case SENSOR_TYPE_ACCELEROMETER:
            Serial.print(F("Acceleration (m/s2)"));
            break;
        case SENSOR_TYPE_MAGNETIC_FIELD:
            Serial.print(F("Magnetic (uT)"));
            break;
        case SENSOR_TYPE_ORIENTATION:
            Serial.print(F("Orientation (degrees)"));
            break;
        case SENSOR_TYPE_GYROSCOPE:
            Serial.print(F("Gyroscopic (rad/s)"));
            break;
        case SENSOR_TYPE_LIGHT:
            Serial.print(F("Light (lux)"));
            break;
        case SENSOR_TYPE_PRESSURE:
            Serial.print(F("Pressure (hPa)"));
            break;
        case SENSOR_TYPE_PROXIMITY:
            Serial.print(F("Distance (cm)"));
            break;
        case SENSOR_TYPE_GRAVITY:
            Serial.print(F("Gravity (m/s2)"));
            break;
        case SENSOR_TYPE_LINEAR_ACCELERATION:
            Serial.print(F("Linear Acceleration (m/s2)"));
            break;
        case SENSOR_TYPE_ROTATION_VECTOR:
            Serial.print(F("Rotation vector"));
            break;
        case SENSOR_TYPE_RELATIVE_HUMIDITY:
            Serial.print(F("Relative Humidity (%)"));
            break;
        case SENSOR_TYPE_AMBIENT_TEMPERATURE:
            Serial.print(F("Ambient Temp (C)"));
            break;
        case SENSOR_TYPE_OBJECT_TEMPERATURE:
            Serial.print(F("Object Temp (C)"));
            break;
        case SENSOR_TYPE_VOLTAGE:
            Serial.print(F("Voltage (V)"));
            break;
        case SENSOR_TYPE_CURRENT:
            Serial.print(F("Current (mA)"));
            break;
        case SENSOR_TYPE_COLOR:
            Serial.print(F("Color (RGBA)"));
            break;
        case SENSOR_TYPE_TVOC:
            Serial.print(F("Total Volatile Organic Compounds (ppb)"));
            break;
        case SENSOR_TYPE_VOC_INDEX:
            Serial.print(F("Volatile Organic Compounds (Index)"));
            break;
        case SENSOR_TYPE_NOX_INDEX:
            Serial.print(F("Nitrogen Oxides (Index)"));
            break;
    }

    Serial.println();
    Serial.print(F("Driver Ver:   "));
    Serial.println(sensor.version);
    Serial.print(F("Unique ID:    "));
    Serial.println(sensor.sensor_id);
    Serial.print(F("Min Value:    "));
    Serial.println(sensor.min_value);
    Serial.print(F("Max Value:    "));
    Serial.println(sensor.max_value);
    Serial.print(F("Resolution:   "));
    Serial.println(sensor.resolution);
    Serial.println(F("------------------------------------\n"));
}
