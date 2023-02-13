/**
 * @file ahrs_telemetry.ino
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This example sketch shows how to send AHRS telemetry data to a CRSF receiver using the CRSFforArduino library.
 * @version 0.2.0
 * @date 2023-02-02
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 */

#include "CRSFforArduino.h"
#include "gps.h"
#include "imu.h"

#define PRINT_SETUP           0
#define PRINT_LOOP            0
#define PRINT_RAW_RC_CHANNELS 0
#define PRINT_GPS_DATA        0
#define PRINT_IMU_DATA        0

#if (CRSF_USE_TELEMETRY == 0)
#error "CRSF_USE_TELEMETRY must be enabled in CRSFconfig.h to use this example."
#elif (CRSF_TELEMETRY_DEVICE_ATTITUDE == 0)
#error "CRSF_TELEMETRY_DEVICE_ATTITUDE must be enabled in CRSFconfig.h to use this example."
#elif (CRSF_TELEMETRY_DEVICE_GPS == 0)
#error "CRSF_TELEMETRY_DEVICE_GPS must be enabled in CRSFconfig.h to use this example."
#endif

// Create a GPS object.
const uint32_t GPSRxPin = 2;
const uint32_t GPSTxPin = 3;
Uart Serial2(&sercom5, GPSRxPin, GPSTxPin, SERCOM_RX_PAD_1, UART_TX_PAD_0);
GPS gps = GPS(&Serial2, GPSRxPin, GPSTxPin);

// Create an IMU object.
IMU Imu = IMU(&Wire);

// Create a CRSFforArduino object.
CRSFforArduino crsf = CRSFforArduino(&Serial1);

void setup()
{
#if (PRINT_SETUP > 0) || (CRSF_DEBUG_ATTITUDE > 0) || \
    (CRSF_DEBUG_GPS > 0) || (CRSF_DEBUG_GPS_NMEA > 0) || (CRSF_DEBUG_RC > 0)
    // Initialize the serial port & wait for the port to open.
    Serial.begin(115200);

    /* The 'while' loop below will prevent the sketch from continuing until the serial port is open.
     * This is important because the sketch will not work if the serial port is not open.
     * If your handset is already connected to your receiver, your handset will call out something along the lines
     * of "Sensor Lost" if the serial port is not open.
     */
    while (!Serial)
    {
        ;
    }

    // Show the user that the sketch is starting.
    Serial.println("AHRS Telemetry Example");
    delay(1000);
#endif

    // Initialize the CRSFforArduino library.
    crsf.begin();

    // Initialize the GPS sensor.
    if (gps.begin() != true)
    {
#if (PRINT_SETUP > 0)
        Serial.println("GPS failed to initialize");
#endif

        // Stop the sketch from continuing.
        while (true)
        {
            ;
        }
    }
    else
    {
        // Set the GPS update rate to 10Hz.
        if (gps.setUpdateRate(GPS_UPDATE_RATE_10HZ) != true)
        {
#if (PRINT_SETUP > 0)
            Serial.println("GPS failed to set update rate");
#endif

            // Stop the sketch from continuing.
            while (true)
            {
                ;
            }
        }
    }

#if (PRINT_SETUP > 0)
    Serial.println("Initializing IMU...");
#endif

    // Initialize the IMU sensor.
    if (Imu.begin() != true)
    {
#if (PRINT_SETUP > 0)
        Serial.println("IMU failed to initialize");
#endif

        // Stop the sketch from continuing.
        while (true)
        {
            ;
        }
    }

#if (PRINT_SETUP > 0)
    // Show the user that the sketch is ready.
    delay(1000);
    Serial.println("Ready");
    delay(1000);
#endif
}

void loop()
{
    // Update the CRSFforArduino library.
    crsf.update();

    // Update the GPS sensor.
    gps.update();

    // Update the IMU sensor.
    static IMU_Data_t imuData;
    if (Imu.update() == true)
    {
        Imu.getData(&imuData);
    }

#if (PRINT_LOOP > 0)
#if (PRINT_GPS_DATA == 0) && (PRINT_IMU_DATA == 0) && (PRINT_RAW_RC_CHANNELS == 0)
#error "No data is being printed. Set PRINT_GPS_DATA, PRINT_IMU_DATA, or PRINT_RAW_RC_CHANNELS to 1 in the sketch."
#endif
    // Schedule printing debug data to the serial port, based on what is enabled.
    // Set the print rate to 5Hz.
    static uint32_t lastDebugPrint = millis();
    if (millis() - lastDebugPrint >= 200)
    {
        lastDebugPrint = millis();

        // Print the raw RC channels.
#if (PRINT_GPS_DATA > 0)
        Serial.print("GPS <Lat: ");
        Serial.print(gps.data.latitude, 6);
        Serial.print(", Lon: ");
        Serial.print(gps.data.longitude, 6);
        Serial.print(", Alt: ");
        Serial.print(gps.data.altitude);
        Serial.print(", Speed: ");
        Serial.print(gps.data.speed);
        Serial.print(", Heading: ");
        Serial.print(gps.data.heading);
        Serial.print(", Satellites: ");
        Serial.print(gps.data.satellites);
        Serial.println(">");
#endif

        // Print the IMU data.
#if (PRINT_IMU_DATA > 0)
        Serial.print("IMU <Euler: ");
        Serial.print(imuData.euler.x);
        Serial.print(", ");
        Serial.print(imuData.euler.y);
        Serial.print(", ");
        Serial.print(imuData.euler.z);
        // Serial.print(", Temp: ");
        // Serial.print(imuData.temp);
        Serial.println(">");
#endif

        // Print the raw RC channels.
#if (PRINT_RAW_RC_CHANNELS > 0)
        Serial.print("RC Channels <A: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(1)));
        Serial.print(", E: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(2)));
        Serial.print(", T: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(3)));
        Serial.print(", R: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(4)));
        Serial.print(", Aux1: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(5)));
        Serial.print(", Aux2: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(6)));
        Serial.print(", Aux3: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(7)));
        Serial.print(", Aux4: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(8)));
        Serial.println(">");
#endif
    }
#endif

    // Schedule writing the telemetry data to the CRSF receiver.
    static uint32_t lastWrite = millis();
    if (millis() - lastWrite >= 20)
    {
        lastWrite = millis();

#if (CRSF_TELEMETRY_DEVICE_ATTITUDE > 0)
        // Write the attitude telemetry.
        crsf.writeAttitudeTelemetry(imuData.euler.x, imuData.euler.y, imuData.euler.z);
#endif

#if (CRSF_TELEMETRY_DEVICE_GPS > 0)
        // Write the GPS telemetry.
        crsf.writeGPStelemetry(gps.data.latitude, gps.data.longitude, gps.data.altitude, gps.data.speed, gps.data.heading, gps.data.satellites);
#endif
    }
}

void SERCOM5_0_Handler()
{
    Serial2.IrqHandler();
}

void SERCOM5_1_Handler()
{
    Serial2.IrqHandler();
}

void SERCOM5_2_Handler()
{
    Serial2.IrqHandler();
}

void SERCOM5_3_Handler()
{
    Serial2.IrqHandler();
}
