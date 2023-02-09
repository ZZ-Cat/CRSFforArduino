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

#define PRINT_RAW_RC_CHANNELS 0
#define PRINT_GPS_DATA 0
#define PRINT_IMU_DATA 0
#define TEST_IMU 0

#if (CRSF_USE_TELEMETRY == 0)
#error "CRSF_USE_TELEMETRY must be enabled in CRSFconfig.h to use this example."
#elif (CRSF_TELEMETRY_DEVICE_GPS == 0)
#error "CRSF_TELEMETRY_DEVICE_GPS must be enabled in CRSFconfig.h to use this example."
#endif

// Create a GPS object.
const uint32_t GPSRxPin = 2;
const uint32_t GPSTxPin = 3;
Uart Serial2(&sercom5, GPSRxPin, GPSTxPin, SERCOM_RX_PAD_1, UART_TX_PAD_0);
GPS gps = GPS(&Serial2, GPSRxPin, GPSTxPin);

#if (TEST_IMU > 0)
// Create an IMU object.
IMU Imu = IMU(&Wire);
#endif

// Create a CRSFforArduino object.
CRSFforArduino crsf = CRSFforArduino(&Serial1);

void setup()
{
#if (PRINT_RAW_RC_CHANNELS > 0) || (PRINT_GPS_DATA > 0) || \
    (PRINT_IMU_DATA) || (CRSF_DEBUG_ATTITUDE) || (CRSF_DEBUG_GPS > 0)
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
#if (CRSF_DEBUG_GPS > 0) || (PRINT_GPS_DATA > 0) || (PRINT_RAW_RC_CHANNELS > 0)
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
#if (CRSF_DEBUG_GPS > 0) || (PRINT_GPS_DATA > 0) || (PRINT_RAW_RC_CHANNELS > 0)
            Serial.println("GPS failed to set update rate");
#endif

            // Stop the sketch from continuing.
            while (true)
            {
                ;
            }
        }
    }

#if (TEST_IMU > 0)
#if (PRINT_IMU_DATA > 0)
    Serial.println("Initializing IMU...");
#endif
    if (Imu.begin() != true)
    {
#if (PRINT_IMU_DATA > 0)
        Serial.println("IMU failed to initialize");
#endif

        // Stop the sketch from continuing.
        while (true)
        {
            ;
        }
    }
    else
    {
#if (PRINT_IMU_DATA > 0)
        Serial.println("IMU initialized");

        delay(1000);
        Serial.println("Ready");
        delay(1000);

        while (true)
        {
            static IMU_Data_t imuData;
            if (Imu.update() == true)
            {
                Imu.getData(&imuData);
            }

            // Print the IMU data at 5 Hz.
            static uint32_t printTimestamp = millis();
            if (millis() - printTimestamp >= 200)
            {
                printTimestamp = millis();
                Serial.print("Gyro <X: ");
                Serial.print(imuData.gyro.x, 4);
                Serial.print(", Y: ");
                Serial.print(imuData.gyro.y, 4);
                Serial.print(", Z: ");
                Serial.print(imuData.gyro.z, 4);
                Serial.println(">");
            }
        }
#endif
    }
#endif

#if (PRINT_RAW_RC_CHANNELS > 0) || (PRINT_GPS_DATA > 0) || (PRINT_IMU_DATA > 0)
    // Show the user that the sketch is ready.
    delay(1000);
    Serial.println("Ready");
    delay(1000);
#endif
}

void loop()
{
    // Update the CRSFforArduino library.
    if (crsf.update())
    {
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

    // Update the GPS sensor.
    gps.update();

#if (PRINT_GPS_DATA > 0)
    // Schedule printing the GPS data to the serial port.
    static uint32_t lastPrint = millis();
    if (millis() - lastPrint >= 200)
    {
        lastPrint = millis();
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
    }
#endif

#if (CRSF_USE_TELEMETRY > 0) && (CRSF_TELEMETRY_DEVICE_GPS > 0)
    // Schedule writing the GPS telemetry data to the CRSF receiver.
    static uint32_t lastWrite = millis();
    if (millis() - lastWrite >= 20)
    {
        lastWrite = millis();
        crsf.writeGPStelemetry(gps.data.latitude, gps.data.longitude, gps.data.altitude, gps.data.speed, gps.data.heading, gps.data.satellites);
    }
#endif
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
