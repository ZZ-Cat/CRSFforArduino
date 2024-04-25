#if __has_include("Arduino.h")
#include "Arduino.h"
#define ARDUINO_IS_INCLUDED 1
#else
#define ARDUINO_IS_INCLUDED 0
#endif

/* Packet rate enumeration. */
typedef enum packet_rate_index_e
{
    PACKET_RATE_4HZ,
    PACKET_RATE_25HZ,
    PACKET_RATE_50HZ,
    PACKET_RATE_100HZ,
    PACKET_RATE_150HZ,
    PACKET_RATE_200HZ,
    PACKET_RATE_250HZ,
    PACKET_RATE_333HZ,
    PACKET_RATE_500HZ,
    PACKET_RATE_1000HZ,
    PACKET_RATE_COUNT
} packet_rate_index_t;

/* Selected packet rate. */
const packet_rate_index_t selected_packet_rate = PACKET_RATE_50HZ;
uint32_t *packet_rate_us = nullptr;

/* Time structure for the packet rate. */
typedef struct time_s
{
    uint32_t time_us = 0;
    uint32_t time_us_last = 0;
    uint32_t time_us_delta = 0;
    int32_t time_us_error = 0;
    const int32_t time_us_max_allowed_error = 2;
} time_t;

/* CRSF Frame structure and union. */
typedef struct crsf_frame_s
{
    uint8_t sync;
    uint8_t length;
    uint8_t type;
    uint8_t payload[60];
    uint8_t crc;
} crsf_frame_t;

const size_t crsf_frame_size = sizeof(crsf_frame_t);

typedef union crsf_tx_frame_u
{
    crsf_frame_t frame;
    uint8_t buffer[crsf_frame_size];
} crsf_tx_frame_t;

/* Time structure instance. */
time_t *time = nullptr;

crsf_tx_frame_t crsf_tx_frame;

/* Exit handlers. */
void exit_success_handler()
{
    /* Clean up and stop. */
    delete[] packet_rate_us;
    delete time;

    /* Print a message to the serial monitor. */
    Serial.println("Program has ended successfully.");
}

void exit_time_max_allowed_error_handler()
{
    /* Print a message to the serial monitor. */
    Serial.print("Program has ended with an error: ");
    Serial.println("Time error is greater than the maximum allowed error.");

    /* Print how far the time error is from the packet rate in microseconds. */
    Serial.print("Time Error: ");
    Serial.print(time->time_us_error);
    Serial.println(" us");

    /* Clean up and stop. */
    delete[] packet_rate_us;
    delete time;
}

#if ARDUINO_IS_INCLUDED == 1
void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    /* Dynamically allocate memory for the packet rate,
    and calculate the time in microseconds for each packet rate. */
    packet_rate_us = new uint32_t[PACKET_RATE_COUNT];
    packet_rate_us[PACKET_RATE_4HZ] = (1000000UL / 4UL);
    packet_rate_us[PACKET_RATE_25HZ] = (1000000UL / 25UL);
    packet_rate_us[PACKET_RATE_50HZ] = (1000000UL / 50UL);
    packet_rate_us[PACKET_RATE_100HZ] = (1000000UL / 100UL);
    packet_rate_us[PACKET_RATE_150HZ] = (1000000UL / 150UL);
    packet_rate_us[PACKET_RATE_200HZ] = (1000000UL / 200UL);
    packet_rate_us[PACKET_RATE_250HZ] = (1000000UL / 250UL);
    packet_rate_us[PACKET_RATE_333HZ] = (1000000UL / 333UL);
    packet_rate_us[PACKET_RATE_500HZ] = (1000000UL / 500UL);
    packet_rate_us[PACKET_RATE_1000HZ] = (1000000UL / 1000UL);

    /* Initialize the time structure. */
    time = new time_t;

    /* Initialise Serial1 with 1.87M baud rate. */
    Serial1.begin(1875000);
    memset(&crsf_tx_frame, 0, crsf_frame_size);

    /* Print a message to the serial monitor. */
    Serial.println("Testing CRSF Serial Transmitter Prototype...");

    /* Set the time in microseconds. */
    time->time_us = micros();
    time->time_us_last = time->time_us;
}

void loop()
{

    static uint32_t iteration = 0;

    /* Calculate the number of iterations based on the selected packet rate and the equivalent total execution time of three seconds. */
    static const uint32_t iterations = (3000000UL / packet_rate_us[selected_packet_rate]);

    if (iteration < iterations)
    {
        /* Calculate the time delta in microseconds. */
        time->time_us = micros();
        time->time_us_delta = time->time_us - time->time_us_last;

        /* Calculate the time error in microseconds. */
        time->time_us_error = time->time_us - (time->time_us_last + packet_rate_us[selected_packet_rate]);

        /* If the time delta is greater than or equal to the packet rate in microseconds and the time error is less than the maximum allowed error. */
        if (time->time_us_delta >= packet_rate_us[selected_packet_rate] && time->time_us_error < time->time_us_max_allowed_error)
        {
            /* Set the last time in microseconds. */
            time->time_us_last = time->time_us;

            /* Write 64 bytes to Serial1. */
            Serial1.write(crsf_tx_frame.buffer, crsf_frame_size);

            /* Increment the iteration. */
            iteration++;
        }

        /* If the time error is greater than or equal to the maximum allowed error. */
        else if (time->time_us_error >= time->time_us_max_allowed_error)
        {
            /* Exit the program. */
            atexit(exit_time_max_allowed_error_handler);
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        /* Exit the program. */
        atexit(exit_success_handler);
        exit(EXIT_SUCCESS);
    }
}
#endif
