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
} time_t;

/* Time structure instance. */
time_t *time = nullptr;

/* Exit handler. */
void exitHandler()
{
    /* Clean up and stop. */
    delete[] packet_rate_us;
    delete time;

    /* Print a message to the serial monitor. */
    Serial.println("Done!");
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

        /* Check if the time delta is greater than the time in microseconds for the selected packet rate. */
        if (time->time_us_delta >= packet_rate_us[selected_packet_rate])
        {
            /* Print the time delta in microseconds. */
            Serial.print("Time Delta: ");
            Serial.print(time->time_us_delta);
            Serial.println(" us");

            /* Set the last time in microseconds. */
            time->time_us_last = time->time_us;

            /* Increment the iteration. */
            iteration++;
        }
    }
    else
    {
        /* Exit the program. */
        atexit(exitHandler);
        exit(EXIT_SUCCESS);
    }
}
#endif
