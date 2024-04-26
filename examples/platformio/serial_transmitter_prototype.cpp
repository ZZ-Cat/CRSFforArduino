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
typedef struct software_realtime_counter_s
{
    uint32_t time_us = 0;
    uint32_t time_us_last = 0;
    uint32_t time_us_delta = 0;
    int32_t time_us_error = 0;
    const int32_t time_us_max_allowed_error = 2;
} software_realtime_counter_t;

/* Packed 11-bit RC Channels. */
struct rc_channels_packed_s
{
    uint16_t ch1 : 11;
    uint16_t ch2 : 11;
    uint16_t ch3 : 11;
    uint16_t ch4 : 11;
    uint16_t ch5 : 11;
    uint16_t ch6 : 11;
    uint16_t ch7 : 11;
    uint16_t ch8 : 11;
    uint16_t ch9 : 11;
    uint16_t ch10 : 11;
    uint16_t ch11 : 11;
    uint16_t ch12 : 11;
    uint16_t ch13 : 11;
    uint16_t ch14 : 11;
    uint16_t ch15 : 11;
    uint16_t ch16 : 11;
} __attribute__((packed));

typedef struct rc_channels_packed_s rc_channels_packed_t;

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
software_realtime_counter_t *sw_timer = nullptr;

crsf_tx_frame_t crsf_tx_frame;

/* CRC8-DVB-S2. */
uint8_t crc8_dvb_s2(uint8_t crc, const uint8_t data)
{
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (crc & 0x80)
        {
            crc = (crc << 1) ^ 0xD5;
        }
        else
        {
            crc <<= 1;
        }
    }
    return crc;
}

/* This function calculates the CRC for the CRSF frame
from the type to the end of the payload. */
uint8_t calculate_crc(const crsf_tx_frame_t *crsf_tx_frame)
{
    uint8_t crc = 0;
    crc = crc8_dvb_s2(crc, crsf_tx_frame->frame.type);
    for (uint8_t i = 0; i < crsf_tx_frame->frame.length - 2; i++)
    {
        crc = crc8_dvb_s2(crc, crsf_tx_frame->frame.payload[i]);
    }
    return crc;
}

/* Exit handlers. */
void exit_success_handler()
{
    /* Clean up and stop. */
    delete[] packet_rate_us;
    delete sw_timer;

    /* Print a message to the serial monitor. */
    Serial.println("Program has ended successfully.");
}

void exit_time_max_allowed_error_handler()
{
    /* Print a message to the serial monitor. */
    Serial.print("Program has ended with an error: ");
    Serial.println("Time error is greater than the maximum allowed error.");

    /* Print how far the sw_timer error is from the packet rate in microseconds. */
    Serial.print("Time Error: ");
    Serial.print(sw_timer->time_us_error);
    Serial.println(" us");

    /* Clean up and stop. */
    delete[] packet_rate_us;
    delete sw_timer;
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
    and calculate the sw_timer in microseconds for each packet rate. */
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

    /* Initialize the sw_timer structure. */
    sw_timer = new software_realtime_counter_t;

    /* Initialise Serial1 with 1.87M baud rate. */
    Serial1.begin(1875000);
    memset(&crsf_tx_frame, 0, crsf_frame_size);

    /* Initialise the RC Channels structure. */
    rc_channels_packed_t rc_channels_packed;
    rc_channels_packed.ch1 = 992;
    rc_channels_packed.ch2 = 992;
    rc_channels_packed.ch3 = 992;
    rc_channels_packed.ch4 = 992;
    rc_channels_packed.ch5 = 178;
    rc_channels_packed.ch6 = 992;
    rc_channels_packed.ch7 = 992;
    rc_channels_packed.ch8 = 992;
    rc_channels_packed.ch9 = 992;
    rc_channels_packed.ch10 = 992;
    rc_channels_packed.ch11 = 992;
    rc_channels_packed.ch12 = 992;
    rc_channels_packed.ch13 = 992;
    rc_channels_packed.ch14 = 992;
    rc_channels_packed.ch15 = 992;
    rc_channels_packed.ch16 = 992;

    /* Prepare the CRSF RC Channels Packed frame. */
    crsf_tx_frame.frame.sync = 0xC8; // NB: EdgeTX uses 0xEE which is incorrect.
    crsf_tx_frame.frame.length = 24;
    crsf_tx_frame.frame.type = 0x16;
    memcpy(crsf_tx_frame.frame.payload, &rc_channels_packed, sizeof(rc_channels_packed));
    crsf_tx_frame.frame.crc = calculate_crc(&crsf_tx_frame);

    /* Print a message to the serial monitor. */
    Serial.println("Testing CRSF Serial Transmitter Prototype...");

    /* Set the time in microseconds. */
    sw_timer->time_us = micros();
    sw_timer->time_us_last = sw_timer->time_us;
}

void loop()
{

    static uint32_t iteration = 0;

    /* Calculate the number of iterations based on the selected packet rate and the equivalent total execution time of three seconds. */
    static const uint32_t iterations = (3000000UL / packet_rate_us[selected_packet_rate]);

    if (iteration < iterations)
    {
        /* Calculate the time delta in microseconds. */
        sw_timer->time_us = micros();
        sw_timer->time_us_delta = sw_timer->time_us - sw_timer->time_us_last;

        /* Calculate the time error in microseconds. */
        sw_timer->time_us_error = sw_timer->time_us - (sw_timer->time_us_last + packet_rate_us[selected_packet_rate]);

        /* If the time delta is greater than or equal to the packet rate in microseconds and the time error is less than the maximum allowed error. */
        if (sw_timer->time_us_delta >= packet_rate_us[selected_packet_rate] && sw_timer->time_us_error < sw_timer->time_us_max_allowed_error)
        {
            /* Set the last time in microseconds. */
            sw_timer->time_us_last = sw_timer->time_us;

            /* Write 64 bytes to Serial1. */
            Serial1.write(crsf_tx_frame.buffer, crsf_tx_frame.frame.length + 1);

            /* Increment the iteration. */
            iteration++;
        }

        /* If the time error is greater than or equal to the maximum allowed error. */
        else if (sw_timer->time_us_error >= sw_timer->time_us_max_allowed_error)
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
