#pragma once
#include <Arduino.h>
// NOLINTNEXTLINE(misc-include-cleaner)
#include <array> // cppcheck-suppress missingIncludeSystem
namespace cfa_internal
{
    class serial_receiver_interface
    {
    public:
        serial_receiver_interface() = default;
        virtual ~serial_receiver_interface() = default;

        // Copy constructor, move constructor, copy assignment operator, and move assignment operator.
        serial_receiver_interface(const serial_receiver_interface &) = delete;
        serial_receiver_interface(serial_receiver_interface &&) = delete;
        auto operator=(const serial_receiver_interface &) -> serial_receiver_interface & = delete;
        auto operator=(serial_receiver_interface &&) -> serial_receiver_interface & = delete;

        using config_t = struct config_s
        {
            unsigned long baud_rate = 0;
            unsigned long config = 0;
        };

        /* BREAKING API CHANGE:
        Use config_t struct for configuration instead of separate
        parameters for CRSFforArduino::begin(). */
        static void begin(config_t cfg = {
            .baud_rate = 420000,
            .config = SERIAL_8N1
        });

        auto receive_data_frame() -> bool;

        void parse_data_frame();

    private:
        static constexpr unsigned char MIN_BUFFER_SIZE = 5;
        static constexpr unsigned char MAX_BUFFER_SIZE = 64;
        static constexpr unsigned char SYNC_BYTE = 0xC8;
        static constexpr unsigned char CRSF_PAYLOAD_SIZE = 59;

        using rx_data_t = struct rx_data_s
        {
            unsigned char byte_read = 0;
            unsigned char index = 0;
            unsigned char length = 0;
            std::array<unsigned char, MAX_BUFFER_SIZE> buffer {};
            bool sync_byte_detected = false;
            bool is_valid = false;
        };
        rx_data_t rx_data;

        using crc8_data_t = struct crc8_s
        {
            static constexpr std::array<unsigned char, 256> CRC8_TABLE = {
                0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
                0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
                0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
                0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
                0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
                0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
                0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
                0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
                0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
                0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
                0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
                0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
                0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
                0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
                0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
                0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
            };
            unsigned char computed = 0;
            unsigned char received = 0;
        };

        crc8_data_t crc8;

        auto crc8_calculate(unsigned char start, std::array<unsigned char, MAX_BUFFER_SIZE> data, unsigned char length) -> unsigned char
        {
            unsigned char crc = 0;
            for (unsigned char i = start; i < length; ++i)
            {
                const unsigned char index = crc ^ data[i];
                crc = crc8.CRC8_TABLE[index];
            }
            return crc;
        }

        using crsf_broadcast_frame_structure_t = struct crsf_broadcast_frame_structure_s
        {
            unsigned char sync_byte = 0;
            unsigned char length = 0;
            unsigned char type = 0;
            std::array<unsigned char, CRSF_PAYLOAD_SIZE> payload = {0};
        };

        static constexpr unsigned char CRSF_FRAME_TYPE_RC_CHANNELS_PACKED = 0x16;

        static constexpr unsigned char CHANNEL_COUNT = 16;

        static constexpr unsigned short CHANNEL_VALUE_MIN = 172;
        static constexpr unsigned short CHANNEL_VALUE_MID = 992;
        static constexpr unsigned short CHANNEL_VALUE_MAX = 1811;

        struct rc_channels_packed_s
        {
            unsigned short rc_channel_1  : 11;
            unsigned short rc_channel_2  : 11;
            unsigned short rc_channel_3  : 11;
            unsigned short rc_channel_4  : 11;
            unsigned short rc_channel_5  : 11;
            unsigned short rc_channel_6  : 11;
            unsigned short rc_channel_7  : 11;
            unsigned short rc_channel_8  : 11;
            unsigned short rc_channel_9  : 11;
            unsigned short rc_channel_10 : 11;
            unsigned short rc_channel_11 : 11;
            unsigned short rc_channel_12 : 11;
            unsigned short rc_channel_13 : 11;
            unsigned short rc_channel_14 : 11;
            unsigned short rc_channel_15 : 11;
            unsigned short rc_channel_16 : 11;
        } __attribute__((packed));

        using rc_channels_t = struct rc_channels_packed_s;
    };
} // namespace cfa_internal
