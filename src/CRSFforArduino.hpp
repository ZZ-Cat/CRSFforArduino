#pragma once
#include <Arduino.h>
// NOLINTNEXTLINE(misc-include-cleaner)
#include <array> // cppcheck-suppress missingIncludeSystem
#include <serial_receiver_interface/serial_receiver_interface.hpp>

namespace crsf_for_arduino
{
    class CRSFforArduino : private __cfa_internal_middleware_serial_receiver_interface::serial_receiver_interface
    {
    public:
        CRSFforArduino() = default;
        ~CRSFforArduino() override = default;

        /* BREAKING API CHANGE:
        Use config_t struct for configuration instead of separate
        parameters for CRSFforArduino::begin(). */
        typedef struct config_s
        {
            unsigned long baud_rate = 0;
            unsigned long config = 0;
        } config_t;
        void begin(config_t cfg = {
            .baud_rate = 420000,
            .config = SERIAL_8N1
        });

        void update();

    private:
        // using serial_receiver_interface::read;
        // using serial_receiver_interface::write;
        // using serial_receiver_interface::available;

        // Buffer index.
        unsigned char buffer_index = 0;

        // Buffer length (variable length packets) - assumed minimum length of 5 bytes until length byte is read.
        unsigned char buffer_length = 5;


        // Maximum buffer size.
        static constexpr unsigned char MAX_BUFFER_SIZE = 64;

        std::array<unsigned char, MAX_BUFFER_SIZE> buffer {};

        // Sync byte detection flag.
        bool sync_byte_detected = false;

        // Sync byte value.
        static constexpr unsigned char SYNC_BYTE = 0xC8;
    };
} // namespace crsf_for_arduino

using namespace crsf_for_arduino;
