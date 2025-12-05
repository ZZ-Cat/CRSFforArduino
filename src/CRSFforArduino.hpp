#pragma once
#include <Arduino.h>
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
    };
} // namespace crsf_for_arduino

using namespace crsf_for_arduino;
