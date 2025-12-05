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

        void receive_data()
        {
            Serial.println("Receiving data via CRSFforArduino...");
            serial_receiver_interface::receive_data();
        }
    };
} // namespace crsf_for_arduino

using namespace crsf_for_arduino;
