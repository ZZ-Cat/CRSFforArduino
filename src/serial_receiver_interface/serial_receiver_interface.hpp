#pragma once
#include <Arduino.h>
namespace __cfa_internal_middleware_serial_receiver_interface
{
    class serial_receiver_interface
    {
    public:
        serial_receiver_interface() = default;
        virtual ~serial_receiver_interface() = default;

        // virtual size_t write(const uint8_t* data, size_t length) = 0;
        // virtual size_t read(uint8_t* buffer, size_t length) = 0;
        // virtual size_t available() = 0;
    };
} // namespace __cfa_internal_middleware_serial_receiver_interface
