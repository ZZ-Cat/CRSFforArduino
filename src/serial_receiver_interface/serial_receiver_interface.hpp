#pragma once
#include <Arduino.h>
namespace __cfa_internal_middleware_serial_receiver_interface
{
    class serial_receiver_interface
    {
    public:
        serial_receiver_interface() = default;
        virtual ~serial_receiver_interface() = default;

        // Example function.
        void receive_data() {}
    };
} // namespace __cfa_internal_middleware_serial_receiver_interface
