#pragma once
#include <Arduino.h>
#include <serial_receiver_interface/serial_receiver_interface.hpp>

namespace CRSF_for_Arduino
{
    // NOLINTNEXTLINE(cppcoreguidelines-special-member-functions, hicpp-special-member-functions)
    class CRSFforArduino : private cfa_internal::serial_receiver_interface
    {
    public:
        CRSFforArduino() = default;
        ~CRSFforArduino() override = default;

        using cfa_internal::serial_receiver_interface::begin;

        void update();

    private:
    };
} // namespace CRSF_for_Arduino

using namespace CRSF_for_Arduino;
