#pragma once
#include <Arduino.h>
#include <serial_receiver_interface/serial_receiver_interface.hpp>

namespace CRSF_for_Arduino
{
    class CRSFforArduino : private cfa_internal::serial_receiver_interface
    {
    public:
        CRSFforArduino() = default;
        ~CRSFforArduino() override = default;

        // Copy constructor, move constructor, copy assignment operator, and move assignment operator.
        CRSFforArduino(const CRSFforArduino &) = delete;
        CRSFforArduino(CRSFforArduino &&) = delete;
        auto operator=(const CRSFforArduino &) -> CRSFforArduino & = delete;
        auto operator=(CRSFforArduino &&) -> CRSFforArduino & = delete;

        using cfa_internal::serial_receiver_interface::begin;
        using cfa_internal::serial_receiver_interface::set_rc_channels_callback;

        void update();

    private:
    };
} // namespace CRSF_for_Arduino

using namespace CRSF_for_Arduino;
