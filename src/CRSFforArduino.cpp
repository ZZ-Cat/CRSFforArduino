#include <CRSFforArduino.hpp>

namespace crsf_for_arduino
{
    void CRSFforArduino::begin(config_t cfg)
    {
        Serial.println("CRSFforArduino begin");
        Serial.print("Baud rate: ");
        Serial.println(cfg.baud_rate);
        Serial.print("Config: 0x");
        Serial.println(cfg.config, HEX);

        // TO-DO: Need to find a way to pass these parameters to the underlying
        // serial receiver interface middleware.

        // Will attempt a Serial1 begin with these parameters, and see
        // if it works.
        Serial1.begin(cfg.baud_rate, cfg.config);
    }

    void CRSFforArduino::update() // cppcheck-suppress unusedFunction
    {
        Serial.println("CRSFforArduino update");
    }
} // namespace crsf_for_arduino