#include <CRSFforArduino.hpp>

namespace CRSF_for_Arduino
{
    void CRSFforArduino::update() // cppcheck-suppress unusedFunction
    {
        // Use the base class method to receive data frames.
        if (this->receive_data_frame())
        {
            // Valid frame received; parse it.
            this->parse_data_frame();
        }
    }
} // namespace CRSF_for_Arduino