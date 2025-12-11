#include <CRSFforArduino.hpp>

namespace crsf_for_arduino
{
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static, readability-function-cognitive-complexity)
    void CRSFforArduino::update() // cppcheck-suppress unusedFunction
    {
        // Use the base class method to receive data frames.
        if (receive_data_frame())
        {
            // Valid frame received; parse it.
            parse_data_frame();
        }
    }
} // namespace crsf_for_arduino