#pragma once

#include "CRSFProtocol.hpp"

#if defined(ARDUINO) && defined(PLATFORMIO)
#include "CRC/CRC.hpp"
#elif defined(ARDUINO) && !defined(PLATFORMIO)
#include "SerialReceiver/CRC/CRC.hpp"
#endif

namespace serialReceiverLayer
{
    class CRSF : private genericCrc::CRC
    {
    public:
        CRSF();
        virtual ~CRSF();
    private:
    };
} // namespace serialReceiverLayer
