#pragma once

#include "CRSFProtocol.hpp"

#if defined(CFA_DEVELOPMENT_MODE)
#include "CRC/CRC.hpp"
#else
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
