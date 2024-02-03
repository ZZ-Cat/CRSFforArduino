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
        void begin();
        void end();
        void setFrameTime(uint32_t baudRate, uint8_t packetCount = 10);
        bool receiveFrames(uint8_t rxByte);
        void getRcChannels(uint16_t *rcChannels);
    private:
        bool rcFrameReceived;
        uint16_t frameCount;
        uint32_t timePerFrame;
        crsfProtocol::frame_t rxFrame;
        crsfProtocol::frame_t rcChannelsFrame;
        uint8_t calculateFrameCRC();
    };
} // namespace serialReceiverLayer
