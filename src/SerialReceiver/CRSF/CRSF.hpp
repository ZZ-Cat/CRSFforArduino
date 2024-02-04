#pragma once

#include "CRSFProtocol.hpp"
#include "../CRC/CRC.hpp"

namespace serialReceiverLayer
{
    class CRSF
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
        genericCrc::CRC *crc8;
        uint8_t calculateFrameCRC();
    };
} // namespace serialReceiverLayer
