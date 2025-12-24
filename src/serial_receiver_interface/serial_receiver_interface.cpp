#include <cstring> // cppcheck-suppress missingIncludeSystem
#include <serial_receiver_interface.hpp>

namespace cfa_internal
{
    void serial_receiver_interface::begin(config_t cfg)
    {
        // Initialise Serial1 with the provided configuration.
        Serial1.begin(cfg.baud_rate, cfg.config);

        // Clear any existing data in the serial buffer.
        Serial1.flush();
        while (Serial1.available() > 0)
        {
            Serial1.read();
        }
    }

    const auto serial_receiver_interface::receive_data_frame() -> bool // cppcheck-suppress unusedFunction
    {
        while (Serial1.available() > 0)
        {
            this->rx_data.byte_read = Serial1.read();

            if (!this->rx_data.sync_byte_detected && this->rx_data.byte_read == SYNC_BYTE)
            {
                this->rx_data.sync_byte_detected = true;
                this->rx_data.buffer.fill(0);
                this->rx_data.index = 0;
                this->rx_data.is_valid = false;
            }

            if (this->rx_data.sync_byte_detected)
            {
                this->rx_data.buffer[this->rx_data.index++] = this->rx_data.byte_read;

                if (this->rx_data.index == 2)
                {
                    this->rx_data.length = this->rx_data.buffer[1] + 2; // Length byte + 2 (for sync and CRC)
                    if (this->rx_data.length > MAX_BUFFER_SIZE)
                    {
                        this->rx_data.sync_byte_detected = false;
                        this->rx_data.index = 0;
                        this->rx_data.is_valid = false;
                        continue;
                    }
                }

                if (this->rx_data.index >= this->rx_data.length && this->rx_data.length >= MIN_BUFFER_SIZE)
                {
                    crc8.computed = crc8_calculate(2, this->rx_data.buffer, this->rx_data.length - 1);
                    crc8.received = this->rx_data.buffer[this->rx_data.length - 1];
                    this->rx_data.sync_byte_detected = false;
                    this->rx_data.index = 0;
                    this->rx_data.is_valid = (crc8.computed == crc8.received);
                    // Debug outputs — valid frame received, or CRC error.
                    // if (this->rx_data.is_valid)
                    // {
                    //     // Print raw data for debugging.
                    //     Serial.print("Received frame: [");
                    //     for (unsigned char i = 0; i < this->rx_data.length; ++i)
                    //     {
                    //         Serial.print("0x");
                    //         if (this->rx_data.buffer[i] < (unsigned char)0x10) Serial.print("0");
                    //         Serial.print(this->rx_data.buffer[i], HEX);
                    //         Serial.print(" ");
                    //     }
                    //     Serial.println("]");
                    // }
                    // else
                    // {
                    //     // CRC error
                    //     Serial.print("CRC error: computed 0x");
                    //     if (crc8.computed < (unsigned char)0x10) Serial.print("0");
                    //     Serial.print(crc8.computed, HEX);
                    //     Serial.print(", received 0x");
                    //     if (crc8.received < (unsigned char)0x10) Serial.print("0");
                    //     Serial.println(crc8.received, HEX);
                    // }
                    return this->rx_data.is_valid;
                }
            }
        }
        return false;
    }

    void serial_receiver_interface::parse_data_frame() // cppcheck-suppress unusedFunction
    {
        // Copy the received data into the frame structure.
        crsf_broadcast_frame_structure_t crsf_broadcast_frame;
        crsf_broadcast_frame.sync_byte = this->rx_data.buffer[0];
        crsf_broadcast_frame.length = this->rx_data.buffer[1];
        crsf_broadcast_frame.type = this->rx_data.buffer[2];
        std::copy(
            this->rx_data.buffer.begin() + 3,
            this->rx_data.buffer.begin() + crsf_broadcast_frame.length,
            crsf_broadcast_frame.payload.begin()
        );

        switch(crsf_broadcast_frame.type)
        {
            case CRSF_FRAME_TYPE_LINK_STATISTICS:
            {
                // Link statistics frame received. Currently not processed.
                // Could extract RSSI, SNR, etc. from payload if needed.
                break;
            }
            case CRSF_FRAME_TYPE_RC_CHANNELS_PACKED:
            {
                // Extract RC channel data from the payload.
                std::memcpy(&control_data.rc_channels, crsf_broadcast_frame.payload.data(), crsf_broadcast_frame.length - 2);

                // Debug output of channel values.
                // Serial.print("RC Channel Values:[");
                // Serial.print(" ch1: "); Serial.print(control_data.rc_channels.rc_channel_1);
                // Serial.print(" ch2: "); Serial.print(control_data.rc_channels.rc_channel_2);
                // Serial.print(" ch3: "); Serial.print(control_data.rc_channels.rc_channel_3);
                // Serial.print(" ch4: "); Serial.print(control_data.rc_channels.rc_channel_4);
                // Serial.print(" ch5: "); Serial.print(control_data.rc_channels.rc_channel_5);
                // Serial.print(" ch6: "); Serial.print(control_data.rc_channels.rc_channel_6);
                // Serial.print(" ch7: "); Serial.print(control_data.rc_channels.rc_channel_7);
                // Serial.print(" ch8: "); Serial.print(control_data.rc_channels.rc_channel_8);
                // Serial.print(" ch9: "); Serial.print(control_data.rc_channels.rc_channel_9);
                // Serial.print(" ch10: "); Serial.print(control_data.rc_channels.rc_channel_10);
                // Serial.print(" ch11: "); Serial.print(control_data.rc_channels.rc_channel_11);
                // Serial.print(" ch12: "); Serial.print(control_data.rc_channels.rc_channel_12);
                // Serial.print(" ch13: "); Serial.print(control_data.rc_channels.rc_channel_13);
                // Serial.print(" ch14: "); Serial.print(control_data.rc_channels.rc_channel_14);
                // Serial.print(" ch15: "); Serial.print(control_data.rc_channels.rc_channel_15);
                // Serial.print(" ch16: "); Serial.println(control_data.rc_channels.rc_channel_16);
                // Serial.println("]");
                break;
            }
            default:
                // Unknown or unhandled frame type.
                break;
        }
    }
} // namespace cfa_internal
