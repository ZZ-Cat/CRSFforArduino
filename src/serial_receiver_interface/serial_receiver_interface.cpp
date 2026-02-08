#include <cstring> // cppcheck-suppress missingIncludeSystem
#include <serial_receiver_interface.hpp> // "No such file or directory" Why?
// #include <serial_receiver_interface/serial_receiver_interface.hpp> // Workaround for the issue above.

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
            this->rx_data.buffer.begin() + (crsf_broadcast_frame.length + 1),
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

                // Invoke the RC channels callback if set.
                if (rc_channels_callback != nullptr)
                {
                    rc_channels_callback(control_data);
                }
                break;
            }
            default:
            {
                // Unknown frame type received.
                Serial.print("Unknown frame type: 0x");
                if (crsf_broadcast_frame.type < (unsigned char)0x10) Serial.print("0");
                Serial.println(crsf_broadcast_frame.type, HEX);
                break;
            }
        }
    }

    void serial_receiver_interface::set_rc_channels_callback(rc_channels_callback_t callback) // cppcheck-suppress unusedFunction
    {
        // Store the provided callback function.
        rc_channels_callback = callback;
    }
} // namespace cfa_internal
