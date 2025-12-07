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

        // Initialise Serial1 with the provided configuration, for now.
        Serial1.begin(cfg.baud_rate, cfg.config);
    }

    // NOLINTNEXTLINE(readability-convert-member-functions-to-static, readability-function-cognitive-complexity)
    void CRSFforArduino::update() // cppcheck-suppress unusedFunction
    {
        while (Serial1.available() > 0)
        {
            byte_read = Serial1.read();

            if (!sync_byte_detected && byte_read == SYNC_BYTE)
            {
                sync_byte_detected = true;
                buffer.fill(0);
                buffer_index = 0;

                // Debug output.
                // Serial.println("Sync byte detected");
            }

            if (sync_byte_detected)
            {
                // If less than three bytes have been read, set the buffer length to minimum.
                if (buffer_index < 3)
                {
                    buffer_length = MIN_BUFFER_SIZE; // Minimum length
                }
                // Otherwise, read the length byte.
                else
                {
                    buffer_length = buffer[1] + 2; // Length byte + 2 (for sync and CRC)
                    if (buffer_length > MAX_BUFFER_SIZE)
                    {
                        Serial.println("Error: Buffer length exceeds maximum size. Resetting.");
                        sync_byte_detected = false;
                        continue;
                    }
                }

                // Update the buffer with the read byte
                buffer[buffer_index] = byte_read;
                buffer_index++;

                // Check if we've read the full packet.
                if (buffer_index >= buffer_length)
                {
                    // Validate CRC8
                    computed_crc = crc8((const unsigned char)2, buffer.data(), buffer_length - 1);
                    received_crc = buffer[buffer_length - 1];

                    if (computed_crc == received_crc)
                    {
                        // Valid packet received
                        // Further processing can be done here.

                        // Debug output.
                        Serial.print("Packet data: [");
                        for (unsigned char i = 0; i < buffer_length; ++i)
                        {
                            Serial.print("0x");
                            Serial.print(buffer[i], HEX);
                            Serial.print(" ");
                        }
                        Serial.println("]");
                    }
                    else
                    {
                        Serial.print("CRC mismatch! Computed: 0x");
                        Serial.print(computed_crc, HEX);
                        Serial.print(", Received: 0x");
                        Serial.println(received_crc, HEX);
                    }

                    // Reset for next packet
                    sync_byte_detected = false;
                }
            }
        }
    }
} // namespace crsf_for_arduino