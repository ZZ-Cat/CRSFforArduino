#include <CRSFforArduino.hpp>
// NOLINTNEXTLINE(misc-include-cleaner)
#include <array> // cppcheck-suppress missingIncludeSystem

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

    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    void CRSFforArduino::update() // cppcheck-suppress unusedFunction
    {
        std::array<uint8_t, MAX_BUFFER_SIZE> buffer;

        while (Serial1.available() > 0)
        {
            uint8_t byte_read = Serial1.read();

            if (!sync_byte_detected && byte_read == SYNC_BYTE)
            {
                sync_byte_detected = true;
                buffer.fill(0);
                // buffer_length = 5; // Reset to minimum length
                buffer_index = 0;
                Serial.println("Sync byte detected");
            }

            if (sync_byte_detected)
            {
                // If less than three bytes have been read, set the buffer length to minimum.
                if (buffer_index < 3)
                {
                    buffer_length = 5; // Minimum length
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
                    Serial.print("Complete packet received: [");
                    for (uint8_t i = 0; i < buffer_length; ++i)
                    {
                        Serial.print("0x");
                        Serial.print(buffer[i], HEX);
                        Serial.print(" ");
                    }
                    Serial.println("]");

                    // Reset for next packet
                    sync_byte_detected = false;
                }
            }
        }
    }
} // namespace crsf_for_arduino