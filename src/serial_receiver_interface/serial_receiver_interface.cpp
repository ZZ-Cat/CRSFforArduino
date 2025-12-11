#include <serial_receiver_interface.hpp>

namespace __cfa_internal_middleware_serial_receiver_interface
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
                    crc8.computed = crc8_calculate(2, this->rx_data.buffer.data(), this->rx_data.length - 1);
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
        // Placeholder for future data frame parsing logic.
        // For now, just print the received data if valid.
        if (this->rx_data.is_valid)
        {
            Serial.print("Received frame: [");
            for (unsigned char i = 0; i < this->rx_data.length; ++i)
            {
                Serial.print("0x");
                if (this->rx_data.buffer[i] < (unsigned char)0x10) Serial.print("0");
                Serial.print(this->rx_data.buffer[i], HEX);
                Serial.print(" ");
            }
            Serial.println("]");

            // Reset validity after parsing.
            this->rx_data.is_valid = false;

            // Further parsing logic can be implemented here.
            // For example, extracting specific fields from the data frame.
        }
    }
} // namespace __cfa_internal_middleware_serial_receiver_interface
