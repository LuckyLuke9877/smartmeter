#pragma once

#ifndef GTEST
#include "esphome/components/uart/uart.h"
#include "esphome/core/helpers.h"
#endif

#include <functional>
#include <vector>

namespace esphome
{
namespace modbus
{
/** Modbus server(slave) class.
 *   Handles the modbus commuinication for one modbus server(slave) address.
 *   This class is needed, cause modbus::Modbus is tailored for Modbus-client(master).
 *   A received modbus-frame is not the same for client and server.
 *   Note: it handles only function-code 0x03
 */
class ModbusServer : public uart::UARTDevice
{
public:
    struct RequestRead
    {
        uint16_t start_address{0};
        uint16_t address_count{0};
    };
    struct ResponseRead
    {
        enum ErrorCode
        {
            NONE             = 0x00,
            ILLEGAL_FUNCTION = 0X01,
            ILLEGAL_ADDRESS  = 0X02,
            ILLEGAL_VALUE    = 0X03
        };

        ErrorCode            errorCode{ErrorCode::NONE};
        std::vector<uint8_t> data;

        void set_error(ErrorCode error) { errorCode = error; }

        void add_payload(const std::vector<uint8_t>& payload)
        {
            const auto dataSize = data.size();
            data.resize(dataSize + payload.size());
            std::copy(payload.begin(), payload.end(), data.begin() + dataSize);
        }

        std::vector<uint8_t> get_payload(uint8_t address, uint8_t function_code)
        {
            if (errorCode != ErrorCode::NONE)
            {
                // error code
                function_code |= 0x80;
                data.clear();
                data.push_back(errorCode);
            }
            else
            {
                // add first param "ByteCount"
                data.insert(data.begin(), static_cast<uint8_t>(data.size()));
            }
            std::vector<uint8_t> payload(data.size() + 2);
            payload[0] = address;
            payload[1] = function_code;
            std::copy(data.begin(), data.end(), payload.begin() + 2);

            return payload;
        }
    };

    using on_receive_request = std::function<ResponseRead(uint8_t function_code, const RequestRead& request)>;

    ModbusServer(uint8_t address, on_receive_request on_receive)
        : address_(address)
        , on_receive_request_(on_receive)
    {
        // access test => is ok:        auto p = id(uart_modbus);
    }

    void process_requests()
    {
        // this is called every ~16ms, so we can not rely on timing (3.5 chars between frames see
        // https://en.wikipedia.org/wiki/Modbus) instead parse the rx_buffer for valid frames(address, function-code,
        // length, crc). Read all from uart
        while (this->available())
        {
            uint8_t byte(0);
            if (read_byte(&byte))
            {
                rx_buffer_.push_back(byte);
                //        ESP_LOGV(TAG, "Modbus received Byte  %d (0X%x)", byte, byte);
            }
        }

        while (rx_buffer_.size() > 0)
        {
            auto removeSize = parse_modbus_frame();
            // std::cout << "loop() removeSize = " << removeSize << "\n";
            if (removeSize == 0)
            {
                break;
            }
            // Remove processed data
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + removeSize);
        }
    }

    // Send command. payload contains data without CRC
    void send(const std::vector<uint8_t>& payload)
    {
        if (payload.empty())
        {
            return;
        }

        auto crc = crc16(payload.data(), payload.size());
        write_array(payload);
        write_byte(crc & 0xFF);
        write_byte((crc >> 8) & 0xFF);
        flush();
        ESP_LOGV(TAG, "Modbus write raw: %s", format_hex_pretty(payload).c_str());
    }

    std::vector<uint8_t> rx_buffer_;

protected:
    uint8_t            address_;
    on_receive_request on_receive_request_;

    size_t get_frame_size(uint8_t function_code)
    {
        // Handle only limited number of function-codes as we do not need more. ( Extend if you need more )
        // do not handle exception code as it makes no sense for a server to receive one.
        return function_code >= 0x01 && function_code <= 0x04 ? 8 : 0;
    }

    uint32_t parse_modbus_frame()
    {
        size_t buf_size = rx_buffer_.size();
        // std::cout << "parse_modbus_frame() buf_size = " << buf_size << "\n";
        // at least address | function_code
        if (buf_size < 2)
        {
            return 0; // need more data
        }

        const auto begin         = rx_buffer_.begin();
        uint8_t    address       = *(begin + 0);
        const auto function_code = *(begin + 1);
        const auto frame_size    = get_frame_size(function_code);
        if (frame_size == 0)
        {
            ESP_LOGD(TAG, "Modbus function-code %02x not supported or invalid frame", function_code);
            return 1; // remove 1 byte
        }

        if (buf_size < frame_size)
        {
            return 0; // need more data
        }

        // Validate crc
        uint16_t computed_crc = crc16(&*begin, frame_size - 2);
        uint16_t remote_crc =
            static_cast<uint16_t>(*(begin + frame_size - 2)) | (static_cast<uint16_t>(*(begin + frame_size - 1)) << 8);
        if (computed_crc != remote_crc)
        {
            ESP_LOGD(TAG, "Invalid CRC");
            // std::cout << std::hex << "invalid crc, computed_crc.lo = 0x" << (computed_crc & 0xFF) << ",
            // computed_crc.hi = 0x" << (computed_crc >> 8) << std::dec << std::endl;
            return 1; // remove 1 byte
        }

        if (address_ == address)
        {
            RequestRead request;
            request.start_address = static_cast<uint16_t>(*(begin + 3)) << 8;
            request.start_address += static_cast<uint16_t>(*(begin + 4));
            request.address_count = static_cast<uint16_t>(*(begin + 5)) << 8;
            request.address_count += static_cast<uint16_t>(*(begin + 6));
            ResponseRead response = on_receive_request_(function_code, request);

            send(response.get_payload(address_, function_code));
        }

        // Frame can be removed
        return frame_size;
    }
};

} // namespace modbus
} // namespace esphome
