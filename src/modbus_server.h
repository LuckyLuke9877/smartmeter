#pragma once

#ifndef GTEST
    #include "esphome/components/uart/uart.h"
    #include "esphome/core/helpers.h"
#endif

#include <cstring>
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
        uint16_t startAddress{0};
        uint16_t addressCount{0};
    };
    class ResponseRead
    {
    public:
        enum ErrorCode
        {
            NONE = 0x00,
            ILLEGAL_FUNCTION = 0X01,
            ILLEGAL_ADDRESS = 0X02,
            ILLEGAL_VALUE = 0X03,
            DEVICE_FAILURE = 0X04
        };

        void SetError(ErrorCode error)
        {
            m_errorCode = error;
        }

        bool IsError()
        {
            return m_errorCode != ErrorCode::NONE;
        }

        void SetData(std::vector<uint8_t>&& data)
        {
            m_data = std::forward<std::vector<uint8_t>>(data);
        }

        std::vector<uint8_t> GetPayload(uint8_t address, uint8_t functionCode)
        {
            std::vector<uint8_t> payload(3 + m_data.size());
            uint8_t byte2 = m_data.size();
            if (m_errorCode != ErrorCode::NONE)
            {
                // error: byte2 is the error-code
                byte2 = m_errorCode;
                functionCode |= 0x80;
                m_data.clear();
            }
            else
            {
                std::memcpy(&payload[3], &m_data[0], m_data.size());
            }

            // set header
            payload[0] = address;
            payload[1] = functionCode;
            payload[2] = byte2;

            return payload;
        }

    private:
        ErrorCode m_errorCode{ErrorCode::NONE};
        std::vector<uint8_t> m_data;
    };

    using OnReceiveRequest = std::function<ResponseRead(uint8_t functionCode, const RequestRead& request)>;

    ModbusServer(uint8_t address, OnReceiveRequest onReceive)
        : m_address(address)
        , m_onReceiveRequest(onReceive)
    { }

    void ProcessRequest()
    {
        // this is called every ~16ms, so we can not rely on timing (3.5 chars between frames see
        // https://en.wikipedia.org/wiki/Modbus) instead parse the rx_buffer for valid frames(address, function-code,
        // length, crc). Read all from uart
        while (available())
        {
            uint8_t byte(0);
            if (read_byte(&byte))
            {
                m_rxBuffer.push_back(byte);
                // ESP_LOGD("mbsrv", "Modbus received Byte  %d (0X%x)", byte, byte);
            }
        }

        while (m_rxBuffer.size() > 0)
        {
            auto removeSize = ParseModbusFrame();
            if (removeSize == 0)
            {
                break;
            }
            // Remove processed data
            m_rxBuffer.erase(m_rxBuffer.begin(), m_rxBuffer.begin() + removeSize);
        }
    }

    // Send command. payload contains data without CRC
    void Send(const std::vector<uint8_t>& payload)
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
        ESP_LOGD("mbsrv", "Modbus sending raw frame: %s, CRC: 0x%02x, 0x%02x", format_hex_pretty(payload).c_str(),
                 crc & 0xFF, (crc >> 8) & 0xFF);
    }

    std::vector<uint8_t> m_rxBuffer;

protected:
    uint8_t m_address;
    OnReceiveRequest m_onReceiveRequest;

    size_t GetFrameSize(uint8_t functionCode)
    {
        // Handle only limited number of function-codes as we do not need more. ( Extend if you need more )
        // do not handle exception code as it makes no sense for a server to receive one.
        return functionCode >= 0x01 && functionCode <= 0x04 ? 8 : 0;
    }

    uint32_t ParseModbusFrame()
    {
        const uint32_t needMoreData = 0;
        const uint32_t tryToFindValidFrame = 1;

        size_t bufSize = m_rxBuffer.size();
        // at least address | functionCode
        if (bufSize < 2)
        {
            return needMoreData;
        }

        const auto begin = m_rxBuffer.begin();
        uint8_t address = *(begin + 0);
        const auto functionCode = *(begin + 1);
        const auto frameSize = GetFrameSize(functionCode);
        if (frameSize == 0)
        {
            ESP_LOGW("mbsrv", "Modbus function-code %02x not supported or invalid frame", functionCode);
            return tryToFindValidFrame;
        }

        if (bufSize < frameSize)
        {
            return needMoreData;
        }

        // Validate crc
        uint16_t computedCrc = crc16(&*begin, frameSize - 2);
        uint16_t remoteCrc
            = static_cast<uint16_t>(*(begin + frameSize - 2)) | (static_cast<uint16_t>(*(begin + frameSize - 1)) << 8);
        if (computedCrc != remoteCrc)
        {
            ESP_LOGW("mbsrv", "Invalid CRC");
            // computed_crc.hi = 0x" << (computed_crc >> 8) << std::dec << std::endl;
            return tryToFindValidFrame;
        }

        if (m_address == address)
        {
            RequestRead request;
            // Note: Received as big endian
            request.startAddress = static_cast<uint16_t>(*(begin + 2)) << 8;
            request.startAddress += static_cast<uint16_t>(*(begin + 3));
            request.addressCount = static_cast<uint16_t>(*(begin + 4)) << 8;
            request.addressCount += static_cast<uint16_t>(*(begin + 5));
            ResponseRead response = m_onReceiveRequest(functionCode, request);

            Send(response.GetPayload(m_address, functionCode));
        }
        else
        {
            ESP_LOGD("mbsrv", "Not our[%d] address = %d", m_address, address);
        }

        // Frame can be removed
        return frameSize;
    }
};

} // namespace modbus
} // namespace esphome
