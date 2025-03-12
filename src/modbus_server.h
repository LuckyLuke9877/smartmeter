#pragma once

#ifndef GTEST
    #include "esphome/components/uart/uart.h"
    #include "esphome/core/helpers.h"
#endif

#include "modbus_request.h"

#include <cstring>
#include <functional>
#include <vector>

namespace modb
{
using namespace esphome;

/** Modbus server(slave) class.
 *   Handles the modbus commuinication for modbus server(slave) with multiple addresses.
 *   This class is needed, cause modbus::Modbus is tailored for Modbus-client(master).
 *   A received modbus-frame is not the same for client and server.
 *   To extend function-code, implement a Request like Request03
 */
class ModbusServer : public uart::UARTDevice
{
public:
    using OnReceiveRequest = std::function<void(Request& request)>;

    ModbusServer(OnReceiveRequest onReceive)
        : m_onReceiveRequest(onReceive)
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
    void Send(const std::vector<uint8_t>* payload)
    {
        if (payload == nullptr || payload->empty())
        {
            return;
        }

        auto crc = crc16(payload->data(), payload->size());
        write_array(*payload);
        write_byte(crc & 0xFF);
        write_byte((crc >> 8) & 0xFF);
        flush();
        ESP_LOGD(
            "mbsrv", "Modbus sending raw frame: %s, CRC: 0x%02x, 0x%02x", format_hex_pretty(*payload).c_str(), crc & 0xFF,
            (crc >> 8) & 0xFF);
    }

    std::vector<uint8_t> m_rxBuffer;

protected:
    OnReceiveRequest m_onReceiveRequest;
    // Requests pool to save memory
    Request03 m_request03;

    Request* GetRequest(uint8_t functionCode)
    {
        // Handle only limited number of function-codes as we do not need more. ( Extend if you need more )
        // do not handle exception code ( ERROR_FLAG ) as it makes no sense for a server to receive one.
        if (functionCode == 0x03)
        {
            return &m_request03;
        }

        return nullptr;
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
        auto request = GetRequest(functionCode);
        if (request == nullptr)
        {
            // We have no idea what the size is, so crc check is not possible.
            // Do not return anything.
            ESP_LOGW("mbsrv", "Modbus function-code %02x not supported", functionCode);
            return tryToFindValidFrame;
        }
        const auto frameSize = request->GetSize();
        if (frameSize == 0)
        {
            ESP_LOGW("mbsrv", "Modbus invalid frame");
            return tryToFindValidFrame;
        }

        if (bufSize < frameSize)
        {
            return needMoreData;
        }

        // Validate crc
        uint16_t computedCrc = crc16(&*begin, frameSize - 2);
        uint16_t remoteCrc = static_cast<uint16_t>(*(begin + frameSize - 2)) | (static_cast<uint16_t>(*(begin + frameSize - 1)) << 8);
        if (computedCrc != remoteCrc)
        {
            ESP_LOGW("mbsrv", "Invalid CRC");
            // computed_crc.hi = 0x" << (computed_crc >> 8) << std::dec << std::endl;
            return tryToFindValidFrame;
        }

        if (request->InitFromBuffer(m_rxBuffer))
        {
            // client validates the modbus-address
            // ESP_LOGI("mbsrv", "Modbus valid request %02x received", functionCode);
            m_onReceiveRequest(*request);
            Send(request->GetResponsePayload());
        }

        // Frame can be removed
        return frameSize;
    }
};

} // namespace modb
