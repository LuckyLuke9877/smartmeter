#pragma once

#ifndef GTEST
    #include "esphome/components/uart/uart.h"
    #include "esphome/core/helpers.h"
#endif
#include "conversion.h"
#include "modbus_interface.h"

#include <algorithm>
#include <stdint.h>
#include <string>
#include <vector>

namespace modb
{

// Error-flag is ored with function-code
constexpr uint8_t ERROR_FLAG = 0x80;
constexpr uint8_t MIN_RESPONSE_HEADER_SIZE = 3;
constexpr uint32_t MIN_REQUEST_FRAME_SIZE = 2;
// returned if frame is not valid
constexpr uint32_t TRY_FIND_VALID_FRAME = 1;

// Base class for all requests ( only for register function codes )
class Request
{
public:
    Request(uint8_t functionCode)
        : m_functionCode(functionCode)
        , m_response(MIN_RESPONSE_HEADER_SIZE)
    { }
    virtual ~Request() { }
    uint8_t GetFunctionCode()
    {
        return m_functionCode;
    }
    uint8_t GetModbusAddress()
    {
        return m_modbusAddress;
    }

    struct InitResult
    {
        InitResult(bool ok, uint32_t frameSize)
            : canProcess(ok)
            , usedFrameSize(frameSize)
        { }

        bool canProcess; // InitFrame is ok and Process() can be called
        uint32_t usedFrameSize; // Number of bytes to remove from buffer
    };
    virtual InitResult InitFrame(const std::vector<uint8_t>& buffer) = 0;
    virtual bool Process(IModbusRegisters& regs) = 0;
    virtual const std::vector<uint8_t>* GetResponsePayload() = 0;
    virtual const std::string ToString() const = 0;

protected:
    uint8_t m_functionCode;
    uint8_t m_modbusAddress{0};
    uint16_t m_startAddress{0};
    uint16_t m_addressCount{0};
    bool m_isResponseValid{false};
    ResponseError m_errorCode{ResponseError::None};
    std::vector<uint8_t> m_response;

    virtual uint32_t ValidateFrameSize(const std::vector<uint8_t>& buffer) const = 0;

    void SetResponseHeader()
    {
        // set header
        m_response[0] = m_modbusAddress;
        m_response[1] = m_functionCode;
    }

    void SetResponseError()
    {
        if (m_errorCode == ResponseError::None)
        {
            return;
        }

        // clear any data
        const uint8_t errorResponseSize(3);
        m_response.resize(errorResponseSize);
        // function-code is flagged
        m_response[1] = m_response[1] | ERROR_FLAG;
        // byte2 is the error-code
        m_response[2] = static_cast<uint8_t>(m_errorCode);
    }

    uint32_t ValidateAndInitFrame(const std::vector<uint8_t>& buffer)
    {
        m_isResponseValid = false;
        m_errorCode = ResponseError::None;

        if (buffer.size() < MIN_REQUEST_FRAME_SIZE)
        {
            return 0; // need more data
        }
        if (buffer[1] != m_functionCode)
        {
            // must never happen => do not init with buffer from other function-code
            return TRY_FIND_VALID_FRAME;
        }
        uint32_t frameSize = ValidateFrameSize(buffer);
        if (frameSize < MIN_REQUEST_FRAME_SIZE)
        {
            // Not enough buffer or corrupted frame
            return frameSize;
        }
        if (ValidateCrc(&buffer[0], frameSize) == false)
        {
            return TRY_FIND_VALID_FRAME;
        }

        // Frame is valid
        m_modbusAddress = buffer[0];

        return frameSize;
    }

    bool ValidateCrc(const uint8_t* buffer, const uint32_t frameSize) const
    {
        const uint16_t computedCrc = esphome::crc16(buffer, frameSize - 2);
        const uint16_t remoteCrc
            = static_cast<uint16_t>(*(buffer + frameSize - 2)) | (static_cast<uint16_t>(*(buffer + frameSize - 1)) << 8);
        if (computedCrc != remoteCrc)
        {
            ESP_LOGW("mbsrv", "Invalid CRC");
            // computed_crc.hi = 0x" << (computed_crc >> 8) << std::dec << std::endl;
            return false;
        }

        return true;
    }

    void SetResponseData(uint16_t index, uint16_t valueLittleEndian)
    {
        if (index + sizeof(uint16_t) > m_response.size())
        {
            m_response.resize(index + sizeof(uint16_t));
        }
        auto temp = Convert2BigEndian(valueLittleEndian);
        std::memcpy(&m_response[index], &temp, sizeof(temp));
    }
};

// Read holding registers
class Request03 : public Request
{
public:
    Request03()
        : Request(03)
    { }
    virtual ~Request03() { }
    virtual InitResult InitFrame(const std::vector<uint8_t>& buffer)
    {
        uint32_t frameSize = ValidateAndInitFrame(buffer);
        if (frameSize < MIN_REQUEST_FRAME_SIZE)
        {
            return InitResult{false, frameSize};
        }
        const auto begin = buffer.begin();
        // Note: Received as big endian => convert to little endian
        m_startAddress = static_cast<uint16_t>(*(begin + 2)) << 8;
        m_startAddress += static_cast<uint16_t>(*(begin + 3));
        m_addressCount = static_cast<uint16_t>(*(begin + 4)) << 8;
        m_addressCount += static_cast<uint16_t>(*(begin + 5));

        return InitResult{true, frameSize};
    }
    virtual bool Process(IModbusRegisters& regs)
    {
        // Also on error response is valid
        m_isResponseValid = true;
        m_response.resize(MIN_RESPONSE_HEADER_SIZE + m_addressCount * modb::REGISTER_SIZE);
        m_errorCode = regs.Read(m_startAddress, m_addressCount, &m_response[MIN_RESPONSE_HEADER_SIZE]);

        return m_errorCode == modb::ResponseError::None;
    }
    virtual const std::vector<uint8_t>* GetResponsePayload()
    {
        if (m_isResponseValid == false)
        {
            return nullptr;
        }

        SetResponseHeader();

        // return Read() - data size; the data are already set by Process()
        m_response[2] = m_response.size() - MIN_RESPONSE_HEADER_SIZE;

        SetResponseError();

        return &m_response;
    }
    virtual const std::string ToString() const
    {
        char temp[128] = {0};
        sprintf(temp, "Request03: mod-addr[%d], reg-start[%d], reg-count[%d]", m_modbusAddress, m_startAddress, m_addressCount);

        return temp;
    }

protected:
    virtual uint32_t ValidateFrameSize(const std::vector<uint8_t>& buffer) const
    {
        // bytes    description
        // 1        address
        // 1        function-code
        // 2        start register address
        // 2        register count
        // 2        CRC
        const uint32_t frameSize(8);
        return buffer.size() < frameSize ? 0 : frameSize;
    }
};

// Read holding registers
class Request16 : public Request
{
public:
    Request16()
        : Request(16)
    { }
    virtual ~Request16() { }
    virtual InitResult InitFrame(const std::vector<uint8_t>& buffer)
    {
        uint32_t frameSize = ValidateAndInitFrame(buffer);
        if (frameSize < MIN_REQUEST_FRAME_SIZE)
        {
            return InitResult{false, frameSize};
        }
        const auto begin = buffer.begin();
        // Note: Received as big endian => convert to little endian
        m_startAddress = static_cast<uint16_t>(*(begin + 2)) << 8;
        m_startAddress += static_cast<uint16_t>(*(begin + 3));
        m_addressCount = static_cast<uint16_t>(*(begin + 4)) << 8;
        m_addressCount += static_cast<uint16_t>(*(begin + 5));
        // write-data
        auto writeDataBegin = begin + 7;
        auto writeDataEnd = writeDataBegin + *(begin + 6);
        m_writeBuffer.assign(writeDataBegin, writeDataEnd);

        return InitResult{true, frameSize};
    }
    virtual bool Process(IModbusRegisters& regs)
    {
        // Also on error response is valid
        m_isResponseValid = true;
        m_errorCode = regs.Write(m_startAddress, m_addressCount, &m_writeBuffer[0]);

        return m_errorCode == modb::ResponseError::None;
    }
    virtual const std::vector<uint8_t>* GetResponsePayload()
    {
        if (m_isResponseValid == false)
        {
            return nullptr;
        }

        SetResponseHeader();

        // function specific response
        // Note: Sent as big endian => convert from little endian
        m_response.resize(6);
        SetResponseData(2, m_startAddress);
        SetResponseData(4, m_addressCount);

        SetResponseError();

        return &m_response;
    }
    virtual const std::string ToString() const
    {
        char temp[128] = {0};
        sprintf(temp, "Request16: mod-addr[%d], reg-start[%d], reg-count[%d]", m_modbusAddress, m_startAddress, m_addressCount);

        std::string hexData;
        const size_t maxDataSize = std::min(size_t(20), m_writeBuffer.size());
        char hex[32] = {0};
        for (size_t i = 0; i < maxDataSize; i++)
        {
            sprintf(hex, ", 0x%02x", m_writeBuffer[i]);
            hexData += hex;
        }
        if (m_writeBuffer.size() > maxDataSize)
        {
            hexData += ", ...";
        }

        return std::string(temp) + hexData;
    }

protected:
    virtual uint32_t ValidateFrameSize(const std::vector<uint8_t>& buffer) const
    {
        // bytes    description
        // 1        address
        // 1        function-code
        // 2        start register address
        // 2        register count
        // 1        n = byte count
        // n        data (uint16_t each)
        // 2        CRC
        uint32_t frameSize(0); // 0 => Need more data
        const size_t minFrameSize = 9;
        const auto bufSize = buffer.size();
        if (bufSize >= minFrameSize)
        {
            const uint8_t byteCount = buffer[6];
            const uint8_t totalFrameSize = byteCount + minFrameSize;
            if (bufSize >= totalFrameSize)
            {
                frameSize = totalFrameSize;
            }
            // Validate byteCount must match the addressCount
            uint16_t addressCount = static_cast<uint16_t>(buffer[4]) << 8;
            addressCount += static_cast<uint16_t>(buffer[5]);
            if (byteCount != (addressCount * 2))
            {
                // This should never happen, but is a protocol error
                frameSize = TRY_FIND_VALID_FRAME;
            }
        }
        return frameSize;
    }

private:
    std::vector<uint8_t> m_writeBuffer;
};

// Illegal function code
// Not used
// class RequestIllegalFunctionCode : public Request
// {
// public:
//     RequestIllegalFunctionCode()
//         : Request(0)
//     { }
//     virtual ~RequestIllegalFunctionCode() { }
//     virtual uint32_t GetSize() const
//     {
//         // bytes    description
//         // 1        address
//         // 1        function-code
//         // 2        CRC
//         // don't know the size but at least 4
//         return 4;
//     }
//     virtual bool InitFrame(const std::vector<uint8_t>& buffer)
//     {
//         m_modbusAddress = buffer[0];
//         m_functionCode = buffer[1];
//         m_errorCode = ResponseError::IllegalFunction;
//         m_isResponseValid = true;

//         return true;
//     }
//     virtual bool Process(IModbusRegisters& /*regs*/)
//     {
//         // Should never be called
//         return false;
//     }
//     virtual const std::vector<uint8_t>* GetResponsePayload()
//     {
//         if (m_isResponseValid == false)
//         {
//             return nullptr;
//         }

//         SetResponseHeader();
//         return &m_response;
//     }
//     virtual const std::string ToString() const
//     {
//         char temp[128] = {0};
//         sprintf(temp, "RequestIllegalFunctionCode: mod-addr[%d], fctcode[%d]", m_modbusAddress, m_functionCode);
//         return temp;
//     }
// };

} // namespace modb
