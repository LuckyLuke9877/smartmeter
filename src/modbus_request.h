#pragma once

#include "modbus_interface.h"

#include <stdint.h>
#include <string>
#include <vector>

namespace modb
{

// Error-flag is ored with function-code
constexpr uint8_t ERROR_FLAG = 0x80;
constexpr uint8_t RESPONSE_HEADER_SIZE = 3;

// Base class for all requests ( only for register function codes )
class Request
{
public:
    Request(uint8_t functionCode)
        : m_functionCode(functionCode)
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
    virtual bool InitFromBuffer(const std::vector<uint8_t>& buffer)
    {
        // CRC is not used here, it is already validated
        m_isResponseValid = false;
        m_errorCode = ResponseError::None;
        const auto begin = buffer.begin();
        const auto functionCode = *(begin + 1);
        if (functionCode != m_functionCode)
        {
            // must never happen => do not init with buffer from other function-code
            return false;
        }
        m_modbusAddress = *(begin + 0);

        return true;
    }

    virtual uint32_t GetSize() const = 0;
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
    std::vector<uint8_t> m_response{RESPONSE_HEADER_SIZE};

    void SetResponseHeader()
    {
        uint8_t functionCode = m_functionCode;
        uint8_t byte2 = m_response.size() - RESPONSE_HEADER_SIZE;
        if (m_errorCode != ResponseError::None)
        {
            // has error: byte2 is the error-code
            byte2 = static_cast<uint8_t>(m_errorCode);
            functionCode |= ERROR_FLAG;
            // clear any data
            m_response.resize(RESPONSE_HEADER_SIZE);
        }

        // set header
        m_response[0] = m_modbusAddress;
        m_response[1] = functionCode;
        m_response[2] = byte2;
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
    virtual uint32_t GetSize() const
    {
        // bytes    description
        // 1        address
        // 1        function-code
        // 2        start register address
        // 2        register count
        // 2        CRC
        return 8;
    }
    virtual bool InitFromBuffer(const std::vector<uint8_t>& buffer)
    {
        if (Request::InitFromBuffer(buffer) == false)
        {
            return false;
        }
        const auto begin = buffer.begin();
        // Note: Received as big endian => convert to little endian
        m_startAddress = static_cast<uint16_t>(*(begin + 2)) << 8;
        m_startAddress += static_cast<uint16_t>(*(begin + 3));
        m_addressCount = static_cast<uint16_t>(*(begin + 4)) << 8;
        m_addressCount += static_cast<uint16_t>(*(begin + 5));

        return true;
    }
    virtual bool Process(IModbusRegisters& regs)
    {
        // Also on error response is valid
        m_isResponseValid = true;
        m_response.resize(RESPONSE_HEADER_SIZE + m_addressCount * modb::REGISTER_SIZE);
        m_errorCode = regs.Read(m_startAddress, m_addressCount, &m_response[RESPONSE_HEADER_SIZE]);

        return m_errorCode == modb::ResponseError::None;
    }
    virtual const std::vector<uint8_t>* GetResponsePayload()
    {
        if (m_isResponseValid == false)
        {
            return nullptr;
        }

        SetResponseHeader();
        // data are already set by Process()
        return &m_response;
    }
    virtual const std::string ToString() const
    {
        char temp[128] = {0};
        sprintf(temp, "Request03: mod-addr[%d], reg-start[%d], reg-count[%d]", m_modbusAddress, m_startAddress, m_addressCount);
        return temp;
    }
};

// Illegal function code
// Not used
class RequestIllegalFunctionCode : public Request
{
public:
    RequestIllegalFunctionCode()
        : Request(0)
    { }
    virtual ~RequestIllegalFunctionCode() { }
    virtual uint32_t GetSize() const
    {
        // bytes    description
        // 1        address
        // 1        function-code
        // 2        CRC
        // don't know the size but at least 4
        return 4;
    }
    virtual bool InitFromBuffer(const std::vector<uint8_t>& buffer)
    {
        m_modbusAddress = buffer[0];
        m_functionCode = buffer[1];
        m_errorCode = ResponseError::IllegalFunction;
        m_isResponseValid = true;

        return true;
    }
    virtual bool Process(IModbusRegisters& /*regs*/)
    {
        // Should never be called
        return false;
    }
    virtual const std::vector<uint8_t>* GetResponsePayload()
    {
        if (m_isResponseValid == false)
        {
            return nullptr;
        }

        SetResponseHeader();
        return &m_response;
    }
    virtual const std::string ToString() const
    {
        char temp[128] = {0};
        sprintf(temp, "RequestIllegalFunctionCode: mod-addr[%d], fctcode[%d]", m_modbusAddress, m_functionCode);
        return temp;
    }
};

} // namespace modb
