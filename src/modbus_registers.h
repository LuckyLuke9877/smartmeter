#pragma once

#include "conversion.h"
#include "modbus_interface.h"

#include <cstring>
#include <stdint.h>
#include <string>
#include <vector>

namespace modb
{

// Values are stored big endian
class ModbusRegisters : public modb::IModbusRegisters
{
public:
    ModbusRegisters(uint16_t startOffset, uint16_t count)
        : m_startOffset(startOffset)
        , m_registers(count, 0x00)
    { }

    virtual ~ModbusRegisters() { }

    virtual modb::ResponseError Read(const uint16_t registerAddress, const uint16_t registerCount, uint8_t* target) const
    {
        const int32_t registerIndex = GetRegisterIndexForRange(registerAddress, registerCount);
        if (registerIndex < 0)
        {
            return modb::ResponseError::IllegalAddress; // invalid index
        }
        std::memcpy(target, &m_registers[registerIndex], registerCount * sizeof(m_registers[0]));

        return modb::ResponseError::None;
    }
    virtual modb::ResponseError Write(const uint16_t registerAddress, const uint16_t registerCount, const uint8_t* source)
    {
        const int32_t registerIndex = GetRegisterIndexForRange(registerAddress, registerCount);
        if (registerIndex < 0)
        {
            return modb::ResponseError::IllegalAddress; // invalid index
        }
        std::memcpy(&m_registers[registerIndex], source, registerCount * sizeof(m_registers[0]));

        return modb::ResponseError::None;
    }

    // For unit-tests
    std::vector<uint16_t> GetRegister(const uint16_t registerAddress, const uint16_t registerCount) const
    {
        const int32_t registerIndex = GetRegisterIndexForRange(registerAddress, registerCount);
        if (registerIndex < 0)
        {
            return {}; // invalid index
        }
        std::vector<uint16_t> reg(registerCount);
        std::memcpy(&reg[0], &m_registers[registerIndex], registerCount * sizeof(m_registers[0]));

        return reg;
    }

protected:
    int32_t GetRegisterIndexForRange(const uint16_t registerAddress, const uint16_t registerCount) const
    {
        // registerAddress is already REGISTER_OFFSET-based! (e.g. sunspec-address: 40001 is
        // registerAddress: 40000)
        const int32_t registerIndex = registerAddress - m_startOffset;
        if (registerCount < 1 || registerIndex < 0 || (registerIndex + registerCount) > m_registers.size())
        {
            return -1; // invalid index
        }

        return registerIndex;
    }

    void SetFloats(uint32_t registerIndex, const std::vector<float>& values)
    {
        for (size_t i = 0; i < values.size(); i++)
        {
            SetRegisterFloat(registerIndex + (i * 2), values[i]);
        }
    }
    void SetRegisterUint16(uint32_t registerIndex, uint16_t value)
    {
        SetRegister(registerIndex, value);
    }
    void SetRegisterUint32(uint32_t registerIndex, uint32_t value)
    {
        SetRegister(registerIndex, value);
    }
    void SetRegisterFloat(uint32_t registerIndex, float value)
    {
        SetRegister(registerIndex, value);
    }
    void SetRegisterString(uint32_t registerIndex, const std::string& text, uint16_t registerCount)
    {
        // No validation, only for internal use
        std::memset(&m_registers[registerIndex], 0x00, registerCount * sizeof(m_registers[0]));
        std::memcpy(&m_registers[registerIndex], text.c_str(), text.length());
    }
    template <typename T>
    void SetRegister(uint32_t registerIndex, T value)
    {
        T temp = Convert2BigEndian(value);
        std::memcpy(&m_registers[registerIndex], &temp, sizeof(temp));
    }

    // Fixed register model
    const uint16_t m_startOffset;
    std::vector<uint16_t> m_registers;
};

} // namespace modb
