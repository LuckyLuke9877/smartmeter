#pragma once

#define GTEST
#include "esphome_mock.h"
#include "../src/conversion.h"
#include "../src/modbus_registers.h"

#include <cstring>
#include <stdint.h>
#include <vector>

namespace testutils
{

constexpr uint8_t INVALID_FCT_CODE = 0xAA;
constexpr uint8_t MAX_REGISTER_COUNT = 4;
constexpr uint8_t INVALID_REGISTER_IDX = MAX_REGISTER_COUNT + 1;

void AssertPayloadError(const std::vector<uint8_t>* payload, modb::ResponseError expectedError);

bool IsEqualString(uint16_t* reg, const std::string& text);
bool IsEqualString(uint8_t* buf, const std::string& text);
bool IsEqualUint16(uint8_t* bufBigEndian, uint16_t valueLittleEndian);

float ToFloatLittleEndian(uint16_t* reg);

class TestRegisters : public modb::ModbusRegisters
{
public:
    bool m_canRead{true};
    bool m_canWrite{true};

    TestRegisters()
        : modb::ModbusRegisters(0, MAX_REGISTER_COUNT)
    {
        auto idx = 0;
        SetRegisterUint16(idx++, 0x010A);
        SetRegisterUint16(idx++, 0x020B);
        SetRegisterUint16(idx++, 0x030C);
        SetRegisterUint16(idx++, 0x040D);
    }

    virtual ~TestRegisters() { }

    virtual modb::ResponseError Read(const uint16_t registerAddress, const uint16_t registerCount, uint8_t* target) const
    {
        if (m_canRead == false)
        {
            return modb::ResponseError::IllegalFunction;
        }

        return modb::ModbusRegisters::Read(registerAddress, registerCount, target);
    }
    virtual modb::ResponseError Write(const uint16_t registerAddress, const uint16_t registerCount, const uint8_t* source)
    {
        if (m_canWrite == false)
        {
            return modb::ResponseError::IllegalFunction;
        }

        return modb::ModbusRegisters::Write(registerAddress, registerCount, source);
    }

    bool IsEqual(const uint8_t startRegister, const uint8_t* data, const size_t count) const
    {
        if (!data || count == 0 || (startRegister * sizeof(m_registers[0]) + count) > (m_registers.size() * sizeof(m_registers[0])))
        {
            return false;
        }

        return std::memcmp(&m_registers[startRegister], data, count) == 0;
    }
};

} // namespace testutils
