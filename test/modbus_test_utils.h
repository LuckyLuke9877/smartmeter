#include <gtest/gtest.h>
#include "../src/modbus_request.h"

#include <cstring>
#include <vector>

namespace
{

constexpr uint8_t MODBUS_ADDRESS = 0x01;
constexpr uint8_t INVALID_FCT_CODE = 0xAA;
constexpr uint8_t MAX_REGISTER_COUNT = 4;
constexpr uint8_t INVALID_REGISTER_IDX = MAX_REGISTER_COUNT + 1;

void AssertPayloadError(const std::vector<uint8_t>* payload, modb::ResponseError expectedError)
{
    ASSERT_NE(payload, nullptr);
    ASSERT_EQ(payload->size(), 3);
    ASSERT_EQ((*payload)[1] & modb::ERROR_FLAG, modb::ERROR_FLAG);
    ASSERT_EQ((*payload)[2], static_cast<uint8_t>(expectedError));
}

} // namespace

class TestRegisters : public modb::IModbusRegisters
{
public:
    bool m_canRead{true};
    bool m_canWrite{true};
    std::vector<uint16_t> m_registers;

    TestRegisters()
        : m_registers(MAX_REGISTER_COUNT)
    {
        auto idx = 0;
        m_registers[idx++] = 0x010A;
        m_registers[idx++] = 0x020B;
        m_registers[idx++] = 0x030C;
        m_registers[idx++] = 0x040D;
    }

    virtual ~TestRegisters() { }

    virtual modb::ResponseError
    Read(const uint16_t registerAddress, const uint16_t registerCount, uint8_t* target) const
    {
        if (m_canRead == false)
        {
            return modb::ResponseError::IllegalFunction;
        }
        if (registerCount == 0 || registerAddress + registerCount > MAX_REGISTER_COUNT)
        {
            return modb::ResponseError::IllegalAddress; // invalid index
        }
        std::memcpy(target, &m_registers[registerAddress], registerCount * sizeof(m_registers[0]));

        return modb::ResponseError::None;
    }
    virtual modb::ResponseError
    Write(const uint16_t registerAddress, const uint16_t registerCount, const uint8_t* source)
    {
        if (m_canWrite == false)
        {
            return modb::ResponseError::IllegalFunction;
        }
        if (registerCount == 0 || registerAddress + registerCount > MAX_REGISTER_COUNT)
        {
            return modb::ResponseError::IllegalAddress; // invalid index
        }
        std::memcpy(&m_registers[registerAddress], source, registerCount * sizeof(m_registers[0]));

        return modb::ResponseError::None;
    }

    bool IsEqual(const uint8_t startRegister, const uint8_t* data, const size_t count) const
    {
        if (!data || count == 0
            || (startRegister * sizeof(m_registers[0]) + count) > (m_registers.size() * sizeof(m_registers[0])))
        {
            return false;
        }

        return std::memcmp(&m_registers[startRegister], data, count) == 0;
    }
};
