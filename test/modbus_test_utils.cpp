#include <gtest/gtest.h>
#include "modbus_test_utils.h"
#include "../src/modbus_request.h"

namespace testutils
{

void AssertPayloadError(const std::vector<uint8_t>* payload, modb::ResponseError expectedError)
{
    ASSERT_NE(payload, nullptr);
    ASSERT_EQ(payload->size(), 3);
    ASSERT_EQ((*payload)[1] & modb::ERROR_FLAG, modb::ERROR_FLAG);
    ASSERT_EQ((*payload)[2], static_cast<uint8_t>(expectedError));
}

bool IsEqualString(uint16_t* reg, const std::string& text)
{
    return std::memcmp(reg, text.c_str(), text.length()) == 0;
}

bool IsEqualString(uint8_t* buf, const std::string& text)
{
    return std::memcmp(buf, text.c_str(), text.length()) == 0;
}

bool IsEqualUint16(uint8_t* bufBigEndian, uint16_t valueLittleEndian)
{
    // convert to little-endian
    uint16_t bufValue = *(bufBigEndian + 1) + (*(bufBigEndian + 0) << 8);
    return bufValue == valueLittleEndian;
}

// Use other conversion method for test
float ToFloatLittleEndian(uint16_t* reg)
{
    // Note: cannot direct cast to float => wrong result!!
    uint32_t tempUint32 = __builtin_bswap32(*(uint32_t*)reg);
    return *(reinterpret_cast<float*>(&tempUint32));
}

} // namespace testutils
