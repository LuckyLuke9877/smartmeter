#include <gtest/gtest.h>
#include "modbus_test_utils.h"
#include "../src/modbus_request.h"

#include <cstring>
#include <vector>

//////////////////////////////////////////////////////////////////////
class Request03Test : public ::testing::Test
{
protected:
    modb::Request03 m_request;
    TestRegisters m_registers;

    void SetUp() override { }
    void TearDown() override { }
};

TEST_F(Request03Test, GetSize_Ok)
{
    ASSERT_EQ(m_request.GetSize(), 8);
}

TEST_F(Request03Test, InitFromBuffer_InvalidFunctionCode_ReturnFalse)
{
    std::vector<uint8_t> testData = {0x01, INVALID_FCT_CODE, 0x00, 0x02, 0x00, 0x01};
    ASSERT_EQ(m_request.InitFromBuffer(testData), false);
    ASSERT_EQ(m_request.GetResponsePayload(), nullptr);
}

TEST_F(Request03Test, InitFromBuffer_ValidFunctionCode_ReturnTrue)
{
    std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x02, 0x00, 0x01};
    ASSERT_EQ(m_request.InitFromBuffer(testData), true);
    ASSERT_EQ(m_request.GetResponsePayload(), nullptr);
}

TEST_F(Request03Test, InitFromBuffer_SecondCall_ResetsPayload)
{
    std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x00, 0x00, MAX_REGISTER_COUNT};
    m_request.InitFromBuffer(testData);
    ASSERT_EQ(m_request.Process(m_registers), true);
    auto payload = m_request.GetResponsePayload();
    ASSERT_NE(payload, nullptr);

    ASSERT_EQ(m_request.InitFromBuffer(testData), true);
    ASSERT_EQ(m_request.GetResponsePayload(), nullptr);
}

TEST_F(Request03Test, InitFromBuffer_SecondCall_ResetsError)
{
    std::vector<uint8_t> testData = {0x01, 0x03, 0x00, INVALID_REGISTER_IDX, 0x00, 0x01};
    m_request.InitFromBuffer(testData);
    ASSERT_EQ(m_request.Process(m_registers), false);
    AssertPayloadError(m_request.GetResponsePayload(), modb::ResponseError::IllegalAddress);

    std::vector<uint8_t> testData2 = {0x01, 0x03, 0x00, 0x00, 0x00, MAX_REGISTER_COUNT};
    m_request.InitFromBuffer(testData2);
    ASSERT_EQ(m_request.Process(m_registers), true);
    auto payload = m_request.GetResponsePayload();
    ASSERT_NE(payload, nullptr);
    ASSERT_EQ((*payload)[1] & modb::ERROR_FLAG, 0);
}

TEST_F(Request03Test, Process_InvalidRegisters_PayloadIsError)
{
    std::vector<uint8_t> testData = {0x01, 0x03, 0x00, INVALID_REGISTER_IDX, 0x00, 0x01};
    m_request.InitFromBuffer(testData);
    ASSERT_EQ(m_request.Process(m_registers), false);
    AssertPayloadError(m_request.GetResponsePayload(), modb::ResponseError::IllegalAddress);
}

TEST_F(Request03Test, Process_UnsupportedFunction_PayloadIsError)
{
    std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01};
    m_request.InitFromBuffer(testData);
    m_registers.m_canRead = false;
    ASSERT_EQ(m_request.Process(m_registers), false);
    AssertPayloadError(m_request.GetResponsePayload(), modb::ResponseError::IllegalFunction);
}

TEST_F(Request03Test, Process_ValidRegisters_PayloadOk)
{
    std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x00, 0x00, MAX_REGISTER_COUNT};
    m_request.InitFromBuffer(testData);
    ASSERT_EQ(m_request.Process(m_registers), true);
    auto payload = m_request.GetResponsePayload();
    ASSERT_NE(payload, nullptr);
    const auto dataSize = MAX_REGISTER_COUNT * sizeof(uint16_t);
    ASSERT_EQ(payload->size(), 3 + dataSize);
    ASSERT_EQ((*payload)[0], 0x01);
    ASSERT_EQ((*payload)[1], 0x03);
    ASSERT_EQ((*payload)[2], dataSize);
    ASSERT_EQ(m_registers.IsEqual(0, &(*payload)[3], dataSize), true);
}

TEST_F(Request03Test, ToString_Ok)
{
    std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x04, 0x00, 0x05};
    m_request.InitFromBuffer(testData);
    auto str = m_request.ToString();
    ASSERT_GE(str.length(), 30);
}

//////////////////////////////////////////////////////////////////////
class RequestIllegalFunctionCodeTest : public ::testing::Test
{
protected:
    modb::RequestIllegalFunctionCode m_request;
    TestRegisters m_registers;
};

TEST_F(RequestIllegalFunctionCodeTest, GetSize_Ok)
{
    ASSERT_EQ(m_request.GetSize(), 4);
}

TEST_F(RequestIllegalFunctionCodeTest, InitFromBuffer_InvalidFunctionCode_PayloadIsError)
{
    std::vector<uint8_t> testData = {0x01, INVALID_FCT_CODE, 0x00, 0x02, 0x00, 0x01};
    ASSERT_EQ(m_request.InitFromBuffer(testData), true);
    auto payload = m_request.GetResponsePayload();
    AssertPayloadError(payload, modb::ResponseError::IllegalFunction);
    ASSERT_EQ((*payload)[1], modb::ERROR_FLAG | INVALID_FCT_CODE);
}

TEST_F(RequestIllegalFunctionCodeTest, Process_ReturnsFalse)
{
    std::vector<uint8_t> testData = {0x01, INVALID_FCT_CODE, 0x00, 0x02, 0x00, 0x01};
    ASSERT_EQ(m_request.InitFromBuffer(testData), true);
    ASSERT_EQ(m_request.Process(m_registers), false);
}
