#include <gtest/gtest.h>
#include "modbus_test_utils.h"
#include "../src/modbus_request.h"

#include <cstring>
#include <vector>

using namespace testutils;

//////////////////////////////////////////////////////////////////////
class Request03Test : public ::testing::Test
{
protected:
    modb::Request03 m_request;
    TestRegisters m_registers;
    const std::vector<uint8_t> m_validTestData{0x01, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x09};

    std::vector<uint8_t> GetTestData(int32_t modifyIndex = -1, uint8_t modifyValue = 0)
    {
        std::vector<uint8_t> data = m_validTestData;
        if (modifyIndex > -1)
        {
            data[modifyIndex] = modifyValue;
        }
        return data;
    }
};

TEST_F(Request03Test, InitFrame_InvalidFunctionCode_InvalidFrame)
{
    const auto result = m_request.InitFrame(GetTestData(1, INVALID_FCT_CODE));
    ASSERT_FALSE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, modb::TRY_FIND_VALID_FRAME);
    ASSERT_EQ(m_request.GetResponsePayload(), nullptr);
}

TEST_F(Request03Test, InitFrame_WrongCrc_InvalidFrame)
{
    const auto result = m_request.InitFrame(GetTestData(7, 0xff));
    ASSERT_FALSE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, modb::TRY_FIND_VALID_FRAME);
    ASSERT_EQ(m_request.GetResponsePayload(), nullptr);
}

TEST_F(Request03Test, InitFrame_NotEnoughBuffer_Return0)
{
    std::vector<uint8_t> testData = GetTestData();
    testData.erase(testData.end() - 1);
    const auto result = m_request.InitFrame(testData);
    ASSERT_FALSE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, 0);
    ASSERT_EQ(m_request.GetResponsePayload(), nullptr);
}

TEST_F(Request03Test, InitFrame_ValidFunctionCode_ValidFrame)
{
    const auto result = m_request.InitFrame(GetTestData());
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, 8);
    ASSERT_EQ(m_request.GetResponsePayload(), nullptr);
}

TEST_F(Request03Test, InitFrame_SecondCall_ResetsPayload)
{
    const std::vector<uint8_t> testData = GetTestData();
    auto result = m_request.InitFrame(testData);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData.size());
    ASSERT_EQ(m_request.Process(m_registers), true);
    auto payload = m_request.GetResponsePayload();
    ASSERT_NE(payload, nullptr);

    result = m_request.InitFrame(testData);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData.size());
    ASSERT_EQ(m_request.GetResponsePayload(), nullptr);
}

TEST_F(Request03Test, InitFrame_SecondCall_ResetsError)
{
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, INVALID_REGISTER_IDX, 0x00, 0x01, 0x94, 0x0b};
    auto result = m_request.InitFrame(testData);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData.size());
    ASSERT_EQ(m_request.Process(m_registers), false);
    AssertPayloadError(m_request.GetResponsePayload(), modb::ResponseError::IllegalAddress);

    const std::vector<uint8_t> testData2 = GetTestData();
    result = m_request.InitFrame(testData2);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData2.size());
    ASSERT_EQ(m_request.Process(m_registers), true);
    auto payload = m_request.GetResponsePayload();
    ASSERT_NE(payload, nullptr);
    ASSERT_EQ((*payload)[1] & modb::ERROR_FLAG, 0);
}

TEST_F(Request03Test, Process_InvalidRegisters_PayloadIsError)
{
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, INVALID_REGISTER_IDX, 0x00, 0x01, 0x94, 0x0b};
    const auto result = m_request.InitFrame(testData);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData.size());
    ASSERT_EQ(m_request.Process(m_registers), false);
    AssertPayloadError(m_request.GetResponsePayload(), modb::ResponseError::IllegalAddress);
}

TEST_F(Request03Test, Process_UnsupportedFunction_PayloadIsError)
{
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x00, 0x00, MAX_REGISTER_COUNT, 0x44, 0x09};
    auto result = m_request.InitFrame(testData);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData.size());
    m_registers.m_canRead = false;
    ASSERT_EQ(m_request.Process(m_registers), false);
    auto payload = m_request.GetResponsePayload();
    AssertPayloadError(payload, modb::ResponseError::IllegalFunction);
    ASSERT_EQ((*payload)[0], testData[0]);
    ASSERT_EQ((*payload)[1] & testData[1], testData[1]);
}

TEST_F(Request03Test, Process_ValidRegisters_PayloadOk)
{
    const std::vector<uint8_t> testData = GetTestData();
    const auto result = m_request.InitFrame(testData);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData.size());
    ASSERT_EQ(m_request.Process(m_registers), true);
    auto payload = m_request.GetResponsePayload();
    ASSERT_NE(payload, nullptr);
    const auto dataSize = MAX_REGISTER_COUNT * sizeof(uint16_t);
    ASSERT_EQ(payload->size(), 3 + dataSize);
    ASSERT_EQ((*payload)[0], 0x01);
    ASSERT_EQ((*payload)[1], 0x03);
    ASSERT_EQ((*payload)[2], dataSize);
    ASSERT_TRUE(m_registers.IsEqual(0, &(*payload)[3], dataSize));
}

TEST_F(Request03Test, ToString_Ok)
{
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x00, 0x00, MAX_REGISTER_COUNT, 0x44, 0x09};
    const auto result = m_request.InitFrame(testData);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData.size());
    auto str = m_request.ToString();
    ASSERT_GE(str.length(), 45);
}

//////////////////////////////////////////////////////////////////////
class Request16Test : public ::testing::Test
{
protected:
    modb::Request16 m_request;
    TestRegisters m_registers;
    const std::vector<uint8_t> m_validTestData{0x01, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x11, 0x22, 0x33, 0x44, 0x42, 0x5A};

    std::vector<uint8_t> GetTestData(int32_t modifyIndex = -1, uint8_t modifyValue = 0)
    {
        std::vector<uint8_t> data = m_validTestData;
        if (modifyIndex > -1)
        {
            data[modifyIndex] = modifyValue;
        }
        return data;
    }
};

TEST_F(Request16Test, InitFrame_InvalidFunctionCode_InvalidFrame)
{
    const auto result = m_request.InitFrame(GetTestData(1, INVALID_FCT_CODE));
    ASSERT_FALSE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, modb::TRY_FIND_VALID_FRAME);
    ASSERT_EQ(m_request.GetResponsePayload(), nullptr);
}

TEST_F(Request16Test, InitFrame_WrongCrc_InvalidFrame)
{
    const auto result = m_request.InitFrame(GetTestData(12, 0xff));
    ASSERT_FALSE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, modb::TRY_FIND_VALID_FRAME);
    ASSERT_EQ(m_request.GetResponsePayload(), nullptr);
}

TEST_F(Request16Test, InitFrame_WrongByteCount_InvalidFrame)
{
    const auto result = m_request.InitFrame(GetTestData(6, 0x05));
    ASSERT_FALSE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, modb::TRY_FIND_VALID_FRAME);
    ASSERT_EQ(m_request.GetResponsePayload(), nullptr);
}

TEST_F(Request16Test, InitFrame_NotEnoughBuffer_Return0)
{
    std::vector<uint8_t> testData = GetTestData();
    testData.erase(testData.end() - 1);
    const auto result = m_request.InitFrame(testData);
    ASSERT_FALSE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, 0);
    ASSERT_EQ(m_request.GetResponsePayload(), nullptr);
}

TEST_F(Request16Test, InitFrame_ValidFunctionCode_ValidFrame)
{
    const auto result = m_request.InitFrame(GetTestData());
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, m_validTestData.size());
    ASSERT_EQ(m_request.GetResponsePayload(), nullptr);
}

TEST_F(Request16Test, InitFrame_SecondCall_ResetsPayload)
{
    const std::vector<uint8_t> testData = GetTestData();
    auto result = m_request.InitFrame(testData);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData.size());
    ASSERT_EQ(m_request.Process(m_registers), true);
    auto payload = m_request.GetResponsePayload();
    ASSERT_NE(payload, nullptr);

    result = m_request.InitFrame(testData);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData.size());
    ASSERT_EQ(m_request.GetResponsePayload(), nullptr);
}

TEST_F(Request16Test, InitFrame_SecondCall_ResetsError)
{
    const std::vector<uint8_t> testData = {0x01, 0x10, 0x00, INVALID_REGISTER_IDX, 0x00, 0x02, 0x04, 0x11, 0x22, 0x33, 0x44, 0x82, 0x65};
    auto result = m_request.InitFrame(testData);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData.size());
    ASSERT_EQ(m_request.Process(m_registers), false);
    AssertPayloadError(m_request.GetResponsePayload(), modb::ResponseError::IllegalAddress);

    const std::vector<uint8_t> testData2 = GetTestData();
    result = m_request.InitFrame(testData2);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData2.size());
    ASSERT_EQ(m_request.Process(m_registers), true);
    auto payload = m_request.GetResponsePayload();
    ASSERT_NE(payload, nullptr);
    ASSERT_EQ((*payload)[1] & modb::ERROR_FLAG, 0);
}

TEST_F(Request16Test, Process_InvalidRegisters_PayloadIsError)
{
    const std::vector<uint8_t> testData = {0x01, 0x10, 0x00, INVALID_REGISTER_IDX, 0x00, 0x02, 0x04, 0x11, 0x22, 0x33, 0x44, 0x82, 0x65};
    const auto result = m_request.InitFrame(testData);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData.size());
    ASSERT_EQ(m_request.Process(m_registers), false);
    AssertPayloadError(m_request.GetResponsePayload(), modb::ResponseError::IllegalAddress);
}

TEST_F(Request16Test, Process_UnsupportedFunction_PayloadIsError)
{
    const std::vector<uint8_t> testData = GetTestData();
    auto result = m_request.InitFrame(testData);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData.size());
    m_registers.m_canWrite = false;
    ASSERT_EQ(m_request.Process(m_registers), false);
    auto payload = m_request.GetResponsePayload();
    AssertPayloadError(payload, modb::ResponseError::IllegalFunction);
    ASSERT_EQ((*payload)[0], testData[0]);
    ASSERT_EQ((*payload)[1] & testData[1], testData[1]);
}

TEST_F(Request16Test, Process_ValidRegisters_PayloadOk)
{
    const std::vector<uint8_t> testData = GetTestData();
    const auto result = m_request.InitFrame(testData);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData.size());
    ASSERT_EQ(m_request.Process(m_registers), true);
    auto payload = m_request.GetResponsePayload();
    ASSERT_NE(payload, nullptr);
    ASSERT_EQ(payload->size(), 6);
    ASSERT_EQ(std::memcmp(&testData[0], &(*payload)[0], 6), 0);
    // are m_registers written correct
    ASSERT_TRUE(m_registers.IsEqual(testData[3], &testData[7], testData[6]));
}

TEST_F(Request16Test, Process_WriteAndRead_PayloadOk)
{
    // Write 2 registers
    const std::vector<uint8_t> testDataWrite = GetTestData();
    auto result = m_request.InitFrame(testDataWrite);
    ASSERT_EQ(m_request.Process(m_registers), true);

    // Read 2 registers
    modb::Request03 req03;
    const std::vector<uint8_t> testDataRead{0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
    result = req03.InitFrame(testDataRead);
    ASSERT_EQ(req03.Process(m_registers), true);
    auto payload = req03.GetResponsePayload();
    ASSERT_NE(payload, nullptr);
    const auto dataSize = testDataRead[5] * sizeof(uint16_t);
    ASSERT_EQ(payload->size(), 3 + dataSize);
    ASSERT_EQ((*payload)[0], testDataRead[0]);
    ASSERT_EQ((*payload)[1], testDataRead[1]);
    ASSERT_EQ((*payload)[2], dataSize);
    ASSERT_EQ(std::memcmp(&testDataWrite[7], &(*payload)[3], dataSize), 0);
}

TEST_F(Request16Test, ToString_Ok)
{
    const std::vector<uint8_t> testData = GetTestData();
    const auto result = m_request.InitFrame(testData);
    ASSERT_TRUE(result.canProcess);
    ASSERT_EQ(result.usedFrameSize, testData.size());
    auto str = m_request.ToString();
    ASSERT_GE(str.length(), 70);
}
