#include <gtest/gtest.h>
#define GTEST
#include "esphome_mock.h"
#include "modbus_test_utils.h"
#include "../src/modbus_server.h"

#include <cstring>

using namespace esphome;

class ModbusServerTest : public ::testing::Test
{
protected:
    uint32_t m_validRequestCount{0};
    std::unique_ptr<modb::ModbusServer> m_server;
    TestRegisters m_registers;

    void SetUp() override
    {
        m_server.reset(
            new modb::ModbusServer([this](modb::Request& request) { return OnModbusReceiveRequest(request); }));
    }
    void TearDown() override { }

    void OnModbusReceiveRequest(modb::Request& request)
    {
        if (request.GetModbusAddress() != 0x01)
        {
            return;
        }
        // std::cout << "Request received\n";
        m_validRequestCount++;
        request.Process(m_registers);
    }
};

TEST_F(ModbusServerTest, OnReceive_IncompleteRequest_RxBufferOk)
{
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25};

    m_server->AddRx(testData);
    ASSERT_EQ(m_server->m_uartRx.size(), 7);
    m_server->ProcessRequest();

    ASSERT_EQ(m_server->m_uartRx.size(), 0);
    ASSERT_EQ(m_server->m_rxBuffer.size(), 7);
    ASSERT_EQ(m_server->m_rxBuffer, testData);
    ASSERT_EQ(m_server->m_uartTx.size(), 0);
    ASSERT_EQ(m_validRequestCount, 0);
}

TEST_F(ModbusServerTest, OnReceive_IncompleteFollowedByValidRequest_ResponseOk)
{
    const uint8_t registerIndex = 2;
    const uint8_t registerCount = 1;
    std::vector<uint8_t> testData = {0x01, 0x03, 0x00, registerIndex, 0x00, registerCount, 0x25};

    m_server->AddRx(testData);
    testData.push_back(0xca);
    m_server->AddRx(testData);
    ASSERT_EQ(m_server->m_uartRx.size(), 15);
    m_server->ProcessRequest();

    ASSERT_EQ(m_server->m_uartRx.size(), 0);
    ASSERT_EQ(m_server->m_rxBuffer.size(), 0);
    ASSERT_EQ(m_server->m_uartTx.size(), 7);
    ASSERT_EQ(m_validRequestCount, 1);
    ASSERT_EQ(m_registers.IsEqual(registerIndex, &m_server->m_uartTx[3], registerCount * sizeof(uint16_t)), true);
}

TEST_F(ModbusServerTest, OnReceive_InvalidCrcFollowedByValidRequest_ResponseOk)
{
    const std::vector<uint8_t> invalidTestData = {0x01, 0x03, 0x15, 0x12, 0x00, 0x01, 0x25, 0xff};
    const uint8_t registerIndex = 2;
    const uint8_t registerCount = 1;
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, registerIndex, 0x00, registerCount, 0x25, 0xca};

    m_server->AddRx(invalidTestData);
    m_server->AddRx(testData);
    m_server->AddRx(invalidTestData);
    m_server->AddRx(testData);
    ASSERT_EQ(m_server->m_uartRx.size(), 32);
    m_server->ProcessRequest();

    ASSERT_EQ(m_server->m_uartRx.size(), 0);
    ASSERT_EQ(m_server->m_rxBuffer.size(), 0);
    ASSERT_EQ(m_server->m_uartTx.size(), 14);
    ASSERT_EQ(m_validRequestCount, 2);
    ASSERT_EQ(m_registers.IsEqual(registerIndex, &m_server->m_uartTx[3], registerCount * sizeof(uint16_t)), true);
    ASSERT_EQ(m_registers.IsEqual(registerIndex, &m_server->m_uartTx[10], registerCount * sizeof(uint16_t)), true);
}

TEST_F(ModbusServerTest, OnReceive_ValidRequest_InvalidRead_Response_IsError)
{
    m_registers.m_canRead = false;
    const uint8_t registerIndex = 2;
    const uint8_t registerCount = 1;
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, registerIndex, 0x00, registerCount, 0x25, 0xca};

    m_server->AddRx(testData);
    ASSERT_EQ(m_server->m_uartRx.size(), 8);
    m_server->ProcessRequest();

    ASSERT_EQ(m_server->m_uartRx.size(), 0);
    ASSERT_EQ(m_server->m_rxBuffer.size(), 0);
    ASSERT_EQ(m_server->m_uartTx.size(), 5);
    ASSERT_EQ(m_validRequestCount, 1);
    ASSERT_EQ(m_server->m_uartTx[0], testData[0]);
    ASSERT_EQ(m_server->m_uartTx[1], testData[1] | modb::ERROR_FLAG);
    ASSERT_EQ(m_server->m_uartTx[2], static_cast<uint8_t>(modb::ResponseError::IllegalFunction));
}

TEST_F(ModbusServerTest, OnReceive_InvalidFunctionCodeFollowedByValidRequest_ResponseOk)
{
    const std::vector<uint8_t> invalidFunctionCodeTestData = {0x01, 0x07, 0x00, 0x02, 0x00, 0x01, 0x90, 0x0a};
    const uint8_t registerIndex = 0;
    const uint8_t registerCount = 2;
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, registerIndex, 0x00, registerCount, 0xc4, 0x0b};

    m_server->AddRx(invalidFunctionCodeTestData);
    m_server->AddRx(testData);
    ASSERT_EQ(m_server->m_uartRx.size(), 16);
    m_server->ProcessRequest();

    ASSERT_EQ(m_server->m_uartRx.size(), 0);
    ASSERT_EQ(m_server->m_rxBuffer.size(), 0);
    ASSERT_EQ(m_server->m_uartTx.size(), 9);
    ASSERT_EQ(m_validRequestCount, 1);
    ASSERT_EQ(m_registers.IsEqual(registerIndex, &m_server->m_uartTx[3], registerCount * sizeof(uint16_t)), true);
}

TEST_F(ModbusServerTest, OnReceive_ValidRequestButWrongAddress_ResponseNone)
{
    const std::vector<uint8_t> testData = {0x02, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25, 0xf9};

    m_server->AddRx(testData);
    ASSERT_EQ(m_server->m_uartRx.size(), 8);
    m_server->ProcessRequest();

    ASSERT_EQ(m_server->m_uartRx.size(), 0);
    ASSERT_EQ(m_server->m_rxBuffer.size(), 0);
    ASSERT_EQ(m_server->m_uartTx.size(), 0);
    ASSERT_EQ(m_validRequestCount, 0);
}

TEST_F(ModbusServerTest, OnReceive_ValidRequest_ResponseOk)
{
    const uint8_t registerIndex = 2;
    const uint8_t registerCount = 1;
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, registerIndex, 0x00, registerCount, 0x25, 0xca};

    // Receive in small peaces and always try to parse
    uint8_t pos = 0;
    // Byte 1
    m_server->m_uartRx.push_back(testData[pos++]);
    m_server->ProcessRequest();
    ASSERT_EQ(m_server->m_uartRx.size(), 0);
    ASSERT_EQ(m_server->m_rxBuffer.size(), 1);
    ASSERT_EQ(m_server->m_rxBuffer[0], 1);

    // Byte 2
    m_server->m_uartRx.push_back(testData[pos++]);
    m_server->ProcessRequest();
    ASSERT_EQ(m_server->m_uartRx.size(), 0);
    ASSERT_EQ(m_server->m_rxBuffer.size(), 2);

    // Byte 3 - 6
    m_server->m_uartRx.push_back(testData[pos++]);
    m_server->m_uartRx.push_back(testData[pos++]);
    m_server->m_uartRx.push_back(testData[pos++]);
    m_server->m_uartRx.push_back(testData[pos++]);
    m_server->ProcessRequest();
    ASSERT_EQ(m_server->m_uartRx.size(), 0);
    ASSERT_EQ(m_server->m_rxBuffer.size(), 6);

    // Byte 7
    m_server->m_uartRx.push_back(testData[pos++]);
    m_server->ProcessRequest();
    ASSERT_EQ(m_server->m_rxBuffer.size(), 7);
    ASSERT_EQ(m_server->m_uartTx.size(), 0);

    // Byte 8: frame is complete => response received
    m_server->m_uartRx.push_back(testData[pos++]);
    m_server->ProcessRequest();
    ASSERT_EQ(m_server->m_rxBuffer.size(), 0);
    ASSERT_EQ(m_server->m_uartTx.size(), 7);

    ASSERT_EQ(m_server->m_uartTx[0], testData[0]);
    ASSERT_EQ(m_server->m_uartTx[1], testData[1]);
    ASSERT_EQ(m_server->m_uartTx[2], registerCount * sizeof(uint16_t));
    auto expectedCrc = crc16(&m_server->m_uartTx[0], m_server->m_uartTx.size() - 2);
    ASSERT_EQ(m_server->m_uartTx[5], expectedCrc & 0xFF);
    ASSERT_EQ(m_server->m_uartTx[6], expectedCrc >> 8);
    ASSERT_EQ(m_validRequestCount, 1);
    ASSERT_EQ(m_registers.IsEqual(registerIndex, &m_server->m_uartTx[3], registerCount * sizeof(uint16_t)), true);
}

TEST_F(ModbusServerTest, Send_Response4Bytes_CrcOk)
{
    const uint8_t registerIndex = 0;
    const uint8_t registerCount = 2;
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, registerIndex, 0x00, registerCount, 0xc4, 0x0b};

    m_server->AddRx(testData);
    ASSERT_EQ(m_server->m_uartRx.size(), 8);
    m_server->ProcessRequest();

    ASSERT_EQ(m_server->m_uartRx.size(), 0);
    ASSERT_EQ(m_server->m_rxBuffer.size(), 0);
    ASSERT_EQ(m_server->m_uartTx.size(), 9);
    auto expectedCrc = crc16(&m_server->m_uartTx[0], m_server->m_uartTx.size() - 2);
    ASSERT_EQ(m_server->m_uartTx[7], expectedCrc & 0xFF);
    ASSERT_EQ(m_server->m_uartTx[8], expectedCrc >> 8);
    ASSERT_EQ(m_validRequestCount, 1);
    ASSERT_EQ(m_registers.IsEqual(registerIndex, &m_server->m_uartTx[3], registerCount * sizeof(uint16_t)), true);
}
