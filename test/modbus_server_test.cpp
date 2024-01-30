#include <gtest/gtest.h>
#define GTEST
#include "esphome_mock.h"
#include "../src/modbus_server.h"

#include <cstring>

using namespace esphome;
using namespace esphome::modbus;

class ModbusServerTest : public ::testing::Test
{
protected:
    std::vector<ModbusServer::RequestRead> m_requests;
    float                                  m_responseValue{0.0f};
    std::unique_ptr<ModbusServer>          m_server;

    void SetUp() override
    {
        m_server.reset(new ModbusServer(0x01U, [this](uint8_t function_code, const ModbusServer::RequestRead& request) {
            return on_modbus_receive_request(function_code, request);
        }));
    }
    void TearDown() override { }

    ModbusServer::ResponseRead
    on_modbus_receive_request(uint8_t function_code, const ModbusServer::RequestRead& request)
    {
        // std::cout << "Request received\n";
        m_requests.push_back(request);

        // Simulate some response
        ModbusServer::ResponseRead response;
        if (m_responseValue != 0.0f)
        {
            // MSB first as it will be in Sunspec
            auto     buffer = response.GetDataBuffer(sizeof(m_responseValue));
            auto     bigEndianValue = convert_big_endian(m_responseValue);
            uint8_t* val = reinterpret_cast<uint8_t*>(&bigEndianValue);
            for (int i = 0; i < sizeof(bigEndianValue); i++)
            {
                buffer[i] = val[i];
            }
        }
        else
        {
            // Error response
            response.set_error(ModbusServer::ResponseRead::ErrorCode::ILLEGAL_FUNCTION);
        }

        return response;
    }
};

TEST_F(ModbusServerTest, IncompleteRequest_RxBufferOk)
{
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25};

    m_server->add_rx(testData);
    ASSERT_EQ(m_server->m_rx.size(), 7);
    m_server->process_requests();

    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 7);
    ASSERT_EQ(m_server->rx_buffer_, testData);
    ASSERT_EQ(m_server->m_tx.size(), 0);
    ASSERT_EQ(m_requests.size(), 0);
}

TEST_F(ModbusServerTest, IncompleteAndValidRequest_ResponseOk)
{
    std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25};
    m_responseValue = 42.3f;

    m_server->add_rx(testData);
    testData.push_back(0xca);
    m_server->add_rx(testData);
    ASSERT_EQ(m_server->m_rx.size(), 15);
    m_server->process_requests();

    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 0);
    ASSERT_EQ(m_server->m_tx.size(), 9);
    ASSERT_EQ(m_requests.size(), 1);
}

TEST_F(ModbusServerTest, InvalidCrcAndValidRequest_ResponseOk)
{
    const std::vector<uint8_t> invalidTestData = {0x01, 0x03, 0x15, 0x12, 0x00, 0x01, 0x25, 0xff};
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25, 0xca};
    m_responseValue = 42.3f;

    m_server->add_rx(invalidTestData);
    m_server->add_rx(testData);
    m_server->add_rx(invalidTestData);
    m_server->add_rx(testData);
    ASSERT_EQ(m_server->m_rx.size(), 32);
    m_server->process_requests();

    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 0);
    ASSERT_EQ(m_server->m_tx.size(), 18);
    ASSERT_EQ(m_requests.size(), 2);
    ASSERT_EQ(::memcmp(&m_requests[0], &m_requests[1], sizeof(m_requests[0])), 0);
}

TEST_F(ModbusServerTest, ValidRequest_InvalidFunctionCode_Response_IsError)
{
    std::vector<uint8_t> testData = {0x01, 0x04, 0x00, 0x02, 0x00, 0x01, 0x90, 0x0a};

    m_server->add_rx(testData);
    ASSERT_EQ(m_server->m_rx.size(), 8);
    m_server->process_requests();

    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 0);
    ASSERT_EQ(m_server->m_tx.size(), 5);
    ASSERT_EQ(m_requests.size(), 1);
    ASSERT_EQ(m_server->m_tx[0], testData[0]);
    ASSERT_EQ(m_server->m_tx[1], testData[1] | 0x80);
    ASSERT_EQ(m_server->m_tx[2], ModbusServer::ResponseRead::ErrorCode::ILLEGAL_FUNCTION);
}

TEST_F(ModbusServerTest, InvalidFunctionCodeAndValidRequest_ResponseOk)
{
    const std::vector<uint8_t> invalidFunctionCodeTestData = {0x01, 0x07, 0x00, 0x02, 0x00, 0x01, 0x90, 0x0a};
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0b};
    m_responseValue = 42.3f;

    m_server->add_rx(invalidFunctionCodeTestData);
    m_server->add_rx(testData);
    ASSERT_EQ(m_server->m_rx.size(), 16);
    m_server->process_requests();

    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 0);
    ASSERT_EQ(m_server->m_tx.size(), 9);
    ASSERT_EQ(m_requests.size(), 1);
}

TEST_F(ModbusServerTest, ValidRequestButWrongAddress_Response_None)
{
    const std::vector<uint8_t> testData = {0x02, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25, 0xf9};

    m_server->add_rx(testData);
    ASSERT_EQ(m_server->m_rx.size(), 8);
    m_server->process_requests();

    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 0);
    ASSERT_EQ(m_server->m_tx.size(), 0);
    ASSERT_EQ(m_requests.size(), 0);
}

TEST_F(ModbusServerTest, ValidRequest_ResponseOk)
{
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25, 0xca};
    m_responseValue = 42.3f;

    // Receive in small peaces and always try to parse
    uint8_t pos = 0;
    m_server->m_rx.push_back(testData[pos++]);
    m_server->process_requests();
    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 1);
    ASSERT_EQ(m_server->rx_buffer_[0], 1);

    m_server->m_rx.push_back(testData[pos++]);
    m_server->process_requests();
    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 2);

    m_server->m_rx.push_back(testData[pos++]);
    m_server->m_rx.push_back(testData[pos++]);
    m_server->m_rx.push_back(testData[pos++]);
    m_server->m_rx.push_back(testData[pos++]);
    m_server->process_requests();
    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 6);

    m_server->m_rx.push_back(testData[pos++]);
    m_server->process_requests();
    ASSERT_EQ(m_server->rx_buffer_.size(), 7);
    ASSERT_EQ(m_server->m_tx.size(), 0);

    m_server->m_rx.push_back(testData[pos++]);
    m_server->process_requests();
    ASSERT_EQ(m_server->rx_buffer_.size(), 0);
    ASSERT_EQ(m_server->m_tx.size(), 9);

    ASSERT_EQ(m_server->m_tx[0], testData[0]);
    ASSERT_EQ(m_server->m_tx[1], testData[1]);
    ASSERT_EQ(m_server->m_tx[2], 4);
    uint8_t* val = (uint8_t*)(&m_responseValue);
    ASSERT_EQ(m_server->m_tx[3], val[3]);
    ASSERT_EQ(m_server->m_tx[4], val[2]);
    ASSERT_EQ(m_server->m_tx[5], val[1]);
    ASSERT_EQ(m_server->m_tx[6], val[0]);
    auto expectedCrc = crc16(&m_server->m_tx[0], m_server->m_tx.size() - 2);
    ASSERT_EQ(m_server->m_tx[7], expectedCrc & 0xFF);
    ASSERT_EQ(m_server->m_tx[8], expectedCrc >> 8);
    ASSERT_EQ(m_requests.size(), 1);
}

TEST_F(ModbusServerTest, ResponseRead_get_payload_SetSomeData_ResultOk)
{
    const std::vector<uint8_t> testData = {0x01, 0x02, 0x03, 0x04};
    ModbusServer::ResponseRead response;
    std::memcpy(response.GetDataBuffer(testData.size()), &testData[0], testData.size());

    const uint8_t address = 0x42;
    const uint8_t functionCode = 0x03;
    auto          result = response.get_payload(address, functionCode);

    ASSERT_EQ(result[0], address);
    ASSERT_EQ(result[1], functionCode);
    ASSERT_EQ(result[2], testData.size());
    ASSERT_EQ(result[3], testData[0]);
    ASSERT_EQ(result[4], testData[1]);
    ASSERT_EQ(result[5], testData[2]);
    ASSERT_EQ(result[6], testData[3]);
}

TEST_F(ModbusServerTest, ResponseRead_get_payload_set_error_ResultIsError)
{
    const std::vector<uint8_t> testData = {0x01, 0x02, 0x03, 0x04};
    ModbusServer::ResponseRead response;
    // first set some data, but error must overrule
    std::memcpy(response.GetDataBuffer(testData.size()), &testData[0], testData.size());
    response.set_error(ModbusServer::ResponseRead::ErrorCode::ILLEGAL_VALUE);

    const uint8_t address = 0x42;
    const uint8_t functionCode = 0x03;
    auto          result = response.get_payload(address, functionCode);

    ASSERT_EQ(result[0], address);
    ASSERT_EQ(result[1], functionCode | 0x80);
    ASSERT_EQ(result[2], ModbusServer::ResponseRead::ErrorCode::ILLEGAL_VALUE);
}

TEST_F(ModbusServerTest, ResponseRead_GetDataBuffer_SizeTooBig_NoBufferReturned)
{
    ModbusServer::ResponseRead response;
    ASSERT_EQ(response.GetDataBuffer(1025), nullptr);
}

TEST_F(ModbusServerTest, ResponseRead_GetDataBuffer_SizeOk_BufferReturned)
{
    ModbusServer::ResponseRead response;
    ASSERT_NE(response.GetDataBuffer(1024), nullptr);
}
