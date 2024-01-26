#include <gtest/gtest.h>
#define GTEST
#include "esphome_mock.h"
#include "../src/modbus_server.h"

using namespace esphome;
using namespace esphome::modbus;

class ModbusServerTest : public ::testing::Test
{
protected:
    std::vector<ModbusServer::RequestRead> m_requests;
    ModbusServer::ResponseRead m_response;
    std::unique_ptr<ModbusServer> m_server;

    void SetUp() override
    {
        m_server.reset(new ModbusServer(0x01U,
                                        [this](uint8_t function_code, const ModbusServer::RequestRead &request)
                                        {
                                            return on_modbus_receive_read(function_code, request);
                                        }));
    }
    void TearDown() override {}

    ModbusServer::ResponseRead on_modbus_receive_read(uint8_t function_code, const ModbusServer::RequestRead &request)
    {
        // std::cout << "Request received\n";
        m_requests.push_back(request);
        return m_response;
    }
};

TEST_F(ModbusServerTest, IncompleteRequest_RxBufferOk)
{
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25};

    m_server->add_rx(testData);
    ASSERT_EQ(m_server->m_rx.size(), 7);
    m_server->loop();

    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 7);
    ASSERT_EQ(m_server->rx_buffer_, testData);
    ASSERT_EQ(m_server->m_tx.size(), 0);
    ASSERT_EQ(m_requests.size(), 0);
}

TEST_F(ModbusServerTest, IncompleteAndValidRequest_ResponseOk)
{
    std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25};
    float responseValue{42.3f};
    m_response.add(responseValue);

    m_server->add_rx(testData);
    testData.push_back(0xca);
    m_server->add_rx(testData);
    ASSERT_EQ(m_server->m_rx.size(), 15);
    m_server->loop();

    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 0);
    ASSERT_EQ(m_server->m_tx.size(), 9);
    ASSERT_EQ(m_requests.size(), 1);
}

TEST_F(ModbusServerTest, InvalidCrcAndValidRequest_ResponseOk)
{
    const std::vector<uint8_t> invalidTestData = {0x01, 0x03, 0x15, 0x12, 0x00, 0x01, 0x25, 0xff};
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25, 0xca};
    float responseValue{42.3f};
    m_response.add(responseValue);

    m_server->add_rx(invalidTestData);
    m_server->add_rx(testData);
    m_server->add_rx(invalidTestData);
    m_server->add_rx(testData);
    ASSERT_EQ(m_server->m_rx.size(), 32);
    m_server->loop();

    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 0);
    ASSERT_EQ(m_server->m_tx.size(), 18);
    ASSERT_EQ(m_requests.size(), 2);
    ASSERT_EQ(::memcmp(&m_requests[0], &m_requests[1], sizeof(m_requests[0])), 0);
}

TEST_F(ModbusServerTest, ValidRequest_InvalidFunctionCode_Response_IsError)
{
    std::vector<uint8_t> testData = {0x01, 0x04, 0x00, 0x02, 0x00, 0x01, 0x90, 0x0a};
    m_response.set_error(ModbusServer::ResponseRead::ErrorCode::ILLEGAL_FUNCTION);

    m_server->add_rx(testData);
    ASSERT_EQ(m_server->m_rx.size(), 8);
    m_server->loop();

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
    float responseValue{42.3f};
    m_response.add(responseValue);

    m_server->add_rx(invalidFunctionCodeTestData);
    m_server->add_rx(testData);
    ASSERT_EQ(m_server->m_rx.size(), 16);
    m_server->loop();

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
    m_server->loop();

    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 0);
    ASSERT_EQ(m_server->m_tx.size(), 0);
    ASSERT_EQ(m_requests.size(), 0);
}

TEST_F(ModbusServerTest, ValidRequest_ResponseOk)
{
    const std::vector<uint8_t> testData = {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25, 0xca};
    float responseValue{42.3f};
    m_response.add(responseValue);

    uint8_t pos = 0;
    m_server->m_rx.push_back(testData[pos++]);
    m_server->loop();
    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 1);
    ASSERT_EQ(m_server->rx_buffer_[0], 1);

    m_server->m_rx.push_back(testData[pos++]);
    m_server->loop();
    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 2);

    m_server->m_rx.push_back(testData[pos++]);
    m_server->m_rx.push_back(testData[pos++]);
    m_server->m_rx.push_back(testData[pos++]);
    m_server->m_rx.push_back(testData[pos++]);
    m_server->loop();
    ASSERT_EQ(m_server->m_rx.size(), 0);
    ASSERT_EQ(m_server->rx_buffer_.size(), 6);

    m_server->m_rx.push_back(testData[pos++]);
    m_server->loop();
    ASSERT_EQ(m_server->rx_buffer_.size(), 7);
    ASSERT_EQ(m_server->m_tx.size(), 0);

    m_server->m_rx.push_back(testData[pos++]);
    m_server->loop();
    ASSERT_EQ(m_server->rx_buffer_.size(), 0);
    ASSERT_EQ(m_server->m_tx.size(), 9);

    ASSERT_EQ(m_server->m_tx[0], testData[0]);
    ASSERT_EQ(m_server->m_tx[1], testData[1]);
    ASSERT_EQ(m_server->m_tx[2], 4);
    uint8_t *val = (uint8_t *)(&responseValue);
    ASSERT_EQ(m_server->m_tx[3], val[3]);
    ASSERT_EQ(m_server->m_tx[4], val[2]);
    ASSERT_EQ(m_server->m_tx[5], val[1]);
    ASSERT_EQ(m_server->m_tx[6], val[0]);
    auto expectedCrc = crc16(&m_server->m_tx[0], m_server->m_tx.size() - 2);
    ASSERT_EQ(m_server->m_tx[7], expectedCrc & 0xFF);
    ASSERT_EQ(m_server->m_tx[8], expectedCrc >> 8);
    ASSERT_EQ(m_requests.size(), 1);
}

// int main(int argc, char **argv)
// {
//     ::testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }
