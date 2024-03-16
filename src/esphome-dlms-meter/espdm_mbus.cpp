#include "espdm_mbus.h"
#include "esphome.h" // for logging

#include <algorithm>
#include <numeric>

namespace
{

// Format:
// Pos  Meaning
// 1    Start(0x68)
// 2    Length from C Field to Check sum
// 3    Length from C Field to Check sum
// 4    Start(0x68)
// 5    C Field
// 6    A Field
// 7    CI Field
// 8    Check sum
// 9    Stop(0x16)
// sample data (min length): 68 03 03 68 53 01 BB 0F 16

constexpr auto HEADER_LENGTH = 4;
constexpr auto FIELDS_LENGTH = 3;
constexpr auto FOOTER_LENGTH = 2;
constexpr auto HEADER_FOOTER_LENGTH = HEADER_LENGTH + FOOTER_LENGTH;
constexpr auto MIN_FRAME_LENGTH = HEADER_FOOTER_LENGTH + FIELDS_LENGTH;
constexpr auto START1_OFFSET = 0; // Offset of first start byte
constexpr auto LENGTH1_OFFSET = 1; // Offset of first length byte
constexpr auto LENGTH2_OFFSET = 2; // Offset of (duplicated) second length byte
constexpr auto START2_OFFSET = 3; // Offset of (duplicated) second start byte
constexpr auto START_VALUE = 0x68;
constexpr auto STOP_VALUE = 0x16;

} // namespace

namespace esphome
{
namespace espdm
{

void MbusProtocol::AddFrameData(uint8_t data)
{
    m_dataBuffer.push_back(data);
}

bool MbusProtocol::GetPayload(std::vector<uint8_t>& payload)
{
    payload.clear();
    bool tryToSyncWithFrame = false;
    while (m_dataBuffer.size() > 0 && payload.empty())
    {
        auto removeSize = ParseFrame(payload);
        if (removeSize == 0)
        {
            // not enough data yet
            break;
        }
        if (!payload.empty())
        {
            ESP_LOGD("mbus", "Got valid mbus-frame, size = %d", removeSize);
            ESP_LOGD("mbus", format_hex_pretty(&m_dataBuffer[0], removeSize).c_str());
        }
        // Remove processed data
        m_dataBuffer.erase(m_dataBuffer.begin(), m_dataBuffer.begin() + removeSize);
        if (removeSize == 1 && !tryToSyncWithFrame)
        {
            // Frame has not the expected format, try to sync with it and log only once
            tryToSyncWithFrame = true;
            ESP_LOGE("mbus", "Mbus frame is not in sync, try to sync it...");
        }
    }
    return !payload.empty();
}

int32_t MbusProtocol::ParseFrame(std::vector<uint8_t>& payload)
{
    if (m_dataBuffer.size() < MIN_FRAME_LENGTH)
    {
        return 0; // not enough data yet
    }
    if (m_dataBuffer[START1_OFFSET] != START_VALUE || m_dataBuffer[START2_OFFSET] != START_VALUE)
    {
        return 1; // wrong start
    }
    const auto payloadLength = m_dataBuffer[LENGTH1_OFFSET];
    if (m_dataBuffer[LENGTH2_OFFSET] != payloadLength)
    {
        return 1; // wrong length
    }
    const auto frameLength = HEADER_FOOTER_LENGTH + payloadLength;
    if (m_dataBuffer.size() < frameLength)
    {
        return 0; // not enough data yet
    }
    const auto checkSum = m_dataBuffer[HEADER_LENGTH + payloadLength];
    if (m_dataBuffer[HEADER_LENGTH + payloadLength + 1] != STOP_VALUE)
    {
        return 1; // wrong stop
    }
    if (CalculateChecksum(m_dataBuffer.begin() + HEADER_LENGTH, m_dataBuffer.begin() + (HEADER_LENGTH + payloadLength))
        != checkSum)
    {
        return 1; // wrong check sum
    }

    // Frame is valid, return payload
    payload.assign(m_dataBuffer.begin() + HEADER_LENGTH, m_dataBuffer.begin() + (HEADER_LENGTH + payloadLength));

    // Remove the frame
    return frameLength;
}

uint8_t MbusProtocol::CalculateChecksum(std::vector<uint8_t>::iterator begin, std::vector<uint8_t>::iterator end) const
{
    // Simply the sum of all data
    return static_cast<uint8_t>(std::accumulate(begin, end, 0U) & 0xFF);
}

} // namespace espdm
} // namespace esphome
