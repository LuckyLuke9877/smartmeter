#pragma once

#include <stdint.h>
#include <vector>

namespace esphome
{
namespace espdm
{

// Handles only LongFrames
class MbusProtocol
{
public:
    void AddFrameData(uint8_t data);
    bool GetPayload(std::vector<uint8_t>& payload);

private:
    std::vector<uint8_t> m_dataBuffer;

    int32_t ParseFrame(std::vector<uint8_t>& payload);
    uint8_t CalculateChecksum(std::vector<uint8_t>::iterator begin, std::vector<uint8_t>::iterator end) const;
    void Log(std::vector<uint8_t> data) const;
};

} // namespace espdm
} // namespace esphome
