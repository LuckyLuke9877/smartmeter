#pragma once

#include <string>

#include "esphome.h"

namespace esphome
{
namespace utils
{

std::string GetTimespanString(int64_t millis)
{
    auto seconds = static_cast<int>(millis / 1000L);
    const int secPerMinute = 60;
    const int secPerHour = secPerMinute * 60;
    const int secPerDay = secPerHour * 24;
    const int days = seconds / secPerDay;
    seconds -= days * secPerDay;
    const int hours = seconds / secPerHour;
    seconds -= hours * secPerHour;
    const int minutes = seconds / secPerMinute;
    seconds -= minutes * secPerMinute;
    char temp[64] = {0};
    sprintf(temp, "%dd %02d:%02d:%02d", days, hours, minutes, seconds);

    return temp;
}

// Stopwatch that handles also uint32_t rollover (~49days),
// but calling GetElapsedMillis() interval must be less than 49days
class Stopwatch
{
public:
    Stopwatch(bool startNow = true)
    {
        if (startNow)
        {
            Start();
        }
    }
    void Start()
    {
        m_start = millis();
        m_lastGet = m_start;
        m_rolloverCount = 0;
    }
    uint64_t GetElapsedMillis()
    {
        const auto now = millis();
        if (now < m_lastGet)
        {
            m_rolloverCount++;
        }
        m_lastGet = now;
        const auto effectiveRollover = now >= m_start ? m_rolloverCount : m_rolloverCount - 1;
        return (static_cast<uint64_t>(effectiveRollover) << 32) + static_cast<uint64_t>(now - m_start);
    }

private:
    uint32_t m_start{0};
    uint32_t m_rolloverCount{0};
    uint32_t m_lastGet{0};
};

} // namespace utils
} // namespace esphome
