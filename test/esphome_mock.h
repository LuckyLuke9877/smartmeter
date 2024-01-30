#include <algorithm>
#include <deque>
#include <functional>
#include <memory>
#include <vector>

#define ESP_LOGV(tag, ...)
#define ESP_LOGD(tag, ...)
#define ESP_LOGW(tag, ...)
#define TAG

namespace esphome
{
namespace uart
{
class UARTDevice
{
public:
    std::deque<uint8_t> m_rx;
    std::deque<uint8_t> m_tx;

    void add_rx(const std::vector<uint8_t> data)
    {
        for (const uint8_t& d : data)
        {
            m_rx.push_back(d);
        }
    }
    bool available()
    {
        return m_rx.size() > 0;
    }
    bool read_byte(uint8_t* byte)
    {
        if (!available())
            return false;
        *byte = m_rx.front();
        m_rx.pop_front();
        return true;
    }
    void write_byte(uint8_t data)
    {
        m_tx.push_back(data);
    }
    void write_array(const std::vector<uint8_t>& data)
    {
        for (const uint8_t& d : data)
        {
            m_tx.push_back(d);
        }
    }
    void flush() { }
};

} // namespace uart

uint16_t crc16(const uint8_t* data, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
        {
            if ((crc & 0x01) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

template <typename T>
T convert_big_endian(T n)
{
    T m;
    for (size_t i = 0; i < sizeof(T); i++)
    {
        reinterpret_cast<uint8_t*>(&m)[i] = reinterpret_cast<uint8_t*>(&n)[sizeof(T) - 1 - i];
    }
    return m;
}

} // namespace esphome
