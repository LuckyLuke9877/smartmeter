#include "esphome_mock.h"

namespace esphome
{
namespace uart
{

void UARTDevice::AddRx(const std::vector<uint8_t> data)
{
    for (const uint8_t& d : data)
    {
        m_uartRx.push_back(d);
    }
}
bool UARTDevice::available()
{
    return m_uartRx.size() > 0;
}
bool UARTDevice::read_byte(uint8_t* byte)
{
    if (!available())
        return false;
    *byte = m_uartRx.front();
    m_uartRx.pop_front();
    return true;
}
void UARTDevice::write_byte(uint8_t data)
{
    m_uartTx.push_back(data);
}
void UARTDevice::write_array(const std::vector<uint8_t>& data)
{
    for (const uint8_t& d : data)
    {
        m_uartTx.push_back(d);
    }
}
void UARTDevice::flush() { }

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

} // namespace esphome
