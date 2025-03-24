#pragma once

#include <algorithm>
#include <deque>
#include <functional>
#include <memory>
#include <vector>

#define ESP_LOGV(tag, ...)
#define ESP_LOGD(tag, ...)
#define ESP_LOGI(tag, ...)
#define ESP_LOGW(tag, ...)
#define ESP_LOGE(tag, ...)
#define TAG

namespace esphome
{
namespace uart
{
class UARTDevice
{
public:
    std::deque<uint8_t> m_uartRx;
    std::deque<uint8_t> m_uartTx;

    void AddRx(const std::vector<uint8_t> data);
    bool available();
    bool read_byte(uint8_t* byte);
    void write_byte(uint8_t data);
    void write_array(const std::vector<uint8_t>& data);
    void flush();
};

} // namespace uart

uint16_t crc16(const uint8_t* data, uint8_t len);

} // namespace esphome
