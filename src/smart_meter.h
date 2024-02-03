#pragma once

#include "esphome.h"
#include "modbus_server.h"
#include "sunspec_meter_model.h"

namespace esphome
{
namespace sm
{
using namespace modbus;
using namespace sunspec;

constexpr uint8_t SMART_METER_ADDRESS = 240;
constexpr uint32_t BLINK_OFF_COUNT = 5; // 5 * 16ms

class SmartMeter : public PollingComponent, public sensor::Sensor
{
public:
    SmartMeter(uart::UARTComponent* uartModbus)
        : PollingComponent(1000)
        , m_modbusServer(SMART_METER_ADDRESS,
                         [this](uint8_t functionCode, const ModbusServer::RequestRead& request) {
                             return OnModbusReceiveRequest(functionCode, request);
                         })
        , m_meterModel(SMART_METER_ADDRESS)
    {
        m_modbusServer.set_uart_parent(uartModbus);
        // None GUI sensor, just needed to get access from yaml.
        set_internal(true);
    }

    void setup() override
    {
        ESP_LOGI("sm", "setup() called");
    }

    void loop() override
    {
        // called in ~16ms interval
        m_modbusServer.ProcessRequest();
        SetStatusLed(false);
    }

    void update() override
    {
        // led test hack
        // SetStatusLed(true, ((millis() / 1000) % 2) == 0);
    }

    // yaml called functions
    void InitOnBoot() { }

    std::vector<sensor::Sensor*> GetSensors()
    {
        std::vector<sensor::Sensor*> sensors;
        sensors.push_back(this);
        // sensors.push_back(&m_sensor);

        return sensors;
    }

    ModbusServer::ResponseRead OnModbusReceiveRequest(uint8_t functionCode, const ModbusServer::RequestRead& request)
    {
        ModbusServer::ResponseRead response;
        if (functionCode != 0x03)
        {
            response.SetError(ModbusServer::ResponseRead::ErrorCode::ILLEGAL_FUNCTION);
            ESP_LOGW("sm", "received wrong functionCode %d", functionCode);
        }
        else
        {
            ESP_LOGI("sm", "received request: address = %d, count = %d", request.startAddress, request.addressCount);
            if (m_meterModel.IsValidAddressRange(request.startAddress, request.addressCount) == false)
            {
                response.SetError(ModbusServer::ResponseRead::ErrorCode::ILLEGAL_ADDRESS);
            }
            else
            {
                response.SetData(m_meterModel.GetRegisterRaw(request.startAddress, request.addressCount));
            }
        }
        SetStatusLed(true, response.IsError());

        return response;
    }

private:
    // sensor::Sensor m_sensor;
    ModbusServer m_modbusServer;
    MeterModel m_meterModel;
    uint32_t m_statusLedBlinkCount{0};

    void SetStatusLed(bool on, bool error = false)
    {
        if (!on)
        {
            if (m_statusLedBlinkCount > 0 && ++m_statusLedBlinkCount > BLINK_OFF_COUNT)
            {
                id(status_led).turn_off().perform();
                m_statusLedBlinkCount = 0;
            }
            return;
        }

        auto call = id(status_led).turn_on();
        call.set_brightness(0.5); // 1.0 is full brightness
        if (error)
        {
            call.set_red(1.0);
            call.set_green(0.0);
        }
        else
        {
            call.set_red(0.0);
            call.set_green(1.0);
        }
        call.perform();
        m_statusLedBlinkCount = 1;
    }
};

} // namespace sm
} // namespace esphome
