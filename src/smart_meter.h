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

class SmartMeter : public PollingComponent, public sensor::Sensor
{
public:
    SmartMeter(uart::UARTComponent* uartModbus)
        : PollingComponent(1000)
        , m_modbusServer(
              SMART_METER_ADDRESS,
              [this](uint8_t function_code, const ModbusServer::RequestRead& request) {
                  return on_modbus_receive_request(function_code, request);
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
        m_modbusServer.process_requests();
    }

    void update() override
    {
        m_value = millis() / 1000.0f;
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

    ModbusServer::ResponseRead
    on_modbus_receive_request(uint8_t function_code, const ModbusServer::RequestRead& request)
    {
        ModbusServer::ResponseRead response;
        if (function_code != 0x03)
        {
            response.set_error(ModbusServer::ResponseRead::ErrorCode::ILLEGAL_FUNCTION);
            ESP_LOGW("sm", "received wrong function_code %d", function_code);
        }
        else
        {
            ESP_LOGI("sm", "received request: address = %d, count = %d", request.start_address, request.address_count);
            if (m_meterModel.IsValidAddressRange(request.start_address, request.address_count) == false)
            {
                response.set_error(ModbusServer::ResponseRead::ErrorCode::ILLEGAL_ADDRESS);
            }
            auto dataBuffer = response.GetDataBuffer(request.address_count * sizeof(uint16_t));
            if (dataBuffer == nullptr)
            {
                response.set_error(ModbusServer::ResponseRead::ErrorCode::DEVICE_FAILURE);
            }
            m_meterModel.GetRegisterRaw(request.start_address, request.address_count, dataBuffer);
        }

        return response;
    }

private:
    // sensor::Sensor m_sensor;
    ModbusServer m_modbusServer;
    MeterModel   m_meterModel;
    float        m_value{0.0f};
};

} // namespace sm
} // namespace esphome
