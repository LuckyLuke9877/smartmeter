#pragma once

#include "esphome.h"
#include "modbus_server.h"
#include "sunspec_meter_model.h"
#include "./esphome-dlms-meter/espdm.h"

namespace esphome
{
namespace sm
{
using namespace modbus;
using namespace sunspec;

constexpr uint8_t SMART_METER_ADDRESS = 240;
constexpr uint32_t BLINK_OFF_COUNT = 5; // 5 * 16ms
constexpr float UNKNOWN_VALUE = 0.0f;

class SmartMeter : public Component, public sensor::Sensor
{
public:
    SmartMeter(uart::UARTComponent* uartModbus, uart::UARTComponent* uartMbus)
        : m_modbusServer(SMART_METER_ADDRESS,
                         [this](uint8_t functionCode, const ModbusServer::RequestRead& request) {
                             return OnModbusReceiveRequest(functionCode, request);
                         })
        , m_dlmsMeter(uartMbus)
        , m_meterModel(SMART_METER_ADDRESS)
    {
        m_dlmsMeter.RegisterForMeterData([this](const espdm::DlmsMeter::MeterData& data) { OnReceiveMeterData(data); });
        m_modbusServer.set_uart_parent(uartModbus);
        // None GUI sensor, just to get access from yaml if needed.
        set_internal(true);
    }

    void setup() override
    {
        ESP_LOGI("sm", "setup() called");
        m_dlmsMeter.setup();
    }

    void loop() override
    {
        // called in ~16ms interval
        m_dlmsMeter.loop();
        m_modbusServer.ProcessRequest();
        SetStatusLed(false);
    }

    std::vector<sensor::Sensor*> GetSensors()
    {
        std::vector<sensor::Sensor*> sensors;
        sensors.push_back(this);

        return sensors;
    }

    void OnReceiveMeterData(const espdm::DlmsMeter::MeterData& data)
    {
        // Set Sunspec meter data
        m_meterModel.SetVoltageToNeutral(data.GetAverageVoltage(), data.voltageL1, data.voltageL2, data.voltageL3);

        m_meterModel.SetAcCurrent(data.currentL1 + data.currentL2 + data.currentL3, data.currentL1, data.currentL2,
                                  data.currentL3);

        m_meterModel.SetVoltagePhaseToPhase(
            data.GetPhaseToPhaseVoltage(data.GetAverageVoltage()), data.GetPhaseToPhaseVoltage(data.voltageL1),
            data.GetPhaseToPhaseVoltage(data.voltageL2), data.GetPhaseToPhaseVoltage(data.voltageL3));

        m_meterModel.SetFrequency(50.0f);

        const float powerPerPhase = data.activePowerPlus / 3.0f;
        m_meterModel.SetPower(data.activePowerPlus, powerPerPhase, powerPerPhase, powerPerPhase);

        m_meterModel.SetPowerFactor(data.GetPowerFactor(), UNKNOWN_VALUE, UNKNOWN_VALUE, UNKNOWN_VALUE);

        const float activeEnergyPerPhase = data.activeEnergyPlus / 3.0f;
        m_meterModel.SetTotalWattHoursImported(data.activeEnergyPlus, activeEnergyPerPhase, activeEnergyPerPhase,
                                               activeEnergyPerPhase);

        const float reactiveEnergyPerPhase = data.reactiveEnergyPlus / 3.0f;
        m_meterModel.SetTotalVaHoursImported(data.reactiveEnergyPlus, reactiveEnergyPerPhase, reactiveEnergyPerPhase,
                                             reactiveEnergyPerPhase);

        ESP_LOGI("sm", "MeterModel data updated");
    }

    ModbusServer::ResponseRead OnModbusReceiveRequest(uint8_t functionCode, const ModbusServer::RequestRead& request)
    {
        ModbusServer::ResponseRead response;
        if (functionCode != 0x03)
        {
            response.SetError(ModbusServer::ResponseRead::ErrorCode::ILLEGAL_FUNCTION);
            ESP_LOGW("sm", "Modbus received wrong functionCode %d", functionCode);
        }
        else
        {
            ESP_LOGI("sm", "Modbus request received: address = %d, count = %d", request.startAddress,
                     request.addressCount);
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
    espdm::DlmsMeter m_dlmsMeter;
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
