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

constexpr uint8_t SMART_METER_ADDRESS = 1;
constexpr uint32_t BLINK_OFF_COUNT = 5; // 5 * 16ms => led is ~80ms on when blinking

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
        m_lastEnergyTime.timestamp = 0;
        m_modbusServer.set_uart_parent(uartModbus);
        // None GUI sensor, just to get access from yaml if needed.
        set_internal(true);

        // 0x38, 0x68, 0x68, 0x69, 0x71, 0x7A, 0x32, 0x45, 0x6B, 0x75, 0x53, 0x48, 0x53, 0x4B, 0x51, 0x37
        uint8_t key[]
            = {0x38, 0x68, 0x68, 0x69, 0x71, 0x7A, 0x32, 0x45, 0x6B, 0x75, 0x53, 0x48, 0x53, 0x4B, 0x51, 0x37};
        m_dlmsMeter.set_key(key, 16); // Pass your decryption key and key length here

        m_dlmsMeter.set_voltage_sensors(&id(voltage_l1), &id(voltage_l2),
                                        &id(voltage_l3)); // Set sensors to use for voltage (optional)

        m_dlmsMeter.set_current_sensors(&id(current_l1), &id(current_l2),
                                        &id(current_l3)); // Set sensors to use for current (optional)

        m_dlmsMeter.set_active_power_sensors(&id(active_power_plus),
                                             &id(active_power_minus)); // Set sensors to use for active power (optional)

        m_dlmsMeter.set_active_energy_sensors(
            &id(active_energy_plus),
            &id(active_energy_minus)); // Set sensors to use for active energy (optional)
        m_dlmsMeter.set_reactive_energy_sensors(
            &id(reactive_energy_plus),
            &id(reactive_energy_minus)); // Set sensors to use for reactive energy (optional)

        m_dlmsMeter.RegisterForMeterData([this](const espdm::DlmsMeter::MeterData& data) { OnReceiveMeterData(data); });
    }

    void setup() override
    {
        ESP_LOGD("sm", "setup() called");
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
        // Note: not all phase related values are available, provide some narrowed values
        float total(0.0f), value1(0.0f), value2(0.0f), value3(0.0f);

        data.GetVoltage(value1, value2, value3);
        m_meterModel.SetVoltageToNeutral(data.GetAverageVoltage(), value1, value2, value3);

        data.GetCurrent(total, value1, value2, value3);
        m_meterModel.SetAcCurrent(total, value1, value2, value3);

        m_meterModel.SetVoltagePhaseToPhase(
            data.GetPhaseToPhaseVoltage(data.GetAverageVoltage()), data.GetPhaseToPhaseVoltage(data.voltageL1),
            data.GetPhaseToPhaseVoltage(data.voltageL2), data.GetPhaseToPhaseVoltage(data.voltageL3));

        m_meterModel.SetFrequency(50.0f);

        // No idea why Fronius inverter shows it as negative number
        const auto powerFactor = data.GetPowerFactor();
        m_meterModel.SetPowerFactor(powerFactor, powerFactor, powerFactor, powerFactor);
        id(power_factor).publish_state(powerFactor);

        const float activeEnergyPerPhase = data.activeEnergyPlus / 3.0f;
        m_meterModel.SetTotalWattHoursImported(data.activeEnergyPlus, activeEnergyPerPhase, activeEnergyPerPhase,
                                               activeEnergyPerPhase);

        const float reactiveEnergyPerPhase = data.reactiveEnergyPlus / 3.0f;
        m_meterModel.SetTotalVaHoursImported(data.reactiveEnergyPlus, reactiveEnergyPerPhase, reactiveEnergyPerPhase,
                                             reactiveEnergyPerPhase);

        data.GetPower(total, value1, value2, value3);
        m_meterModel.SetPower(total, value1, value2, value3);

        data.GetApparentPower(total, value1, value2, value3);
        m_meterModel.SetApparentPower(total, value1, value2, value3);
        id(apparent_power).publish_state(total);

        data.GetReactivePower(total, value1, value2, value3);
        m_meterModel.SetReactivePower(total, value1, value2, value3);

        SetEnergyFlow();
        ESP_LOGD("sm", "MeterModel data updated");
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
            ESP_LOGD("sm", "Modbus request received: address = %d, count = %d", request.startAddress,
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

    void ResetEnergyFlow()
    {
        auto now = id(sntp_time).now();
        if (!now.is_valid())
        {
            return;
        }
        m_lastEnergyTime = now;
        m_lastEnergyPlus = id(active_energy_plus).state;
        m_lastEnergyMinus = id(active_energy_minus).state;
        SetEnergyFlow();
        ESP_LOGI("sm", "Reset energy flow at %s", now.strftime("%y-%m-%d %H:%M:%S").c_str());
    }

private:
    ModbusServer m_modbusServer;
    espdm::DlmsMeter m_dlmsMeter;
    MeterModel m_meterModel;
    uint32_t m_statusLedBlinkCount{0};
    float m_lastEnergyPlus{0.0f};
    float m_lastEnergyMinus{0.0f};
    time::ESPTime m_lastEnergyTime;

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
            call.set_blue(0.0);
        }
        else
        {
            call.set_red(0.0);
            call.set_green(1.0);
            call.set_blue(0.0);
        }
        call.perform();
        m_statusLedBlinkCount = 1;
    }

    void SetEnergyFlow()
    {
        auto now = id(sntp_time).now();
        if (!now.is_valid() || m_lastEnergyTime.timestamp == 0)
        {
            id(energy_flow).publish_state("-- : --");
            return;
        }
        auto durationSec = now.timestamp - m_lastEnergyTime.timestamp;
        auto energyFlow
            = ((id(active_energy_plus).state - m_lastEnergyPlus) - (id(active_energy_minus).state - m_lastEnergyMinus))
            * 1000.0f;

        const auto secPerHour = 3600.0f;
        const auto secPerDay = secPerHour * 24.0f;
        const uint32_t days = durationSec / secPerDay;
        const float hours = (durationSec - days * secPerDay) / secPerHour;
        char temp[64] = {0};
        sprintf(temp, "%dd %.2fh : %.0fWh", days, hours, energyFlow);
        id(energy_flow).publish_state(temp);
    }
};

} // namespace sm
} // namespace esphome
