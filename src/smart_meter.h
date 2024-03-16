#pragma once

#include "esphome.h"
#include "modbus_server.h"
#include "sunspec_meter_model.h"
#include "./esphome-dlms-meter/espdm.h"

#define SMART_METER_VERSION "1.0.0"

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
        ESP_LOGI("sm", "Smart-Meter starting, version = %s", SMART_METER_VERSION);
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

private:
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
        const float preventCastError = 0.5f;
        time::ESPTime begin;
        std::memset(&begin, 0, sizeof(begin));
        begin.day_of_month = static_cast<uint32_t>(id(energy_day_begin).state + preventCastError);
        begin.month = static_cast<uint32_t>(id(energy_month_begin).state + preventCastError);
        begin.year = static_cast<uint32_t>(id(energy_year_begin).state + preventCastError);
        auto now = id(sntp_time).now();
        if (begin.year != 1970U && now.is_valid())
        {
            // make fields_in_range() happy, otherwise recalc_timestamp_utc() fails
            const uint8_t doesNotMatter = 1;
            begin.day_of_week = doesNotMatter;
            begin.day_of_year = doesNotMatter;
            begin.recalc_timestamp_utc(false);
            now.recalc_timestamp_utc(false);
            long int durationSec = now.timestamp - begin.timestamp;
            const int secPerHour = 3600;
            const int secPerDay = secPerHour * 24;
            const int days = durationSec / secPerDay;
            const float hours = static_cast<float>(durationSec % secPerDay) / static_cast<float>(secPerHour);
            char temp[64] = {0};
            sprintf(temp, "%dd %.2fh", days, hours);
            id(energy_interval_duration).publish_state(temp);

            // Plus
            const auto plus = id(active_energy_plus).state - id(energy_plus_begin).state;
            sprintf(temp, "%.3fkWh", plus);
            id(energy_interval_plus).publish_state(temp);

            // Minus
            const auto minus = id(active_energy_minus).state - id(energy_minus_begin).state;
            sprintf(temp, "%.3fkWh", minus);
            id(energy_interval_minus).publish_state(temp);

            // Sum
            sprintf(temp, "%.3fkWh", plus - minus);
            id(energy_interval_sum).publish_state(temp);
        }
        else
        {
            const char invalid[] = {"--"};
            id(energy_interval_duration).publish_state(invalid);
            id(energy_interval_plus).publish_state(invalid);
            id(energy_interval_minus).publish_state(invalid);
            id(energy_interval_sum).publish_state(invalid);
        }
    }
};

} // namespace sm
} // namespace esphome
