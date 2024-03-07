#pragma once

#include "esphome.h"
#if defined(ESP32)
    #include "mbedtls/gcm.h"
#endif
#include "espdm_mbus.h"

#include <math.h>

static const char* ESPDM_VERSION = "0.9.0";
static const char* TAG = "espdm";

namespace esphome
{
namespace espdm
{
constexpr float SQRT3 = 1.732050808f;

class DlmsMeter : public Component, public uart::UARTDevice
{
public:
    struct MeterData
    {
        void GetVoltage(float& value1, float& value2, float& value3) const
        {
            value1 = voltageL1;
            value2 = voltageL2;
            value3 = voltageL3;
        }
        float GetAverageVoltage() const
        {
            int count(0);
            count += voltageL1 != 0.0f ? 1 : 0;
            count += voltageL2 != 0.0f ? 1 : 0;
            count += voltageL3 != 0.0f ? 1 : 0;

            return count == 0 ? 0.0f : (voltageL1 + voltageL2 + voltageL3) / static_cast<float>(count);
        }
        void GetCurrent(float& total, float& value1, float& value2, float& value3) const
        {
            value1 = currentL1;
            value2 = currentL2;
            value3 = currentL3;
            total = value1 + value2 + value3;
        }
        void GetApparentPower(float& total, float& value1, float& value2, float& value3) const
        {
            // Scheinleistung
            value1 = voltageL1 * currentL1;
            value2 = voltageL2 * currentL2;
            value3 = voltageL3 * currentL3;
            total = value1 + value2 + value3;
        }
        void GetPower(float& total, float& value1, float& value2, float& value3) const
        {
            // Wirkleistung
            const auto powerFactor = GetPowerFactor();
            value1 = voltageL1 * currentL1 * powerFactor;
            value2 = voltageL2 * currentL2 * powerFactor;
            value3 = voltageL3 * currentL3 * powerFactor;
            total = value1 + value2 + value3;
        }
        void GetReactivePower(float& total, float& value1, float& value2, float& value3) const
        {
            // Blindleistung
            const auto reactivePowerFactor = 1.0f - GetPowerFactor();
            value1 = voltageL1 * currentL1 * reactivePowerFactor;
            value2 = voltageL2 * currentL2 * reactivePowerFactor;
            value3 = voltageL3 * currentL3 * reactivePowerFactor;
            total = value1 + value2 + value3;
        }
        float GetPowerFactor() const
        {
            float total(0.0f), value1(0.0f), value2(0.0f), value3(0.0f);
            GetApparentPower(total, value1, value2, value3);
            // cos-phi = Wirkleistung / Scheinleistung
            return total != 0.0f ? std::fabs((activePowerPlus - activePowerMinus) / total) : 1.0f;
        }
        static float GetPhaseToPhaseVoltage(float voltage)
        {
            return voltage * SQRT3;
        }

        float voltageL1{0.0f};
        float voltageL2{0.0f};
        float voltageL3{0.0f};
        float currentL1{0.0f};
        float currentL2{0.0f};
        float currentL3{0.0f};
        float activePowerPlus{0.0f}; // Wirkleistung
        float activePowerMinus{0.0f};
        float activeEnergyPlus{0.0f};
        float activeEnergyMinus{0.0f};
        float reactiveEnergyPlus{0.0f};
        float reactiveEnergyMinus{0.0f};
    };

    using OnReceiveMeterData = std::function<void(const MeterData& data)>;

    DlmsMeter(uart::UARTComponent* parent);

    void setup() override;
    void loop() override;

    void set_voltage_sensors(sensor::Sensor* voltage_l1, sensor::Sensor* voltage_l2, sensor::Sensor* voltage_l3);
    void set_current_sensors(sensor::Sensor* current_l1, sensor::Sensor* current_l2, sensor::Sensor* current_l3);

    void set_active_power_sensors(sensor::Sensor* active_power_plus, sensor::Sensor* active_power_minus);
    void set_active_energy_sensors(sensor::Sensor* active_energy_plus, sensor::Sensor* active_energy_minus);
    void set_reactive_energy_sensors(sensor::Sensor* reactive_energy_plus, sensor::Sensor* reactive_energy_minus);
#if defined(USE_MQTT)
    void set_timestamp_sensor(text_sensor::TextSensor* timestamp);

    void enable_mqtt(mqtt::MQTTClientComponent* mqtt_client, const char* topic);
#endif
    void set_key(uint8_t key[], size_t keyLength);

    void RegisterForMeterData(OnReceiveMeterData onReceive);

private:
    MbusProtocol m_mbus;
    std::vector<uint8_t> m_dlmsData;

    uint8_t key[16]; // Stores the decryption key
    size_t keyLength; // Stores the decryption key length (usually 16 bytes)

    const char* topic; // Stores the MQTT topic

#if defined(ESP32)
    mbedtls_gcm_context aes; // AES context used for decryption
#endif

    sensor::Sensor* voltage_l1 = NULL; // Voltage L1
    sensor::Sensor* voltage_l2 = NULL; // Voltage L2
    sensor::Sensor* voltage_l3 = NULL; // Voltage L3

    sensor::Sensor* current_l1 = NULL; // Current L1
    sensor::Sensor* current_l2 = NULL; // Current L2
    sensor::Sensor* current_l3 = NULL; // Current L3

    sensor::Sensor* active_power_plus = NULL; // Active power taken from grid
    sensor::Sensor* active_power_minus = NULL; // Active power put into grid

    sensor::Sensor* active_energy_plus = NULL; // Active energy taken from grid
    sensor::Sensor* active_energy_minus = NULL; // Active energy put into grid

    sensor::Sensor* reactive_energy_plus = NULL; // Reactive energy taken from grid
    sensor::Sensor* reactive_energy_minus = NULL; // Reactive energy put into grid

#if defined(USE_MQTT)
    text_sensor::TextSensor* timestamp = NULL; // Text sensor for the timestamp value

    mqtt::MQTTClientComponent* mqtt_client = NULL;
#endif
    OnReceiveMeterData m_onReceiveMeterData{nullptr};

    uint16_t swap_uint16(uint16_t val);
    uint32_t swap_uint32(uint32_t val);
    void log_packet(std::vector<uint8_t> data);
    void AbortDlmsParsing();
};
} // namespace espdm
} // namespace esphome
