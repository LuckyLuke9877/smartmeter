#include "espdm.h"
#include "espdm_dlms.h"
#include "espdm_obis.h"
#if defined(ESP8266)
    #include <bearssl/bearssl.h>
#endif

namespace
{
const char ESPDM_VERSION[] = {"0.9.1"};
const char TAG[] = {"espdm"};
} // namespace

namespace esphome
{

void PublishSensorState(sensor::Sensor& sensor, float value, float impossibleLimit = 0.0f)
{
    if (impossibleLimit != 0.0f && value > impossibleLimit)
    {
        ESP_LOGE(TAG, "%s value[%f] is greater than limit[%f]. Set it to 0.0.", sensor.get_name().c_str(), value,
                 impossibleLimit);
        value = 0.0f;
    }
    sensor.publish_state(value);
}

namespace espdm
{
constexpr auto IMPOSSIBLE_VOLTAGE_LIMIT = 300.0f;
constexpr auto IMPOSSIBLE_CURRENT_LIMIT = 32.0f; // No more than 32Ampere for normal house
constexpr auto IMPOSSIBLE_POWER_LIMIT = IMPOSSIBLE_CURRENT_LIMIT * 230.0f * 3.0f;

DlmsMeter::DlmsMeter(uart::UARTComponent* parent)
    : uart::UARTDevice(parent)
{ }

void DlmsMeter::setup()
{
    ESP_LOGI(TAG, "DLMS smart meter component v%s started", ESPDM_VERSION);
}

void DlmsMeter::loop()
{
    while (available()) // Read while data is available
    {
        uint8_t c(0);
        this->read_byte(&c);
        m_mbus.AddFrameData(c);
    }

    std::vector<uint8_t> mbusPayload; // Contains the data of the payload
    while (m_mbus.GetPayload(mbusPayload))
    {
        ESP_LOGD(TAG, "mbusPayload.size() = %d bytes", mbusPayload.size());
        log_packet(m_dlmsData);

        // trim mbusPayload to work with original code where first 5 bytes were skipped
        const auto originalCodeRemovedByte = 5;
        m_dlmsData.insert(m_dlmsData.end(), mbusPayload.begin() + originalCodeRemovedByte, mbusPayload.end());

        // Verify and parse DLMS header
        // Always abort parsing if the data do not match the protocol

        ESP_LOGV(TAG, "Parsing DLMS header");

        if (m_dlmsData.size() < 20) // If the payload is too short we need to abort
        {
            ESP_LOGE(TAG, "DLMS: Payload too short");
            return AbortDlmsParsing();
        }

        if (m_dlmsData[DLMS_CIPHER_OFFSET] != 0xDB) // Only general-glo-ciphering is supported (0xDB)
        {
            ESP_LOGE(TAG, "DLMS: Unsupported cipher");
            return AbortDlmsParsing();
        }

        uint8_t systitleLength = m_dlmsData[DLMS_SYST_OFFSET];

        if (systitleLength != 0x08) // Only system titles with length of 8 are supported
        {
            ESP_LOGE(TAG, "DLMS: Unsupported system title length");
            return AbortDlmsParsing();
        }

        uint16_t messageLength = m_dlmsData[DLMS_LENGTH_OFFSET];
        int headerOffset = 0;

        if (messageLength == 0x82)
        {
            ESP_LOGV(TAG, "DLMS: Message length > 127");

            memcpy(&messageLength, &m_dlmsData[DLMS_LENGTH_OFFSET + 1], 2);
            messageLength = swap_uint16(messageLength);

            headerOffset = DLMS_HEADER_EXT_OFFSET; // Header is now 2 bytes longer due to length > 127
        }
        else
        {
            ESP_LOGV(TAG, "DLMS: Message length <= 127");
        }

        messageLength
            -= DLMS_LENGTH_CORRECTION; // Correct message length due to part of header being included in length

        if (m_dlmsData.size() - DLMS_HEADER_LENGTH - headerOffset != messageLength)
        {
            // Note: Kaifa309M sends multiple(2) mbus-frames for one dlms-frame, this is normal flow.
            ESP_LOGD(TAG, "DLMS: Frame[%d] has not enough data yet, current length[%d]", messageLength,
                     m_dlmsData.size() - DLMS_HEADER_LENGTH - headerOffset);
            continue; // Wait for more data to come
        }

        // Now we have enough data for the dlms frame.

        if (m_dlmsData[headerOffset + DLMS_SECBYTE_OFFSET] != 0x21) // Only certain security suite is supported (0x21)
        {
            ESP_LOGE(TAG, "DLMS: Unsupported security control byte");
            return AbortDlmsParsing();
        }

        // Decryption

        ESP_LOGV(TAG, "Decrypting payload");

        uint8_t iv[12]; // Reserve space for the IV, always 12 bytes
        // Copy system title to IV (System title is before length; no header offset needed!)
        // Add 1 to the offset in order to skip the system title length byte
        memcpy(&iv[0], &m_dlmsData[DLMS_SYST_OFFSET + 1], systitleLength);
        memcpy(&iv[8], &m_dlmsData[headerOffset + DLMS_FRAMECOUNTER_OFFSET],
               DLMS_FRAMECOUNTER_LENGTH); // Copy frame counter to IV

        std::vector<uint8_t> plaintext(messageLength);

#if defined(ESP8266)
        memcpy(&plaintext[0], &m_dlmsData[headerOffset + DLMS_PAYLOAD_OFFSET], messageLength);
        br_gcm_context gcmCtx;
        br_aes_ct_ctr_keys bc;
        br_aes_ct_ctr_init(&bc, this->key, this->keyLength);
        br_gcm_init(&gcmCtx, &bc.vtable, br_ghash_ctmul32);
        br_gcm_reset(&gcmCtx, iv, sizeof(iv));
        br_gcm_flip(&gcmCtx);
        br_gcm_run(&gcmCtx, 0, &plaintext[0], messageLength);
#elif defined(ESP32)
        mbedtls_gcm_init(&this->aes);
        mbedtls_gcm_setkey(&this->aes, MBEDTLS_CIPHER_ID_AES, this->key, this->keyLength * 8);

        mbedtls_gcm_auth_decrypt(&this->aes, messageLength, iv, sizeof(iv), NULL, 0, NULL, 0,
                                 &m_dlmsData[headerOffset + DLMS_PAYLOAD_OFFSET], &plaintext[0]);

        mbedtls_gcm_free(&this->aes);
#else
    #error "Invalid Platform"
#endif

        if (plaintext[0] != 0x0F || plaintext[5] != 0x0C)
        {
            ESP_LOGE(TAG, "OBIS: Packet was decrypted but data is invalid");
            return AbortDlmsParsing();
        }

        // Decoding

        ESP_LOGV(TAG, "Decoding payload");

        int currentPosition = DECODER_START_OFFSET;

        do
        {
            if (plaintext[currentPosition + OBIS_TYPE_OFFSET] != DataType::OctetString)
            {
                ESP_LOGE(TAG, "OBIS: Unsupported OBIS header type");
                return AbortDlmsParsing();
            }

            uint8_t obisCodeLength = plaintext[currentPosition + OBIS_LENGTH_OFFSET];

            if (obisCodeLength != 0x06)
            {
                ESP_LOGE(TAG, "OBIS: Unsupported OBIS header length");
                return AbortDlmsParsing();
            }

            uint8_t obisCode[obisCodeLength];
            memcpy(&obisCode[0], &plaintext[currentPosition + OBIS_CODE_OFFSET],
                   obisCodeLength); // Copy OBIS code to array

            currentPosition += obisCodeLength + 2; // Advance past code, position and type

            uint8_t dataType = plaintext[currentPosition];
            currentPosition++; // Advance past data type

            uint8_t dataLength = 0x00;

            CodeType codeType = CodeType::Unknown;

            if (obisCode[OBIS_A] == Medium::Electricity)
            {
                // Compare C and D against code
                if (memcmp(&obisCode[OBIS_C], ESPDM_VOLTAGE_L1, 2) == 0)
                {
                    codeType = CodeType::VoltageL1;
                }
                else if (memcmp(&obisCode[OBIS_C], ESPDM_VOLTAGE_L2, 2) == 0)
                {
                    codeType = CodeType::VoltageL2;
                }
                else if (memcmp(&obisCode[OBIS_C], ESPDM_VOLTAGE_L3, 2) == 0)
                {
                    codeType = CodeType::VoltageL3;
                }

                else if (memcmp(&obisCode[OBIS_C], ESPDM_CURRENT_L1, 2) == 0)
                {
                    codeType = CodeType::CurrentL1;
                }
                else if (memcmp(&obisCode[OBIS_C], ESPDM_CURRENT_L2, 2) == 0)
                {
                    codeType = CodeType::CurrentL2;
                }
                else if (memcmp(&obisCode[OBIS_C], ESPDM_CURRENT_L3, 2) == 0)
                {
                    codeType = CodeType::CurrentL3;
                }

                else if (memcmp(&obisCode[OBIS_C], ESPDM_ACTIVE_POWER_PLUS, 2) == 0)
                {
                    codeType = CodeType::ActivePowerPlus;
                }
                else if (memcmp(&obisCode[OBIS_C], ESPDM_ACTIVE_POWER_MINUS, 2) == 0)
                {
                    codeType = CodeType::ActivePowerMinus;
                }

                else if (memcmp(&obisCode[OBIS_C], ESPDM_ACTIVE_ENERGY_PLUS, 2) == 0)
                {
                    codeType = CodeType::ActiveEnergyPlus;
                }
                else if (memcmp(&obisCode[OBIS_C], ESPDM_ACTIVE_ENERGY_MINUS, 2) == 0)
                {
                    codeType = CodeType::ActiveEnergyMinus;
                }

                else if (memcmp(&obisCode[OBIS_C], ESPDM_REACTIVE_ENERGY_PLUS, 2) == 0)
                {
                    codeType = CodeType::ReactiveEnergyPlus;
                }
                else if (memcmp(&obisCode[OBIS_C], ESPDM_REACTIVE_ENERGY_MINUS, 2) == 0)
                {
                    codeType = CodeType::ReactiveEnergyMinus;
                }
                else
                {
                    ESP_LOGW(TAG, "OBIS: Unsupported OBIS code");
                }
            }
            else if (obisCode[OBIS_A] == Medium::Abstract)
            {
                if (memcmp(&obisCode[OBIS_C], ESPDM_TIMESTAMP, 2) == 0)
                {
                    codeType = CodeType::Timestamp;
                }
                else if (memcmp(&obisCode[OBIS_C], ESPDM_SERIAL_NUMBER, 2) == 0)
                {
                    codeType = CodeType::SerialNumber;
                }
                else if (memcmp(&obisCode[OBIS_C], ESPDM_DEVICE_NAME, 2) == 0)
                {
                    codeType = CodeType::DeviceName;
                }
                else
                {
                    ESP_LOGW(TAG, "OBIS: Unsupported OBIS code");
                }
            }
            else
            {
                ESP_LOGE(TAG, "OBIS: Unsupported OBIS medium");
                return AbortDlmsParsing();
            }

            uint8_t uint8Value;
            uint16_t uint16Value;
            uint32_t uint32Value;
            float floatValue;

            switch (dataType)
            {
            case DataType::DoubleLongUnsigned:
                dataLength = 4;

                memcpy(&uint32Value, &plaintext[currentPosition], 4); // Copy bytes to integer
                uint32Value = swap_uint32(uint32Value); // Swap bytes

                floatValue = uint32Value; // Ignore decimal digits for now

                if (codeType == CodeType::ActivePowerPlus && this->active_power_plus != NULL
                    && this->active_power_plus->state != floatValue)
                    PublishSensorState(*active_power_plus, floatValue, IMPOSSIBLE_POWER_LIMIT);
                else if (codeType == CodeType::ActivePowerMinus && this->active_power_minus != NULL
                         && this->active_power_minus->state != floatValue)
                    PublishSensorState(*active_power_minus, floatValue, IMPOSSIBLE_POWER_LIMIT);

                else if (codeType == CodeType::ActiveEnergyPlus && this->active_energy_plus != NULL
                         && this->active_energy_plus->state != floatValue)
                    this->active_energy_plus->publish_state(floatValue);
                else if (codeType == CodeType::ActiveEnergyMinus && this->active_energy_minus != NULL
                         && this->active_energy_minus->state != floatValue)
                    this->active_energy_minus->publish_state(floatValue);

                else if (codeType == CodeType::ReactiveEnergyPlus && this->reactive_energy_plus != NULL
                         && this->reactive_energy_plus->state != floatValue)
                    this->reactive_energy_plus->publish_state(floatValue);
                else if (codeType == CodeType::ReactiveEnergyMinus && this->reactive_energy_minus != NULL
                         && this->reactive_energy_minus->state != floatValue)
                    this->reactive_energy_minus->publish_state(floatValue);

                break;
            case DataType::LongUnsigned:
                dataLength = 2;

                memcpy(&uint16Value, &plaintext[currentPosition], 2); // Copy bytes to integer
                uint16Value = swap_uint16(uint16Value); // Swap bytes

                if (plaintext[currentPosition + 5] == Accuracy::SingleDigit)
                    floatValue = uint16Value / 10.0; // Divide by 10 to get decimal places
                else if (plaintext[currentPosition + 5] == Accuracy::DoubleDigit)
                    floatValue = uint16Value / 100.0; // Divide by 100 to get decimal places
                else
                    floatValue = uint16Value; // No decimal places

                if (codeType == CodeType::VoltageL1 && this->voltage_l1 != NULL
                    && this->voltage_l1->state != floatValue)
                    PublishSensorState(*voltage_l1, floatValue, IMPOSSIBLE_VOLTAGE_LIMIT);
                else if (codeType == CodeType::VoltageL2 && this->voltage_l2 != NULL
                         && this->voltage_l2->state != floatValue)
                    PublishSensorState(*voltage_l2, floatValue, IMPOSSIBLE_VOLTAGE_LIMIT);
                else if (codeType == CodeType::VoltageL3 && this->voltage_l3 != NULL
                         && this->voltage_l3->state != floatValue)
                    PublishSensorState(*voltage_l3, floatValue, IMPOSSIBLE_VOLTAGE_LIMIT);

                else if (codeType == CodeType::CurrentL1 && this->current_l1 != NULL
                         && this->current_l1->state != floatValue)
                    PublishSensorState(*current_l1, floatValue, IMPOSSIBLE_CURRENT_LIMIT);
                else if (codeType == CodeType::CurrentL2 && this->current_l2 != NULL
                         && this->current_l2->state != floatValue)
                    PublishSensorState(*current_l2, floatValue, IMPOSSIBLE_CURRENT_LIMIT);
                else if (codeType == CodeType::CurrentL3 && this->current_l3 != NULL
                         && this->current_l3->state != floatValue)
                    PublishSensorState(*current_l3, floatValue, IMPOSSIBLE_CURRENT_LIMIT);

                break;
            case DataType::OctetString:
                dataLength = plaintext[currentPosition];
                currentPosition++; // Advance past string length

                if (codeType == CodeType::Timestamp) // Handle timestamp generation
                {
                    char timestamp[21]; // 0000-00-00T00:00:00Z

                    uint16_t year;
                    uint8_t month;
                    uint8_t day;

                    uint8_t hour;
                    uint8_t minute;
                    uint8_t second;

                    memcpy(&uint16Value, &plaintext[currentPosition], 2);
                    year = swap_uint16(uint16Value);

                    memcpy(&month, &plaintext[currentPosition + 2], 1);
                    memcpy(&day, &plaintext[currentPosition + 3], 1);

                    memcpy(&hour, &plaintext[currentPosition + 5], 1);
                    memcpy(&minute, &plaintext[currentPosition + 6], 1);
                    memcpy(&second, &plaintext[currentPosition + 7], 1);

                    sprintf(timestamp, "%04u-%02u-%02uT%02u:%02u:%02uZ", year, month, day, hour, minute, second);

#if defined(USE_MQTT)
                    this->timestamp->publish_state(timestamp);
#endif
                }

                break;
            default:
                ESP_LOGE(TAG, "OBIS: Unsupported OBIS data type");
                return AbortDlmsParsing();
            }

            currentPosition += dataLength; // Skip data length

            currentPosition += 2; // Skip break after data

            if (plaintext[currentPosition] == 0x0F) // There is still additional data for this type, skip it
                currentPosition
                    += 6; // Skip additional data and additional break; this will jump out of bounds on last frame
        } while (currentPosition <= messageLength); // Loop until arrived at end

        ESP_LOGD(TAG, "Received valid data");
        m_dlmsData.clear();

        // Apply sign to current to show the direction of current flow
        if ((active_power_plus->state - active_power_minus->state) < 0.0f)
        {
            // Providing power to grid ( Einspeisung ) => negative current flow
            current_l1->publish_state(-current_l1->state);
            current_l2->publish_state(-current_l2->state);
            current_l3->publish_state(-current_l3->state);
        }

#if defined(USE_MQTT)
        if (this->mqtt_client != NULL)
        {
            this->mqtt_client->publish_json(this->topic.c_str(), [=](JsonObject root) {
                if (this->voltage_l1 != NULL)
                {
                    root["voltage_l1"] = this->voltage_l1->state;
                    root["voltage_l2"] = this->voltage_l2->state;
                    root["voltage_l3"] = this->voltage_l3->state;
                }

                if (this->current_l1 != NULL)
                {
                    root["current_l1"] = this->current_l1->state;
                    root["current_l2"] = this->current_l2->state;
                    root["current_l3"] = this->current_l3->state;
                }

                if (this->active_power_plus != NULL)
                {
                    root["active_power_plus"] = this->active_power_plus->state;
                    root["active_power_minus"] = this->active_power_minus->state;
                }

                if (this->active_energy_plus != NULL)
                {
                    root["active_energy_plus"] = this->active_energy_plus->state;
                    root["active_energy_minus"] = this->active_energy_minus->state;
                }

                if (this->reactive_energy_plus != NULL)
                {
                    root["reactive_energy_plus"] = this->reactive_energy_plus->state;
                    root["reactive_energy_minus"] = this->reactive_energy_minus->state;
                }

                if (this->timestamp != NULL)
                {
                    root["timestamp"] = this->timestamp->state;
                }
            });
        }
#endif

        // Note: Extension expects all sensors
        if (m_onReceiveMeterData)
        {
            MeterData data;
            data.voltageL1 = voltage_l1->state;
            data.voltageL2 = voltage_l2->state;
            data.voltageL3 = voltage_l3->state;
            data.currentL1 = current_l1->state;
            data.currentL2 = current_l2->state;
            data.currentL3 = current_l3->state;
            data.activePowerPlus = active_power_plus->state;
            data.activePowerMinus = active_power_minus->state;
            data.activeEnergyPlus = active_energy_plus->state;
            data.activeEnergyMinus = active_energy_minus->state;
            data.reactiveEnergyPlus = reactive_energy_plus->state;
            data.reactiveEnergyMinus = reactive_energy_minus->state;
            m_onReceiveMeterData(data);
        }
    }
}

void DlmsMeter::AbortDlmsParsing()
{
    m_dlmsData.clear();
}

uint16_t DlmsMeter::swap_uint16(uint16_t val)
{
    return (val << 8) | (val >> 8);
}

uint32_t DlmsMeter::swap_uint32(uint32_t val)
{
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
    return (val << 16) | (val >> 16);
}

void DlmsMeter::set_key(uint8_t key[], size_t keyLength)
{
    // Important: Ensure no more than 16bytes.
    memcpy(&this->key[0], &key[0], keyLength);
    this->keyLength = keyLength;
}

void DlmsMeter::set_voltage_sensors(sensor::Sensor* voltage_l1, sensor::Sensor* voltage_l2, sensor::Sensor* voltage_l3)
{
    this->voltage_l1 = voltage_l1;
    this->voltage_l2 = voltage_l2;
    this->voltage_l3 = voltage_l3;
}
void DlmsMeter::set_current_sensors(sensor::Sensor* current_l1, sensor::Sensor* current_l2, sensor::Sensor* current_l3)
{
    this->current_l1 = current_l1;
    this->current_l2 = current_l2;
    this->current_l3 = current_l3;
}

void DlmsMeter::set_active_power_sensors(sensor::Sensor* active_power_plus, sensor::Sensor* active_power_minus)
{
    this->active_power_plus = active_power_plus;
    this->active_power_minus = active_power_minus;
}

void DlmsMeter::set_active_energy_sensors(sensor::Sensor* active_energy_plus, sensor::Sensor* active_energy_minus)
{
    this->active_energy_plus = active_energy_plus;
    this->active_energy_minus = active_energy_minus;
}

void DlmsMeter::set_reactive_energy_sensors(sensor::Sensor* reactive_energy_plus, sensor::Sensor* reactive_energy_minus)
{
    this->reactive_energy_plus = reactive_energy_plus;
    this->reactive_energy_minus = reactive_energy_minus;
}

#if defined(USE_MQTT)
void DlmsMeter::set_timestamp_sensor(text_sensor::TextSensor* timestamp)
{
    this->timestamp = timestamp;
}

void DlmsMeter::enable_mqtt(mqtt::MQTTClientComponent* mqtt_client, const char* topic)
{
    this->mqtt_client = mqtt_client;
    this->topic = topic;
}
#endif

void DlmsMeter::log_packet(std::vector<uint8_t> data)
{
    ESP_LOGD(TAG, format_hex_pretty(data).c_str());
}

void DlmsMeter::RegisterForMeterData(DlmsMeter::OnReceiveMeterData onReceive)
{
    m_onReceiveMeterData = onReceive;
}

} // namespace espdm
} // namespace esphome
