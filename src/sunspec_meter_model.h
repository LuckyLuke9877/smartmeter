#pragma once

#include <cstring>
#include <stdint.h>
#include <vector>

#define CHAR2UINT16(c1, c2) uint16_t(uint8_t(c1) << 8) | uint8_t(c2)

namespace sunspec
{
// Infos from "Fronius Datamanager Register Map: Floating Point Meter Model (211, 212, 213)"
// Note: Here only 213 is supported : 3-phase float model
// The smallest data element ( called register ) is uint16 ( e.g. a float32 requires 2 registers)
// Values are converted from little to big endian

constexpr auto REGISTER_OFFSET = 40000;
constexpr auto REGISTER_COMMON_COUNT = 4 + 65;
constexpr auto REGISTER_METER_COUNT = 2 + 124;
constexpr auto REGISTER_END_COUNT = 2;
constexpr auto REGISTER_TOTAL_COUNT = REGISTER_COMMON_COUNT + REGISTER_METER_COUNT + REGISTER_END_COUNT;

template <typename T>
T Convert2BigEndian(T n)
{
    T m;
    for (size_t i = 0; i < sizeof(T); i++)
    {
        reinterpret_cast<uint8_t*>(&m)[i] = reinterpret_cast<uint8_t*>(&n)[sizeof(T) - 1 - i];
    }
    return m;
}

class MeterModel
{
public:
    MeterModel(uint8_t modbusAddress)
    {
        // Init static data
        std::memset(m_registers, 0, sizeof(m_registers));
        // Common block
        SetRegisterUint32(0, 0x53756e53); // "SunS"
        SetRegisterUint16(2, 0x0001);
        SetRegisterUint16(3, REGISTER_COMMON_COUNT - 4); // Number of registers in this block following this entry

        SetRegisterUint16(4, CHAR2UINT16(':', ')'));

        SetRegisterUint16(20, CHAR2UINT16('K', 'a'));
        SetRegisterUint16(21, CHAR2UINT16('i', '2'));
        SetRegisterUint16(22, CHAR2UINT16('S', 'u'));
        SetRegisterUint16(23, CHAR2UINT16('n', 'M'));
        SetRegisterUint16(24, CHAR2UINT16('o', 'd'));

        SetRegisterUint16(44, CHAR2UINT16('V', '0'));
        SetRegisterUint16(45, CHAR2UINT16('.', '1'));
        SetRegisterUint16(46, CHAR2UINT16('.', '0'));

        SetRegisterUint16(68, modbusAddress);

        // Meter block
        SetRegisterUint16(69, 213); // float, 3-phase meter.
        SetRegisterUint16(70, REGISTER_METER_COUNT - 2); // Number of registers in this block following this entry

        // End block
        SetRegisterUint16(195, 0xFFFF);
        SetRegisterUint16(196, 0); // Number of registers in this block following this entry
    }

    void SetAcCurrent(float total, float phaseA, float phaseB, float phaseC)
    {
        SetFloats(71, {total, phaseA, phaseB, phaseC});
    }
    void SetVoltageToNeutral(float average, float phaseA, float phaseB, float phaseC)
    {
        SetFloats(79, {average, phaseA, phaseB, phaseC});
    }
    void SetVoltagePhaseToPhase(float average, float phaseAB, float phaseBC, float phaseCA)
    {
        SetFloats(87, {average, phaseAB, phaseBC, phaseCA});
    }
    void SetFrequency(float value)
    {
        SetFloats(95, {value});
    }
    void SetPower(float total, float phaseA, float phaseB, float phaseC)
    {
        SetFloats(97, {total, phaseA, phaseB, phaseC});
    }
    void SetApparentPower(float total, float phaseA, float phaseB, float phaseC)
    {
        SetFloats(105, {total, phaseA, phaseB, phaseC});
    }
    void SetReactivePower(float total, float phaseA, float phaseB, float phaseC)
    {
        SetFloats(113, {total, phaseA, phaseB, phaseC});
    }
    void SetPowerFactor(float total, float phaseA, float phaseB, float phaseC) // cos-phi
    {
        SetFloats(121, {total, phaseA, phaseB, phaseC});
    }
    void SetTotalWattHoursExported(float total, float phaseA, float phaseB, float phaseC)
    {
        SetFloats(129, {total, phaseA, phaseB, phaseC});
    }
    void SetTotalWattHoursImported(float total, float phaseA, float phaseB, float phaseC)
    {
        SetFloats(137, {total, phaseA, phaseB, phaseC});
    }
    void SetTotalVaHoursExported(float total, float phaseA, float phaseB, float phaseC)
    {
        SetFloats(145, {total, phaseA, phaseB, phaseC});
    }
    void SetTotalVaHoursImported(float total, float phaseA, float phaseB, float phaseC)
    {
        SetFloats(153, {total, phaseA, phaseB, phaseC});
    }
    // Rest is not needed

    std::vector<uint16_t> GetRegister(uint32_t registerAddress, uint8_t registerCount)
    {
        const int32_t registerIndex = GetRegisterIndexForRange(registerAddress, registerCount);
        if (registerIndex < 0)
        {
            return {}; // invalid index
        }
        std::vector<uint16_t> reg(registerCount);
        std::memcpy(&reg[0], &m_registers[registerIndex], reg.size() * sizeof(m_registers[0]));

        return reg;
    }

    std::vector<uint8_t> GetRegisterRaw(uint32_t registerAddress, uint8_t registerCount)
    {
        const int32_t registerIndex = GetRegisterIndexForRange(registerAddress, registerCount);
        if (registerIndex < 0)
        {
            return {}; // invalid index
        }
        std::vector<uint8_t> raw(registerCount * sizeof(m_registers[0]));
        std::memcpy(&raw[0], &m_registers[registerIndex], raw.size());

        return raw;
    }

    bool IsValidAddressRange(uint32_t registerAddress, uint8_t registerCount)
    {
        return GetRegisterIndexForRange(registerAddress, registerCount) >= 0;
    }

private:
    int32_t GetRegisterIndexForRange(uint32_t registerAddress, uint8_t registerCount)
    {
        // registerAddress is already REGISTER_OFFSET-based! (e.g. sunspec-address: 40001 is
        // registerAddress: 40000)
        const int32_t registerIndex = registerAddress - REGISTER_OFFSET;
        if (registerCount < 1 || registerIndex < 0 || (registerIndex + registerCount - 1) >= REGISTER_TOTAL_COUNT)
        {
            return -1; // invalid index
        }

        return registerIndex;
    }

    void SetFloats(uint32_t registerIndex, const std::vector<float>& values)
    {
        for (size_t i = 0; i < values.size(); i++)
        {
            SetRegisterFloat(registerIndex + (i * 2), values[i]);
        }
    }
    void SetRegisterUint16(uint32_t registerIndex, uint16_t value)
    {
        SetRegister(registerIndex, value);
    }
    void SetRegisterUint32(uint32_t registerIndex, uint32_t value)
    {
        SetRegister(registerIndex, value);
    }
    void SetRegisterFloat(uint32_t registerIndex, float value)
    {
        SetRegister(registerIndex, value);
    }
    template <typename T>
    void SetRegister(uint32_t registerIndex, T value)
    {
        T temp = Convert2BigEndian(value);
        std::memcpy(m_registers + registerIndex, &temp, sizeof(temp));
    }

    uint16_t m_registers[REGISTER_TOTAL_COUNT];
};

} // namespace sunspec
