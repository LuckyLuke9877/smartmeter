#pragma once

#include "conversion.h"
#include "modbus_registers.h"

#include <cstring>
#include <stdint.h>
#include <vector>

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

class MeterModel : public modb::ModbusRegisters
{
public:
    MeterModel(uint8_t modbusAddress)
        : modb::ModbusRegisters(REGISTER_OFFSET, REGISTER_TOTAL_COUNT)
    {
        // Init static data
        // Common block
        SetRegisterString(0, "SunS", 2);
        SetRegisterUint16(2, 0x0001);
        SetRegisterUint16(3, REGISTER_COMMON_COUNT - 4); // Number of registers in this block following this entry

        SetRegisterString(4, ":)", 16);

        SetRegisterString(20, "Kai2SunMod", 24);

        SetRegisterString(44, "V0.1.0", 24);

        SetRegisterUint16(68, modbusAddress);

        // Meter block
        SetRegisterUint16(69, 213); // float, 3-phase meter.
        SetRegisterUint16(70, REGISTER_METER_COUNT - 2); // Number of registers in this block following this entry

        // End block
        SetRegisterUint16(195, 0xFFFF);
        SetRegisterUint16(196, 0); // Number of registers in this block following this entry
    }

    virtual ~MeterModel() { }

    virtual modb::ResponseError Write(const uint16_t /*registerAddress*/, const uint16_t /*registerCount*/, const uint8_t* /*source*/)
    {
        // Not supported
        return modb::ResponseError::IllegalFunction;
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
};

} // namespace sunspec
