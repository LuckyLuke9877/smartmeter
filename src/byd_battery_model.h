#pragma once

#include "conversion.h"
#include "modbus_registers.h"

#include <cstring>
#include <stdint.h>
#include <vector>

namespace byd
{
// Note: got all data from: https://gitlab.com/pelle8/inverter_resources/-/blob/main/byd_registers_modbus_rtu.md?ref_type=heads
// for register index in doc must sub 1 (e.g. 101 => 100)

constexpr auto REG100_OFFSET = 100;
constexpr auto REG100_COUNT = 68;
constexpr auto REG200_OFFSET = 200;
constexpr auto REG200_COUNT = 13;
constexpr auto REG300_OFFSET = 300;
constexpr auto REG300_COUNT = 24;
constexpr auto REG400_OFFSET = 400;
constexpr auto REG400_COUNT = 19;
constexpr auto REG1000_OFFSET = 1000;
constexpr auto REG1000_COUNT = 100;
constexpr auto REG12288_OFFSET = 12288;
constexpr auto REG12288_COUNT = 768;

class BatteryModel : public modb::IModbusRegisters
{
public:
    BatteryModel() { }

    virtual ~BatteryModel() { }

    virtual modb::ResponseError Read(const uint16_t registerAddress, const uint16_t registerCount, uint8_t* target) const
    {
        modb::ModbusRegisters* regBlock = GetRegisterBlock(registerAddress);
        if (!regBlock)
        {
            return modb::ResponseError::IllegalAddress; // invalid index
        }

        return regBlock->Read(registerAddress, registerCount, target);
    }
    virtual modb::ResponseError Write(const uint16_t registerAddress, const uint16_t registerCount, const uint8_t* source)
    {

        modb::ModbusRegisters* regBlock = GetRegisterBlock(registerAddress);
        if (!regBlock)
        {
            return modb::ResponseError::IllegalAddress; // invalid index
        }

        return regBlock->Write(registerAddress, registerCount, source);
    }

private:
    modb::ModbusRegisters* GetRegisterBlock(const uint16_t registerAddress) const
    {
        const uint16_t startAddress = registerAddress - (registerAddress % 100);
        switch (startAddress)
        {
        case REG100_OFFSET:
            return &m_reg100;
        case REG200_OFFSET:
            return &m_reg200;
        case REG300_OFFSET:
            return &m_reg300;
        case REG400_OFFSET:
            return &m_reg400;
        case REG1000_OFFSET:
            return &m_reg1000;

        default:
            break;
        }
        if (registerAddress >= REG12288_OFFSET && registerAddress < REG12288_OFFSET + REG12288_COUNT)
        {
            return &m_reg12288;
        }
        return nullptr;
    }

    class Reg100 : public modb::ModbusRegisters
    {
    public:
        Reg100()
            : modb::ModbusRegisters(REG100_OFFSET, REG100_COUNT)
        {
            // 101 - "SI",1
            SetRegisterString(0, "SI", 1);
            SetRegisterUint16(1, 0x0001);
            // 103 - "BYD",0,0,0,0,0,0,0,0,0,0,0,0,0,0                     type
            SetRegisterString(2, "BYD", 16);
            // 119 - "BYD Battery-Box Premium HV", 0, 0, 0                 descr
            SetRegisterString(18, "BYD Battery-Box Premium HV", 16);
            // 135 - "5.0", 0, 0, 0, 0, 0, 0, "3.16", 0, 0, 0, 0, 0, 0     version
            SetRegisterString(34, "5.0", 8);
            SetRegisterString(42, "3.16", 8);
            // 151 - "2025-9876", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0    serial number
            SetRegisterString(50, "2025-9876", 16);
            // 167 - 1, 0
            SetRegisterUint16(66, 1);
            SetRegisterUint16(67, 0);
        }

        virtual ~Reg100() { }

        virtual modb::ResponseError Write(const uint16_t /*registerAddress*/, const uint16_t /*registerCount*/, const uint8_t* /*source*/)
        {
            // Not supported
            return modb::ResponseError::IllegalFunction;
        }
    };

    class Reg200 : public modb::ModbusRegisters
    {
    public:
        Reg200()
            : modb::ModbusRegisters(REG200_OFFSET, REG200_COUNT)
        {
            // 201 - 0         always 0
            // 202 - 0         always 0
            // 203 - 44236     Capacity (44kWh)
            // 204 - 40960     Max Power (41kW)
            // 205 - 40960     Max Power (41kW), lowest value of 204 and 205 will be enforced by Gen24
            // 206 - 4672      Max Voltage (467.2V), if higher charging is not possible (goes into forced discharge)
            // 207 - 3200      Min Voltage (320.0V), if lower Gen24 disables battery
            // 208 - 53248     always 53248 for this BYD, Peak power (53kW)
            // 209 - 10        always 10
            // 210 - 53248     always 53248 for this BYD, Peak power (53kW)
            // 211 - 10        always 10
            // 212 - 0         always 0
            // 213 - 0         always 0
            SetRegisterUint16(0, 0);
            SetRegisterUint16(1, 0);
            SetRegisterUint16(2, 44236);
            SetRegisterUint16(3, 40959);
            SetRegisterUint16(4, 40960);
            SetRegisterUint16(5, 4672);
            SetRegisterUint16(6, 3200);
            SetRegisterUint16(7, 53247);
            SetRegisterUint16(8, 10);
            SetRegisterUint16(9, 53248);
            SetRegisterUint16(10, 10);
            SetRegisterUint16(11, 0);
            SetRegisterUint16(12, 0);
        }

        virtual ~Reg200() { }

#ifndef ENABLE_BYD_BAT_WRITE
        virtual modb::ResponseError Write(const uint16_t /*registerAddress*/, const uint16_t /*registerCount*/, const uint8_t* /*source*/)
        {
            // Not supported
            return modb::ResponseError::IllegalFunction;
        }
#endif
    };

    class Reg300 : public modb::ModbusRegisters
    {
    public:
        Reg300()
            : modb::ModbusRegisters(REG300_OFFSET, REG300_COUNT)
        {
            // 301,303-308 matters

            // 301 - 3         status(*): ACTIVE - [0..5]<>[STANDBY,INACTIVE,DARKSTART,ACTIVE,FAULT,UPDATING]
            // 302 - 0         always 0
            // 303 - 128       mode(*): normal
            // 304 - 3200      soc: 32% // ccs => ChargeLoopReq.display_parameters.present_soc
            // 305 - 24000     tot cap: 24kWh // ccs => ChargeLoopReq.display_parameters.battery_energy_capacity
            // 306 - 7680      remaining cap: 7.68kWh // ccs => calculate from 304, 305
            // 307 - 0         max/target discharge power: 0W (0W > restricts to no discharge)
            // 308 - 4312      max/target charge power: 4.3kW (during charge), both 307&308 can be set (>0) at the same time
            // 309 - 1734      Batt Voltage outer (0 if status !=3, maybe a contactor closes when active): 173.4V // ccs => 311
            // 310 - 61760     Current Power to API: if>32768... -(65535-61760)=3775W; ll9877: uint16 to int16: 61760 => -3776W
            // 311 - 1732      Batt Voltage inner: 173.2V // ccs => DCChargeLoopReq.ev_present_voltage
            // 312 - 61760     =r310
            // 313 - 140       temp min: 14 degrees (if below 0....65535-t)
            // 314 - 150       temp max: 15 degrees (if below 0....65535-t)
            // 315 - 0         always 0
            // 316 - 0         always 0
            // 317 - 101       counter charge hi
            // 318 - 9912      counter charge lo....65536*101+9912 = 6629048 Wh?
            // 319 - 0         always 0
            // 320 - 0         always 0
            // 321 - 92        counter discharge hi
            // 322 - 7448      counter discharge lo....65536*92+7448 = 6036760 Wh?
            // 323 - 230       device temp (23 degrees)
            // 324 - 9850      soh (value/100 %) = 98.5%
            SetRegisterUint16(0, 1); // *
            SetRegisterUint16(1, 0);
            SetRegisterUint16(2, 128);
            SetRegisterUint16(3, 3200);
            SetRegisterUint16(4, 24000);
            SetRegisterUint16(5, 7680);
            SetRegisterUint16(6, 0); // *
            SetRegisterUint16(7, 0); // *
            SetRegisterUint16(8, 1734);
            SetRegisterUint16(9, 61760);
            SetRegisterUint16(10, 1732);
            SetRegisterUint16(11, 61760);
            SetRegisterUint16(12, 140);
            SetRegisterUint16(13, 150);
            SetRegisterUint16(14, 0);
            SetRegisterUint16(15, 0);
            SetRegisterUint16(16, 101);
            SetRegisterUint16(17, 9912);
            SetRegisterUint16(18, 0);
            SetRegisterUint16(19, 0);
            SetRegisterUint16(20, 92);
            SetRegisterUint16(21, 7448);
            SetRegisterUint16(22, 230);
            SetRegisterUint16(23, 9850);
        }

        virtual ~Reg300() { }

#ifndef ENABLE_BYD_BAT_WRITE
        virtual modb::ResponseError Write(const uint16_t /*registerAddress*/, const uint16_t /*registerCount*/, const uint8_t* /*source*/)
        {
            // Not supported
            return modb::ResponseError::IllegalFunction;
        }
#endif
    };

    class Reg400 : public modb::ModbusRegisters
    {
    public:
        Reg400()
            : modb::ModbusRegisters(REG400_OFFSET, REG400_COUNT)
        {
            // All registers are written by Gen24

            // 401 0002 0000 => r301=0, 0001 => r301=1, 0002 => r301=3, Gen24 writes
            // if 401=0002 and r301=3(active) then Gen24 measures bat. voltage
            // 402 ff00 toggles 00ff-ff00 (Gen24 reads value from 402, and then toggles it and writes to BYD)
            // 403 003c always 003c
            // 404 0000 always 0
            // 405 0000 always 0
            // 406 63bf = unix epoch timestamp hi ...timestamp for last error/restart?
            // 407 9b67 = unix epoch timestamp lo ...63bf 9b67 => Thursday 12 January 2023 09:59:24
            // 408 0000 always 0
            // 409 0001 always 0001 when 406 and 407 has been written
            // 410 0202 0202 when 301=3, 0002 when 301=0
            // 411 0101 0101,0102,0103 seen
            // 412 0000 always 0
            // 413 0000 always 0
            // 414 0000 always 0
            // 415 0000 always 0
            // 416 0000 always 0
            // 417 0000 always 0
            // 418 0fb2 =r311 Gen24 Measured Voltage? (4018 => 401,8V)
            // 419 0fb7 =r309 BYD reported Voltage (4023 => 402,3V)
            SetRegisterUint16(1, 0xff00);
        }

        virtual ~Reg400() { }
    };

    class Reg1000 : public modb::ModbusRegisters
    {
    public:
        Reg1000()
            : modb::ModbusRegisters(REG1000_OFFSET, REG1000_COUNT)
        { }

        virtual ~Reg1000() { }
    };

    class Reg12288 : public modb::ModbusRegisters
    {
    public:
        Reg12288()
            : modb::ModbusRegisters(REG12288_OFFSET, REG12288_COUNT)
        { }

        virtual ~Reg12288() { }
    };

    // mutable: cause Read() is const
    mutable Reg100 m_reg100;
    mutable Reg200 m_reg200;
    mutable Reg300 m_reg300;
    mutable Reg400 m_reg400;
    mutable Reg1000 m_reg1000;
    mutable Reg12288 m_reg12288;
};

} // namespace byd
