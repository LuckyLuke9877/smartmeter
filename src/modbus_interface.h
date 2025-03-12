#pragma once

#include <stdint.h>

// Do not use "modbus", it conflicts with existing namespace esphome::modbus
namespace modb
{

enum class ResponseError : uint8_t
{
    None = 0x00,
    IllegalFunction = 0X01,
    IllegalAddress = 0X02,
    IllegalValue = 0X03,
    DeviceFailure = 0X04
};

// Byte size of 1 register, to calculate bytes needed for Read()
constexpr auto REGISTER_SIZE = sizeof(uint16_t);

// Interface to read from or write to data-model (e.g. sunspec, battery)
// Note: all data read or write are big endian
class IModbusRegisters
{
public:
    virtual ~IModbusRegisters() { }

    // Read registers into target buffer
    // Use REGISTER_SIZE * registerCount to determine required bytes for target buffer
    virtual ResponseError Read(const uint16_t registerAddress, const uint16_t registerCount, uint8_t* target) const = 0;
    // Write registers from source buffer
    virtual ResponseError Write(const uint16_t registerAddress, const uint16_t registerCount, const uint8_t* source) = 0;
};

} // namespace modb
