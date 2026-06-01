#include <gtest/gtest.h>
#include "modbus_test_utils.h"
#include "../src/sunspec_meter_model.h"

using namespace sunspec;
using namespace testutils;

namespace
{

constexpr auto VALUE1 = 1.1f;
constexpr auto VALUE2 = 22.22f;
constexpr auto VALUE3 = 333.333f;
constexpr auto VALUE4 = 0.4444f;

} // namespace

class SunspecMeterModelTest : public ::testing::Test
{
protected:
    void CheckFloatValues(uint32_t registerAddress)
    {
        auto reg = m_meter.GetRegister(registerAddress, 8);
        ASSERT_EQ(ToFloatLittleEndian(&reg[0]), VALUE1);
        ASSERT_EQ(ToFloatLittleEndian(&reg[2]), VALUE2);
        ASSERT_EQ(ToFloatLittleEndian(&reg[4]), VALUE3);
        ASSERT_EQ(ToFloatLittleEndian(&reg[6]), VALUE4);
    }

    MeterModel m_meter;
};

TEST_F(SunspecMeterModelTest, Constructor_InitializedRegisters)
{
    auto reg = m_meter.GetRegister(40000, 197);
    // register are in big endian
    // uint32_t : 0x53 75 6e 53
    // uint32_be: 0x53 6e 75 53
    ASSERT_EQ(reg[0], 0x7553);
    ASSERT_EQ(reg[1], 0x536e);
    ASSERT_EQ(__builtin_bswap32(*(uint32_t*)&reg[0]), 0x53756e53); // same test as before
    ASSERT_EQ(__builtin_bswap16(reg[2]), 1);
    ASSERT_EQ(__builtin_bswap16(reg[3]), 65);
    ASSERT_EQ(IsEqualString(&reg[4], "Fronius"), true);
    ASSERT_EQ(IsEqualString(&reg[20], "Smart Meter 63A"), true);
    ASSERT_EQ(IsEqualString(&reg[36], "<primary>"), true);
    ASSERT_EQ(IsEqualString(&reg[44], "1.0"), true);
    ASSERT_EQ(IsEqualString(&reg[52], "18370117"), true);
    ASSERT_EQ(__builtin_bswap16(reg[68]), 240);
    ASSERT_EQ(__builtin_bswap16(reg[69]), 213);
    ASSERT_EQ(__builtin_bswap16(reg[70]), 124);
    ASSERT_EQ(__builtin_bswap16(reg[195]), 0xFFFF);
    ASSERT_EQ(__builtin_bswap16(reg[196]), 0);
    for (size_t i = 71; i < 195; i++)
    {
        ASSERT_EQ(reg[i], 0);
    }
}

TEST_F(SunspecMeterModelTest, GetRegister_InvalidRegisterIndex_NoResult)
{
    ASSERT_EQ(m_meter.GetRegister(39999, 1).size(), 0);
    ASSERT_EQ(m_meter.GetRegister(40197, 1).size(), 0);
}

TEST_F(SunspecMeterModelTest, GetRegister_ValidRegisterIndex_ResultOk)
{
    ASSERT_EQ(m_meter.GetRegister(40000, 1).size(), 1);
    ASSERT_EQ(m_meter.GetRegister(40000, 197).size(), 197);
    ASSERT_EQ(m_meter.GetRegister(40189, 8).size(), 8);
    ASSERT_EQ(m_meter.GetRegister(40196, 1).size(), 1);
}

TEST_F(SunspecMeterModelTest, GetRegister_InvalidRegisterCount_NoResult)
{
    ASSERT_EQ(m_meter.GetRegister(40000, 0).size(), 0);
    ASSERT_EQ(m_meter.GetRegister(40000, 198).size(), 0);
    ASSERT_EQ(m_meter.GetRegister(40189, 9).size(), 0);
    ASSERT_EQ(m_meter.GetRegister(40196, 2).size(), 0);
}

TEST_F(SunspecMeterModelTest, SetAcCurrent_ResultOk)
{
    m_meter.SetAcCurrent(VALUE1, VALUE2, VALUE3, VALUE4);
    CheckFloatValues(40071);
}

TEST_F(SunspecMeterModelTest, SetVoltageToNeutral_ResultOk)
{
    m_meter.SetVoltageToNeutral(VALUE1, VALUE2, VALUE3, VALUE4);
    CheckFloatValues(40079);
}

TEST_F(SunspecMeterModelTest, SetVoltagePhaseToPhase_ResultOk)
{
    m_meter.SetVoltagePhaseToPhase(VALUE1, VALUE2, VALUE3, VALUE4);
    CheckFloatValues(40087);
}

TEST_F(SunspecMeterModelTest, SetFrequency_ResultOk)
{
    m_meter.SetFrequency(VALUE1);

    auto reg = m_meter.GetRegister(40095, 2);
    ASSERT_EQ(ToFloatLittleEndian(&reg[0]), VALUE1);
}

TEST_F(SunspecMeterModelTest, SetPower_ResultOk)
{
    m_meter.SetPower(VALUE1, VALUE2, VALUE3, VALUE4);
    CheckFloatValues(40097);
}

TEST_F(SunspecMeterModelTest, SetApparentPower_ResultOk)
{
    m_meter.SetApparentPower(VALUE1, VALUE2, VALUE3, VALUE4);
    CheckFloatValues(40105);
}

TEST_F(SunspecMeterModelTest, SetReactivePower_ResultOk)
{
    m_meter.SetReactivePower(VALUE1, VALUE2, VALUE3, VALUE4);
    CheckFloatValues(40113);
}

TEST_F(SunspecMeterModelTest, SetPowerFactor_ResultOk)
{
    m_meter.SetPowerFactor(VALUE1, VALUE2, VALUE3, VALUE4);
    CheckFloatValues(40121);
}

TEST_F(SunspecMeterModelTest, SetTotalWattHoursExported_ResultOk)
{
    m_meter.SetTotalWattHoursExported(VALUE1, VALUE2, VALUE3, VALUE4);
    CheckFloatValues(40129);
}

TEST_F(SunspecMeterModelTest, SetTotalWattHoursImported_ResultOk)
{
    m_meter.SetTotalWattHoursImported(VALUE1, VALUE2, VALUE3, VALUE4);
    CheckFloatValues(40137);
}

TEST_F(SunspecMeterModelTest, SetTotalVaHoursExported_ResultOk)
{
    m_meter.SetTotalVaHoursExported(VALUE1, VALUE2, VALUE3, VALUE4);
    CheckFloatValues(40145);
}

TEST_F(SunspecMeterModelTest, SetTotalVaHoursImported)
{
    m_meter.SetTotalVaHoursImported(VALUE1, VALUE2, VALUE3, VALUE4);
    CheckFloatValues(40153);
}

TEST_F(SunspecMeterModelTest, Read_Valid_ReturnsOk)
{
    const auto address = sunspec::REGISTER_OFFSET;
    const auto count = sunspec::REGISTER_TOTAL_COUNT;
    std::vector<uint8_t> data(count * modb::REGISTER_SIZE);
    ASSERT_EQ(m_meter.Read(address, count, &data[0]), modb::ResponseError::None);
    auto reg = m_meter.GetRegister(address, count);
    ASSERT_EQ(std::memcmp(&reg[0], &data[0], count * sizeof(uint16_t)), 0);

    // change some values
    m_meter.SetTotalVaHoursImported(VALUE1, VALUE2, VALUE3, VALUE4);
    ASSERT_EQ(m_meter.Read(address, count, &data[0]), modb::ResponseError::None);
    reg = m_meter.GetRegister(address, count);
    ASSERT_EQ(std::memcmp(&reg[0], &data[0], count * sizeof(uint16_t)), 0);
}

TEST_F(SunspecMeterModelTest, Read_InvalidAddress_ReturnsError)
{
    const auto address = sunspec::REGISTER_OFFSET + sunspec::REGISTER_TOTAL_COUNT;
    const auto count = 5;
    std::vector<uint8_t> data(count * sizeof(uint16_t));
    ASSERT_EQ(m_meter.Read(address, count, &data[0]), modb::ResponseError::IllegalAddress);
}

TEST_F(SunspecMeterModelTest, Read_InvalidCount_ReturnsError)
{
    const auto address = sunspec::REGISTER_OFFSET;
    const auto count = sunspec::REGISTER_TOTAL_COUNT + 1;
    std::vector<uint8_t> data(count * sizeof(uint16_t));
    ASSERT_EQ(m_meter.Read(address, count, &data[0]), modb::ResponseError::IllegalAddress);
}

TEST_F(SunspecMeterModelTest, Read_ValidAddress768_ReturnsOk)
{
    const auto address = 768;
    const auto count = 1;
    std::vector<uint8_t> data(count * modb::REGISTER_SIZE);
    ASSERT_EQ(m_meter.Read(address, count, &data[0]), modb::ResponseError::None);
    ASSERT_EQ(data[0], 0);
    ASSERT_EQ(data[1], 0);
}

TEST_F(SunspecMeterModelTest, Read_InvalidAddress768Count_ReturnsError)
{
    const auto address = 768;
    const auto count = 2;
    std::vector<uint8_t> data(count * modb::REGISTER_SIZE);
    ASSERT_EQ(m_meter.Read(address, count, &data[0]), modb::ResponseError::IllegalAddress);
}

TEST_F(SunspecMeterModelTest, Read_ValidAddress1706_ReturnsOk)
{
    const auto address = 1706;
    const auto count = 1;
    std::vector<uint8_t> data(count * modb::REGISTER_SIZE);
    ASSERT_EQ(m_meter.Read(address, count, &data[0]), modb::ResponseError::None);
    ASSERT_EQ(data[0], 0);
    ASSERT_EQ(data[1], 0);
}

TEST_F(SunspecMeterModelTest, Read_InvalidAddress1706Count_ReturnsError)
{
    const auto address = 1706;
    const auto count = 2;
    std::vector<uint8_t> data(count * modb::REGISTER_SIZE);
    ASSERT_EQ(m_meter.Read(address, count, &data[0]), modb::ResponseError::IllegalAddress);
}

TEST_F(SunspecMeterModelTest, Write_ReturnsError)
{
    const auto address = 40000;
    const auto count = 5;
    std::vector<uint8_t> data(count * sizeof(uint16_t));
    ASSERT_EQ(m_meter.Write(address, count, &data[0]), modb::ResponseError::IllegalFunction);
}
